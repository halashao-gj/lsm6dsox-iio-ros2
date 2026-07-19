#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <glob.h>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Lsm6dsoxImuPublisher : public rclcpp::Node {
public:
  Lsm6dsoxImuPublisher() : Node("lsm6dsox_imu_publisher") {
    const std::string requested_device_path =
        declare_parameter<std::string>("device_path", "");
    const std::string device_name =
        declare_parameter<std::string>("device_name", "lsm6dsox");

    device_path_ = requested_device_path.empty()
                       ? find_iio_device(device_name)
                       : requested_device_path;
    device_node_ = make_device_node_path(device_path_);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    gyro_bias_ = read_vector_parameter("gyro_bias", {0.0, 0.0, 0.0});
    angular_velocity_covariance_ = read_vector_parameter(
        "angular_velocity_covariance_diagonal", {0.0, 0.0, 0.0});
    linear_acceleration_covariance_ = read_vector_parameter(
        "linear_acceleration_covariance_diagonal", {0.0, 0.0, 0.0});

    // Scale values change only when the sensor range changes, so read them once.
    accel_scale_ = read_iio_value("in_accel_scale");
    gyro_scale_ = read_iio_value("in_anglvel_scale");

    warn_if_timestamp_clock_is_not_realtime();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS());
    diagnostic_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);
    diagnostic_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&Lsm6dsoxImuPublisher::publish_diagnostics, this));

    device_fd_ = ::open(device_node_.c_str(), O_RDONLY | O_CLOEXEC);
    if (device_fd_ < 0) {
      throw std::runtime_error("failed to open " + device_node_ + ": " +
                               std::strerror(errno));
    }

    running_.store(true);
    try {
      worker_thread_ =
          std::thread(&Lsm6dsoxImuPublisher::read_loop, this);
    } catch (...) {
      running_.store(false);
      ::close(device_fd_);
      device_fd_ = -1;
      throw;
    }

    RCLCPP_INFO(get_logger(),
                "reading buffered IIO frames from %s (sysfs: %s)",
                device_node_.c_str(), device_path_.c_str());
    RCLCPP_INFO(get_logger(), "accel_scale=%.9f gyro_scale=%.9f",
                accel_scale_, gyro_scale_);
    RCLCPP_INFO(get_logger(), "frame_id=%s gyro_bias=[%.9f, %.9f, %.9f]",
                frame_id_.c_str(), gyro_bias_[0], gyro_bias_[1],
                gyro_bias_[2]);
  }

  ~Lsm6dsoxImuPublisher() override {
    // poll() wakes every 100 ms, notices this flag, and leaves the read loop.
    running_.store(false);

    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    if (device_fd_ >= 0) {
      ::close(device_fd_);
      device_fd_ = -1;
    }
  }

private:
  static constexpr size_t kFrameSize = 24;

  static std::string read_text(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("failed to open " + path);
    }

    std::string value;
    std::getline(file, value);

    const size_t null_position = value.find('\0');
    if (null_position != std::string::npos) {
      value.resize(null_position);
    }

    return value;
  }

  static double read_number(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("failed to open " + path);
    }

    double value = 0.0;
    file >> value;
    if (!file.good() && !file.eof()) {
      throw std::runtime_error("failed to read " + path);
    }

    return value;
  }

  static int64_t read_integer(const std::string &path) {
    std::ifstream file(path);
    int64_t value = 0;

    if (!file.is_open()) {
      throw std::runtime_error("failed to open " + path);
    }

    file >> value;
    if (!file.good() && !file.eof()) {
      throw std::runtime_error("failed to read " + path);
    }

    return value;
  }

  static std::string find_iio_device(const std::string &device_name) {
    glob_t matches{};
    const int ret = glob("/sys/bus/iio/devices/iio:device*", 0, nullptr,
                         &matches);

    if (ret != 0) {
      globfree(&matches);
      throw std::runtime_error("no IIO devices found");
    }

    for (size_t i = 0; i < matches.gl_pathc; ++i) {
      const std::string path = matches.gl_pathv[i];

      try {
        if (read_text(path + "/name") == device_name) {
          globfree(&matches);
          return path;
        }
      } catch (const std::exception &) {
      }
    }

    globfree(&matches);
    throw std::runtime_error("IIO device named " + device_name +
                             " not found");
  }

  static std::string make_device_node_path(const std::string &device_path) {
    const size_t separator = device_path.find_last_of('/');

    if (separator == std::string::npos || separator + 1 >= device_path.size()) {
      throw std::runtime_error("invalid IIO sysfs path: " + device_path);
    }

    return "/dev/" + device_path.substr(separator + 1);
  }

  static int16_t read_le16(const uint8_t *data) {
    const uint16_t value = static_cast<uint16_t>(data[0]) |
                           (static_cast<uint16_t>(data[1]) << 8);

    return static_cast<int16_t>(value);
  }

  static int64_t read_le64(const uint8_t *data) {
    uint64_t value = 0;

    for (size_t i = 0; i < sizeof(value); ++i) {
      value |= static_cast<uint64_t>(data[i]) << (i * 8);
    }

    return static_cast<int64_t>(value);
  }

  std::array<double, 3> read_vector_parameter(
      const std::string &name, const std::array<double, 3> &default_value) {
    const auto values = declare_parameter<std::vector<double>>(
        name, {default_value[0], default_value[1], default_value[2]});

    if (values.size() != default_value.size()) {
      throw std::runtime_error("parameter " + name + " must contain 3 values");
    }

    return {values[0], values[1], values[2]};
  }

  double read_iio_value(const std::string &name) const {
    return read_number(device_path_ + "/" + name);
  }

  void warn_if_timestamp_clock_is_not_realtime() {
    try {
      const std::string clock =
          read_text(device_path_ + "/current_timestamp_clock");

      if (clock != "realtime") {
        RCLCPP_WARN(get_logger(),
                    "IIO timestamp clock is '%s', but ROS header timestamps "
                    "expect realtime",
                    clock.c_str());
      }
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "failed to check IIO timestamp clock: %s",
                  error.what());
    }
  }

  void read_loop() {
    pollfd descriptor{};
    descriptor.fd = device_fd_;
    descriptor.events = POLLIN;

    while (running_.load() && rclcpp::ok()) {
      descriptor.revents = 0;
      const int ready = ::poll(&descriptor, 1, 100);

      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }

        RCLCPP_ERROR(get_logger(), "IIO poll failed: %s",
                     std::strerror(errno));
        break;
      }

      if (ready == 0) {
        continue;
      }

      if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        RCLCPP_ERROR(get_logger(), "IIO poll returned error events: 0x%x",
                     descriptor.revents);
        break;
      }

      if (!(descriptor.revents & POLLIN)) {
        continue;
      }

      std::array<uint8_t, kFrameSize> frame{};
      const ssize_t bytes =
          ::read(device_fd_, frame.data(), frame.size());

      if (bytes < 0) {
        if (errno == EINTR) {
          continue;
        }

        RCLCPP_ERROR(get_logger(), "IIO read failed: %s",
                     std::strerror(errno));
        break;
      }

      if (bytes != static_cast<ssize_t>(frame.size())) {
        RCLCPP_WARN(get_logger(), "short IIO frame: got %zd, expected %zu",
                    bytes, frame.size());
        continue;
      }

      publish_frame(frame);
    }
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    diagnostic_msgs::msg::DiagnosticStatus status;

    array.header.stamp = now();
    status.name = "lsm6dsox/FIFO";
    status.hardware_id = device_path_;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "FIFO healthy";

    const auto add_value = [&status](const std::string &name, int64_t value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = name;
      item.value = std::to_string(value);
      status.values.push_back(std::move(item));
    };

    try {
      const std::string root = device_path_ + "/buffer/";
      const int64_t faulted = read_integer(root + "hwfifo_faulted");
      const int64_t overflows =
          read_integer(root + "hwfifo_overflow_count");
      const int64_t i2c_errors =
          read_integer(root + "hwfifo_i2c_error_count");
      const int64_t tag_errors =
          read_integer(root + "hwfifo_tag_error_count");
      const int64_t dropped =
          read_integer(root + "hwfifo_dropped_scan_count");
      const int64_t unknown_losses =
          read_integer(root + "hwfifo_unknown_loss_count");
      const int64_t discontinuities =
          read_integer(root + "hwfifo_discontinuity_count");
      const int64_t recoveries =
          read_integer(root + "hwfifo_recovery_count");
      const int64_t recovery_failures =
          read_integer(root + "hwfifo_recovery_failure_count");
      const int64_t timestamp_backwards =
          read_integer(root + "hwfifo_timestamp_backward_count");
      const int64_t timestamp_gaps =
          read_integer(root + "hwfifo_timestamp_gap_count");
      const int64_t timestamp_rollovers =
          read_integer(root + "hwfifo_timestamp_rollover_count");

      add_value("faulted", faulted);
      add_value("overflows", overflows);
      add_value("i2c_errors", i2c_errors);
      add_value("tag_errors", tag_errors);
      add_value("dropped_scans", dropped);
      add_value("unknown_loss_events", unknown_losses);
      add_value("discontinuities", discontinuities);
      add_value("recoveries", recoveries);
      add_value("recovery_failures", recovery_failures);
      add_value("timestamp_backwards", timestamp_backwards);
      add_value("timestamp_gaps", timestamp_gaps);
      add_value("timestamp_rollovers", timestamp_rollovers);
      add_value("timestamp_tick_ns",
                read_integer(root + "hwfifo_timestamp_tick_ns"));
      add_value("watermark",
                read_integer(root + "hwfifo_watermark"));

      if (faulted || recovery_failures) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "FIFO recovery failed; interrupt remains masked";
      } else if (overflows || i2c_errors || tag_errors || dropped ||
                 unknown_losses || discontinuities || timestamp_backwards ||
                 timestamp_gaps) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "FIFO recovered from a data discontinuity";
      }
    } catch (const std::exception &error) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = std::string("failed to read FIFO diagnostics: ") +
                       error.what();
    }

    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  void publish_frame(const std::array<uint8_t, kFrameSize> &frame) {
    const int16_t accel_x = read_le16(frame.data() + 0);
    const int16_t accel_y = read_le16(frame.data() + 2);
    const int16_t accel_z = read_le16(frame.data() + 4);
    const int16_t gyro_x = read_le16(frame.data() + 6);
    const int16_t gyro_y = read_le16(frame.data() + 8);
    const int16_t gyro_z = read_le16(frame.data() + 10);
    const int64_t timestamp_ns = read_le64(frame.data() + 16);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(timestamp_ns, RCL_SYSTEM_TIME);
    msg.header.frame_id = frame_id_;

    msg.linear_acceleration.x = accel_x * accel_scale_;
    msg.linear_acceleration.y = accel_y * accel_scale_;
    msg.linear_acceleration.z = accel_z * accel_scale_;

    msg.angular_velocity.x = gyro_x * gyro_scale_ - gyro_bias_[0];
    msg.angular_velocity.y = gyro_y * gyro_scale_ - gyro_bias_[1];
    msg.angular_velocity.z = gyro_z * gyro_scale_ - gyro_bias_[2];

    msg.angular_velocity_covariance[0] = angular_velocity_covariance_[0];
    msg.angular_velocity_covariance[4] = angular_velocity_covariance_[1];
    msg.angular_velocity_covariance[8] = angular_velocity_covariance_[2];
    msg.linear_acceleration_covariance[0] = linear_acceleration_covariance_[0];
    msg.linear_acceleration_covariance[4] = linear_acceleration_covariance_[1];
    msg.linear_acceleration_covariance[8] = linear_acceleration_covariance_[2];

    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;

    publisher_->publish(msg);
  }

  std::string device_path_;
  std::string device_node_;
  std::string frame_id_;
  double accel_scale_{0.0};
  double gyro_scale_{0.0};
  std::array<double, 3> gyro_bias_{};
  std::array<double, 3> angular_velocity_covariance_{};
  std::array<double, 3> linear_acceleration_covariance_{};
  int device_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread worker_thread_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Lsm6dsoxImuPublisher>();
  rclcpp::spin(node);
  node.reset();

  rclcpp::shutdown();
  return 0;
}
