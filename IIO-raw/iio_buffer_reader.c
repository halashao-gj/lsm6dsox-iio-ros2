// SPDX-License-Identifier: GPL-2.0-only

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define LSM6DSOX_FRAME_SIZE 24
#define DEFAULT_FRAME_COUNT 20

static volatile sig_atomic_t stop_requested;

static void handle_signal(int signo)
{
	(void)signo;
	stop_requested = 1;
}

static int16_t read_le16(const uint8_t *data)
{
	uint16_t value = (uint16_t)data[0] |
			 ((uint16_t)data[1] << 8);

	return (int16_t)value;
}

static int64_t read_le64(const uint8_t *data)
{
	uint64_t value = 0;
	unsigned int i;

	for (i = 0; i < sizeof(value); i++)
		value |= (uint64_t)data[i] << (i * 8);

	return (int64_t)value;
}

static int read_full_frame(int fd, uint8_t *frame)
{
	size_t offset = 0;

	while (offset < LSM6DSOX_FRAME_SIZE) {
		ssize_t ret = read(fd, frame + offset,
				   LSM6DSOX_FRAME_SIZE - offset);

		if (ret > 0) {
			offset += (size_t)ret;
			continue;
		}

		if (ret == 0) {
			fprintf(stderr, "unexpected end of IIO device\n");
			return -1;
		}

		if (errno == EINTR) {
			if (stop_requested)
				return 1;
			continue;
		}

		fprintf(stderr, "failed to read IIO frame: %s\n",
			strerror(errno));
		return -1;
	}

	return 0;
}

static unsigned long parse_frame_count(const char *text)
{
	char *end;
	unsigned long value;

	errno = 0;
	value = strtoul(text, &end, 10);
	if (errno || *text == '\0' || *end != '\0') {
		fprintf(stderr, "invalid frame count: %s\n", text);
		exit(EXIT_FAILURE);
	}

	return value;
}

int main(int argc, char **argv)
{
	const char *device = "/dev/iio:device1";
	unsigned long frame_count = DEFAULT_FRAME_COUNT;
	int64_t previous_timestamp = 0;
	unsigned long index = 0;
	int fd;

	if (argc > 3) {
		fprintf(stderr, "usage: %s [device] [frame_count]\n", argv[0]);
		return EXIT_FAILURE;
	}

	if (argc >= 2)
		device = argv[1];
	if (argc == 3)
		frame_count = parse_frame_count(argv[2]);

	signal(SIGINT, handle_signal);
	signal(SIGTERM, handle_signal);

	fd = open(device, O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "failed to open %s: %s\n",
			device, strerror(errno));
		return EXIT_FAILURE;
	}

	printf("device=%s frame_size=%d count=%lu (0 means continuous)\n",
	       device, LSM6DSOX_FRAME_SIZE, frame_count);
	printf("index accel_x accel_y accel_z gyro_x gyro_y gyro_z "
	       "timestamp_ns delta_ms\n");

	while (!stop_requested && (!frame_count || index < frame_count)) {
		uint8_t frame[LSM6DSOX_FRAME_SIZE];
		int64_t timestamp;
		double delta_ms = 0.0;
		int ret;

		ret = read_full_frame(fd, frame);
		if (ret != 0) {
			close(fd);
			return ret > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
		}

		timestamp = read_le64(frame + 16);
		if (previous_timestamp)
			delta_ms = (timestamp - previous_timestamp) / 1000000.0;

		printf("%5lu %7d %7d %7d %7d %7d %7d %" PRId64
		       " %8.3f\n",
		       index,
		       read_le16(frame + 0), read_le16(frame + 2),
		       read_le16(frame + 4), read_le16(frame + 6),
		       read_le16(frame + 8), read_le16(frame + 10),
		       timestamp, delta_ms);

		previous_timestamp = timestamp;
		index++;
	}

	close(fd);
	return EXIT_SUCCESS;
}
