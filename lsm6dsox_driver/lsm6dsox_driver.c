// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/string.h>

/* Register addresses. */
#define LSM6DSOX_REG_WHO_AM_I		0x0f
#define LSM6DSOX_REG_CTRL1_XL		0x10
#define LSM6DSOX_REG_CTRL2_G		0x11
#define LSM6DSOX_REG_CTRL3_C		0x12
#define LSM6DSOX_REG_INT1_CTRL		0x0d
#define LSM6DSOX_REG_OUTX_L_G		0x22
#define LSM6DSOX_REG_OUTX_L_A		0x28

#define LSM6DSOX_WHO_AM_I_VALUE		0x6c

/* CTRL1_XL: accelerometer output data rate and full scale. */
#define LSM6DSOX_ODR_XL_MASK		GENMASK(7, 4)
#define LSM6DSOX_ODR_XL_104HZ		(0x4 << 4)
#define LSM6DSOX_FS_XL_MASK		GENMASK(3, 2)
#define LSM6DSOX_FS_XL_2G		(0x0 << 2)

/* CTRL2_G: gyroscope output data rate and full scale. */
#define LSM6DSOX_ODR_G_MASK		GENMASK(7, 4)
#define LSM6DSOX_ODR_G_104HZ		(0x4 << 4)
#define LSM6DSOX_FS_G_MASK		GENMASK(3, 2)
#define LSM6DSOX_FS_G_250DPS		(0x0 << 2)
#define LSM6DSOX_FS_125_MASK		BIT(1)

/* CTRL3_C control bits. */
#define LSM6DSOX_SW_RESET		BIT(0)
#define LSM6DSOX_BDU			BIT(6)

/* INT1_CTRL: route data-ready events to INT1. */
#define LSM6DSOX_INT1_DRDY_XL		BIT(0)
#define LSM6DSOX_INT1_DRDY_G		BIT(1)

#define LSM6DSOX_SAMP_FREQ_HZ		104
#define LSM6DSOX_ACCEL_SCALE_UG		61000
#define LSM6DSOX_GYRO_SCALE_UDPS	8750000
#define LSM6DSOX_RESET_DELAY_MS		50
#define LSM6DSOX_STARTUP_DELAY_MS	20
#define LSM6DSOX_INT1_RETRIES		3
#define LSM6DSOX_INT1_RETRY_DELAY_MS	20

struct lsm6dsox_odr_entry {
	int hz;
	u8 value;
};

static const struct lsm6dsox_odr_entry lsm6dsox_odr_table[] = {
	{ 26,  0x2 << 4 },
	{ 52,  0x3 << 4 },
	{ 104, 0x4 << 4 },
	{ 208, 0x5 << 4 },
};

struct lsm6dsox_data {
	struct i2c_client *client;
	struct iio_trigger *trig;
	struct mutex lock;
	int accel_odr;
	int gyro_odr;
	bool buffer_enabled;
	atomic_t irq_count;
	atomic_t sample_count;
};
struct lsm6dsox_scan {
	s16 accel_x;
	s16 accel_y;
	s16 accel_z;
	s16 gyro_x;
	s16 gyro_y;
	s16 gyro_z;
	s64 timestamp;
} __aligned(8);

static int lsm6dsox_read_xyz(struct i2c_client *client, u8 start_reg,
			     s16 *x, s16 *y, s16 *z);

static const struct iio_chan_spec lsm6dsox_channels[] = {
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.address = 0x28,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.address = 0x2a,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.address = 0x2c,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 2,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.address = 0x22,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 3,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.address = 0x24,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 4,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.address = 0x26,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 5,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(6),
};

static const unsigned long lsm6dsox_scan_masks[] = {
	GENMASK(5, 0), /* accel XYZ + gyro XYZ */
	0,             /* 结束标志 */
};

static irqreturn_t lsm6dsox_irq_thread(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int count = atomic_inc_return(&data->irq_count);

	if (count <= 10 || count % 100 == 0)
		dev_info(&data->client->dev, "data-ready irq count=%d\n", count);

	iio_trigger_poll(data->trig);

	return IRQ_HANDLED;
}

static irqreturn_t lsm6dsox_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	struct lsm6dsox_scan scan;
	s16 ax, ay, az;
	s16 gx, gy, gz;
	int ret;

	memset(&scan, 0, sizeof(scan));

	ret = lsm6dsox_read_xyz(data->client, LSM6DSOX_REG_OUTX_L_A,
				&ax, &ay, &az);
	if (ret < 0)
		goto done;

	ret = lsm6dsox_read_xyz(data->client, LSM6DSOX_REG_OUTX_L_G,
				&gx, &gy, &gz);
	if (ret < 0)
		goto done;

	scan.accel_x = ax;
	scan.accel_y = ay;
	scan.accel_z = az;
	scan.gyro_x = gx;
	scan.gyro_y = gy;
	scan.gyro_z = gz;

	iio_push_to_buffers_with_timestamp(indio_dev, &scan,
					  pf->timestamp);
	if (atomic_inc_return(&data->sample_count) == 1)
		dev_info(&data->client->dev, "first IIO buffer frame queued\n");

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int lsm6dsox_validate_trigger(struct iio_dev *indio_dev,
				     struct iio_trigger *trig)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);

	return data->trig == trig ? 0 : -EINVAL;
}

static int lsm6dsox_write_int1_ctrl(struct i2c_client *client, u8 value,
					    const char *operation)
{
	int attempt;
	int ret = 0;

	for (attempt = 0; attempt < LSM6DSOX_INT1_RETRIES; attempt++) {
		ret = i2c_smbus_write_byte_data(client, LSM6DSOX_REG_INT1_CTRL,
						       value);
		if (ret >= 0)
			return 0;

		if (attempt + 1 < LSM6DSOX_INT1_RETRIES)
			msleep(LSM6DSOX_INT1_RETRY_DELAY_MS);
	}

	dev_err(&client->dev, "failed to %s INT1_CTRL after %d attempts: %d\n",
		operation, LSM6DSOX_INT1_RETRIES, ret);
	return ret;
}

static int lsm6dsox_config_int1_drdy(struct i2c_client *client)
{
	int ret;

	ret = lsm6dsox_write_int1_ctrl(client,
					 LSM6DSOX_INT1_DRDY_XL |
					 LSM6DSOX_INT1_DRDY_G,
					 "enable");
	if (ret < 0)
		return ret;

	dev_info(&client->dev, "INT1 data-ready interrupt enabled\n");
	return 0;
}

static int lsm6dsox_disable_int1_drdy(struct i2c_client *client)
{
	int ret;

	ret = lsm6dsox_write_int1_ctrl(client, 0, "disable");
	if (ret < 0)
		return ret;

	dev_info(&client->dev, "INT1 data-ready interrupt disabled\n");
	return 0;
}

static int lsm6dsox_set_trigger_state(struct iio_trigger *trig, bool state)
{
	/*
	 * INT1 is owned by the buffer lifecycle below.  A trigger can be selected
	 * before the buffer is enabled, so using this callback to toggle hardware
	 * leaves the sensor state dependent on IIO trigger reference counting.
	 */
	return 0;
}

static const struct iio_trigger_ops lsm6dsox_trigger_ops = {
	.set_trigger_state = lsm6dsox_set_trigger_state,
	.validate_device = iio_trigger_validate_own_device,
};

static int lsm6dsox_buffer_postenable(struct iio_dev *indio_dev)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->lock);
	if (!data->buffer_enabled) {
		atomic_set(&data->irq_count, 0);
		atomic_set(&data->sample_count, 0);
		ret = lsm6dsox_config_int1_drdy(data->client);
		if (!ret)
			data->buffer_enabled = true;
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int lsm6dsox_buffer_predisable(struct iio_dev *indio_dev)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->lock);
	if (data->buffer_enabled) {
		ret = lsm6dsox_disable_int1_drdy(data->client);
		/*
		 * Teardown must not poison the next enable cycle.  The hardware state
		 * is unknown after an I2C error, so force postenable() to reprogram it.
		 */
		data->buffer_enabled = false;
	}
	mutex_unlock(&data->lock);

	if (ret < 0)
		dev_warn(&data->client->dev,
			 "continuing buffer teardown after INT1 disable failure: %d\n",
			 ret);

	return 0;
}

static const struct iio_buffer_setup_ops lsm6dsox_buffer_ops = {
	.postenable = lsm6dsox_buffer_postenable,
	.predisable = lsm6dsox_buffer_predisable,
};

static int lsm6dsox_soft_reset(struct i2c_client *client)
{
	int value;
	int ret;

	value = i2c_smbus_read_byte_data(client, LSM6DSOX_REG_CTRL3_C);
	if (value < 0) {
		dev_err(&client->dev, "failed to read CTRL3_C: %d\n", value);
		return value;
	}

	dev_info(&client->dev, "CTRL3_C=0x%02x\n", value);

	ret = i2c_smbus_write_byte_data(client, LSM6DSOX_REG_CTRL3_C,
					 value | LSM6DSOX_SW_RESET);
	if (ret < 0) {
		dev_err(&client->dev, "failed to write CTRL3_C: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "successfully started software reset\n");
	msleep(LSM6DSOX_RESET_DELAY_MS);

	value = i2c_smbus_read_byte_data(client, LSM6DSOX_REG_CTRL3_C);
	if (value < 0) {
		dev_err(&client->dev, "failed to read CTRL3_C: %d\n", value);
		return value;
	}

	if (value & LSM6DSOX_SW_RESET) {
		dev_err(&client->dev, "software reset did not complete\n");
		return -ETIMEDOUT;
	}

	dev_info(&client->dev,
		 "software reset complete, CTRL3_C=0x%02x\n", value);
	return 0;
}

static int lsm6dsox_enable_bdu(struct i2c_client *client)
{
	int value;
	int ret;

	value = i2c_smbus_read_byte_data(client, LSM6DSOX_REG_CTRL3_C);
	if (value < 0) {
		dev_err(&client->dev, "failed to read CTRL3_C: %d\n", value);
		return value;
	}

	ret = i2c_smbus_write_byte_data(client, LSM6DSOX_REG_CTRL3_C,
					 value | LSM6DSOX_BDU);
	if (ret < 0) {
		dev_err(&client->dev, "failed to write CTRL3_C: %d\n", ret);
		return ret;
	}

	value = i2c_smbus_read_byte_data(client, LSM6DSOX_REG_CTRL3_C);
	if (value < 0) {
		dev_err(&client->dev, "failed to read CTRL3_C: %d\n", value);
		return value;
	}

	if (!(value & LSM6DSOX_BDU)) {
		dev_err(&client->dev, "failed to enable BDU\n");
		return -EIO;
	}

	dev_info(&client->dev, "BDU enabled, CTRL3_C=0x%02x\n", value);
	return 0;
}

static int lsm6dsox_config_accel(struct i2c_client *client)
{
	const int config_mask = LSM6DSOX_ODR_XL_MASK |
				LSM6DSOX_FS_XL_MASK;
	const int config_value = LSM6DSOX_ODR_XL_104HZ |
				 LSM6DSOX_FS_XL_2G;
	int value;
	int ret;

	value = i2c_smbus_read_byte_data(client, LSM6DSOX_REG_CTRL1_XL);
	if (value < 0) {
		dev_err(&client->dev, "failed to read CTRL1_XL: %d\n", value);
		return value;
	}

	value &= ~config_mask;
	value |= config_value;

	ret = i2c_smbus_write_byte_data(client, LSM6DSOX_REG_CTRL1_XL,
					 value);
	if (ret < 0) {
		dev_err(&client->dev, "failed to write CTRL1_XL: %d\n", ret);
		return ret;
	}

	value = i2c_smbus_read_byte_data(client, LSM6DSOX_REG_CTRL1_XL);
	if (value < 0) {
		dev_err(&client->dev, "failed to read CTRL1_XL: %d\n", value);
		return value;
	}

	if ((value & config_mask) != config_value) {
		dev_err(&client->dev,
			"failed to configure accelerometer, CTRL1_XL=0x%02x\n",
			value);
		return -EIO;
	}

	dev_info(&client->dev,
		 "accelerometer configured, CTRL1_XL=0x%02x\n", value);
	return 0;
}

static int lsm6dsox_config_gyro(struct i2c_client *client)
{
	const int config_mask = LSM6DSOX_ODR_G_MASK |
				LSM6DSOX_FS_G_MASK |
				LSM6DSOX_FS_125_MASK;
	const int config_value = LSM6DSOX_ODR_G_104HZ |
				 LSM6DSOX_FS_G_250DPS;
	int value;
	int ret;

	value = i2c_smbus_read_byte_data(client, LSM6DSOX_REG_CTRL2_G);
	if (value < 0) {
		dev_err(&client->dev, "failed to read CTRL2_G: %d\n", value);
		return value;
	}

	value &= ~config_mask;
	value |= config_value;

	ret = i2c_smbus_write_byte_data(client, LSM6DSOX_REG_CTRL2_G,
					 value);
	if (ret < 0) {
		dev_err(&client->dev, "failed to write CTRL2_G: %d\n", ret);
		return ret;
	}

	value = i2c_smbus_read_byte_data(client, LSM6DSOX_REG_CTRL2_G);
	if (value < 0) {
		dev_err(&client->dev, "failed to read CTRL2_G: %d\n", value);
		return value;
	}

	if ((value & config_mask) != config_value) {
		dev_err(&client->dev,
			"failed to configure gyroscope, CTRL2_G=0x%02x\n",
			value);
		return -EIO;
	}

	dev_info(&client->dev,
		 "gyroscope configured, CTRL2_G=0x%02x\n", value);
	return 0;
}
static int lsm6dsox_set_odr(struct lsm6dsox_data *data,
			    enum iio_chan_type type, int hz)
{
	struct i2c_client *client = data->client;
	const struct lsm6dsox_odr_entry *odr = NULL;
	int reg;
	int mask;
	int value;
	int ret;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lsm6dsox_odr_table); i++) {
		if (lsm6dsox_odr_table[i].hz == hz) {
			odr = &lsm6dsox_odr_table[i];
			break;
		}
	}

	if (!odr) {
		dev_err(&client->dev,
			"unsupported sampling frequency: %d Hz\n", hz);
		return -EINVAL;
	}

	if (type == IIO_ACCEL) {
		reg = LSM6DSOX_REG_CTRL1_XL;
		mask = LSM6DSOX_ODR_XL_MASK;
	} else if (type == IIO_ANGL_VEL) {
		reg = LSM6DSOX_REG_CTRL2_G;
		mask = LSM6DSOX_ODR_G_MASK;
	} else {
		return -EINVAL;
	}

	value = i2c_smbus_read_byte_data(client, reg);
	if (value < 0)
		return value;

	value &= ~mask;
	value |= odr->value;

	ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0)
		return ret;

	value = i2c_smbus_read_byte_data(client, reg);
	if (value < 0)
		return value;

	if ((value & mask) != odr->value) {
		dev_err(&client->dev,
			"failed to set %d Hz, register 0x%02x=0x%02x\n",
			hz, reg, value);
		return -EIO;
	}

	if (type == IIO_ACCEL)
		data->accel_odr = hz;
	else
		data->gyro_odr = hz;

	dev_info(&client->dev,
		 "sampling frequency set to %d Hz, register 0x%02x=0x%02x\n",
		 hz, reg, value);

	return 0;
}

static int lsm6dsox_read_xyz(struct i2c_client *client, u8 start_reg,
			     s16 *x, s16 *y, s16 *z)
{
	u8 data[6];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, start_reg,
					    sizeof(data), data);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed to read XYZ block at 0x%02x: %d\n",
			start_reg, ret);
		return ret;
	}

	if (ret != sizeof(data)) {
		dev_err(&client->dev,
			"short XYZ read at 0x%02x: got %d bytes\n",
			start_reg, ret);
		return -EIO;
	}

	*x = (s16)(((u16)data[1] << 8) | data[0]);
	*y = (s16)(((u16)data[3] << 8) | data[2]);
	*z = (s16)(((u16)data[5] << 8) | data[4]);

	return 0;
}

static int lsm6dsox_read_raw(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     int *val, int *val2, long mask)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	u8 raw[2];
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = i2c_smbus_read_i2c_block_data(data->client,
						chan->address,
						sizeof(raw), raw);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"failed to read raw channel at 0x%02lx: %d\n",
				chan->address, ret);
			return ret;
		}

		if (ret != sizeof(raw)) {
			dev_err(&data->client->dev,
				"short raw read at 0x%02lx: got %d bytes\n",
				chan->address, ret);
			return -EIO;
		}

		/* Samples are signed 16-bit values stored low byte first. */
		*val = (s16)(((u16)raw[1] << 8) | raw[0]);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;

		if (chan->type == IIO_ACCEL)
			*val2 = IIO_G_TO_M_S_2(LSM6DSOX_ACCEL_SCALE_UG);
		else if (chan->type == IIO_ANGL_VEL)
			*val2 = IIO_DEGREE_TO_RAD(LSM6DSOX_GYRO_SCALE_UDPS);
		else
			return -EINVAL;

		return IIO_VAL_INT_PLUS_NANO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->type == IIO_ACCEL)
			*val = data->accel_odr;
		else if (chan->type == IIO_ANGL_VEL)
			*val = data->gyro_odr;
		else
			return -EINVAL;

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}
static int lsm6dsox_write_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      int val, int val2, long mask)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ || val2 != 0)
		return -EINVAL;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = lsm6dsox_set_odr(data, chan->type, val);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static const struct iio_info lsm6dsox_iio_info = {
	.read_raw = lsm6dsox_read_raw,
	.write_raw = lsm6dsox_write_raw,
	.validate_trigger = lsm6dsox_validate_trigger,
};

static int lsm6dsox_probe(struct i2c_client *client)
{
	struct lsm6dsox_data *data;
	struct iio_dev *indio_dev;
	s16 ax, ay, az;
	s16 gx, gy, gz;
	int ret;

	dev_info(&client->dev, "my probe entered, addr=0x%02x\n",
		 client->addr);

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev,
			"adapter lacks required SMBus byte/block access\n");
		return -EOPNOTSUPP;
	}

	ret = i2c_smbus_read_byte_data(client, LSM6DSOX_REG_WHO_AM_I);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read WHO_AM_I: %d\n", ret);
		return ret;
	}

	if (ret != LSM6DSOX_WHO_AM_I_VALUE) {
		dev_err(&client->dev,
			"unexpected WHO_AM_I: got 0x%02x, expected 0x%02x\n",
			ret, LSM6DSOX_WHO_AM_I_VALUE);
		return -ENODEV;
	}

	dev_info(&client->dev, "WHO_AM_I=0x%02x\n", ret);

	ret = lsm6dsox_soft_reset(client);
	if (ret < 0)
		return ret;

	ret = lsm6dsox_enable_bdu(client);
	if (ret < 0)
		return ret;

	ret = lsm6dsox_config_accel(client);
	if (ret < 0)
		return ret;

	ret = lsm6dsox_config_gyro(client);
	if (ret < 0)
		return ret;

	msleep(LSM6DSOX_STARTUP_DELAY_MS);

	ret = lsm6dsox_read_xyz(client, LSM6DSOX_REG_OUTX_L_A,
				&ax, &ay, &az);
	if (ret < 0)
		return ret;

	dev_info(&client->dev, "accel raw: x=%d y=%d z=%d\n", ax, ay, az);

	ret = lsm6dsox_read_xyz(client, LSM6DSOX_REG_OUTX_L_G,
				&gx, &gy, &gz);
	if (ret < 0)
		return ret;

	dev_info(&client->dev, "gyro raw: x=%d y=%d z=%d\n", gx, gy, gz);

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);
	data->accel_odr = LSM6DSOX_SAMP_FREQ_HZ;
	data->gyro_odr = LSM6DSOX_SAMP_FREQ_HZ;

	indio_dev->name = "lsm6dsox";
	indio_dev->info = &lsm6dsox_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = lsm6dsox_channels;
	indio_dev->num_channels = ARRAY_SIZE(lsm6dsox_channels);
	indio_dev->available_scan_masks = lsm6dsox_scan_masks;

	i2c_set_clientdata(client, indio_dev);

	data->trig = devm_iio_trigger_alloc(&client->dev, "%s-dev%d",
					    indio_dev->name,
					    iio_device_id(indio_dev));
	if (!data->trig)
		return -ENOMEM;

	data->trig->ops = &lsm6dsox_trigger_ops;
	data->trig->dev.parent = &client->dev;
	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(&client->dev, data->trig);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register trigger: %d\n", ret);
		return ret;
	}

	indio_dev->trig = iio_trigger_get(data->trig);
	dev_info(&client->dev, "IIO trigger registered\n");

	ret = devm_iio_triggered_buffer_setup(&client->dev,
					      indio_dev,
					      iio_pollfunc_store_time,
					      lsm6dsox_trigger_handler,
					      &lsm6dsox_buffer_ops);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed to setup triggered buffer: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "IIO triggered buffer setup complete\n");

	if (client->irq > 0) {
		atomic_set(&data->irq_count, 0);
		atomic_set(&data->sample_count, 0);

		ret = devm_request_threaded_irq(&client->dev,
						client->irq,
						NULL,
						lsm6dsox_irq_thread,
						IRQF_ONESHOT,
						dev_name(&client->dev),
						indio_dev);
		if (ret < 0) {
			dev_err(&client->dev, "failed to request irq %d: %d\n",
				client->irq, ret);
			return ret;
		}

		dev_info(&client->dev, "data-ready irq registered: irq=%d\n",
			 client->irq);
	} else {
		dev_warn(&client->dev, "no irq configured in device tree\n");
	}

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register IIO device: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "IIO device registered\n");
	return 0;
}

static void lsm6dsox_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	/* Balance the reference taken when the driver's trigger became current. */
	if (indio_dev && indio_dev->trig)
		iio_trigger_put(indio_dev->trig);
}

static const struct of_device_id lsm6dsox_of_match[] = {
	{ .compatible = "study,lsm6dsox-minimal" },
	{ }
};
MODULE_DEVICE_TABLE(of, lsm6dsox_of_match);

static struct i2c_driver lsm6dsox_driver = {
	.driver = {
		.name = "my_lsm6dsox",
		.of_match_table = lsm6dsox_of_match,
	},
	.probe_new = lsm6dsox_probe,
	.remove = lsm6dsox_remove,
};
module_i2c_driver(lsm6dsox_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("GJ-King");
MODULE_DESCRIPTION("My LSM6DSOX I2C driver");
