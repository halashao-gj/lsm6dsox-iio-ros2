// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/slab.h>

/* Register addresses. */
#define LSM6DSOX_REG_WHO_AM_I		0x0f
#define LSM6DSOX_REG_CTRL1_XL		0x10
#define LSM6DSOX_REG_CTRL2_G		0x11
#define LSM6DSOX_REG_CTRL3_C		0x12
#define LSM6DSOX_REG_CTRL10_C		0x19
#define LSM6DSOX_REG_INT1_CTRL		0x0d
#define LSM6DSOX_REG_FIFO_CTRL1		0x07
#define LSM6DSOX_REG_FIFO_CTRL2		0x08
#define LSM6DSOX_REG_FIFO_CTRL3		0x09
#define LSM6DSOX_REG_FIFO_CTRL4		0x0a
#define LSM6DSOX_REG_FIFO_STATUS1	0x3a
#define LSM6DSOX_REG_TIMESTAMP2		0x42
#define LSM6DSOX_REG_INTERNAL_FREQ_FINE	0x63
#define LSM6DSOX_REG_FIFO_DATA_OUT_TAG	0x78
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
#define LSM6DSOX_TIMESTAMP_EN		BIT(5)

/* INT1_CTRL: route FIFO watermark to INT1. */
#define LSM6DSOX_INT1_FIFO_TH		BIT(3)

/* FIFO configuration and tagged output. */
#define LSM6DSOX_FIFO_WTM_MASK		GENMASK(8, 0)
#define LSM6DSOX_FIFO_BDR_XL_MASK	GENMASK(3, 0)
#define LSM6DSOX_FIFO_BDR_G_MASK		GENMASK(7, 4)
#define LSM6DSOX_FIFO_TIMESTAMP_DEC_MASK	GENMASK(7, 6)
#define LSM6DSOX_FIFO_TIMESTAMP_DEC_1	FIELD_PREP(LSM6DSOX_FIFO_TIMESTAMP_DEC_MASK, 1)
#define LSM6DSOX_FIFO_MODE_MASK		GENMASK(2, 0)
#define LSM6DSOX_FIFO_MODE_BYPASS	0x0
#define LSM6DSOX_FIFO_MODE_CONTINUOUS	0x6
#define LSM6DSOX_FIFO_DIFF_MASK		GENMASK(9, 0)
#define LSM6DSOX_FIFO_EMPTY		BIT(12)
#define LSM6DSOX_FIFO_FULL		BIT(13)
#define LSM6DSOX_FIFO_OVERRUN		BIT(14)
#define LSM6DSOX_FIFO_WATERMARK		BIT(15)
#define LSM6DSOX_FIFO_GYRO_TAG		0x01
#define LSM6DSOX_FIFO_ACCEL_TAG		0x02
#define LSM6DSOX_FIFO_TIMESTAMP_TAG	0x04
#define LSM6DSOX_FIFO_ENTRY_SIZE		7
#define LSM6DSOX_FIFO_ENTRIES_PER_SCAN	3
#define LSM6DSOX_FIFO_MAX_ENTRIES	512
#define LSM6DSOX_FIFO_MAX_SCANS		170
#define LSM6DSOX_FIFO_TARGET_READ_CHUNK	112
#define LSM6DSOX_FIFO_DEFAULT_WATERMARK	4
#define LSM6DSOX_TIMESTAMP_RESET_VALUE	0xaa
#define LSM6DSOX_TIMESTAMP_NOMINAL_NS	25000

#define LSM6DSOX_SAMP_FREQ_HZ		104
#define LSM6DSOX_ACCEL_SCALE_UG		61000
#define LSM6DSOX_GYRO_SCALE_UDPS	8750000
#define LSM6DSOX_RESET_DELAY_MS		50
#define LSM6DSOX_STARTUP_DELAY_MS	20
#define LSM6DSOX_INT1_RETRIES		3
#define LSM6DSOX_INT1_RETRY_DELAY_MS	20
#define LSM6DSOX_RESUME_RETRIES		3
#define LSM6DSOX_RESUME_RETRY_DELAY_MS	20
#define LSM6DSOX_AUTOSUSPEND_DELAY_MS	2000

struct lsm6dsox_odr_entry {
	int hz;
	u8 value;
};

struct lsm6dsox_fs_entry {
	int scale_nano;
	u8 value;
};

static const struct lsm6dsox_odr_entry lsm6dsox_odr_table[] = {
	{ 26,  0x2 << 4 },
	{ 52,  0x3 << 4 },
	{ 104, 0x4 << 4 },
	{ 208, 0x5 << 4 },
};

static const int lsm6dsox_odr_available[] = { 26, 52, 104, 208 };

static const struct lsm6dsox_fs_entry lsm6dsox_accel_fs_table[] = {
	{ IIO_G_TO_M_S_2(61000),  0x0 << 2 }, /* +/-2 g */
	{ IIO_G_TO_M_S_2(122000), 0x2 << 2 }, /* +/-4 g */
	{ IIO_G_TO_M_S_2(244000), 0x3 << 2 }, /* +/-8 g */
	{ IIO_G_TO_M_S_2(488000), 0x1 << 2 }, /* +/-16 g */
};

static const struct lsm6dsox_fs_entry lsm6dsox_gyro_fs_table[] = {
	{ IIO_DEGREE_TO_RAD(4375000), BIT(1) },  /* +/-125 dps */
	{ IIO_DEGREE_TO_RAD(8750000), 0x0 << 2 }, /* +/-250 dps */
	{ IIO_DEGREE_TO_RAD(17500000), 0x1 << 2 }, /* +/-500 dps */
	{ IIO_DEGREE_TO_RAD(35000000), 0x2 << 2 }, /* +/-1000 dps */
	{ IIO_DEGREE_TO_RAD(70000000), 0x3 << 2 }, /* +/-2000 dps */
};

static const int lsm6dsox_accel_scale_available[][2] = {
	{ 0, IIO_G_TO_M_S_2(61000) },
	{ 0, IIO_G_TO_M_S_2(122000) },
	{ 0, IIO_G_TO_M_S_2(244000) },
	{ 0, IIO_G_TO_M_S_2(488000) },
};

static const int lsm6dsox_gyro_scale_available[][2] = {
	{ 0, IIO_DEGREE_TO_RAD(4375000) },
	{ 0, IIO_DEGREE_TO_RAD(8750000) },
	{ 0, IIO_DEGREE_TO_RAD(17500000) },
	{ 0, IIO_DEGREE_TO_RAD(35000000) },
	{ 0, IIO_DEGREE_TO_RAD(70000000) },
};

struct lsm6dsox_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct mutex lock;
	int accel_odr;
	int gyro_odr;
	int accel_scale_nano;
	int gyro_scale_nano;
	unsigned int fifo_watermark;
	unsigned int fifo_hw_entries;
	unsigned int fifo_read_chunk;
	bool buffer_enabled;
	bool fifo_running;
	bool fifo_faulted;
	bool irq_disabled_for_suspend;
	bool timestamp_valid;
	bool hw_timestamp_valid;
	u32 last_hw_timestamp;
	u64 hw_timestamp_wraps;
	s64 hw_timestamp_ref_ns;
	s64 hw_timestamp_tick_ns;
	s64 last_timestamp_ns;
	u8 *fifo_raw;
	u32 *fifo_hw_timestamps;
	struct lsm6dsox_scan *fifo_scans;
	atomic_t irq_count;
	atomic_t sample_count;
	atomic64_t fifo_overflow_count;
	atomic64_t i2c_error_count;
	atomic64_t fifo_tag_error_count;
	atomic64_t fifo_dropped_scan_count;
	atomic64_t fifo_unknown_loss_count;
	atomic64_t fifo_discontinuity_count;
	atomic64_t fifo_recovery_count;
	atomic64_t fifo_recovery_failure_count;
	atomic64_t timestamp_backward_count;
	atomic64_t timestamp_gap_count;
	atomic64_t timestamp_rollover_count;
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

static int lsm6dsox_read_xyz(struct lsm6dsox_data *data, u8 start_reg,
			     s16 *x, s16 *y, s16 *z);

static const struct regmap_config lsm6dsox_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LSM6DSOX_REG_FIFO_DATA_OUT_TAG +
			LSM6DSOX_FIFO_ENTRY_SIZE - 1,
	/* A sensor reset changes hardware registers behind regmap's back. */
	.cache_type = REGCACHE_NONE,
};

static const struct iio_chan_spec lsm6dsox_channels[] = {
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.address = 0x28,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_shared_by_type_available =
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_SCALE),
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
		.info_mask_shared_by_type_available =
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_SCALE),
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
		.info_mask_shared_by_type_available =
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_SCALE),
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
		.info_mask_shared_by_type_available =
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_SCALE),
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
		.info_mask_shared_by_type_available =
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_SCALE),
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
		.info_mask_shared_by_type_available =
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_SCALE),
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

static int lsm6dsox_fifo_drain(struct iio_dev *indio_dev,
			       unsigned int max_scans);
static int lsm6dsox_config_int1_fifo(struct lsm6dsox_data *data);
static int lsm6dsox_disable_int1_fifo(struct lsm6dsox_data *data);

static irqreturn_t lsm6dsox_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct lsm6dsox_data *data = iio_priv(indio_dev);

	atomic_inc(&data->irq_count);
	iio_trigger_poll(data->trig);

	return IRQ_HANDLED;
}

static irqreturn_t lsm6dsox_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	ret = lsm6dsox_fifo_drain(indio_dev, LSM6DSOX_FIFO_MAX_SCANS);
	mutex_unlock(&data->lock);
	if (ret < 0)
		dev_err_ratelimited(&data->client->dev,
				    "failed to drain FIFO: %d\n", ret);

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int lsm6dsox_validate_trigger(struct iio_dev *indio_dev,
				     struct iio_trigger *trig)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);

	return data->trig == trig ? 0 : -EINVAL;
}

static int lsm6dsox_write_int1_ctrl(struct lsm6dsox_data *data, u8 value,
					    const char *operation)
{
	int attempt;
	int ret = 0;

	for (attempt = 0; attempt < LSM6DSOX_INT1_RETRIES; attempt++) {
		ret = regmap_write(data->regmap, LSM6DSOX_REG_INT1_CTRL, value);
		if (!ret)
			return 0;

		if (attempt + 1 < LSM6DSOX_INT1_RETRIES)
			msleep(LSM6DSOX_INT1_RETRY_DELAY_MS);
	}

	dev_err(&data->client->dev, "failed to %s INT1_CTRL after %d attempts: %d\n",
		operation, LSM6DSOX_INT1_RETRIES, ret);
	return ret;
}

static int lsm6dsox_fifo_set_mode(struct lsm6dsox_data *data, u8 mode)
{
	return regmap_update_bits(data->regmap, LSM6DSOX_REG_FIFO_CTRL4,
				  LSM6DSOX_FIFO_MODE_MASK, mode);
}

static int lsm6dsox_fifo_set_batching(struct lsm6dsox_data *data, bool enable)
{
	u8 value = 0;
	size_t i;

	if (enable) {
		for (i = 0; i < ARRAY_SIZE(lsm6dsox_odr_table); i++) {
			if (lsm6dsox_odr_table[i].hz == data->accel_odr) {
				value = lsm6dsox_odr_table[i].value;
				break;
			}
		}

		if (!value)
			return -EINVAL;

		/* Gyro occupies the high nibble, accel the low nibble. */
		value |= value >> 4;
	}

	return regmap_update_bits(data->regmap, LSM6DSOX_REG_FIFO_CTRL3,
				  LSM6DSOX_FIFO_BDR_XL_MASK |
				  LSM6DSOX_FIFO_BDR_G_MASK,
				  value);
}

static int lsm6dsox_fifo_set_timestamp_batching(struct lsm6dsox_data *data,
						 bool enable)
{
	return regmap_update_bits(data->regmap, LSM6DSOX_REG_FIFO_CTRL4,
				  LSM6DSOX_FIFO_TIMESTAMP_DEC_MASK,
				  enable ? LSM6DSOX_FIFO_TIMESTAMP_DEC_1 : 0);
}

static int lsm6dsox_fifo_program_watermark(struct lsm6dsox_data *data)
{
	unsigned int entries = clamp_val(data->fifo_watermark *
					 LSM6DSOX_FIFO_ENTRIES_PER_SCAN,
					 LSM6DSOX_FIFO_ENTRIES_PER_SCAN,
					 LSM6DSOX_FIFO_WTM_MASK);
	int ret;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_FIFO_CTRL1,
				  GENMASK(7, 0), entries & 0xff);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_FIFO_CTRL2,
				  BIT(0), (entries >> 8) & BIT(0));
	if (!ret)
		data->fifo_hw_entries = entries;

	return ret;
}

static int lsm6dsox_reset_hw_timestamp(struct iio_dev *indio_dev)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret;

	ret = regmap_write(data->regmap, LSM6DSOX_REG_TIMESTAMP2,
			   LSM6DSOX_TIMESTAMP_RESET_VALUE);
	if (ret < 0)
		return ret;

	data->hw_timestamp_ref_ns = iio_get_time_ns(indio_dev);
	data->hw_timestamp_valid = false;
	data->hw_timestamp_wraps = 0;
	return 0;
}

static int lsm6dsox_fifo_start_hw(struct iio_dev *indio_dev)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret;

	if (data->fifo_running)
		return 0;

	ret = lsm6dsox_fifo_set_mode(data, LSM6DSOX_FIFO_MODE_BYPASS);
	if (ret < 0)
		return ret;

	ret = lsm6dsox_fifo_set_batching(data, true);
	if (ret < 0)
		return ret;

	ret = lsm6dsox_fifo_set_timestamp_batching(data, true);
	if (ret < 0)
		goto err_disable_batching;

	ret = lsm6dsox_fifo_program_watermark(data);
	if (ret < 0)
		goto err_disable_timestamp;

	ret = lsm6dsox_reset_hw_timestamp(indio_dev);
	if (ret < 0)
		goto err_disable_timestamp;

	ret = lsm6dsox_fifo_set_mode(data, LSM6DSOX_FIFO_MODE_CONTINUOUS);
	if (ret < 0)
		goto err_disable_timestamp;

	ret = lsm6dsox_config_int1_fifo(data);
	if (ret < 0)
		goto err_bypass;

	data->fifo_running = true;
	return 0;

err_bypass:
	lsm6dsox_fifo_set_mode(data, LSM6DSOX_FIFO_MODE_BYPASS);
err_disable_timestamp:
	lsm6dsox_fifo_set_timestamp_batching(data, false);
err_disable_batching:
	lsm6dsox_fifo_set_batching(data, false);
	return ret;
}

static int lsm6dsox_fifo_stop_hw(struct lsm6dsox_data *data)
{
	int first_error = 0;
	int ret;

	ret = lsm6dsox_disable_int1_fifo(data);
	if (ret < 0)
		first_error = ret;

	ret = lsm6dsox_fifo_set_mode(data, LSM6DSOX_FIFO_MODE_BYPASS);
	if (ret < 0 && !first_error)
		first_error = ret;

	ret = lsm6dsox_fifo_set_batching(data, false);
	if (ret < 0 && !first_error)
		first_error = ret;

	ret = lsm6dsox_fifo_set_timestamp_batching(data, false);
	if (ret < 0 && !first_error)
		first_error = ret;

	data->fifo_running = false;
	return first_error;
}

static int lsm6dsox_fifo_recover(struct iio_dev *indio_dev)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	bool restore_irq = data->buffer_enabled;
	int ret;

	atomic64_inc(&data->fifo_discontinuity_count);
	atomic64_inc(&data->fifo_recovery_count);
	data->timestamp_valid = false;
	data->hw_timestamp_valid = false;
	data->fifo_faulted = true;

	ret = lsm6dsox_write_int1_ctrl(data, 0, "mask for recovery");
	if (ret < 0)
		goto fail;

	ret = lsm6dsox_fifo_set_mode(data, LSM6DSOX_FIFO_MODE_BYPASS);
	if (ret < 0)
		goto fail;
	data->fifo_running = false;

	ret = lsm6dsox_fifo_set_batching(data, false);
	if (ret < 0)
		goto fail;

	ret = lsm6dsox_fifo_set_timestamp_batching(data, false);
	if (ret < 0)
		goto fail;

	ret = lsm6dsox_fifo_program_watermark(data);
	if (ret < 0)
		goto fail;

	ret = lsm6dsox_fifo_set_batching(data, true);
	if (ret < 0)
		goto fail;

	ret = lsm6dsox_fifo_set_timestamp_batching(data, true);
	if (ret < 0)
		goto fail;

	ret = lsm6dsox_reset_hw_timestamp(indio_dev);
	if (ret < 0)
		goto fail;

	ret = lsm6dsox_fifo_set_mode(data, LSM6DSOX_FIFO_MODE_CONTINUOUS);
	if (ret < 0)
		goto fail;

	if (restore_irq) {
		ret = lsm6dsox_write_int1_ctrl(data, LSM6DSOX_INT1_FIFO_TH,
					       "restore after recovery");
		if (ret < 0)
			goto fail;
	}

	data->fifo_faulted = false;
	data->fifo_running = true;
	return 0;

fail:
	atomic64_inc(&data->fifo_recovery_failure_count);
	lsm6dsox_write_int1_ctrl(data, 0, "leave masked after recovery failure");
	lsm6dsox_fifo_set_mode(data, LSM6DSOX_FIFO_MODE_BYPASS);
	data->fifo_running = false;
	return ret;
}

static int lsm6dsox_fifo_read_status(struct lsm6dsox_data *data, u16 *status)
{
	u8 raw[2];
	int ret;

	ret = regmap_bulk_read(data->regmap, LSM6DSOX_REG_FIFO_STATUS1,
				 raw, sizeof(raw));
	if (ret < 0) {
		atomic64_inc(&data->i2c_error_count);
		return ret;
	}

	*status = raw[0] | ((u16)raw[1] << 8);
	return 0;
}

static void lsm6dsox_fifo_decode_xyz(const u8 *raw, s16 *x, s16 *y,
				     s16 *z)
{
	*x = (s16)(((u16)raw[1] << 8) | raw[0]);
	*y = (s16)(((u16)raw[3] << 8) | raw[2]);
	*z = (s16)(((u16)raw[5] << 8) | raw[4]);
}

static u32 lsm6dsox_fifo_decode_timestamp(const u8 *raw)
{
	return raw[0] | ((u32)raw[1] << 8) | ((u32)raw[2] << 16) |
	       ((u32)raw[3] << 24);
}

static int lsm6dsox_hw_timestamp_to_ns(struct lsm6dsox_data *data,
					      u32 raw, s64 *timestamp)
{
	u64 extended;

	if (data->hw_timestamp_valid && raw < data->last_hw_timestamp) {
		if (data->last_hw_timestamp - raw > BIT(31)) {
			data->hw_timestamp_wraps++;
			atomic64_inc(&data->timestamp_rollover_count);
		} else {
			atomic64_inc(&data->timestamp_backward_count);
			return -ERANGE;
		}
	}

	data->last_hw_timestamp = raw;
	data->hw_timestamp_valid = true;
	extended = (data->hw_timestamp_wraps << 32) | raw;
	*timestamp = data->hw_timestamp_ref_ns +
		     extended * data->hw_timestamp_tick_ns;
	return 0;
}

static int lsm6dsox_fifo_read_entries(struct iio_dev *indio_dev,
				      unsigned int entries)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	struct lsm6dsox_scan pending = { };
	bool have_accel = false, have_gyro = false, malformed = false;
	unsigned int bytes = entries * LSM6DSOX_FIFO_ENTRY_SIZE;
	unsigned int offset = 0, scan_count = 0, timestamp_count = 0, i;
	s64 period_ns;
	int ret;

	while (offset < bytes) {
		unsigned int chunk = min_t(unsigned int, bytes - offset,
					       data->fifo_read_chunk);

		ret = regmap_bulk_read(data->regmap,
				       LSM6DSOX_REG_FIFO_DATA_OUT_TAG,
				       data->fifo_raw + offset, chunk);
		if (ret < 0) {
			atomic64_inc(&data->i2c_error_count);
			lsm6dsox_fifo_recover(indio_dev);
			return ret;
		}
		offset += chunk;
	}

	for (i = 0; i < entries; i++) {
		const u8 *entry = data->fifo_raw +
				  i * LSM6DSOX_FIFO_ENTRY_SIZE;
		u8 tag = entry[0] >> 3;

		switch (tag) {
		case LSM6DSOX_FIFO_GYRO_TAG:
			if (have_gyro) {
				atomic64_inc(&data->fifo_tag_error_count);
				malformed = true;
			}
			lsm6dsox_fifo_decode_xyz(entry + 1, &pending.gyro_x,
						 &pending.gyro_y, &pending.gyro_z);
			have_gyro = true;
			break;
		case LSM6DSOX_FIFO_ACCEL_TAG:
			if (have_accel) {
				atomic64_inc(&data->fifo_tag_error_count);
				malformed = true;
			}
			lsm6dsox_fifo_decode_xyz(entry + 1, &pending.accel_x,
						 &pending.accel_y, &pending.accel_z);
			have_accel = true;
			break;
		case LSM6DSOX_FIFO_TIMESTAMP_TAG:
			if (timestamp_count >= LSM6DSOX_FIFO_MAX_SCANS) {
				malformed = true;
				break;
			}
			data->fifo_hw_timestamps[timestamp_count++] =
				lsm6dsox_fifo_decode_timestamp(entry + 1);
			break;
		default:
			atomic64_inc(&data->fifo_tag_error_count);
			malformed = true;
			dev_warn_ratelimited(&data->client->dev,
					     "unexpected FIFO tag 0x%02x\n", tag);
			continue;
		}

		if (have_accel && have_gyro) {
			if (scan_count < LSM6DSOX_FIFO_MAX_SCANS)
				data->fifo_scans[scan_count++] = pending;
			else
				malformed = true;
			memset(&pending, 0, sizeof(pending));
			have_accel = false;
			have_gyro = false;
		}
	}
	if (have_accel || have_gyro) {
		atomic64_inc(&data->fifo_tag_error_count);
		malformed = true;
	}

	if (malformed || !scan_count || scan_count != timestamp_count) {
		atomic64_add(scan_count ? scan_count : 1,
			     &data->fifo_dropped_scan_count);
		dev_warn_ratelimited(&data->client->dev,
			"discarding malformed FIFO batch: scans=%u timestamps=%u\n",
			scan_count, timestamp_count);
		lsm6dsox_fifo_recover(indio_dev);
		return -EBADMSG;
	}

	period_ns = div_s64(NSEC_PER_SEC, data->accel_odr);
	for (i = 0; i < scan_count; i++) {
		s64 timestamp;
		s64 previous = i ? data->fifo_scans[i - 1].timestamp :
				     data->last_timestamp_ns;

		ret = lsm6dsox_hw_timestamp_to_ns(data,
						 data->fifo_hw_timestamps[i],
						 &timestamp);
		if (ret < 0 || ((i || data->timestamp_valid) &&
				 timestamp <= previous)) {
			if (!ret)
				atomic64_inc(&data->timestamp_backward_count);
			atomic64_add(scan_count, &data->fifo_dropped_scan_count);
			lsm6dsox_fifo_recover(indio_dev);
			return -ERANGE;
		}

		if ((i || data->timestamp_valid) &&
		    timestamp - previous > period_ns * 2) {
			atomic64_inc(&data->timestamp_gap_count);
			atomic64_inc(&data->fifo_discontinuity_count);
		}
		data->fifo_scans[i].timestamp = timestamp;
	}

	for (i = 0; i < scan_count; i++) {
		s64 timestamp = data->fifo_scans[i].timestamp;

		iio_push_to_buffers_with_timestamp(indio_dev,
						   &data->fifo_scans[i], timestamp);
		data->last_timestamp_ns = timestamp;
	}

	data->timestamp_valid = true;
	atomic_add(scan_count, &data->sample_count);
	return scan_count;
}

static int lsm6dsox_fifo_drain(struct iio_dev *indio_dev,
			       unsigned int max_scans)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	unsigned int total = 0;
	bool first_pass = true;

	while (total < max_scans) {
		unsigned int entries, scans;
		bool recheck;
		u16 status;
		int ret;

		ret = lsm6dsox_fifo_read_status(data, &status);
		if (ret < 0) {
			lsm6dsox_fifo_recover(indio_dev);
			return ret;
		}

		if (status & LSM6DSOX_FIFO_OVERRUN) {
			atomic64_inc(&data->fifo_overflow_count);
			atomic64_inc(&data->fifo_unknown_loss_count);
			dev_warn_ratelimited(&data->client->dev,
					     "FIFO overrun, resetting stream\n");
			ret = lsm6dsox_fifo_recover(indio_dev);
			return ret < 0 ? ret : -EOVERFLOW;
		}

		entries = status & LSM6DSOX_FIFO_DIFF_MASK;
		if ((status & LSM6DSOX_FIFO_EMPTY) ||
		    entries < LSM6DSOX_FIFO_ENTRIES_PER_SCAN)
			break;

		if (!first_pass && entries < data->fifo_hw_entries)
			break;

		scans = min_t(unsigned int,
				  entries / LSM6DSOX_FIFO_ENTRIES_PER_SCAN,
				  max_scans - total);
		entries = scans * LSM6DSOX_FIFO_ENTRIES_PER_SCAN;
		/*
		 * A normal watermark batch is fully consumed by this read and will
		 * deassert INT1. Re-read status only after a delayed handler sees at
		 * least two complete batches, where arrivals during the larger burst
		 * could otherwise leave the FIFO above its threshold.
		 */
		recheck = scans >= data->fifo_watermark * 2;
		ret = lsm6dsox_fifo_read_entries(indio_dev, entries);
		if (ret < 0)
			return ret;

		total += ret;
		if (ret < scans || !recheck)
			break;
		first_pass = false;
	}

	return total;
}

static int lsm6dsox_config_int1_fifo(struct lsm6dsox_data *data)
{
	int ret;

	ret = lsm6dsox_write_int1_ctrl(data,
					 LSM6DSOX_INT1_FIFO_TH,
					 "enable");
	if (ret < 0)
		return ret;

	dev_info(&data->client->dev, "INT1 FIFO watermark interrupt enabled\n");
	return 0;
}

static int lsm6dsox_disable_int1_fifo(struct lsm6dsox_data *data)
{
	int ret;

	ret = lsm6dsox_write_int1_ctrl(data, 0, "disable");
	if (ret < 0)
		return ret;

	dev_info(&data->client->dev, "INT1 FIFO watermark interrupt disabled\n");
	return 0;
}

static int lsm6dsox_set_trigger_state(struct iio_trigger *trig, bool state)
{
	/*
	 * INT1 and the FIFO are owned by the buffer lifecycle below. A trigger can be selected
	 * before the buffer is enabled, so using this callback to toggle hardware
	 * leaves the sensor state dependent on IIO trigger reference counting.
	 */
	return 0;
}

static const struct iio_trigger_ops lsm6dsox_trigger_ops = {
	.set_trigger_state = lsm6dsox_set_trigger_state,
	.validate_device = iio_trigger_validate_own_device,
};

static int lsm6dsox_buffer_preenable(struct iio_dev *indio_dev)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);

	return pm_runtime_resume_and_get(&data->client->dev);
}

static int lsm6dsox_buffer_postenable(struct iio_dev *indio_dev)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->lock);
	if (!data->buffer_enabled) {
		if (data->client->irq <= 0) {
			ret = -ENXIO;
			goto out;
		}

		if (data->accel_odr != data->gyro_odr) {
			dev_err(&data->client->dev,
				"FIFO requires matching accel/gyro ODRs\n");
			ret = -EINVAL;
			goto out;
		}

		atomic_set(&data->irq_count, 0);
		atomic_set(&data->sample_count, 0);
		atomic64_set(&data->fifo_overflow_count, 0);
		atomic64_set(&data->i2c_error_count, 0);
		atomic64_set(&data->fifo_tag_error_count, 0);
		atomic64_set(&data->fifo_dropped_scan_count, 0);
		atomic64_set(&data->fifo_unknown_loss_count, 0);
		atomic64_set(&data->fifo_discontinuity_count, 0);
		atomic64_set(&data->fifo_recovery_count, 0);
		atomic64_set(&data->fifo_recovery_failure_count, 0);
		atomic64_set(&data->timestamp_backward_count, 0);
		atomic64_set(&data->timestamp_gap_count, 0);
		atomic64_set(&data->timestamp_rollover_count, 0);
		data->timestamp_valid = false;
		data->hw_timestamp_valid = false;
		data->fifo_faulted = false;

		ret = lsm6dsox_fifo_start_hw(indio_dev);
		if (ret < 0)
			goto out;

		data->buffer_enabled = true;
		dev_info(&data->client->dev,
			 "FIFO enabled: watermark=%u scans (%u tagged entries), ODR=%d Hz, burst=%u bytes\n",
			 data->fifo_watermark, data->fifo_hw_entries,
			 data->accel_odr, data->fifo_read_chunk);
		goto out;
	}
out:
	mutex_unlock(&data->lock);

	return ret;
}

static int lsm6dsox_buffer_predisable(struct iio_dev *indio_dev)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->lock);
	if (data->buffer_enabled) {
		ret = lsm6dsox_fifo_stop_hw(data);
		/*
		 * Teardown must not poison the next enable cycle.  The hardware state
		 * is unknown after an I2C error, so force postenable() to reprogram it.
		 */
		data->buffer_enabled = false;
		data->fifo_faulted = false;
		data->timestamp_valid = false;
		data->hw_timestamp_valid = false;
		dev_info(&data->client->dev,
			 "FIFO disabled: irqs=%d samples=%d overflows=%lld i2c_errors=%lld tag_errors=%lld\n",
			 atomic_read(&data->irq_count),
			 atomic_read(&data->sample_count),
			 atomic64_read(&data->fifo_overflow_count),
			 atomic64_read(&data->i2c_error_count),
			 atomic64_read(&data->fifo_tag_error_count));
		dev_info(&data->client->dev,
			 "FIFO diagnostics: dropped=%lld unknown_losses=%lld discontinuities=%lld recoveries=%lld recovery_failures=%lld ts_backwards=%lld ts_gaps=%lld ts_rollovers=%lld\n",
			 atomic64_read(&data->fifo_dropped_scan_count),
			 atomic64_read(&data->fifo_unknown_loss_count),
			 atomic64_read(&data->fifo_discontinuity_count),
			 atomic64_read(&data->fifo_recovery_count),
			 atomic64_read(&data->fifo_recovery_failure_count),
			 atomic64_read(&data->timestamp_backward_count),
			 atomic64_read(&data->timestamp_gap_count),
			 atomic64_read(&data->timestamp_rollover_count));
	}
	mutex_unlock(&data->lock);

	if (ret < 0)
		dev_warn(&data->client->dev,
			 "continuing buffer teardown after INT1 disable failure: %d\n",
			 ret);

	return 0;
}

static int lsm6dsox_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	struct device *dev = &data->client->dev;

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return 0;
}

static const struct iio_buffer_setup_ops lsm6dsox_buffer_ops = {
	.preenable = lsm6dsox_buffer_preenable,
	.postenable = lsm6dsox_buffer_postenable,
	.predisable = lsm6dsox_buffer_predisable,
	.postdisable = lsm6dsox_buffer_postdisable,
};

static int lsm6dsox_soft_reset(struct lsm6dsox_data *data)
{
	unsigned int value;
	int ret;

	ret = regmap_read(data->regmap, LSM6DSOX_REG_CTRL3_C, &value);
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to read CTRL3_C: %d\n", ret);
		return ret;
	}

	dev_info(&data->client->dev, "CTRL3_C=0x%02x\n", value);

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL3_C,
				 LSM6DSOX_SW_RESET, LSM6DSOX_SW_RESET);
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to write CTRL3_C: %d\n", ret);
		return ret;
	}

	dev_info(&data->client->dev, "successfully started software reset\n");
	msleep(LSM6DSOX_RESET_DELAY_MS);

	ret = regmap_read(data->regmap, LSM6DSOX_REG_CTRL3_C, &value);
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to read CTRL3_C: %d\n", ret);
		return ret;
	}

	if (value & LSM6DSOX_SW_RESET) {
		dev_err(&data->client->dev, "software reset did not complete\n");
		return -ETIMEDOUT;
	}

	dev_info(&data->client->dev,
		 "software reset complete, CTRL3_C=0x%02x\n", value);
	return 0;
}

static int lsm6dsox_enable_bdu(struct lsm6dsox_data *data)
{
	unsigned int value;
	int ret;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL3_C,
				 LSM6DSOX_BDU, LSM6DSOX_BDU);
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to write CTRL3_C: %d\n", ret);
		return ret;
	}

	ret = regmap_read(data->regmap, LSM6DSOX_REG_CTRL3_C, &value);
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to read CTRL3_C: %d\n", ret);
		return ret;
	}

	if (!(value & LSM6DSOX_BDU)) {
		dev_err(&data->client->dev, "failed to enable BDU\n");
		return -EIO;
	}

	dev_info(&data->client->dev, "BDU enabled, CTRL3_C=0x%02x\n", value);
	return 0;
}

static int lsm6dsox_init_hw_timestamp(struct lsm6dsox_data *data)
{
	unsigned int fine;
	int ret;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL10_C,
				 LSM6DSOX_TIMESTAMP_EN, LSM6DSOX_TIMESTAMP_EN);
	if (ret < 0)
		return ret;

	ret = regmap_read(data->regmap, LSM6DSOX_REG_INTERNAL_FREQ_FINE,
			  &fine);
	if (ret < 0)
		return ret;

	/* AN5192: 25 us nominal tick, corrected by signed FREQ_FINE trim. */
	data->hw_timestamp_tick_ns = LSM6DSOX_TIMESTAMP_NOMINAL_NS -
				       ((s8)fine * 37500) / 1000;
	dev_info(&data->client->dev,
		 "hardware timestamp enabled: fine=%d tick=%lld ns\n",
		 (s8)fine, data->hw_timestamp_tick_ns);
	return 0;
}

static int lsm6dsox_config_accel(struct lsm6dsox_data *data)
{
	const int config_mask = LSM6DSOX_ODR_XL_MASK |
				LSM6DSOX_FS_XL_MASK;
	const int config_value = LSM6DSOX_ODR_XL_104HZ |
				 LSM6DSOX_FS_XL_2G;
	unsigned int value;
	int ret;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL1_XL,
				 config_mask, config_value);
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to write CTRL1_XL: %d\n", ret);
		return ret;
	}

	ret = regmap_read(data->regmap, LSM6DSOX_REG_CTRL1_XL, &value);
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to read CTRL1_XL: %d\n", ret);
		return ret;
	}

	if ((value & config_mask) != config_value) {
		dev_err(&data->client->dev,
			"failed to configure accelerometer, CTRL1_XL=0x%02x\n",
			value);
		return -EIO;
	}

	dev_info(&data->client->dev,
		 "accelerometer configured, CTRL1_XL=0x%02x\n", value);
	return 0;
}

static int lsm6dsox_config_gyro(struct lsm6dsox_data *data)
{
	const int config_mask = LSM6DSOX_ODR_G_MASK |
				LSM6DSOX_FS_G_MASK |
				LSM6DSOX_FS_125_MASK;
	const int config_value = LSM6DSOX_ODR_G_104HZ |
				 LSM6DSOX_FS_G_250DPS;
	unsigned int value;
	int ret;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL2_G,
				 config_mask, config_value);
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to write CTRL2_G: %d\n", ret);
		return ret;
	}

	ret = regmap_read(data->regmap, LSM6DSOX_REG_CTRL2_G, &value);
	if (ret < 0) {
		dev_err(&data->client->dev, "failed to read CTRL2_G: %d\n", ret);
		return ret;
	}

	if ((value & config_mask) != config_value) {
		dev_err(&data->client->dev,
			"failed to configure gyroscope, CTRL2_G=0x%02x\n",
			value);
		return -EIO;
	}

	dev_info(&data->client->dev,
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
	unsigned int value;
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

	ret = regmap_update_bits(data->regmap, reg, mask, odr->value);
	if (ret < 0)
		return ret;

	ret = regmap_read(data->regmap, reg, &value);
	if (ret < 0)
		return ret;

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

static int lsm6dsox_set_full_scale(struct lsm6dsox_data *data,
				   enum iio_chan_type type, int scale_nano)
{
	const struct lsm6dsox_fs_entry *table;
	const struct lsm6dsox_fs_entry *new_fs = NULL, *old_fs = NULL;
	struct device *dev = &data->client->dev;
	int *cached_scale;
	unsigned int value;
	unsigned int table_size;
	u8 reg, mask;
	int ret, rollback_ret;
	unsigned int i;

	if (type == IIO_ACCEL) {
		table = lsm6dsox_accel_fs_table;
		table_size = ARRAY_SIZE(lsm6dsox_accel_fs_table);
		cached_scale = &data->accel_scale_nano;
		reg = LSM6DSOX_REG_CTRL1_XL;
		mask = LSM6DSOX_FS_XL_MASK;
	} else if (type == IIO_ANGL_VEL) {
		table = lsm6dsox_gyro_fs_table;
		table_size = ARRAY_SIZE(lsm6dsox_gyro_fs_table);
		cached_scale = &data->gyro_scale_nano;
		reg = LSM6DSOX_REG_CTRL2_G;
		mask = LSM6DSOX_FS_G_MASK | LSM6DSOX_FS_125_MASK;
	} else {
		return -EINVAL;
	}

	for (i = 0; i < table_size; i++) {
		if (table[i].scale_nano == scale_nano)
			new_fs = &table[i];
		if (table[i].scale_nano == *cached_scale)
			old_fs = &table[i];
	}

	if (!new_fs || !old_fs)
		return -EINVAL;
	if (new_fs == old_fs)
		return 0;

	ret = regmap_update_bits(data->regmap, reg, mask, new_fs->value);
	if (ret < 0)
		goto rollback;

	ret = regmap_read(data->regmap, reg, &value);
	if (ret < 0)
		goto rollback;

	if ((value & mask) != new_fs->value) {
		dev_err(dev,
			"failed to verify scale 0.%09d, register 0x%02x=0x%02x\n",
			scale_nano, reg, value);
		ret = -EIO;
		goto rollback;
	}

	*cached_scale = scale_nano;
	dev_info(dev, "scale set to 0.%09d, register 0x%02x=0x%02x\n",
		 scale_nano, reg, value);
	return 0;

rollback:
	rollback_ret = regmap_update_bits(data->regmap, reg, mask,
					  old_fs->value);
	if (rollback_ret < 0)
		dev_err(dev, "failed to roll back scale at register 0x%02x: %d\n",
			reg, rollback_ret);
	return ret;
}

static int lsm6dsox_restore_sensor_config(struct lsm6dsox_data *data)
{
	const struct lsm6dsox_odr_entry *accel_odr = NULL, *gyro_odr = NULL;
	const struct lsm6dsox_fs_entry *accel_fs = NULL, *gyro_fs = NULL;
	unsigned int accel_value, gyro_value;
	unsigned int value;
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(lsm6dsox_odr_table); i++) {
		if (lsm6dsox_odr_table[i].hz == data->accel_odr)
			accel_odr = &lsm6dsox_odr_table[i];
		if (lsm6dsox_odr_table[i].hz == data->gyro_odr)
			gyro_odr = &lsm6dsox_odr_table[i];
	}

	for (i = 0; i < ARRAY_SIZE(lsm6dsox_accel_fs_table); i++)
		if (lsm6dsox_accel_fs_table[i].scale_nano ==
		    data->accel_scale_nano)
			accel_fs = &lsm6dsox_accel_fs_table[i];

	for (i = 0; i < ARRAY_SIZE(lsm6dsox_gyro_fs_table); i++)
		if (lsm6dsox_gyro_fs_table[i].scale_nano ==
		    data->gyro_scale_nano)
			gyro_fs = &lsm6dsox_gyro_fs_table[i];

	if (!accel_odr || !gyro_odr || !accel_fs || !gyro_fs)
		return -EINVAL;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL3_C,
				 LSM6DSOX_BDU, LSM6DSOX_BDU);
	if (ret < 0)
		return ret;

	accel_value = accel_odr->value | accel_fs->value;
	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL1_XL,
				 LSM6DSOX_ODR_XL_MASK | LSM6DSOX_FS_XL_MASK,
				 accel_value);
	if (ret < 0)
		return ret;

	gyro_value = gyro_odr->value | gyro_fs->value;
	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL2_G,
				 LSM6DSOX_ODR_G_MASK | LSM6DSOX_FS_G_MASK |
				 LSM6DSOX_FS_125_MASK, gyro_value);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL10_C,
				 LSM6DSOX_TIMESTAMP_EN, LSM6DSOX_TIMESTAMP_EN);
	if (ret < 0)
		return ret;

	ret = regmap_read(data->regmap, LSM6DSOX_REG_CTRL1_XL, &value);
	if (ret < 0)
		return ret;
	if ((value & (LSM6DSOX_ODR_XL_MASK | LSM6DSOX_FS_XL_MASK)) !=
	    accel_value)
		return -EIO;

	ret = regmap_read(data->regmap, LSM6DSOX_REG_CTRL2_G, &value);
	if (ret < 0)
		return ret;
	if ((value & (LSM6DSOX_ODR_G_MASK | LSM6DSOX_FS_G_MASK |
		      LSM6DSOX_FS_125_MASK)) != gyro_value)
		return -EIO;

	data->timestamp_valid = false;
	data->hw_timestamp_valid = false;
	return 0;
}

static int lsm6dsox_power_down(struct lsm6dsox_data *data)
{
	int first_error = 0;
	int ret;

	if (data->fifo_running) {
		ret = lsm6dsox_fifo_stop_hw(data);
		if (ret < 0)
			first_error = ret;
	}

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL1_XL,
				 LSM6DSOX_ODR_XL_MASK, 0);
	if (ret < 0 && !first_error)
		first_error = ret;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL2_G,
				 LSM6DSOX_ODR_G_MASK, 0);
	if (ret < 0 && !first_error)
		first_error = ret;

	ret = regmap_update_bits(data->regmap, LSM6DSOX_REG_CTRL10_C,
				 LSM6DSOX_TIMESTAMP_EN, 0);
	if (ret < 0 && !first_error)
		first_error = ret;

	data->timestamp_valid = false;
	data->hw_timestamp_valid = false;
	return first_error;
}

static int lsm6dsox_read_xyz(struct lsm6dsox_data *data, u8 start_reg,
			     s16 *x, s16 *y, s16 *z)
{
	u8 raw[6];
	int ret;

	ret = regmap_bulk_read(data->regmap, start_reg, raw, sizeof(raw));
	if (ret < 0) {
		dev_err(&data->client->dev,
			"failed to read XYZ block at 0x%02x: %d\n",
			start_reg, ret);
		return ret;
	}

	*x = (s16)(((u16)raw[1] << 8) | raw[0]);
	*y = (s16)(((u16)raw[3] << 8) | raw[2]);
	*z = (s16)(((u16)raw[5] << 8) | raw[4]);

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
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = pm_runtime_resume_and_get(&data->client->dev);
		if (ret < 0) {
			iio_device_release_direct_mode(indio_dev);
			return ret;
		}

		mutex_lock(&data->lock);
		ret = regmap_bulk_read(data->regmap, chan->address,
					       raw, sizeof(raw));
		mutex_unlock(&data->lock);
		pm_runtime_mark_last_busy(&data->client->dev);
		pm_runtime_put_autosuspend(&data->client->dev);
		iio_device_release_direct_mode(indio_dev);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"failed to read raw channel at 0x%02lx: %d\n",
				chan->address, ret);
			return ret;
		}

		/* Samples are signed 16-bit values stored low byte first. */
		*val = (s16)(((u16)raw[1] << 8) | raw[0]);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;

		if (chan->type == IIO_ACCEL)
			*val2 = data->accel_scale_nano;
		else if (chan->type == IIO_ANGL_VEL)
			*val2 = data->gyro_scale_nano;
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

static int lsm6dsox_read_avail(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       const int **vals, int *type, int *length,
			       long mask)
{
	if (mask == IIO_CHAN_INFO_SAMP_FREQ) {
		*vals = lsm6dsox_odr_available;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(lsm6dsox_odr_available);
		return IIO_AVAIL_LIST;
	}

	if (mask != IIO_CHAN_INFO_SCALE)
		return -EINVAL;

	*type = IIO_VAL_INT_PLUS_NANO;
	if (chan->type == IIO_ACCEL) {
		*vals = (const int *)lsm6dsox_accel_scale_available;
		*length = ARRAY_SIZE(lsm6dsox_accel_scale_available) * 2;
	} else if (chan->type == IIO_ANGL_VEL) {
		*vals = (const int *)lsm6dsox_gyro_scale_available;
		*length = ARRAY_SIZE(lsm6dsox_gyro_scale_available) * 2;
	} else {
		return -EINVAL;
	}

	return IIO_AVAIL_LIST;
}

static int lsm6dsox_write_raw_get_fmt(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      long mask)
{
	if (mask == IIO_CHAN_INFO_SCALE &&
	    (chan->type == IIO_ACCEL || chan->type == IIO_ANGL_VEL))
		return IIO_VAL_INT_PLUS_NANO;
	if (mask == IIO_CHAN_INFO_SAMP_FREQ)
		return IIO_VAL_INT;

	return -EINVAL;
}

static int lsm6dsox_write_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      int val, int val2, long mask)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int old_accel_odr, old_gyro_odr;
	int ret;

	if (mask == IIO_CHAN_INFO_SAMP_FREQ && val2 != 0)
		return -EINVAL;
	if (mask == IIO_CHAN_INFO_SCALE && val != 0)
		return -EINVAL;
	if (mask != IIO_CHAN_INFO_SAMP_FREQ && mask != IIO_CHAN_INFO_SCALE)
		return -EINVAL;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(&data->client->dev);
	if (ret < 0) {
		iio_device_release_direct_mode(indio_dev);
		return ret;
	}

	mutex_lock(&data->lock);
	if (mask == IIO_CHAN_INFO_SCALE) {
		ret = lsm6dsox_set_full_scale(data, chan->type, val2);
		goto out_unlock;
	}

	old_accel_odr = data->accel_odr;
	old_gyro_odr = data->gyro_odr;
	ret = lsm6dsox_set_odr(data, IIO_ACCEL, val);
	if (ret)
		goto odr_rollback;

	ret = lsm6dsox_set_odr(data, IIO_ANGL_VEL, val);
	if (!ret)
		goto out_unlock;

odr_rollback:
	if (lsm6dsox_set_odr(data, IIO_ACCEL, old_accel_odr))
		dev_err(&data->client->dev, "failed to roll back accel ODR\n");
	if (lsm6dsox_set_odr(data, IIO_ANGL_VEL, old_gyro_odr))
		dev_err(&data->client->dev, "failed to roll back gyro ODR\n");
out_unlock:
	mutex_unlock(&data->lock);

	pm_runtime_mark_last_busy(&data->client->dev);
	pm_runtime_put_autosuspend(&data->client->dev);
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int lsm6dsox_hwfifo_set_watermark(struct iio_dev *indio_dev,
					 unsigned int val)
{
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret = 0;

	if (val < 1 || val > LSM6DSOX_FIFO_MAX_SCANS)
		return -EINVAL;

	mutex_lock(&data->lock);
	if (data->buffer_enabled)
		ret = -EBUSY;
	else
		data->fifo_watermark = val;
	mutex_unlock(&data->lock);
	return ret;
}

static ssize_t hwfifo_watermark_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct lsm6dsox_data *data = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", data->fifo_watermark);
}

static ssize_t hwfifo_watermark_min_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "1\n");
}

static ssize_t hwfifo_watermark_max_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", LSM6DSOX_FIFO_MAX_SCANS);
}

static ssize_t hwfifo_overflow_count_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct lsm6dsox_data *data = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lld\n",
			  atomic64_read(&data->fifo_overflow_count));
}

static ssize_t hwfifo_i2c_error_count_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct lsm6dsox_data *data = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lld\n",
			  atomic64_read(&data->i2c_error_count));
}

static ssize_t hwfifo_tag_error_count_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	struct lsm6dsox_data *data = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lld\n",
			  atomic64_read(&data->fifo_tag_error_count));
}

#define LSM6DSOX_COUNTER_ATTR_SHOW(_name, _member) \
static ssize_t _name##_show(struct device *dev, \
			    struct device_attribute *attr, char *buf) \
{ \
	struct lsm6dsox_data *data = iio_priv(dev_to_iio_dev(dev)); \
\
	return sysfs_emit(buf, "%lld\n", atomic64_read(&data->_member)); \
}

LSM6DSOX_COUNTER_ATTR_SHOW(hwfifo_dropped_scan_count,
			   fifo_dropped_scan_count)
LSM6DSOX_COUNTER_ATTR_SHOW(hwfifo_unknown_loss_count,
			   fifo_unknown_loss_count)
LSM6DSOX_COUNTER_ATTR_SHOW(hwfifo_discontinuity_count,
			   fifo_discontinuity_count)
LSM6DSOX_COUNTER_ATTR_SHOW(hwfifo_recovery_count, fifo_recovery_count)
LSM6DSOX_COUNTER_ATTR_SHOW(hwfifo_recovery_failure_count,
			   fifo_recovery_failure_count)
LSM6DSOX_COUNTER_ATTR_SHOW(hwfifo_timestamp_backward_count,
			   timestamp_backward_count)
LSM6DSOX_COUNTER_ATTR_SHOW(hwfifo_timestamp_gap_count, timestamp_gap_count)
LSM6DSOX_COUNTER_ATTR_SHOW(hwfifo_timestamp_rollover_count,
			   timestamp_rollover_count)

static ssize_t hwfifo_read_chunk_bytes_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct lsm6dsox_data *data = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%u\n", data->fifo_read_chunk);
}

static ssize_t hwfifo_timestamp_tick_ns_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct lsm6dsox_data *data = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lld\n", data->hw_timestamp_tick_ns);
}

static ssize_t hwfifo_faulted_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct lsm6dsox_data *data = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%u\n", data->fifo_faulted);
}

static IIO_DEVICE_ATTR_RO(hwfifo_watermark, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_watermark_min, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_watermark_max, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_overflow_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_i2c_error_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_tag_error_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_dropped_scan_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_unknown_loss_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_discontinuity_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_recovery_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_recovery_failure_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_timestamp_backward_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_timestamp_gap_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_timestamp_rollover_count, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_read_chunk_bytes, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_timestamp_tick_ns, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_faulted, 0);

static const struct attribute *lsm6dsox_fifo_attributes[] = {
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_overflow_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_i2c_error_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_tag_error_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_dropped_scan_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_unknown_loss_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_discontinuity_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_recovery_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_recovery_failure_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_timestamp_backward_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_timestamp_gap_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_timestamp_rollover_count.dev_attr.attr,
	&iio_dev_attr_hwfifo_read_chunk_bytes.dev_attr.attr,
	&iio_dev_attr_hwfifo_timestamp_tick_ns.dev_attr.attr,
	&iio_dev_attr_hwfifo_faulted.dev_attr.attr,
	NULL,
};

static const struct iio_info lsm6dsox_iio_info = {
	.read_raw = lsm6dsox_read_raw,
	.read_avail = lsm6dsox_read_avail,
	.write_raw = lsm6dsox_write_raw,
	.write_raw_get_fmt = lsm6dsox_write_raw_get_fmt,
	.validate_trigger = lsm6dsox_validate_trigger,
	.hwfifo_set_watermark = lsm6dsox_hwfifo_set_watermark,
};

static int __maybe_unused lsm6dsox_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	bool restore_fifo;
	int restore_ret;
	int ret;

	mutex_lock(&data->lock);
	restore_fifo = data->fifo_running;
	if (data->buffer_enabled)
		atomic64_inc(&data->fifo_discontinuity_count);

	ret = lsm6dsox_power_down(data);
	if (ret < 0) {
		restore_ret = lsm6dsox_restore_sensor_config(data);
		if (!restore_ret && restore_fifo)
			restore_ret = lsm6dsox_fifo_start_hw(indio_dev);
		if (restore_ret < 0)
			dev_err(dev,
				"failed to restore hardware after suspend error: %d\n",
				restore_ret);
	}
	mutex_unlock(&data->lock);

	if (!ret)
		dev_dbg(dev, "runtime suspended: accel and gyro powered down\n");
	return ret;
}

static int __maybe_unused lsm6dsox_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int attempt;
	int ret;

	mutex_lock(&data->lock);
	for (attempt = 0; attempt < LSM6DSOX_RESUME_RETRIES; attempt++) {
		ret = lsm6dsox_restore_sensor_config(data);
		if (!ret)
			break;
		if (attempt + 1 < LSM6DSOX_RESUME_RETRIES)
			msleep(LSM6DSOX_RESUME_RETRY_DELAY_MS);
	}
	if (ret < 0)
		goto out_power_down;

	msleep(LSM6DSOX_STARTUP_DELAY_MS);
	if (data->buffer_enabled) {
		ret = lsm6dsox_fifo_start_hw(indio_dev);
		if (ret < 0)
			goto out_power_down;
	}

	data->fifo_faulted = false;
	mutex_unlock(&data->lock);
	dev_dbg(dev, "runtime resumed: cached sensor state restored\n");
	return 0;

out_power_down:
	data->fifo_faulted = data->buffer_enabled;
	lsm6dsox_power_down(data);
	mutex_unlock(&data->lock);
	dev_err(dev, "failed to restore hardware after resume: %d\n", ret);
	return ret;
}

static int __maybe_unused lsm6dsox_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret;

	if (data->client->irq > 0) {
		disable_irq(data->client->irq);
		data->irq_disabled_for_suspend = true;
	}
	if (data->buffer_enabled && indio_dev->pollfunc &&
	    indio_dev->pollfunc->irq > 0)
		synchronize_irq(indio_dev->pollfunc->irq);

	ret = pm_runtime_force_suspend(dev);
	if (ret < 0 && data->irq_disabled_for_suspend) {
		enable_irq(data->client->irq);
		data->irq_disabled_for_suspend = false;
	}
	if (!ret)
		dev_info(dev, "system suspended: buffer=%u\n",
			 data->buffer_enabled);

	return ret;
}

static int __maybe_unused lsm6dsox_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (!ret && pm_runtime_suspended(dev)) {
		ret = pm_runtime_resume_and_get(dev);
		if (!ret) {
			pm_runtime_mark_last_busy(dev);
			pm_runtime_put_autosuspend(dev);
		}
	}
	if (data->irq_disabled_for_suspend) {
		enable_irq(data->client->irq);
		data->irq_disabled_for_suspend = false;
	}
	if (!ret)
		dev_info(dev, "system resumed: buffer=%u fifo=%u\n",
			 data->buffer_enabled, data->fifo_running);

	return ret;
}

static const struct dev_pm_ops lsm6dsox_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm6dsox_suspend, lsm6dsox_resume)
	SET_RUNTIME_PM_OPS(lsm6dsox_runtime_suspend,
			   lsm6dsox_runtime_resume, NULL)
};

static int lsm6dsox_probe(struct i2c_client *client)
{
	const struct i2c_adapter_quirks *quirks = client->adapter->quirks;
	struct lsm6dsox_data *data;
	struct iio_dev *indio_dev;
	s16 ax, ay, az;
	s16 gx, gy, gz;
	unsigned int whoami;
	int ret;

	dev_info(&client->dev, "my probe entered, addr=0x%02x\n",
		 client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"adapter lacks required I2C transfer support\n");
		return -EOPNOTSUPP;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	data->fifo_read_chunk = LSM6DSOX_FIFO_TARGET_READ_CHUNK;
	if (quirks && quirks->max_read_len)
		data->fifo_read_chunk = min_t(unsigned int,
						 data->fifo_read_chunk,
						 quirks->max_read_len);
	if (quirks && quirks->max_comb_2nd_msg_len)
		data->fifo_read_chunk = min_t(unsigned int,
						 data->fifo_read_chunk,
						 quirks->max_comb_2nd_msg_len);
	data->fifo_read_chunk -= data->fifo_read_chunk %
				 LSM6DSOX_FIFO_ENTRY_SIZE;
	if (data->fifo_read_chunk < LSM6DSOX_FIFO_ENTRY_SIZE) {
		dev_err(&client->dev,
			"I2C adapter cannot read one %u-byte FIFO entry\n",
			LSM6DSOX_FIFO_ENTRY_SIZE);
		return -EOPNOTSUPP;
	}
	dev_info(&client->dev, "FIFO burst read size: %u bytes\n",
		 data->fifo_read_chunk);

	data->regmap = devm_regmap_init_i2c(client, &lsm6dsox_regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(&client->dev, "failed to initialize regmap: %d\n", ret);
		return ret;
	}
	data->fifo_raw = devm_kmalloc(&client->dev,
				      LSM6DSOX_FIFO_MAX_ENTRIES *
				      LSM6DSOX_FIFO_ENTRY_SIZE, GFP_KERNEL);
	if (!data->fifo_raw)
		return -ENOMEM;

	data->fifo_scans = devm_kcalloc(&client->dev,
					LSM6DSOX_FIFO_MAX_SCANS,
					sizeof(*data->fifo_scans), GFP_KERNEL);
	if (!data->fifo_scans)
		return -ENOMEM;

	data->fifo_hw_timestamps = devm_kcalloc(&client->dev,
						LSM6DSOX_FIFO_MAX_SCANS,
						sizeof(*data->fifo_hw_timestamps),
						GFP_KERNEL);
	if (!data->fifo_hw_timestamps)
		return -ENOMEM;

	ret = regmap_read(data->regmap, LSM6DSOX_REG_WHO_AM_I, &whoami);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read WHO_AM_I: %d\n", ret);
		return ret;
	}

	if (whoami != LSM6DSOX_WHO_AM_I_VALUE) {
		dev_err(&client->dev,
			"unexpected WHO_AM_I: got 0x%02x, expected 0x%02x\n", whoami,
			LSM6DSOX_WHO_AM_I_VALUE);
		return -ENODEV;
	}

	dev_info(&client->dev, "WHO_AM_I=0x%02x\n", whoami);

	ret = lsm6dsox_soft_reset(data);
	if (ret < 0)
		return ret;

	ret = lsm6dsox_enable_bdu(data);
	if (ret < 0)
		return ret;

	ret = lsm6dsox_init_hw_timestamp(data);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed to initialize hardware timestamp: %d\n", ret);
		return ret;
	}

	ret = lsm6dsox_config_accel(data);
	if (ret < 0)
		return ret;

	ret = lsm6dsox_config_gyro(data);
	if (ret < 0)
		return ret;

	msleep(LSM6DSOX_STARTUP_DELAY_MS);

	ret = lsm6dsox_read_xyz(data, LSM6DSOX_REG_OUTX_L_A,
				&ax, &ay, &az);
	if (ret < 0)
		return ret;

	dev_info(&client->dev, "accel raw: x=%d y=%d z=%d\n", ax, ay, az);

	ret = lsm6dsox_read_xyz(data, LSM6DSOX_REG_OUTX_L_G,
				&gx, &gy, &gz);
	if (ret < 0)
		return ret;

	dev_info(&client->dev, "gyro raw: x=%d y=%d z=%d\n", gx, gy, gz);

	mutex_init(&data->lock);
	data->accel_odr = LSM6DSOX_SAMP_FREQ_HZ;
	data->gyro_odr = LSM6DSOX_SAMP_FREQ_HZ;
	data->accel_scale_nano =
		IIO_G_TO_M_S_2(LSM6DSOX_ACCEL_SCALE_UG);
	data->gyro_scale_nano =
		IIO_DEGREE_TO_RAD(LSM6DSOX_GYRO_SCALE_UDPS);
	data->fifo_watermark = LSM6DSOX_FIFO_DEFAULT_WATERMARK;

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

	ret = devm_iio_triggered_buffer_setup_ext(&client->dev,
						  indio_dev,
						  iio_pollfunc_store_time,
						  lsm6dsox_trigger_handler,
						  IIO_BUFFER_DIRECTION_IN,
						  &lsm6dsox_buffer_ops,
						  lsm6dsox_fifo_attributes);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed to setup triggered buffer: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "IIO triggered buffer setup complete\n");

	if (client->irq > 0) {
		atomic_set(&data->irq_count, 0);
		atomic_set(&data->sample_count, 0);

		ret = devm_request_irq(&client->dev, client->irq,
				       lsm6dsox_irq_handler, 0,
				       dev_name(&client->dev), indio_dev);
		if (ret < 0) {
			dev_err(&client->dev, "failed to request irq %d: %d\n",
				client->irq, ret);
			return ret;
		}

		dev_info(&client->dev, "FIFO watermark irq registered: irq=%d\n",
			 client->irq);
	} else {
		dev_warn(&client->dev, "no irq configured in device tree\n");
	}

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register IIO device: %d\n", ret);
		return ret;
	}

	ret = pm_runtime_set_active(&client->dev);
	if (ret < 0)
		return ret;

	pm_runtime_get_noresume(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev,
					 LSM6DSOX_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	dev_info(&client->dev, "IIO device registered\n");
	return 0;
}

static void lsm6dsox_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct lsm6dsox_data *data = iio_priv(indio_dev);
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	pm_runtime_disable(&client->dev);
	if (ret < 0)
		return;

	mutex_lock(&data->lock);
	ret = lsm6dsox_power_down(data);
	mutex_unlock(&data->lock);
	if (ret < 0)
		dev_warn(&client->dev, "failed to power down on remove: %d\n", ret);
	pm_runtime_put_noidle(&client->dev);
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
		.pm = &lsm6dsox_pm_ops,
	},
	.probe_new = lsm6dsox_probe,
	.remove = lsm6dsox_remove,
};
module_i2c_driver(lsm6dsox_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("GJ-King");
MODULE_DESCRIPTION("My LSM6DSOX I2C driver");
