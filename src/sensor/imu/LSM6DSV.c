#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "LSM6DSV.h"
#include "sensor/sensor_none.h"

#define PACKET_SIZE 7

// TODO: shared with LSM
float accel_sensitivity = 16.0f / 32768.0f; // Default 16G (FS = ±16 g: 0.488 mg/LSB)
float gyro_sensitivity = 0.070f; // Default 2000dps (FS = ±2000 dps: 70 mdps/LSB)

static const uint16_t accel_ranges[] = {16, 8, 4, 2, 0};
static const uint8_t accel_fss[] = {FS_XL_16G, FS_XL_8G, FS_XL_4G, FS_XL_2G};
static const uint16_t gyro_ranges[] = {4000, 2000, 1000, 500, 250, 125, 0};
static const uint8_t gyro_fss[] = {FS_G_4000DPS, FS_G_2000DPS, FS_G_1000DPS, FS_G_500DPS, FS_G_250DPS, FS_G_125DPS};

static uint8_t accel_fs = FS_XL_16G;
static uint8_t gyro_fs = FS_G_2000DPS;

static const uint16_t intervals[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 4096, 0};
static const uint8_t odrs[] = {ODR_7_68kHz, ODR_3_84kHz, ODR_1_92kHz, ODR_960Hz, ODR_480Hz, ODR_240Hz, ODR_120Hz, ODR_60Hz, ODR_30Hz, ODR_15Hz, ODR_7_5Hz, ODR_1_875Hz};

// TODO: shared with LSM
uint8_t last_accel_mode = 0xff;
uint8_t last_gyro_mode = 0xff;
uint8_t last_accel_odr = 0xff;
uint8_t last_gyro_odr = 0xff;

static float freq_scale = 1; // ODR is scaled by INTERNAL_FREQ_FINE

LOG_MODULE_REGISTER(LSM6DSV, LOG_LEVEL_DBG);

int lsm_init(float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	// setup interface for SPI
	sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(10), 0);
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL6, gyro_fs); // set gyro FS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL8, accel_fs); // set accel FS
	if (err)
		LOG_ERR("Communication error");
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, 0x18); // INT H_LACTIVE active low, PP_OD open-drain
	int8_t internal_freq_fine;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INTERNAL_FREQ_FINE, &internal_freq_fine); // affects ODR
	freq_scale = 1.0f + 0.0013f * (float)internal_freq_fine;
	err |= lsm_update_odr(accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL4, 0x06); // enable Continuous mode
	if (err)
		LOG_ERR("Communication error");

	/* ================= DIAG: read back every config register =================
	 * ssi_reg_write_byte() is write-only on SPI, so err only reports SPI
	 * peripheral status -- it says NOTHING about whether the chip accepted it.
	 * Expected values if the writes landed:
	 *   who=70 CTRL6=gyro_fs CTRL8=accel_fs CTRL3=00
	 *   CTRL1=OP_MODE_XL_HP<<4|ODR_XL  CTRL2=OP_MODE_G_HP<<4|ODR_G
	 *   FIFO_CTRL3=ODR_XL|(ODR_G<<4)   FIFO_CTRL4=06   IF_CFG=18
	 * Anything 00 or FF here means the writes did NOT land.
	 */
	{
		uint8_t who = 0, c1 = 0, c2 = 0, c3 = 0, c6 = 0, c8 = 0, fc3 = 0, fc4 = 0, ifc = 0;
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x0F, &who);
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x10, &c1);
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x11, &c2);
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x12, &c3);
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x15, &c6);
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x17, &c8);
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x09, &fc3);
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x0A, &fc4);
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x03, &ifc);
		LOG_WRN("DIAGREG who=%02X CTRL1=%02X CTRL2=%02X CTRL3=%02X CTRL6=%02X CTRL8=%02X FC3=%02X FC4=%02X IFCFG=%02X",
			who, c1, c2, c3, c6, c8, fc3, fc4, ifc);
	}
	/* ======================================================================== */

	return (err < 0 ? err : 0);
}

void lsm_shutdown(void)
{
	last_accel_odr = 0xff; // reset last odr
	last_gyro_odr = 0xff; // reset last odr
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL3, 0x01); // SW_RESET
	if (err)
		LOG_ERR("Communication error");
	// TODO : Correctly wait for the reset to finish
	k_msleep(1);
}

void lsm_update_fs(float accel_range, float gyro_range, float *accel_actual_range, float *gyro_actual_range)
{
	if (accel_range < 0)
		accel_range = 0;

	if (gyro_range < 0)
		gyro_range = 0;

	for (int i = 1; i < ARRAY_SIZE(accel_ranges); i++)
	{
		if (accel_range <= accel_ranges[i])
			continue;
		accel_fs = accel_fss[i - 1];
		accel_range = accel_ranges[i - 1];
		break;
	}

	for (int i = 1; i < ARRAY_SIZE(gyro_ranges); i++)
	{
		if (gyro_range <= gyro_ranges[i])
			continue;
		gyro_fs = gyro_fss[i - 1];
		gyro_range = gyro_ranges[i - 1];
		break;
	}

	accel_sensitivity = accel_range / 32768.0f;
	gyro_sensitivity = 35.0f * gyro_range / 1000000.0f;

	*accel_actual_range = accel_range;
	*gyro_actual_range = gyro_range;
}

int lsm_update_odr(float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int interval;
	uint8_t OP_MODE_XL;
	uint8_t OP_MODE_G;
	uint8_t ODR_XL = 0;
	uint8_t ODR_G = 0;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY) // off, standby interpreted as off
	{
		OP_MODE_XL = OP_MODE_XL_HP;
		ODR_XL = ODR_OFF;
		accel_time = 0; // off
	}
	else
	{
		OP_MODE_XL = OP_MODE_XL_HP;
		interval = accel_time * freq_scale * 7680; // scale by internal freq adjustment
		for (int i = 1; i < ARRAY_SIZE(intervals); i++)
		{
			if (intervals[i] && interval >= intervals[i])
				continue;
			ODR_XL = odrs[i - 1];
			accel_time = intervals[i - 1] / 7680.0f;
			break;
		}
	}
	accel_time /= freq_scale; // scale by internal freq adjustment

	// Calculate gyro
	if (gyro_time <= 0) // off
	{
		OP_MODE_G = OP_MODE_G_HP;
		ODR_G = ODR_OFF;
		gyro_time = 0; // off
	}
	else if (gyro_time == INFINITY) // sleep
	{
		OP_MODE_G = OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		gyro_time = 0; // off
	}
	else
	{
		OP_MODE_G = OP_MODE_G_HP;
		interval = gyro_time * freq_scale * 7680; // scale by internal freq adjustment
		for (int i = 1; i < ARRAY_SIZE(intervals); i++)
		{
			if (intervals[i] && interval >= intervals[i])
				continue;
			ODR_G = odrs[i - 1];
			gyro_time = intervals[i - 1] / 7680.0f;
			break;
		}
	}
	gyro_time /= freq_scale; // scale by internal freq adjustment

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL && last_gyro_odr == ODR_G) // if both were already configured
		return 1;

	last_accel_mode = OP_MODE_XL;
	last_gyro_mode = OP_MODE_G;
	last_accel_odr = ODR_XL;
	last_gyro_odr = ODR_G;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, OP_MODE_XL << 4 | ODR_XL); // set accel ODR and mode
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, OP_MODE_G << 4 | ODR_G); // set gyro ODR and mode

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_CTRL3, ODR_XL | (ODR_G << 4)); // set accel and gyro batch rate
	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t lsm_data_read(uint8_t *data, uint16_t len)
{
	uint8_t rawCount[2] = {0};
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_STATUS1, &rawCount[0], 2);

	uint16_t count = (uint16_t)((rawCount[1] & 3) << 8 | rawCount[0]); // ORIGINAL: masks 2 bits
	/* DIAG: DIFF_FIFO is 9 bits. FIFO_STATUS2 bit0 = DIFF_FIFO[8], bit1 is RESERVED.
	 * Masking 2 bits adds a phantom +512 whenever the reserved bit reads as 1. */
	uint16_t count_fixed = (uint16_t)((rawCount[1] & 1) << 8 | rawCount[0]);

	const uint16_t limit = len / PACKET_SIZE;
	if (count > limit)
	{
		LOG_WRN("FIFO read buffer limit reached, %d packets dropped", count - limit);
		count = limit;
	}

	err |= ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FIFO_DATA_OUT_TAG, data, count * PACKET_SIZE, PACKET_SIZE);
	if (err)
		LOG_ERR("Communication error");

	/* ============ DIAG: brownout / CS discriminator ============
	 * On a garbage read (both FIFO_STATUS bytes 0xFF = nobody driving MISO)
	 * immediately read back WHO_AM_I and the config registers.
	 *   who=70 CTRL1/CTRL2 = configured -> chip alive, this transaction glitched
	 *   who=70 CTRL1/CTRL2 = 00         -> CHIP WAS RESET (brownout / power loss)
	 *   who=FF                          -> whole bus dead at this instant (CS or power)
	 * CTRL1/CTRL2 only return to 0x00 after a power-on reset, so they are a
	 * reliable "did the chip lose power" probe.
	 */
	if (rawCount[0] == 0xFF && rawCount[1] == 0xFF)
	{
		static uint32_t nbad = 0;
		if ((nbad++ % 8) == 0)
		{
			uint8_t w2 = 0, c1 = 0, c2 = 0, fc4 = 0, c3 = 0;
			ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x0F, &w2);
			ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x10, &c1);
			ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x11, &c2);
			ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x0A, &fc4);
			ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x12, &c3);
			LOG_WRN("DIAGBAD who=%02X CTRL1=%02X CTRL2=%02X FC4=%02X CTRL3=%02X",
				w2, c1, c2, fc4, c3);
		}
	}
	/* ========================================================== */

	/* ================= DIAG ================= */
	static uint32_t diag_n = 0;
	if ((diag_n++ % 64) == 0)
	{
		uint8_t who = 0;
		ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, 0x0F, &who);

		int n = (int)(count * PACKET_SIZE);
		if (n > 96) n = 96;
		int nff = 0;
		for (int i = 0; i < n; i++)
			if (data[i] == 0xFF) nff++;

		uint8_t t0 = count > 0 ? data[0] : 0;
		uint8_t t1 = count > 1 ? data[7] : 0;
		uint8_t t2 = count > 2 ? data[14] : 0;

		LOG_WRN("DIAG ST2=%02X ST1=%02X cnt=%u fixed=%u who=0x%02X tags=%02X,%02X,%02X ff=%d/%d",
			rawCount[1], rawCount[0], (unsigned)count, (unsigned)count_fixed, who,
			t0, t1, t2, nff, n);
	}
	/* ======================================== */

	return count;
}

sensor_data_attrs_t lsm_data_process(uint16_t index, uint8_t *data, float a[3], float g[3], float m[3])
{
	index *= PACKET_SIZE;
	switch (data[index] >> 3)
	{
	case 0x02: // Accelerometer NC (Accelerometer uncompressed data)
		for (int i = 0; i < 3; i++) // x, y, z
		{
			a[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			a[i] *= accel_sensitivity;
		}
		return DATA_VALID_ACCEL;
	case 0x01: // Gyroscope NC (Gyroscope uncompressed data)
		for (int i = 0; i < 3; i++) // x, y, z
		{
			g[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			g[i] *= gyro_sensitivity;
			
		}
		return DATA_VALID_GYRO;
	default:
	}
	// TODO: need to skip invalid data
	return DATA_INVALID;
}

void lsm_accel_read(float a[3])
{
	uint8_t rawAccel[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_L_A, &rawAccel[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] = (int16_t)((((uint16_t)rawAccel[1 + (i * 2)]) << 8) | rawAccel[i * 2]);
		a[i] *= accel_sensitivity;
	}
	// TODO: for ISM330BX, the accelerometer data is in ZYX order
}

void lsm_gyro_read(float g[3])
{
	uint8_t rawGyro[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_L_G, &rawGyro[0], 6);
	if (err)
		LOG_ERR("Communication error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[1 + (i * 2)]) << 8) | rawGyro[i * 2]);
		g[i] *= gyro_sensitivity;
	}
}

int lsm_temp_read(float *data)
{
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUT_TEMP_L, &rawTemp[0], 2);
	if (err)
	{
		LOG_ERR("Communication error");
		return -1;
	}
	// TSen Temperature sensitivity 256 LSB/°C
	// The output of the temperature sensor is 0 LSB (typ.) at 25°C
	*data = (int16_t)((((uint16_t)rawTemp[1]) << 8) | rawTemp[0]);
	*data /= 256;
	*data += 25;
	return 0;
}

uint8_t lsm_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];
	buf[0] = ((threshold >> 8) & 0x03) | (last_gyro_odr > last_accel_odr ? 0x20 : 0x00); // use gyro for BDR if gyro rate is higher // NOTE: using 0x03 for DSV, but DSO allows 0x07
	buf[1] = threshold & 0xFF;
	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_COUNTER_BDR_REG1, buf, 2);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_INT1_CTRL, 0x40); // COUNTER_BDR interrupt
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

uint8_t lsm_setup_WOM(void)
{ // TODO: should be off by the time WOM will be setup
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, ODR_OFF); // set accel off
//	ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL2, ODR_OFF); // set gyro off

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL8, 0xE0 | FS_XL_8G); // set accel FS, set HP_LPF2_XL_BW to lowest bandwidth, enable HP_REF_MODE (set HP_LPF2_XL_BW)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL1, OP_MODE_XL_LP1 << 4 | ODR_240Hz); // set accel low power mode 1, set accel ODR (enable accel)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_CTRL9, 0x50); // enable HP_REF_MODE (set HP_REF_MODE_XL and HP_SLOPE_XL_EN)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_TAP_CFG0, 0x10); // set SLOPE_FDS
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_WAKE_UP_THS, 0x04); // set threshold, 4 * 7.8125 mg is ~31.25 mg
	k_msleep(11); // need to wait for accel to settle

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNCTIONS_ENABLE, 0x80); // enable interrupts
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MD1_CFG, 0x20); // route wake-up to INT1
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, 0x18); // INT H_LACTIVE active low, PP_OD open-drain
	if (err)
		LOG_ERR("Communication error");
	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW; // active low
}

int lsm_ext_setup(sensor_ext_mode_t mode, const sensor_mag_t *mag, uint8_t mag_addr) {
	int err = 0;
	switch (mode) {
		case SENSOR_EXT_MODE_OFF:
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // passthrough off
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
			break;

		case SENSOR_EXT_MODE_I2C_PASSTHROUGH:
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x10); // passthrough on
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
			break;
			
		case SENSOR_EXT_MODE_I2CM_PROXY:
			// enable internal pull-up for auxiliary I2C
			err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_IF_CFG, 0x58); // SHUB_PU_EN, INT H_LACTIVE active low, PP_OD open-drain
			sensor_interface_ext_configure(&sensor_ext_lsm6dsv);
			break;
		
		case SENSOR_EXT_MODE_I2CM_AUTONOMOUS:
			return -1;
	}

	if (err) {
		LOG_ERR("Communication error: %d", err);
	}

	return err;
}

int lsm_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes)
{
	if (num_bytes != 2)
	{
		LOG_ERR("Unsupported write");
		return -1;
	}
	// Configure transaction and begin one-shot (AN5922, page 80, One-shot write routine)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	uint8_t slv0[3] = {(addr << 1) | 0x00, buf[0], 0xC0 | 0x00}; // write, SHUB_ODR = 440Hz, reading no bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, 3);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, (addr << 1) | 0x00); // write
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_SUBADD, buf[0]);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_CONFIG, 0xC0 | 0x00); // SHUB_ODR = 440Hz, reading no bytes
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_DATAWRITE_SLV0, buf[1]);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x44); // WRITE_ONCE, enable I2C master
	// Wait for transaction
	uint8_t status = 0;
	int64_t timeout = k_uptime_get() + 10;
	while ((status & 0x80) && k_uptime_get() < timeout) // WR_ONCE_DONE
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER, &status);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // disable I2C master
	k_usleep(300);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	if (~status & 0x80)
	{
		LOG_ERR("Write timeout");
		return -1;
	}
	return err;
}

int lsm_ext_write_read(const uint8_t addr, const uint8_t *write_buf, size_t num_write, uint8_t *read_buf, size_t num_read)
{
	if (num_write != 1 || num_read < 1 || num_read > 8)
	{
		LOG_ERR("Unsupported write_read");
		return -1;
	}
	// Configure transaction and begin one-shot (AN5922, page 79, One-shot read routine)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	uint8_t slv0[3] = {(addr << 1) | 0x01, ((const uint8_t *)write_buf)[0], 0xC0 | num_read}; // read, SHUB_ODR = 440Hz, reading num_read bytes
	err |= ssi_burst_write(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, slv0, 3);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_ADD, (addr << 1) | 0x01); // read
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_SUBADD, ((const uint8_t *)write_buf)[0]);
//	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SLV0_CONFIG, 0xC0 | num_read); // SHUB_ODR = 440Hz, reading num_read bytes
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x44); // WRITE_ONCE mandatory for read, enable I2C master
	// Wait for transaction
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	uint8_t tmp;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_OUTX_H_A, &tmp); // clear XLDA
	uint8_t status = 0;
	int64_t timeout = k_uptime_get() + 10;
	while ((status & 0x01) && k_uptime_get() < timeout) // XLDA
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_REG, &status);
	status = 0;
	timeout = k_uptime_get() + 10;
	while ((status & 0x01) && k_uptime_get() < timeout) // SENS_HUB_ENDOP
		err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_STATUS_MASTER_MAINPAGE, &status);
	// Read data
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_MASTER_CONFIG, 0x00); // disable I2C master
	k_usleep(300);
	err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_SENSOR_HUB_1, read_buf, num_read);
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	return err;
}

const sensor_imu_t sensor_imu_lsm6dsv = {
	*lsm_init,
	*lsm_shutdown,

	*lsm_update_fs,
	*lsm_update_odr,

	*lsm_data_read,
	*lsm_data_process,
	*lsm_accel_read,
	*lsm_gyro_read,
	*lsm_temp_read,

	*lsm_setup_DRDY,
	*lsm_setup_WOM,

	*lsm_ext_setup,
};

const sensor_ext_ssi_t sensor_ext_lsm6dsv = {
	*lsm_ext_write,
	*lsm_ext_write_read,
	8
};
