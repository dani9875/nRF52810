/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
// #include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <assert.h>


#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/settings/settings.h>
#define DEVICE_NAME		"Remote"
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)


#define LED_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

// #define PMW3360_DEFINE(n)						       \
// 	static struct pmw3360_data data##n;				       \
// 									       \
// 	static const struct pmw3360_config config##n = {		       \
// 		.irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),	       \
// 		.bus = {						       \
// 			.bus = DEVICE_DT_GET(DT_INST_BUS(n)),		       \
// 			.config = {					       \
// 				.frequency = DT_INST_PROP(n,		       \
// 							  spi_max_frequency),  \
// 				.operation = SPI_WORD_SET(8) |		       \
// 					     SPI_TRANSFER_MSB |		       \
// 					     SPI_MODE_CPOL | SPI_MODE_CPHA,    \
// 				.slave = DT_INST_REG_ADDR(n),		       \
// 			},						       \
// 		},							       \
// 		.cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),	       \
// 	};								       \
// 									       \
// 	DEVICE_DT_INST_DEFINE(n, pmw3360_init, NULL, &data##n, &config##n,     \
// 			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	       \
// 			      &pmw3360_driver_api);

// DT_INST_FOREACH_STATUS_OKAY(PMW3360_DEFINE)






/** @brief UUID of the Remote Service. **/
#define BT_UUID_REMOTE_SERV_VAL \
	BT_UUID_128_ENCODE(0xe9ea0001, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)

/** @brief UUID of the Button Characteristic. **/
#define BT_UUID_REMOTE_BUTTON_CHRC_VAL \
	BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)

/** @brief UUID of the Message Characteristic. **/
#define BT_UUID_REMOTE_MESSAGE_CHRC_VAL \
	BT_UUID_128_ENCODE(0xe9ea0003, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)

#define BT_UUID_REMOTE_SERVICE          BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)
#define BT_UUID_REMOTE_BUTTON_CHRC 	    BT_UUID_DECLARE_128(BT_UUID_REMOTE_BUTTON_CHRC_VAL)
#define BT_UUID_REMOTE_MESSAGE_CHRC     BT_UUID_DECLARE_128(BT_UUID_REMOTE_MESSAGE_CHRC_VAL)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_REMOTE_SERV_VAL),
};

enum bt_button_notifications_enabled {
	BT_BUTTON_NOTIFICATIONS_ENABLED,
	BT_BUTTON_NOTIFICATIONS_DISABLED,
};

struct bt_remote_service_cb {
	void (*notif_changed)(enum bt_button_notifications_enabled status);
    void (*data_received)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
};

static K_SEM_DEFINE(bt_init_ok, 1, 1);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

static struct bt_remote_service_cb remote_callbacks;

static struct bt_conn *current_conn;

/* Declarations */
void on_connected(struct bt_conn *conn, uint8_t err);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_notif_changed(enum bt_button_notifications_enabled status);
void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len);

struct bt_conn_cb bluetooth_callbacks = {
	.connected 		= on_connected,
	.disconnected 	= on_disconnected,
};
struct bt_remote_service_cb remote_callbacks_s = {
	.notif_changed = on_notif_changed,
    .data_received = on_data_received,
};

/**************************************************************/
extern const uint8_t pmw3360_firmware_data[];

#define DT_DRV_COMPAT pixart_pmw3360

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <sensor/pmw3360.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3360, 0);

/* Timings defined by spec */
#define T_NCS_SCLK	1			/* 120 ns */
#define T_SRX		(20 - T_NCS_SCLK)	/* 20 us */
#define T_SCLK_NCS_WR	(35 - T_NCS_SCLK)	/* 35 us */
#define T_SWX		(180 - T_SCLK_NCS_WR)	/* 180 us */
#define T_SRAD		160			/* 160 us */
#define T_SRAD_MOTBR	35			/* 35 us */
#define T_BEXIT		1			/* 500 ns */

/* Timing defined on SROM download burst mode figure */
#define T_BRSEP		15			/* 15 us */


/* Sensor registers */
#define PMW3360_REG_PRODUCT_ID			0x00
#define PMW3360_REG_REVISION_ID			0x01
#define PMW3360_REG_MOTION			0x02
#define PMW3360_REG_DELTA_X_L			0x03
#define PMW3360_REG_DELTA_X_H			0x04
#define PMW3360_REG_DELTA_Y_L			0x05
#define PMW3360_REG_DELTA_Y_H			0x06
#define PMW3360_REG_SQUAL			0x07
#define PMW3360_REG_RAW_DATA_SUM		0x08
#define PMW3360_REG_MAXIMUM_RAW_DATA		0x09
#define PMW3360_REG_MINIMUM_RAW_DATA		0x0A
#define PMW3360_REG_SHUTTER_LOWER		0x0B
#define PMW3360_REG_SHUTTER_UPPER		0x0C
#define PMW3360_REG_CONTROL			0x0D
#define PMW3360_REG_CONFIG1			0x0F
#define PMW3360_REG_CONFIG2			0x10
#define PMW3360_REG_ANGLE_TUNE			0x11
#define PMW3360_REG_FRAME_CAPTURE		0x12
#define PMW3360_REG_SROM_ENABLE			0x13
#define PMW3360_REG_RUN_DOWNSHIFT		0x14
#define PMW3360_REG_REST1_RATE_LOWER		0x15
#define PMW3360_REG_REST1_RATE_UPPER		0x16
#define PMW3360_REG_REST1_DOWNSHIFT		0x17
#define PMW3360_REG_REST2_RATE_LOWER		0x18
#define PMW3360_REG_REST2_RATE_UPPER		0x19
#define PMW3360_REG_REST2_DOWNSHIFT		0x1A
#define PMW3360_REG_REST3_RATE_LOWER		0x1B
#define PMW3360_REG_REST3_RATE_UPPER		0x1C
#define PMW3360_REG_OBSERVATION			0x24
#define PMW3360_REG_DATA_OUT_LOWER		0x25
#define PMW3360_REG_DATA_OUT_UPPER		0x26
#define PMW3360_REG_RAW_DATA_DUMP		0x29
#define PMW3360_REG_SROM_ID			0x2A
#define PMW3360_REG_MIN_SQ_RUN			0x2B
#define PMW3360_REG_RAW_DATA_THRESHOLD		0x2C
#define PMW3360_REG_CONFIG5			0x2F
#define PMW3360_REG_POWER_UP_RESET		0x3A
#define PMW3360_REG_SHUTDOWN			0x3B
#define PMW3360_REG_INVERSE_PRODUCT_ID		0x3F
#define PMW3360_REG_LIFTCUTOFF_TUNE3		0x41
#define PMW3360_REG_ANGLE_SNAP			0x42
#define PMW3360_REG_LIFTCUTOFF_TUNE1		0x4A
#define PMW3360_REG_MOTION_BURST		0x50
#define PMW3360_REG_LIFTCUTOFF_TUNE_TIMEOUT	0x58
#define PMW3360_REG_LIFTCUTOFF_TUNE_MIN_LENGTH	0x5A
#define PMW3360_REG_SROM_LOAD_BURST		0x62
#define PMW3360_REG_LIFT_CONFIG			0x63
#define PMW3360_REG_RAW_DATA_BURST		0x64
#define PMW3360_REG_LIFTCUTOFF_TUNE2		0x65

/* Sensor identification values */
#define PMW3360_PRODUCT_ID			0x42
#define PMW3360_FIRMWARE_ID			0x04

/* Max register count readable in a single motion burst */
#define PMW3360_MAX_BURST_SIZE			12

/* Register count used for reading a single motion burst */
#define PMW3360_BURST_SIZE			6

/* Position of X in motion burst data */
#define PMW3360_DX_POS				2
#define PMW3360_DY_POS				4

/* Rest_En position in Config2 register. */
#define PMW3360_REST_EN_POS			5

#define PMW3360_MAX_CPI				12000
#define PMW3360_MIN_CPI				100


#define SPI_WRITE_BIT				BIT(7)

/* Helper macros used to convert sensor values. */
#define PMW3360_SVALUE_TO_CPI(svalue) ((uint32_t)(svalue).val1)
#define PMW3360_SVALUE_TO_TIME(svalue) ((uint32_t)(svalue).val1)
#define PMW3360_SVALUE_TO_BOOL(svalue) ((svalue).val1 != 0)


extern const size_t pmw3360_firmware_length;
extern const uint8_t pmw3360_firmware_data[];


enum async_init_step {
	ASYNC_INIT_STEP_POWER_UP,
	ASYNC_INIT_STEP_FW_LOAD_START,
	ASYNC_INIT_STEP_FW_LOAD_CONTINUE,
	ASYNC_INIT_STEP_FW_LOAD_VERIFY,
	ASYNC_INIT_STEP_CONFIGURE,

	ASYNC_INIT_STEP_COUNT
};

struct pmw3360_data {
	const struct device          *dev;
	struct gpio_callback         irq_gpio_cb;
	struct k_spinlock            lock;
	int16_t                      x;
	int16_t                      y;
	sensor_trigger_handler_t     data_ready_handler;
	struct k_work                trigger_handler_work;
	struct k_work_delayable      init_work;
	enum async_init_step         async_init_step;
	int                          err;
	bool                         ready;
	bool                         last_read_burst;
};

struct pmw3360_config {
	struct gpio_dt_spec irq_gpio;
	struct spi_dt_spec bus;
	struct gpio_dt_spec cs_gpio;
};

static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
	[ASYNC_INIT_STEP_POWER_UP]         = 1,
	[ASYNC_INIT_STEP_FW_LOAD_START]    = 50,
	[ASYNC_INIT_STEP_FW_LOAD_CONTINUE] = 10,
	[ASYNC_INIT_STEP_FW_LOAD_VERIFY]   = 1,
	[ASYNC_INIT_STEP_CONFIGURE]        = 0,
};


static int pmw3360_async_init_power_up(const struct device *dev);
static int pmw3360_async_init_configure(const struct device *dev);
static int pmw3360_async_init_fw_load_verify(const struct device *dev);
static int pmw3360_async_init_fw_load_continue(const struct device *dev);
static int pmw3360_async_init_fw_load_start(const struct device *dev);

static int (* const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
	[ASYNC_INIT_STEP_POWER_UP] = pmw3360_async_init_power_up,
	[ASYNC_INIT_STEP_FW_LOAD_START] = pmw3360_async_init_fw_load_start,
	[ASYNC_INIT_STEP_FW_LOAD_CONTINUE] = pmw3360_async_init_fw_load_continue,
	[ASYNC_INIT_STEP_FW_LOAD_VERIFY] = pmw3360_async_init_fw_load_verify,
	[ASYNC_INIT_STEP_CONFIGURE] = pmw3360_async_init_configure,
};

static int spi_cs_ctrl(const struct device *dev, bool enable)
{
	const struct pmw3360_config *config = dev->config;
	int err;

	if (!enable) {
		k_busy_wait(T_NCS_SCLK);
	}

	err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
	if (err) {
		printk("SPI CS ctrl failed");
	}

	if (enable) {
		k_busy_wait(T_NCS_SCLK);
	}

	return err;
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf)
{
	int err;
	struct pmw3360_data *data = dev->data;
	const struct pmw3360_config *config = dev->config;

	__ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

	err = spi_cs_ctrl(dev, true);
	if (err) {
		return err;
	}

	/* Write register address. */
	const struct spi_buf tx_buf = {
		.buf = &reg,
		.len = 1
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	err = spi_write_dt(&config->bus, &tx);
	if (err) {
		printk("Reg read failed on SPI write");
		return err;
	}

	k_busy_wait(T_SRAD);

	/* Read register value. */
	struct spi_buf rx_buf = {
		.buf = buf,
		.len = 1,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	err = spi_read_dt(&config->bus, &rx);
	if (err) {
		printk("Reg read failed on SPI read");
		return err;
	}

	err = spi_cs_ctrl(dev, false);
	if (err) {
		return err;
	}

	k_busy_wait(T_SRX);

	data->last_read_burst = false;

	return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	int err;
	struct pmw3360_data *data = dev->data;
	const struct pmw3360_config *config = dev->config;

	__ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

	err = spi_cs_ctrl(dev, true);
	if (err) {
		return err;
	}

	uint8_t buf[] = {
		SPI_WRITE_BIT | reg,
		val
	};
	const struct spi_buf tx_buf = {
		.buf = buf,
		.len = ARRAY_SIZE(buf)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	err = spi_write_dt(&config->bus, &tx);
	if (err) {
		printk("Reg write failed on SPI write");
		return err;
	}

	k_busy_wait(T_SCLK_NCS_WR);

	err = spi_cs_ctrl(dev, false);
	if (err) {
		return err;
	}

	k_busy_wait(T_SWX);

	data->last_read_burst = false;

	return 0;
}

static int motion_burst_read(const struct device *dev, uint8_t *buf,
			     size_t burst_size)
{
	int err;
	struct pmw3360_data *data = dev->data;
	const struct pmw3360_config *config = dev->config;

	__ASSERT_NO_MSG(burst_size <= PMW3360_MAX_BURST_SIZE);

	/* Write any value to motion burst register only if there have been
	 * other SPI transmissions with sensor since last burst read.
	 */
	if (!data->last_read_burst) {
		err = reg_write(dev, PMW3360_REG_MOTION_BURST, 0x00);
		if (err) {
			return err;
		}
	}

	err = spi_cs_ctrl(dev, true);
	if (err) {
		return err;
	}

	/* Send motion burst address */
	uint8_t reg_buf[] = {
		PMW3360_REG_MOTION_BURST
	};
	const struct spi_buf tx_buf = {
		.buf = reg_buf,
		.len = ARRAY_SIZE(reg_buf)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	err = spi_write_dt(&config->bus, &tx);
	if (err) {
		printk("Motion burst failed on SPI write");
		return err;
	}

	k_busy_wait(T_SRAD_MOTBR);

	const struct spi_buf rx_buf = {
		.buf = buf,
		.len = burst_size,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	err = spi_read_dt(&config->bus, &rx);
	if (err) {
		printk("Motion burst failed on SPI read");
		return err;
	}

	/* Terminate burst */
	err = spi_cs_ctrl(dev, false);
	if (err) {
		return err;
	}

	k_busy_wait(T_BEXIT);

	data->last_read_burst = true;

	return 0;
}

static int burst_write(const struct device *dev, uint8_t reg, const uint8_t *buf,
		       size_t size)
{
	int err;
	struct pmw3360_data *data = dev->data;
	const struct pmw3360_config *config = dev->config;

	/* Write address of burst register */
	uint8_t write_buf = reg | SPI_WRITE_BIT;
	struct spi_buf tx_buf = {
		.buf = &write_buf,
		.len = 1
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	err = spi_cs_ctrl(dev, true);
	if (err) {
		return err;
	}

	err = spi_write_dt(&config->bus, &tx);
	if (err) {
		printk("Burst write failed on SPI write");
		return err;
	}

	/* Write data */
	for (size_t i = 0; i < size; i++) {
		write_buf = buf[i];

		err = spi_write_dt(&config->bus, &tx);
		if (err) {
			printk("Burst write failed on SPI write (data)");
			return err;
		}

		k_busy_wait(T_BRSEP);
	}

	/* Terminate burst mode. */
	err = spi_cs_ctrl(dev, false);
	if (err) {
		return err;
	}

	k_busy_wait(T_BEXIT);

	data->last_read_burst = false;

	return 0;
}

static int update_cpi(const struct device *dev, uint32_t cpi)
{
	/* Set resolution with CPI step of 100 cpi
	 * 0x00: 100 cpi (minimum cpi)
	 * 0x01: 200 cpi
	 * :
	 * 0x31: 5000 cpi (default cpi)
	 * :
	 * 0x77: 12000 cpi (maximum cpi)
	 */

	if ((cpi > PMW3360_MAX_CPI) || (cpi < PMW3360_MIN_CPI)) {
		printk("CPI value %u out of range", cpi);
		return -EINVAL;
	}

	/* Convert CPI to register value */
	uint8_t value = (cpi / 100) - 1;

	printk("Setting CPI to %u (reg value 0x%x)", cpi, value);

	int err = reg_write(dev, PMW3360_REG_CONFIG1, value);
	if (err) {
		printk("Failed to change CPI");
	}

	return err;
}

static int update_downshift_time(const struct device *dev, uint8_t reg_addr,
				 uint32_t time)
{
	/* Set downshift time:
	 * - Run downshift time (from Run to Rest1 mode)
	 * - Rest 1 downshift time (from Rest1 to Rest2 mode)
	 * - Rest 2 downshift time (from Rest2 to Rest3 mode)
	 */
	uint32_t maxtime;
	uint32_t mintime;

	switch (reg_addr) {
	case PMW3360_REG_RUN_DOWNSHIFT:
		/*
		 * Run downshift time = PMW3360_REG_RUN_DOWNSHIFT * 10 ms
		 */
		maxtime = 2550;
		mintime = 10;
		break;

	case PMW3360_REG_REST1_DOWNSHIFT:
		/*
		 * Rest1 downshift time = PMW3360_REG_RUN_DOWNSHIFT
		 *                        * 320 * Rest1 rate (default 1 ms)
		 */
		maxtime = 81600;
		mintime = 320;
		break;

	case PMW3360_REG_REST2_DOWNSHIFT:
		/*
		 * Rest2 downshift time = PMW3360_REG_REST2_DOWNSHIFT
		 *                        * 32 * Rest2 rate (default 100 ms)
		 */
		maxtime = 816000;
		mintime = 3200;
		break;

	default:
		printk("Not supported");
		return -ENOTSUP;
	}

	if ((time > maxtime) || (time < mintime)) {
		printk("Downshift time %u out of range", time);
		return -EINVAL;
	}

	__ASSERT_NO_MSG((mintime > 0) && (maxtime/mintime <= UINT8_MAX));

	/* Convert time to register value */
	uint8_t value = time / mintime;

	printk("Set downshift time to %u ms (reg value 0x%x)", time, value);

	int err = reg_write(dev, reg_addr, value);
	if (err) {
		printk("Failed to change downshift time");
	}

	return err;
}

static int update_sample_time(const struct device *dev,
			      uint8_t reg_addr_lower,
			      uint8_t reg_addr_upper,
			      uint32_t sample_time)
{
	/* Set sample time for the Rest1-Rest3 modes.
	 * Values above 0x09B0 will trigger internal watchdog reset.
	 */
	uint32_t maxtime = 0x9B0;
	uint32_t mintime = 1;

	if ((sample_time > maxtime) || (sample_time < mintime)) {
		printk("Sample time %u out of range", sample_time);
		return -EINVAL;
	}

	printk("Set sample time to %u ms", sample_time);

	/* The sample time is (reg_value + 1) ms. */
	sample_time--;
	uint8_t buf[2];

	sys_put_le16((uint16_t)sample_time, buf);

	int err = reg_write(dev, reg_addr_lower, buf[0]);

	if (!err) {
		err = reg_write(dev, reg_addr_upper, buf[1]);
	} else {
		printk("Failed to change sample time");
	}

	return err;
}

static int toggle_rest_modes(const struct device *dev, uint8_t reg_addr,
			     bool enable)
{
	uint8_t value;
	int err = reg_read(dev, reg_addr, &value);

	if (err) {
		printk("Failed to read Config2 register");
		return err;
	}

	WRITE_BIT(value, PMW3360_REST_EN_POS, enable);

	printk("%sable rest modes", (enable) ? ("En") : ("Dis"));
	err = reg_write(dev, reg_addr, value);

	if (err) {
		printk("Failed to set rest mode");
	}

	return err;
}

static int pmw3360_async_init_fw_load_start(const struct device *dev)
{
	int err = 0;

	/* Read from registers 0x02-0x06 regardless of the motion pin state. */
	for (uint8_t reg = 0x02; (reg <= 0x06) && !err; reg++) {
		uint8_t buf[1];
		err = reg_read(dev, reg, buf);
	}

	if (err) {
		printk("Cannot read from data registers");
		return err;
	}

	/* Write 0 to Rest_En bit of Config2 register to disable Rest mode. */
	err = reg_write(dev, PMW3360_REG_CONFIG2, 0x00);
	if (err) {
		printk("Cannot disable REST mode");
		return err;
	}

	/* Write 0x1D in SROM_enable register to initialize the operation */
	err = reg_write(dev, PMW3360_REG_SROM_ENABLE, 0x1D);
	if (err) {
		printk("Cannot initialize SROM");
		return err;
	}

	return err;
}

static int pmw3360_async_init_fw_load_continue(const struct device *dev)
{
	int err;

	printk("Uploading optical sensor firmware...");

	/* Write 0x18 to SROM_enable to start SROM download */
	err = reg_write(dev, PMW3360_REG_SROM_ENABLE, 0x18);
	if (err) {
		printk("Cannot start SROM download");
		return err;
	}

	/* Write SROM file into SROM_Load_Burst register.
	 * Data must start with SROM_Load_Burst address.
	 */
	err = burst_write(dev, PMW3360_REG_SROM_LOAD_BURST,
			  pmw3360_firmware_data, pmw3360_firmware_length);
	if (err) {
		printk("Cannot write firmware to sensor");
	}

	return err;
}

static int pmw3360_async_init_fw_load_verify(const struct device *dev)
{
	int err;

	/* Read the SROM_ID register to verify the firmware ID before any
	 * other register reads or writes
	 */

	uint8_t fw_id;
	err = reg_read(dev, PMW3360_REG_SROM_ID, &fw_id);
	if (err) {
		printk("Cannot obtain firmware id");
		return err;
	}

	printk("Optical chip firmware ID: 0x%x", fw_id);
	if (fw_id != PMW3360_FIRMWARE_ID) {
		printk("Chip is not running from SROM!");
		return -EIO;
	}

	uint8_t product_id;
	err = reg_read(dev, PMW3360_REG_PRODUCT_ID, &product_id);
	if (err) {
		printk("Cannot obtain product id");
		return err;
	}

	if (product_id != PMW3360_PRODUCT_ID) {
		printk("Invalid product id!");
		return -EIO;
	}

	/* Write 0x20 to Config2 register for wireless mouse design.
	 * This enables entering rest modes.
	 */
	err = reg_write(dev, PMW3360_REG_CONFIG2, 0x20);
	if (err) {
		printk("Cannot enable REST modes");
	}

	return err;
}

static void irq_handler(const struct device *gpiob, struct gpio_callback *cb,
			uint32_t pins)
{
	int err;
	struct pmw3360_data *data = CONTAINER_OF(cb, struct pmw3360_data,
						 irq_gpio_cb);
	const struct device *dev = data->dev;
	const struct pmw3360_config *config = dev->config;

	err = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
					      GPIO_INT_DISABLE);
	if (unlikely(err)) {
		printk("Cannot disable IRQ");
		k_panic();
	}

	k_work_submit(&data->trigger_handler_work);
}

static void trigger_handler(struct k_work *work)
{
	sensor_trigger_handler_t handler;
	int err = 0;
	struct pmw3360_data *data = CONTAINER_OF(work, struct pmw3360_data,
						 trigger_handler_work);
	const struct device *dev = data->dev;
	const struct pmw3360_config *config = dev->config;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	handler = data->data_ready_handler;
	k_spin_unlock(&data->lock, key);

	if (!handler) {
		return;
	}

	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	handler(dev, &trig);

	key = k_spin_lock(&data->lock);
	if (data->data_ready_handler) {
		err = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
						      GPIO_INT_LEVEL_ACTIVE);
	}
	k_spin_unlock(&data->lock, key);

	if (unlikely(err)) {
		printk("Cannot re-enable IRQ");
		k_panic();
	}
}

static int pmw3360_async_init_power_up(const struct device *dev)
{
	/* Reset sensor */

	return reg_write(dev, PMW3360_REG_POWER_UP_RESET, 0x5A);
}

static int pmw3360_async_init_configure(const struct device *dev)
{
	int err;

	err = update_cpi(dev, CONFIG_PMW3360_CPI);

	if (!err) {
		err = update_downshift_time(dev,
					    PMW3360_REG_RUN_DOWNSHIFT,
					    CONFIG_PMW3360_RUN_DOWNSHIFT_TIME_MS);
	}

	if (!err) {
		err = update_downshift_time(dev,
					    PMW3360_REG_REST1_DOWNSHIFT,
					    CONFIG_PMW3360_REST1_DOWNSHIFT_TIME_MS);
	}

	if (!err) {
		err = update_downshift_time(dev,
					    PMW3360_REG_REST2_DOWNSHIFT,
					    CONFIG_PMW3360_REST2_DOWNSHIFT_TIME_MS);
	}

	return err;
}

static void pmw3360_async_init(struct k_work *work)
{
	struct pmw3360_data *data = CONTAINER_OF(work, struct pmw3360_data,
						 init_work.work);
	const struct device *dev = data->dev;

	printk("PMW3360 async init step %d", data->async_init_step);

	data->err = async_init_fn[data->async_init_step](dev);
	if (data->err) {
		printk("PMW3360 initialization failed");
	} else {
		data->async_init_step++;

		if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
			data->ready = true;
			printk("PMW3360 initialized");
		} else {
			k_work_schedule(&data->init_work,
					K_MSEC(async_init_delay[
						data->async_init_step]));
		}
	}
}

static int pmw3360_init_irq(const struct device *dev)
{
	int err;
	struct pmw3360_data *data = dev->data;
	const struct pmw3360_config *config = dev->config;

	if (!device_is_ready(config->irq_gpio.port)) {
		printk("IRQ GPIO device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
	if (err) {
		printk("Cannot configure IRQ GPIO");
		return err;
	}

	gpio_init_callback(&data->irq_gpio_cb, irq_handler,
			   BIT(config->irq_gpio.pin));

	err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
	if (err) {
		printk("Cannot add IRQ GPIO callback");
	}

	return err;
}

static int pmw3360_init(const struct device *dev)
{
	struct pmw3360_data *data = dev->data;
	const struct pmw3360_config *config = dev->config;
	int err;
	printk("Init invoked.");


	data->dev = dev;
	k_work_init(&data->trigger_handler_work, trigger_handler);

	if (!spi_is_ready_dt(&config->bus)) {
		printk("SPI device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(config->cs_gpio.port)) {
		printk("SPI CS device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (err) {
		printk("Cannot configure SPI CS GPIO");
		return err;
	}

	err = pmw3360_init_irq(dev);
	if (err) {
		return err;
	}

	k_work_init_delayable(&data->init_work, pmw3360_async_init);

	k_work_schedule(&data->init_work,
			K_MSEC(async_init_delay[data->async_init_step]));

	return err;
}

static int pmw3360_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct pmw3360_data *data = dev->data;
	uint8_t buf[PMW3360_BURST_SIZE];

	if (unlikely(chan != SENSOR_CHAN_ALL)) {
		return -ENOTSUP;
	}

	if (unlikely(!data->ready)) {
		printk("Device is not initialized yet");
		return -EBUSY;
	}

	int err = motion_burst_read(dev, buf, sizeof(buf));

	if (!err) {
		int16_t x = sys_get_le16(&buf[PMW3360_DX_POS]);
		int16_t y = sys_get_le16(&buf[PMW3360_DY_POS]);

		if (IS_ENABLED(CONFIG_PMW3360_ORIENTATION_0)) {
			data->x = -x;
			data->y = y;
		} else if (IS_ENABLED(CONFIG_PMW3360_ORIENTATION_90)) {
			data->x = y;
			data->y = x;
		} else if (IS_ENABLED(CONFIG_PMW3360_ORIENTATION_180)) {
			data->x = x;
			data->y = -y;
		} else if (IS_ENABLED(CONFIG_PMW3360_ORIENTATION_270)) {
			data->x = -y;
			data->y = -x;
		}
	}

	return err;
}

static int pmw3360_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct pmw3360_data *data = dev->data;

	if (unlikely(!data->ready)) {
		printk("Device is not initialized yet");
		return -EBUSY;
	}

	switch (chan) {
	case SENSOR_CHAN_POS_DX:
		val->val1 = data->x;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_POS_DY:
		val->val1 = data->y;
		val->val2 = 0;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int pmw3360_trigger_set(const struct device *dev,
			       const struct sensor_trigger *trig,
			       sensor_trigger_handler_t handler)
{
	struct pmw3360_data *data = dev->data;
	const struct pmw3360_config *config = dev->config;
	int err;

	if (unlikely(trig->type != SENSOR_TRIG_DATA_READY)) {
		return -ENOTSUP;
	}

	if (unlikely(trig->chan != SENSOR_CHAN_ALL)) {
		return -ENOTSUP;
	}

	if (unlikely(!data->ready)) {
		printk("Device is not initialized yet");
		return -EBUSY;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (handler) {
		err = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
						      GPIO_INT_LEVEL_ACTIVE);
	} else {
		err = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
						      GPIO_INT_DISABLE);
	}

	if (!err) {
		data->data_ready_handler = handler;
	}

	k_spin_unlock(&data->lock, key);

	return err;
}

static int pmw3360_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	struct pmw3360_data *data = dev->data;
	int err;

	if (unlikely(chan != SENSOR_CHAN_ALL)) {
		return -ENOTSUP;
	}

	if (unlikely(!data->ready)) {
		printk("Device is not initialized yet");
		return -EBUSY;
	}

	switch ((uint32_t)attr) {
	case PMW3360_ATTR_CPI:
		err = update_cpi(dev, PMW3360_SVALUE_TO_CPI(*val));
		break;

	case PMW3360_ATTR_REST_ENABLE:
		err = toggle_rest_modes(dev,
					PMW3360_REG_CONFIG2,
					PMW3360_SVALUE_TO_BOOL(*val));
		break;

	case PMW3360_ATTR_RUN_DOWNSHIFT_TIME:
		err = update_downshift_time(dev,
					    PMW3360_REG_RUN_DOWNSHIFT,
					    PMW3360_SVALUE_TO_TIME(*val));
		break;

	case PMW3360_ATTR_REST1_DOWNSHIFT_TIME:
		err = update_downshift_time(dev,
					    PMW3360_REG_REST1_DOWNSHIFT,
					    PMW3360_SVALUE_TO_TIME(*val));
		break;

	case PMW3360_ATTR_REST2_DOWNSHIFT_TIME:
		err = update_downshift_time(dev,
					    PMW3360_REG_REST2_DOWNSHIFT,
					    PMW3360_SVALUE_TO_TIME(*val));
		break;

	case PMW3360_ATTR_REST1_SAMPLE_TIME:
		err = update_sample_time(dev,
					 PMW3360_REG_REST1_RATE_LOWER,
					 PMW3360_REG_REST1_RATE_UPPER,
					 PMW3360_SVALUE_TO_TIME(*val));
		break;

	case PMW3360_ATTR_REST2_SAMPLE_TIME:
		err = update_sample_time(dev,
					 PMW3360_REG_REST2_RATE_LOWER,
					 PMW3360_REG_REST2_RATE_UPPER,
					 PMW3360_SVALUE_TO_TIME(*val));
		break;

	case PMW3360_ATTR_REST3_SAMPLE_TIME:
		err = update_sample_time(dev,
					 PMW3360_REG_REST3_RATE_LOWER,
					 PMW3360_REG_REST3_RATE_UPPER,
					 PMW3360_SVALUE_TO_TIME(*val));
		break;

	default:
		printk("Unknown attribute");
		return -ENOTSUP;
	}

	return err;
}

static const struct sensor_driver_api pmw3360_driver_api = {
	.sample_fetch = pmw3360_sample_fetch,
	.channel_get  = pmw3360_channel_get,
	.trigger_set  = pmw3360_trigger_set,
	.attr_set     = pmw3360_attr_set,
};

#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB
static struct pmw3360_data pmw3360_data;

static const struct pmw3360_config pmw3360_config = {
    .irq_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(button2), gpios), // Adjust manually
	.bus =  SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0),
    .cs_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(spi0), cs_gpios), // Adjust Chip Select manually
};

DEVICE_DEFINE(pmw3360, "PMW3360", pmw3360_init, NULL,
              &pmw3360_data, &pmw3360_config,
              POST_KERNEL, 50,
              &pmw3360_driver_api);
/**************************************************************/


// /* Callbacks */

void bt_ready(int err)
{
    if (err) {
        printk("bt_ready returned %d", err);
    }
    k_sem_give(&bt_init_ok);
}

void on_connected(struct bt_conn *conn, uint8_t err)
{
	if(err) {
		printk("connection err: %d", err);
		return;
	}
	printk("Connected.");
	current_conn = bt_conn_ref(conn);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason: %d)", reason);
	if(current_conn) 
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

void on_notif_changed(enum bt_button_notifications_enabled status)
{
	if (status == BT_BUTTON_NOTIFICATIONS_ENABLED) {
		printk("Notifications enabled");
	}
	else {
		printk("Notificatons disabled");
	}
}

void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	uint8_t temp_str[len+1];
	memcpy(temp_str, data, len);
	temp_str[len] = 0x00;

	printk("Received data on conn %p. Len: %d", (void *)conn, len);
	// printk("Data: %d", log_strdup(temp_str));
}

// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/hci.h>
// #include <zephyr/bluetooth/conn.h>
// #include <zephyr/bluetooth/uuid.h>
// #include <zephyr/bluetooth/gatt.h>

// #include <zephyr/bluetooth/services/bas.h>
// #include <bluetooth/services/hids.h>
// #include <zephyr/bluetooth/services/dis.h>

// #define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
// #define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// #define BASE_USB_HID_SPEC_VERSION   0x0101
// #define CONFIG_BT_DIRECTED_ADVERTISING 1
// // #define CONFIG_BT_HIDS_SECURITY_ENABLED 1



// /* Number of pixels by which the cursor is moved when a button is pushed. */
// #define MOVEMENT_SPEED              5
// /* Number of input reports in this application. */
// #define INPUT_REPORT_COUNT          3
// /* Length of Mouse Input Report containing button data. */
// #define INPUT_REP_BUTTONS_LEN       3
// /* Length of Mouse Input Report containing movement data. */
// #define INPUT_REP_MOVEMENT_LEN      3
// /* Length of Mouse Input Report containing media player data. */
// #define INPUT_REP_MEDIA_PLAYER_LEN  1
// /* Index of Mouse Input Report containing button data. */
// #define INPUT_REP_BUTTONS_INDEX     0
// /* Index of Mouse Input Report containing movement data. */
// #define INPUT_REP_MOVEMENT_INDEX    1
// /* Index of Mouse Input Report containing media player data. */
// #define INPUT_REP_MPLAYER_INDEX     2
// /* Id of reference to Mouse Input Report containing button data. */
// #define INPUT_REP_REF_BUTTONS_ID    1
// /* Id of reference to Mouse Input Report containing movement data. */
// #define INPUT_REP_REF_MOVEMENT_ID   2
// /* Id of reference to Mouse Input Report containing media player data. */
// #define INPUT_REP_REF_MPLAYER_ID    3

// // /* HIDs queue size. */
// // #define HIDS_QUEUE_SIZE 10

// // /* Key used to move cursor left */
// // #define KEY_LEFT_MASK   DK_BTN1_MSK
// // /* Key used to move cursor up */
// // #define KEY_UP_MASK     DK_BTN2_MSK
// // /* Key used to move cursor right */
// // #define KEY_RIGHT_MASK  DK_BTN3_MSK
// // /* Key used to move cursor down */
// // #define KEY_DOWN_MASK   DK_BTN4_MSK

// // /* Key used to accept or reject passkey value */
// // #define KEY_PAIRING_ACCEPT DK_BTN1_MSK
// // #define KEY_PAIRING_REJECT DK_BTN2_MSK

// // // /* HIDS instance. */
// // // BT_HIDS_DEF(hids_obj,
// // // 	    INPUT_REP_BUTTONS_LEN,
// // // 	    INPUT_REP_MOVEMENT_LEN,
// // // 	    INPUT_REP_MEDIA_PLAYER_LEN);

// static struct k_work hids_work;
// struct mouse_pos {
// 	int16_t x_val;
// 	int16_t y_val;
// };

// // /* Mouse movement queue. */
// // K_MSGQ_DEFINE(hids_queue,
// // 	      sizeof(struct mouse_pos),
// // 	      HIDS_QUEUE_SIZE,
// // 	      4);

// // // #if CONFIG_BT_DIRECTED_ADVERTISING
// // // /* Bonded address queue. */
// // // K_MSGQ_DEFINE(bonds_queue,
// // // 	      sizeof(bt_addr_le_t),
// // // 	      CONFIG_BT_MAX_PAIRED,
// // // 	      4);
// // // #endif

// // // static const struct bt_data ad[] = {
// // // 	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
// // // 		      (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
// // // 		      (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
// // // 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// // // 	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
// // // 					  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
// // // };

// // // static const struct bt_data sd[] = {
// // // 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
// // // };

// static struct conn_mode {
// 	struct bt_conn *conn;
// 	bool in_boot_mode;
// } conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

// static volatile bool is_adv_running;

// static struct k_work adv_work;

// // // static struct k_work pairing_work;
// // // struct pairing_data_mitm {
// // // 	struct bt_conn *conn;
// // // 	unsigned int passkey;
// // // };

// // // K_MSGQ_DEFINE(mitm_queue,
// // // 	      sizeof(struct pairing_data_mitm),
// // // 	      CONFIG_BT_HIDS_MAX_CLIENT_COUNT,
// // // 	      4);

// // // #if CONFIG_BT_DIRECTED_ADVERTISING
// // // static void bond_find(const struct bt_bond_info *info, void *user_data)
// // // {
// // // 	int err;

// // // 	/* Filter already connected peers. */
// // // 	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
// // // 		if (conn_mode[i].conn) {
// // // 			const bt_addr_le_t *dst =
// // // 				bt_conn_get_dst(conn_mode[i].conn);

// // // 			if (!bt_addr_le_cmp(&info->addr, dst)) {
// // // 				return;
// // // 			}
// // // 		}
// // // 	}

// // // 	err = k_msgq_put(&bonds_queue, (void *) &info->addr, K_NO_WAIT);
// // // 	if (err) {
// // // 		printk("No space in the queue for the bond.\n");
// // // 	}
// // // }
// // // #endif

// // // static void advertising_continue(void)
// // // {
// // // 	struct bt_le_adv_param adv_param;
// // // #if CONFIG_BT_DIRECTED_ADVERTISING
// // // 	bt_addr_le_t addr;

// // // 	if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
// // // 		char addr_buf[BT_ADDR_LE_STR_LEN];
// // // 		int err;

// // // 		if (is_adv_running) {
// // // 			err = bt_le_adv_stop();
// // // 			if (err) {
// // // 				printk("Advertising failed to stop (err %d)\n", err);
// // // 				return;
// // // 			}
// // // 			is_adv_running = false;
// // // 		}

// // // 		adv_param = *BT_LE_ADV_CONN_DIR(&addr);
// // // 		adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

// // // 		err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);

// // // 		if (err) {
// // // 			printk("Directed advertising failed to start (err %d)\n", err);
// // // 			return;
// // // 		}

// // // 		bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
// // // 		printk("Direct advertising to %s started\n", addr_buf);
// // // 	} else
// // // #endif
// // // 	{
// // // 		int err;

// // // 		if (is_adv_running) {
// // // 			return;
// // // 		}

// // // 		adv_param = *BT_LE_ADV_CONN;
// // // 		adv_param.options |= BT_LE_ADV_OPT_ONE_TIME;
// // // 		err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
// // // 				  sd, ARRAY_SIZE(sd));
// // // 		if (err) {
// // // 			printk("Advertising failed to start (err %d)\n", err);
// // // 			return;
// // // 		}

// // // 		printk("Regular advertising started\n");
// // // 	}

// // // 	is_adv_running = true;
// // // }

// // // static void advertising_start(void)
// // // {
// // // #if CONFIG_BT_DIRECTED_ADVERTISING
// // // 	k_msgq_purge(&bonds_queue);
// // // 	bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);
// // // #endif

// // // 	k_work_submit(&adv_work);
// // // }

// // // static void advertising_process(struct k_work *work)
// // // {
// // // 	advertising_continue();
// // // }

// // // static void pairing_process(struct k_work *work)
// // // {
// // // 	int err;
// // // 	struct pairing_data_mitm pairing_data;

// // // 	char addr[BT_ADDR_LE_STR_LEN];

// // // 	err = k_msgq_peek(&mitm_queue, &pairing_data);
// // // 	if (err) {
// // // 		return;
// // // 	}

// // // 	bt_addr_le_to_str(bt_conn_get_dst(pairing_data.conn),
// // // 			  addr, sizeof(addr));

// // // 	printk("Passkey for %s: %06u\n", addr, pairing_data.passkey);

// // // 	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54HX) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54LX)) {
// // // 		printk("Press Button 0 to confirm, Button 1 to reject.\n");
// // // 	} else {
// // // 		printk("Press Button 1 to confirm, Button 2 to reject.\n");
// // // 	}
// // // }


// // // static void insert_conn_object(struct bt_conn *conn)
// // // {
// // // 	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
// // // 		if (!conn_mode[i].conn) {
// // // 			conn_mode[i].conn = conn;
// // // 			conn_mode[i].in_boot_mode = false;

// // // 			return;
// // // 		}
// // // 	}

// // // 	printk("Connection object could not be inserted %p\n", conn);
// // // }


// // // static bool is_conn_slot_free(void)
// // // {
// // // 	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
// // // 		if (!conn_mode[i].conn) {
// // // 			return true;
// // // 		}
// // // 	}

// // // 	return false;
// // // }


// // // static void connected(struct bt_conn *conn, uint8_t err)
// // // {
// // // 	char addr[BT_ADDR_LE_STR_LEN];

// // // 	is_adv_running = false;

// // // 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// // // 	if (err) {
// // // 		if (err == BT_HCI_ERR_ADV_TIMEOUT) {
// // // 			printk("Direct advertising to %s timed out\n", addr);
// // // 			k_work_submit(&adv_work);
// // // 		} else {
// // // 			printk("Failed to connect to %s 0x%02x %s\n", addr, err,
// // // 			       bt_hci_err_to_str(err));
// // // 		}
// // // 		return;
// // // 	}

// // // 	printk("Connected %s\n", addr);

// // // 	err = bt_hids_connected(&hids_obj, conn);

// // // 	if (err) {
// // // 		printk("Failed to notify HID service about connection\n");
// // // 		return;
// // // 	}

// // // 	insert_conn_object(conn);

// // // 	if (is_conn_slot_free()) {
// // // 		advertising_start();
// // // 	}
// // // }


// // // static void disconnected(struct bt_conn *conn, uint8_t reason)
// // // {
// // // 	int err;
// // // 	char addr[BT_ADDR_LE_STR_LEN];

// // // 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// // // 	printk("Disconnected from %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

// // // 	err = bt_hids_disconnected(&hids_obj, conn);

// // // 	if (err) {
// // // 		printk("Failed to notify HID service about disconnection\n");
// // // 	}

// // // 	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
// // // 		if (conn_mode[i].conn == conn) {
// // // 			conn_mode[i].conn = NULL;
// // // 			break;
// // // 		}
// // // 	}

// // // 	advertising_start();
// // // }


// // // #ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
// // // static void security_changed(struct bt_conn *conn, bt_security_t level,
// // // 			     enum bt_security_err err)
// // // {
// // // 	char addr[BT_ADDR_LE_STR_LEN];

// // // 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// // // 	if (!err) {
// // // 		printk("Security changed: %s level %u\n", addr, level);
// // // 	} else {
// // // 		printk("Security failed: %s level %u err %d %s\n", addr, level, err,
// // // 		       bt_security_err_to_str(err));
// // // 	}
// // // }
// // // #endif

// // // BT_CONN_CB_DEFINE(conn_callbacks) = {
// // // 	.connected = connected,
// // // 	.disconnected = disconnected,
// // // #ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
// // // 	.security_changed = security_changed,
// // // #endif
// // // };


// static void hids_pm_evt_handler(enum bt_hids_pm_evt evt,
// 				struct bt_conn *conn)
// {
// 	char addr[BT_ADDR_LE_STR_LEN];
// 	size_t i;

// 	for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
// 		if (conn_mode[i].conn == conn) {
// 			break;
// 		}
// 	}

// 	if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
// 		return;
// 	}

// 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// 	switch (evt) {
// 	case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
// 		printk("Boot mode entered %s\n", addr);
// 		conn_mode[i].in_boot_mode = true;
// 		break;

// 	case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
// 		printk("Report mode entered %s\n", addr);
// 		conn_mode[i].in_boot_mode = false;
// 		break;

// 	default:
// 		break;
// 	}
// }


// static void hid_init(void)
// {
// 	int err;
// 	struct bt_hids_init_param hids_init_param = { 0 };
// 	struct bt_hids_inp_rep *hids_inp_rep;
// 	static const uint8_t mouse_movement_mask[DIV_ROUND_UP(INPUT_REP_MOVEMENT_LEN, 8)] = {0};

// 	static const uint8_t report_map[] = {
// 		0x05, 0x01,     /* Usage Page (Generic Desktop) */
// 		0x09, 0x02,     /* Usage (Mouse) */

// 		0xA1, 0x01,     /* Collection (Application) */

// 		/* Report ID 1: Mouse buttons + scroll/pan */
// 		0x85, 0x01,       /* Report Id 1 */
// 		0x09, 0x01,       /* Usage (Pointer) */
// 		0xA1, 0x00,       /* Collection (Physical) */
// 		0x95, 0x05,       /* Report Count (3) */
// 		0x75, 0x01,       /* Report Size (1) */
// 		0x05, 0x09,       /* Usage Page (Buttons) */
// 		0x19, 0x01,       /* Usage Minimum (01) */
// 		0x29, 0x05,       /* Usage Maximum (05) */
// 		0x15, 0x00,       /* Logical Minimum (0) */
// 		0x25, 0x01,       /* Logical Maximum (1) */
// 		0x81, 0x02,       /* Input (Data, Variable, Absolute) */
// 		0x95, 0x01,       /* Report Count (1) */
// 		0x75, 0x03,       /* Report Size (3) */
// 		0x81, 0x01,       /* Input (Constant) for padding */
// 		0x75, 0x08,       /* Report Size (8) */
// 		0x95, 0x01,       /* Report Count (1) */
// 		0x05, 0x01,       /* Usage Page (Generic Desktop) */
// 		0x09, 0x38,       /* Usage (Wheel) */
// 		0x15, 0x81,       /* Logical Minimum (-127) */
// 		0x25, 0x7F,       /* Logical Maximum (127) */
// 		0x81, 0x06,       /* Input (Data, Variable, Relative) */
// 		0x05, 0x0C,       /* Usage Page (Consumer) */
// 		0x0A, 0x38, 0x02, /* Usage (AC Pan) */
// 		0x95, 0x01,       /* Report Count (1) */
// 		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
// 		0xC0,             /* End Collection (Physical) */

// 		/* Report ID 2: Mouse motion */
// 		0x85, 0x02,       /* Report Id 2 */
// 		0x09, 0x01,       /* Usage (Pointer) */
// 		0xA1, 0x00,       /* Collection (Physical) */
// 		0x75, 0x0C,       /* Report Size (12) */
// 		0x95, 0x02,       /* Report Count (2) */
// 		0x05, 0x01,       /* Usage Page (Generic Desktop) */
// 		0x09, 0x30,       /* Usage (X) */
// 		0x09, 0x31,       /* Usage (Y) */
// 		0x16, 0x01, 0xF8, /* Logical maximum (2047) */
// 		0x26, 0xFF, 0x07, /* Logical minimum (-2047) */
// 		0x81, 0x06,       /* Input (Data, Variable, Relative) */
// 		0xC0,             /* End Collection (Physical) */
// 		0xC0,             /* End Collection (Application) */

// 		/* Report ID 3: Advanced buttons */
// 		0x05, 0x0C,       /* Usage Page (Consumer) */
// 		0x09, 0x01,       /* Usage (Consumer Control) */
// 		0xA1, 0x01,       /* Collection (Application) */
// 		0x85, 0x03,       /* Report Id (3) */
// 		0x15, 0x00,       /* Logical minimum (0) */
// 		0x25, 0x01,       /* Logical maximum (1) */
// 		0x75, 0x01,       /* Report Size (1) */
// 		0x95, 0x01,       /* Report Count (1) */

// 		0x09, 0xCD,       /* Usage (Play/Pause) */
// 		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
// 		0x0A, 0x83, 0x01, /* Usage (Consumer Control Configuration) */
// 		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
// 		0x09, 0xB5,       /* Usage (Scan Next Track) */
// 		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
// 		0x09, 0xB6,       /* Usage (Scan Previous Track) */
// 		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */

// 		0x09, 0xEA,       /* Usage (Volume Down) */
// 		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
// 		0x09, 0xE9,       /* Usage (Volume Up) */
// 		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
// 		0x0A, 0x25, 0x02, /* Usage (AC Forward) */
// 		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
// 		0x0A, 0x24, 0x02, /* Usage (AC Back) */
// 		0x81, 0x06,       /* Input (Data,Value,Relative,Bit Field) */
// 		0xC0              /* End Collection */
// 	};

// 	hids_init_param.rep_map.data = report_map;
// 	hids_init_param.rep_map.size = sizeof(report_map);

// 	hids_init_param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
// 	hids_init_param.info.b_country_code = 0x00;
// 	hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE |
// 				      BT_HIDS_NORMALLY_CONNECTABLE);

// 	hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
// 	hids_inp_rep->size = INPUT_REP_BUTTONS_LEN;
// 	hids_inp_rep->id = INPUT_REP_REF_BUTTONS_ID;
// 	hids_init_param.inp_rep_group_init.cnt++;

// 	hids_inp_rep++;
// 	hids_inp_rep->size = INPUT_REP_MOVEMENT_LEN;
// 	hids_inp_rep->id = INPUT_REP_REF_MOVEMENT_ID;
// 	hids_inp_rep->rep_mask = mouse_movement_mask;
// 	hids_init_param.inp_rep_group_init.cnt++;

// 	hids_inp_rep++;
// 	hids_inp_rep->size = INPUT_REP_MEDIA_PLAYER_LEN;
// 	hids_inp_rep->id = INPUT_REP_REF_MPLAYER_ID;
// 	hids_init_param.inp_rep_group_init.cnt++;

// 	hids_init_param.is_mouse = true;
// 	hids_init_param.pm_evt_handler = hids_pm_evt_handler;

// 	// err = bt_hids_init(&hids_obj, &hids_init_param);
// 	// __ASSERT(err == 0, "HIDS initialization failed\n");
// }


// // static void mouse_movement_send(int16_t x_delta, int16_t y_delta)
// // {
// // 	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {

// // 		if (!conn_mode[i].conn) {
// // 			continue;
// // 		}

// // 		if (conn_mode[i].in_boot_mode) {
// // 			x_delta = MAX(MIN(x_delta, SCHAR_MAX), SCHAR_MIN);
// // 			y_delta = MAX(MIN(y_delta, SCHAR_MAX), SCHAR_MIN);

// // 			bt_hids_boot_mouse_inp_rep_send(&hids_obj,
// // 							     conn_mode[i].conn,
// // 							     NULL,
// // 							     (int8_t) x_delta,
// // 							     (int8_t) y_delta,
// // 							     NULL);
// // 		} else {
// // 			uint8_t x_buff[2];
// // 			uint8_t y_buff[2];
// // 			uint8_t buffer[INPUT_REP_MOVEMENT_LEN];

// // 			int16_t x = MAX(MIN(x_delta, 0x07ff), -0x07ff);
// // 			int16_t y = MAX(MIN(y_delta, 0x07ff), -0x07ff);

// // 			/* Convert to little-endian. */
// // 			sys_put_le16(x, x_buff);
// // 			sys_put_le16(y, y_buff);

// // 			/* Encode report. */
// // 			BUILD_ASSERT(sizeof(buffer) == 3,
// // 					 "Only 2 axis, 12-bit each, are supported");

// // 			buffer[0] = x_buff[0];
// // 			buffer[1] = (y_buff[0] << 4) | (x_buff[1] & 0x0f);
// // 			buffer[2] = (y_buff[1] << 4) | (y_buff[0] >> 4);


// // 			bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn,
// // 						  INPUT_REP_MOVEMENT_INDEX,
// // 						  buffer, sizeof(buffer), NULL);
// // 		}
// // 	}
// // }


// // static void mouse_handler(struct k_work *work)
// // {
// // 	struct mouse_pos pos;

// // 	while (!k_msgq_get(&hids_queue, &pos, K_NO_WAIT)) {
// // 		mouse_movement_send(pos.x_val, pos.y_val);
// // 	}
// // }
// // #if defined(CONFIG_BT_HIDS_SECURITY_ENABLED)
// // static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
// // {
// // 	char addr[BT_ADDR_LE_STR_LEN];

// // 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// // 	printk("Passkey for %s: %06u\n", addr, passkey);
// // }


// // static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
// // {
// // 	int err;

// // 	struct pairing_data_mitm pairing_data;

// // 	pairing_data.conn    = bt_conn_ref(conn);
// // 	pairing_data.passkey = passkey;

// // 	err = k_msgq_put(&mitm_queue, &pairing_data, K_NO_WAIT);
// // 	if (err) {
// // 		printk("Pairing queue is full. Purge previous data.\n");
// // 	}

// // 	/* In the case of multiple pairing requests, trigger
// // 	 * pairing confirmation which needed user interaction only
// // 	 * once to avoid display information about all devices at
// // 	 * the same time. Passkey confirmation for next devices will
// // 	 * be proccess from queue after handling the earlier ones.
// // 	 */
// // 	if (k_msgq_num_used_get(&mitm_queue) == 1) {
// // 		k_work_submit(&pairing_work);
// // 	}
// // }


// // static void auth_cancel(struct bt_conn *conn)
// // {
// // 	char addr[BT_ADDR_LE_STR_LEN];

// // 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// // 	printk("Pairing cancelled: %s\n", addr);
// // }


// // static void pairing_complete(struct bt_conn *conn, bool bonded)
// // {
// // 	char addr[BT_ADDR_LE_STR_LEN];

// // 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// // 	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
// // }


// // static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
// // {
// // 	char addr[BT_ADDR_LE_STR_LEN];
// // 	struct pairing_data_mitm pairing_data;

// // 	if (k_msgq_peek(&mitm_queue, &pairing_data) != 0) {
// // 		return;
// // 	}

// // 	if (pairing_data.conn == conn) {
// // 		bt_conn_unref(pairing_data.conn);
// // 		k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT);
// // 	}

// // 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// // 	printk("Pairing failed conn: %s, reason %d %s\n", addr, reason,
// // 	       bt_security_err_to_str(reason));
// // }

// // static struct bt_conn_auth_cb conn_auth_callbacks = {
// // 	.passkey_display = auth_passkey_display,
// // 	.passkey_confirm = auth_passkey_confirm,
// // 	.cancel = auth_cancel,
// // };

// // static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
// // 	.pairing_complete = pairing_complete,
// // 	.pairing_failed = pairing_failed
// // };
// // #else
// // static struct bt_conn_auth_cb conn_auth_callbacks;
// // static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
// // #endif /* defined(CONFIG_BT_HIDS_SECURITY_ENABLED) */


// // static void num_comp_reply(bool accept)
// // {
// // 	struct pairing_data_mitm pairing_data;
// // 	struct bt_conn *conn;

// // 	if (k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT) != 0) {
// // 		return;
// // 	}

// // 	conn = pairing_data.conn;

// // 	if (accept) {
// // 		bt_conn_auth_passkey_confirm(conn);
// // 		printk("Numeric Match, conn %p\n", conn);
// // 	} else {
// // 		bt_conn_auth_cancel(conn);
// // 		printk("Numeric Reject, conn %p\n", conn);
// // 	}

// // 	bt_conn_unref(pairing_data.conn);

// // 	if (k_msgq_num_used_get(&mitm_queue)) {
// // 		k_work_submit(&pairing_work);
// // 	}
// // }


// // void button_changed(uint32_t button_state, uint32_t has_changed)
// // {
// // 	bool data_to_send = false;
// // 	struct mouse_pos pos;
// // 	uint32_t buttons = button_state & has_changed;

// // 	memset(&pos, 0, sizeof(struct mouse_pos));

// // 	if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
// // 		if (k_msgq_num_used_get(&mitm_queue)) {
// // 			if (buttons & KEY_PAIRING_ACCEPT) {
// // 				num_comp_reply(true);

// // 				return;
// // 			}

// // 			if (buttons & KEY_PAIRING_REJECT) {
// // 				num_comp_reply(false);

// // 				return;
// // 			}
// // 		}
// // 	}

// // 	if (buttons & KEY_LEFT_MASK) {
// // 		pos.x_val -= MOVEMENT_SPEED;
// // 		printk("%s(): left\n", __func__);
// // 		data_to_send = true;
// // 	}
// // 	if (buttons & KEY_UP_MASK) {
// // 		pos.y_val -= MOVEMENT_SPEED;
// // 		printk("%s(): up\n", __func__);
// // 		data_to_send = true;
// // 	}
// // 	if (buttons & KEY_RIGHT_MASK) {
// // 		pos.x_val += MOVEMENT_SPEED;
// // 		printk("%s(): right\n", __func__);
// // 		data_to_send = true;
// // 	}
// // 	if (buttons & KEY_DOWN_MASK) {
// // 		pos.y_val += MOVEMENT_SPEED;
// // 		printk("%s(): down\n", __func__);
// // 		data_to_send = true;
// // 	}

// // 	if (data_to_send) {
// // 		int err;

// // 		err = k_msgq_put(&hids_queue, &pos, K_NO_WAIT);
// // 		if (err) {
// // 			printk("No space in the queue for button pressed\n");
// // 			return;
// // 		}
// // 		if (k_msgq_num_used_get(&hids_queue) == 1) {
// // 			k_work_submit(&hids_work);
// // 		}
// // 	}
// // }


// // void configure_buttons(void)
// // {
// // 	int err;

// // 	err = dk_buttons_init(button_changed);
// // 	if (err) {
// // 		printk("Cannot init buttons (err: %d)\n", err);
// // 	}
// // }


// // static void bas_notify(void)
// // {
// // 	uint8_t battery_level = bt_bas_get_battery_level();

// // 	battery_level--;

// // 	if (!battery_level) {
// // 		battery_level = 100U;
// // 	}

// // 	bt_bas_set_battery_level(battery_level);
// // }


int main(void)
{
	int err;

	// printk("Starting Bluetooth Peripheral HIDS mouse example\n");

	// if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
	// 	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	// 	if (err) {
	// 		printk("Failed to register authorization callbacks.\n");
	// 		return 0;
	// 	}

	// 	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	// 	if (err) {
	// 		printk("Failed to register authorization info callbacks.\n");
	// 		return 0;
	// 	}
	// }

	/* DIS initialized at system boot with SYS_INIT macro. */
	// hid_init();

	// // err = bt_enable(NULL);
	// if (err) {
	// 	printk("Bluetooth init failed (err %d)\n", err);
	// 	return 0;
	// }

	// printk("Bluetooth initialized\n");

	// k_work_init(&hids_work, mouse_handler);
	// k_work_init(&adv_work, advertising_process);
	// if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
	// 	k_work_init(&pairing_work, pairing_process);
	// }

	// if (IS_ENABLED(CONFIG_SETTINGS)) {
	// 	settings_load();
	// }

	// advertising_start();

	// configure_buttons();


    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Failed to configure LED pin");
        return;
    }

    printk("Blinking LED on P0.20\n");
	bt_conn_cb_register(&bluetooth_callbacks);
    remote_callbacks.notif_changed = remote_callbacks_s.notif_changed;

    err = bt_enable(bt_ready);
    if (err) {
        printk("bt_enable returned %d", err);
        return err;
    }

	k_sem_take(&bt_init_ok, K_FOREVER);

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Couldn't start advertising (err = %d)", err);
        return err;
    }

	// #define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB
	// static struct pmw3360_data data;
    // static const struct pmw3360_config config = {		       
	// 	.irq_gpio = 0,	       
	// 	.bus =  SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0),							       
	// 	.cs_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(spi0), cs_gpios),
	// };		
	

	while (1) {
		// k_sleep(K_SECONDS(1));
		gpio_pin_toggle_dt(&led);
        k_sleep(K_MSEC(3000));
		printk("Blinking LED on P0.20\n");

	// 	/* Battery level simulation */
		// bas_notify();
	}
}