/* drivers/input/touchscreen/sec_ts.h
 *
 * Copyright (C) 2015 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SEC_TS_H
#define _SEC_TS_H

#include <linux/input/sec_cmd.h>

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif


#define CONFIG_6FT0
#define USE_OPEN_CLOSE
#define CONFIG_FW_UPDATE_ON_PROBE


#define SEC_TS_I2C_NAME			"sec_ts"

/* register address */
#define SEC_TS_CMD_SENSE_ON		0x40
#define SEC_TS_CMD_SENSE_OFF		0x41
#define SEC_TS_CMD_SW_RESET		0x42

#define SEC_TS_READ_DEVICE_ID		0x52
#define SEC_TS_READ_SUB_ID		0x53
#define SEC_TS_READ_BOOT_STATUS		0x55

#define SEC_TS_CMD_EDGE_DEADZONE        0x6E

#define SEC_TS_READ_ONE_EVENT		0x71


#define SEC_TS_EVENT_BUFF_SIZE		8
#define I2C_WRITE_BUFFER_SIZE		255


#define DEVICE_ID_RETRY_COUNT		10
#define SEC_TS_I2C_RETRY_CNT		10
#define SEC_TS_WAIT_RETRY_CNT		100
#define MAX_SUPPORT_TOUCH_COUNT		10


#define SEC_TS_STATUS_BOOT_MODE		0X10
#define SEC_TS_STATUS_APP_MODE		0X20


#define SEC_TS_STATE_POWER_ON		1
#define SEC_TS_STATE_POWER_OFF		0


#define SEC_TS_STATUS_EVENT		0
#define SEC_TS_COORDINATE_EVENT		1
#define SEC_TS_GESTURE_EVENT		2


#define SEC_TS_COORD_ACT_NONE		0
#define SEC_TS_COORD_ACT_PRESS		1
#define SEC_TS_COORD_ACT_MOVE		2
#define SEC_TS_COORD_ACT_RELEASE	3


#define SEC_TS_TOUCHTYPE_FINGER		0
#define SEC_TS_TOUCHTYPE_PROXIMITY	1
#define SEC_TS_TOUCHTYPE_GLOVE		3
#define SEC_TS_TOUCHTYPE_HOVER		5


#define SEC_TS_DEFAULT_UMS_FW		"/sdcard/Firmware/TSP/lsi.bin"
#define SEC_TS_MAX_FW_PATH		64
#define SEC_TS_FW_HEADER_SIGN		0x53494654 /* TFIS */
#define SEC_TS_FW_CHUNK_SIGN		0x53434654 /* TFCS */


enum {
	BUILT_IN = 0,
	UMS,
	NONE,
};

typedef struct {
	u32 signature;			/* signature */
	u32 version;			/* version */
	u32 totalsize;			/* total size */
	u32 param_area;			/* parameter area */
	u32 flag;			/* mode select/bootloader mode */
	u32 setting;			/* HWB settings */
	u32 checksum;			/* checksum */
	u32 boot_addr[3];
	u32 flash_addr[3];
	u32 chunk_num[3];
} fw_header;


typedef struct
{
	u32 signature;
	u32 addr;
	u32 size;
	u32 check_sum;
}fw_chunk;

struct sec_ts_coordinate {
	u8 id;
	u16 x;
	u16 y;
	u8 z;
	u8 major;
	u8 minor;
	u8 type;
	u8 action;
	u16 mcount;
};

struct sec_ts_event_coordinate{
	u8 state:3;
	u8 type:3;
	u8 eid:2;

	u8 tid;
	u8 x_11_4;
	u8 y_11_4;

	u8 y_3_0:4;
	u8 x_3_0:4;

	u8 z;
	u8 major;
	u8 minor;
} __attribute__ ((packed));


struct sec_ts_data {
	struct device *dev;
	struct i2c_client *client;
	struct sec_ts_plat_data *pdata;
	int irq;
	struct input_dev *input;
	struct mutex i2c_mutex;
	struct mutex device_mutex;
#ifdef CONFIG_FB
	struct notifier_block fb_notifier;
	//bool fb_ready;
#endif
	struct sec_cmd_data sec;

	struct sec_ts_coordinate coord[MAX_SUPPORT_TOUCH_COUNT + 1];
	int touch_count;

	int power_status;
};

struct sec_ts_plat_data {
	int max_x;
	int max_y;
	unsigned gpio;
	int irq_type;
	int tx_count;
	int rx_count;
	int panel_revision;
	u8 ic_fw_ver[4];
	u8 bin_fw_ver[4];
	const char *firmware_name;
	bool bringup;
	bool support_mt_pressure;
};

int sec_ts_i2c_write_reg(struct sec_ts_data *ts_data, u8 reg, u8 *data, int len);
int sec_ts_i2c_read_reg(struct sec_ts_data *ts_data, u8 reg, u8 *data, int len);
int sec_ts_i2c_read(struct sec_ts_data *ts_data, u8 *data, int len);
int sec_ts_i2c_write(struct sec_ts_data *ts_data, u8 *data, int len);

int sec_ts_firmware_update(struct sec_ts_data *data, bool force);
int sec_ts_firmware_update_on_hidden_menu(struct sec_ts_data *ts,int update_type);

void sec_ts_delay(unsigned int ms);
int sec_ts_fn_init(struct sec_ts_data *ts);

int sec_ts_start_device(struct sec_ts_data *data);
int sec_ts_stop_device(struct sec_ts_data *data);

extern unsigned int lcdtype;

#ifdef CONFIG_BATTERY_SAMSUNG
extern unsigned int lpcharge;
#endif

#endif
