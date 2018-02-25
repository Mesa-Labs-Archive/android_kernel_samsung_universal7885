/* drivers/input/touchscreen/sec_ts.c
 *
 * Copyright (C) 2017 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_SEC_SYSFS
#include <linux/sec_sysfs.h>
#endif

#include "sec_ts.h"


int sec_ts_i2c_write_reg(struct sec_ts_data *ts_data, u8 reg, u8 *data, int len)
{
	struct i2c_msg msg;
	u8 buf[I2C_WRITE_BUFFER_SIZE + 1];
	u8 retry;
	int ret = 0;

	if (len > I2C_WRITE_BUFFER_SIZE) {
		input_err(true, &ts_data->client->dev, "sec_ts_i2c_write len is larger than buffer size\n");
		return -EIO;
	}

	buf[0] = reg;
	memcpy(buf+1, data, len);

	msg.addr = ts_data->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;

	mutex_lock(&ts_data->i2c_mutex);

	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		if (ts_data->power_status == SEC_TS_STATE_POWER_OFF) {
			input_err(true, &ts_data->client->dev, "%s: POWER_STATUS OFF\n",
					__func__);
			ret = -EIO;
			goto err;
		}

		if ((ret = i2c_transfer(ts_data->client->adapter, &msg, 1)) == 1) {
			ret = 0;
			break;
		}
	}

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		input_err(true, &ts_data->client->dev, "%s: I2C write over retry limit %d\n",
				__func__, ret);
	}
err:
	mutex_unlock(&ts_data->i2c_mutex);

	return ret;
}

int sec_ts_i2c_read_reg(struct sec_ts_data *ts_data, u8 reg, u8 *data, int len)
{
	struct i2c_msg msg[2];
	u8 buf[4];
	u8 retry;
	int ret = 0;

	buf[0] = reg;
	msg[0].addr = ts_data->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	mutex_lock(&ts_data->i2c_mutex);

	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		if (ts_data->power_status == SEC_TS_STATE_POWER_OFF) {
			input_err(true, &ts_data->client->dev, "%s: POWER_STATUS OFF\n",
					__func__);
			ret = -EIO;
			goto err;
		}

		if ((ret = i2c_transfer(ts_data->client->adapter, msg, 1)) == 1) {
			ret = 0;
			break;
		}
	}

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		input_err(true, &ts_data->client->dev, "%s: I2C write over retry limit %d\n",
				__func__, ret);
		goto err;
	}

	msg[0].addr = ts_data->client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = len;
	msg[0].buf = data;

	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		if (ts_data->power_status == SEC_TS_STATE_POWER_OFF) {
			input_err(true, &ts_data->client->dev, "%s: POWER_STATUS OFF\n",
					__func__);
			ret = -EIO;
			goto err;
		}

		if ((ret = i2c_transfer(ts_data->client->adapter, msg, 1)) == 1) {
			ret = 0;
			break;
		}
	}

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		input_err(true, &ts_data->client->dev, "%s: I2C read over retry limit %d\n",
				__func__, ret);
	}
err:
	mutex_unlock(&ts_data->i2c_mutex);

	return ret;
}
/*
static int sec_ts_i2c_read(struct sec_ts_data *ts_data, u8 *data, int len)
{
	struct i2c_msg msg;
	u8 retry;
	int ret;

	msg.addr = ts_data->client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	mutex_lock(&ts_data->i2c_mutex);

	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		if ((ret = i2c_transfer(ts_data->client->adapter, &msg, 1)) == 1) {
			break;
		}
		if (ts_data->power_status == SEC_TS_STATE_POWER_OFF) {
			input_err(true, &ts_data->client->dev,
				"%s: fail to POWER_STATUS=OFF ret = %d\n", __func__, ret);
			mutex_unlock(&ts_data->i2c_mutex);
			goto err;
		}
	}

	mutex_unlock(&ts_data->i2c_mutex);

	if (retry == 10) {
		input_err(true, &ts_data->client->dev, "%s: I2C read over retry limit\n",
				__func__);
		ret = -EIO;
	}

	if (ret == 1)
		return 0;
err:
	return -EIO;
}
*/
int sec_ts_i2c_write(struct sec_ts_data *ts_data, u8 *data, int len)
{
	int retry;
	int ret;

	mutex_lock(&ts_data->i2c_mutex);
	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		if (ts_data->power_status == SEC_TS_STATE_POWER_OFF) {
			input_err(true, &ts_data->client->dev, "%s: POWER_STATUS OFF\n",
					__func__);
			ret = -EIO;
			goto err;
		}

		if ((ret = i2c_master_send(ts_data->client, data, len)) == len) {
			ret = 0;
			break;
		}
	}

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		input_err(true, &ts_data->client->dev, "%s: I2C read over retry limit %d\n",
				__func__, ret);
	}

err:
	mutex_unlock(&ts_data->i2c_mutex);

	return ret;
}

void sec_ts_delay(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 900, ms * 1100);
	else
		msleep(ms);
}

static int sec_ts_wait_for_ready(struct sec_ts_data *data)
{
	u8 buff[3];
	int retry = DEVICE_ID_RETRY_COUNT;
	int ret;

retry:
	do {
		if (retry != DEVICE_ID_RETRY_COUNT)
			sec_ts_delay(20);

		ret = sec_ts_i2c_read_reg(data, SEC_TS_READ_DEVICE_ID, buff,
					sizeof(buff));
	} while (--retry && (ret < 0));

	if (ret < 0) {
		input_err(true, &data->client->dev, "%s: failed to read device ID\n",
				__func__);
		return ret;
	} else {
		/* check Device ID */
		if (retry && (buff[1] != 0x6F || buff[2] != 0x70)) {
			/* need to power reset*/
			goto retry;
		}

		input_info(true, &data->client->dev, "%s: %02X, %02X, %02X[%d]\n",
				__func__, buff[0], buff[1], buff[2],
				(DEVICE_ID_RETRY_COUNT - retry));
	}

	return 0;
}

static void sec_ts_release_all_finger(struct sec_ts_data *data)
{
	int i;

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		input_mt_slot(data->input, i);
		input_mt_report_slot_state(data->input, MT_TOOL_FINGER, false);

		if ((data->coord[i].action == SEC_TS_COORD_ACT_PRESS) ||
		    (data->coord[i].action == SEC_TS_COORD_ACT_MOVE)) {

			data->coord[i].action = SEC_TS_COORD_ACT_RELEASE;

			input_info(true, &data->client->dev,
					"%s: [RA] tID:%d mc:%d tc:%d[SE%02X%02X%02X]\n",
					__func__, i, data->coord[i].mcount,
					data->touch_count,
					data->pdata->panel_revision,
					data->pdata->ic_fw_ver[2],
					data->pdata->ic_fw_ver[3]);
		}

		data->coord[i].mcount = 0;
	}

	input_mt_slot(data->input, 0);

	input_report_key(data->input, BTN_TOUCH, false);
	input_report_key(data->input, BTN_TOOL_FINGER, false);

	data->touch_count = 0;

	input_sync(data->input);
}

int sec_ts_stop_device(struct sec_ts_data *data)
{
	input_info(true, &data->client->dev, "%s\n", __func__);

	mutex_lock(&data->device_mutex);

	if (data->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &data->client->dev, "%s: already power off\n",
				__func__);
		goto out;
	}

	disable_irq(data->irq);
	sec_ts_release_all_finger(data);

	data->power_status = SEC_TS_STATE_POWER_OFF;

out:
	mutex_unlock(&data->device_mutex);

	return 0;
}

int sec_ts_start_device(struct sec_ts_data *data)
{
	input_info(true, &data->client->dev, "%s\n", __func__);

	mutex_lock(&data->device_mutex);

	if (data->power_status == SEC_TS_STATE_POWER_ON) {
		input_err(true, &data->client->dev, "%s: already power on\n",
				__func__);
		goto out;
	}

	data->power_status = SEC_TS_STATE_POWER_ON;

	sec_ts_release_all_finger(data);
	enable_irq(data->irq);

	sec_ts_wait_for_ready(data);

out:
	mutex_unlock(&data->device_mutex);

	return 0;
}

#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev)
{
	struct sec_ts_data *data = input_get_drvdata(dev);
	int ret;

	input_info(true, &data->client->dev, "%s\n", __func__);

	ret = sec_ts_start_device(data);
	if (ret < 0) {
		input_err(true, &data->client->dev, "%s: Failed to start device\n",
				__func__);
	}

	return 0;
}

static void sec_ts_input_close(struct input_dev *dev)
{
	struct sec_ts_data *data = input_get_drvdata(dev);

	input_info(true, &data->client->dev, "%s\n", __func__);

	sec_ts_stop_device(data);
}

#ifdef CONFIG_FB
static int sec_ts_fb_notifier_cb(struct notifier_block *nb, unsigned long event,
					void *data)
{
	struct fb_event *evdata = data;
	struct sec_ts_data *ts_data = container_of(nb, struct sec_ts_data, fb_notifier);
	int *blank = evdata->data;

	if (evdata && evdata->data && ts_data) {
		if (event == FB_EARLY_EVENT_BLANK &&
		    *blank == FB_BLANK_POWERDOWN) {
			sec_ts_input_close(ts_data->input);
			//ts_data->fb_ready = false;
		} else if (event == FB_EVENT_BLANK &&
			   *blank == FB_BLANK_UNBLANK) {
				sec_ts_input_open(ts_data->input);
				//ts_data->fb_ready = true;
		}
	}

	return 0;
}
#endif
#endif

static void coord_event_handler(struct sec_ts_data *data, const u8 *buff)
{
	struct sec_ts_event_coordinate *p_event_coord;
	struct sec_ts_coordinate coord;
	int t_id;

	p_event_coord = (struct sec_ts_event_coordinate *)buff;

	memset(&coord, 0x00, sizeof(struct sec_ts_coordinate));

	t_id = (p_event_coord->tid - 1);

	if (t_id >= MAX_SUPPORT_TOUCH_COUNT) {
		input_err(true, &data->client->dev, "%s: tid(%d) is  out of range\n",
				__func__, t_id);
		return;
	}

	coord.id = t_id;
	coord.action = p_event_coord->state & 0x7;
	coord.x = (p_event_coord->x_11_4 << 4) | (p_event_coord->x_3_0);
	coord.y = (p_event_coord->y_11_4 << 4) | (p_event_coord->y_3_0);
	coord.z = p_event_coord->z;
	coord.type = p_event_coord->type & 0x7;
	coord.major = p_event_coord->major;
	coord.minor = p_event_coord->minor;
	coord.mcount = data->coord[t_id].mcount;

	if (coord.z <= 0)
		coord.z = 1;

	if (coord.type == SEC_TS_TOUCHTYPE_FINGER) {
		if (coord.action == SEC_TS_COORD_ACT_RELEASE) {
			if (data->touch_count > 0)
				data->touch_count--;

			input_mt_slot(data->input, t_id);
			input_mt_report_slot_state(data->input, MT_TOOL_FINGER, 0);

			if (data->pdata->support_mt_pressure)
				input_report_abs(data->input, ABS_MT_PRESSURE, 0);

			if (!data->touch_count) {
				input_report_key(data->input, BTN_TOUCH, 0);
				input_report_key(data->input, BTN_TOOL_FINGER, 0);
			}
		} else if (coord.action == SEC_TS_COORD_ACT_PRESS) {
			data->touch_count++;

			input_mt_slot(data->input, t_id);
			input_mt_report_slot_state(data->input, MT_TOOL_FINGER, 1);
			input_report_key(data->input, BTN_TOUCH, 1);
			input_report_key(data->input, BTN_TOOL_FINGER, 1);

			input_report_abs(data->input, ABS_MT_POSITION_X, coord.x);
			input_report_abs(data->input, ABS_MT_POSITION_Y, coord.y);

			if (data->pdata->support_mt_pressure)
				input_report_abs(data->input, ABS_MT_PRESSURE, coord.z);

			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, coord.major);
			input_report_abs(data->input, ABS_MT_TOUCH_MINOR, coord.minor);
		} else if (coord.action == SEC_TS_COORD_ACT_MOVE) {
			input_mt_slot(data->input, t_id);
			input_mt_report_slot_state(data->input, MT_TOOL_FINGER, 1);
			input_report_key(data->input, BTN_TOUCH, 1);
			input_report_key(data->input, BTN_TOOL_FINGER, 1);

			input_report_abs(data->input, ABS_MT_POSITION_X, coord.x);
			input_report_abs(data->input, ABS_MT_POSITION_Y, coord.y);

			if (data->pdata->support_mt_pressure)
				input_report_abs(data->input, ABS_MT_PRESSURE, coord.z);

			input_report_abs(data->input, ABS_MT_TOUCH_MAJOR, coord.major);
			input_report_abs(data->input, ABS_MT_TOUCH_MINOR, coord.minor);

			coord.mcount++;
		}

		memcpy(&data->coord[t_id], &coord, sizeof(struct sec_ts_coordinate));
	}

	input_sync(data->input);

	if (coord.action == SEC_TS_COORD_ACT_PRESS) {
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
		input_info(true, &data->client->dev, "%s: [P] tID:%d, x:%d, y:%d, major:%d, minor:%d, tc:%d\n",
				__func__, t_id, coord.x, coord.y,
				coord.major, coord.minor,
				data->touch_count);
#else
		input_info(true, &data->client->dev, "%s: [P] tID:%d, tc:%d\n",
				__func__, t_id, data->touch_count);
#endif
	} else if (coord.action == SEC_TS_COORD_ACT_RELEASE) {
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
		input_info(true, &data->client->dev, "%s: [R] tID:%d mc:%d tc:%d lx:%d ly:%d[SE%02X%02X%02X]\n",
				__func__, t_id, data->coord[t_id].mcount,
				data->touch_count, data->coord[t_id].x,
				data->coord[t_id].y, data->pdata->panel_revision,
				data->pdata->ic_fw_ver[2],
				data->pdata->ic_fw_ver[3]);
#else
		input_info(true, &data->client->dev, "%s: [R] tID:%d mc:%d tc:%d[SE%02X%02X%02X]\n",
				__func__, t_id, data->coord[t_id].mcount,
				data->touch_count, data->pdata->panel_revision,
				data->pdata->ic_fw_ver[2],
				data->pdata->ic_fw_ver[3]);
#endif
		data->coord[t_id].mcount = 0;
	}
}

static void sec_ts_read_event(struct sec_ts_data *data)
{
	u8 buff[SEC_TS_EVENT_BUFF_SIZE];
	int event_id;
	int ret;

	ret = sec_ts_i2c_read_reg(data, SEC_TS_READ_ONE_EVENT, buff,
				sizeof(buff));
	if (ret < 0) {
		input_err(true, &data->client->dev, "%s: failed to read one event\n",
				__func__);
		return;
	}

	event_id = buff[0] >> 6;
	switch (event_id) {
	case SEC_TS_STATUS_EVENT:
		input_info(true, &data->client->dev, "%s: status event %x.%x.%x.%x.%x.%x.%x.%x\n",
				__func__, buff[0], buff[1], buff[2], buff[3],
				buff[4], buff[5], buff[6], buff[7]);

		break;

	case SEC_TS_COORDINATE_EVENT:
		coord_event_handler(data, buff);
		break;

	default:
		input_err(true, &data->client->dev, "%s: unknown event %x.%x.%x.%x.%x.%x.%x.%x\n",
				__func__, buff[0], buff[1], buff[2], buff[3],
				buff[4], buff[5], buff[6], buff[7]);

		break;
	}
}

static irqreturn_t sec_ts_irq_thread(int irq, void *dev_id)
{
	struct sec_ts_data *data = dev_id;

	sec_ts_read_event(data);

	return IRQ_HANDLED;
}

static void sec_ts_set_input_values(struct sec_ts_data *data,
					struct input_dev *input)
{
	struct sec_ts_plat_data *pdata = data->pdata;

	input_set_drvdata(input, data);

	input->name = "sec_touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &data->client->dev;

	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_capability(input, EV_KEY, BTN_TOOL_FINGER);

	input_mt_init_slots(input, MAX_SUPPORT_TOUCH_COUNT, INPUT_MT_DIRECT);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, pdata->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, pdata->max_y, 0, 0);

	if (data->pdata->support_mt_pressure)
		input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);

#ifdef USE_OPEN_CLOSE
	input->open = sec_ts_input_open;
	input->close = sec_ts_input_close;
#endif
}

static struct sec_ts_plat_data *sec_ts_parse_dt(struct device *dev)
{
	struct sec_ts_plat_data *pdata;
	struct device_node *np = dev->of_node;
	u32 coords[2];
	int ret = 0;

	if (!np)
		return ERR_PTR(-ENODEV);

	if (!lcdtype) {
		input_err(true, dev, "lcd is disconnected\n");
		return ERR_PTR(-ENODEV);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->gpio = of_get_named_gpio(np, "sec,irq_gpio", 0);
	if (gpio_is_valid(pdata->gpio)) {
		ret = devm_gpio_request_one(dev, pdata->gpio, GPIOF_DIR_IN, "sec,tsp_int");
		if (ret) {
			input_err(true, dev, "failed to request tsp int[%d]\n",
					pdata->gpio);
			return ERR_PTR(ret);
		}
	} else {
		input_err(true, dev, "failed to get irq gpio\n");
		return ERR_PTR(pdata->gpio);
	}

	ret = of_property_read_u32(np, "sec,irq_type", &pdata->irq_type);
	if (ret) {
		input_err(true, dev, "failed to get irq type\n");
		return ERR_PTR(ret);
	}

	ret = of_property_read_u32_array(np, "sec,max_coords", coords, 2);
	if (ret) {
		input_err(true, dev, "failed to get max coords\n");
		return ERR_PTR(ret);
	}
	pdata->max_x = coords[0];
	pdata->max_y = coords[1];

	of_property_read_string(np, "sec,firmware_name", &pdata->firmware_name);

#ifdef CONFIG_SEC_FACTORY
	pdata->support_mt_pressure = true;
#endif

	input_info(true, dev, "irq: %d, irq_type: 0x%04x, max[x,y]: [%d,%d], lcdtype: 0x%x\n",
			pdata->gpio, pdata->irq_type, pdata->max_x, pdata->max_y,
			lcdtype);

	return pdata;
}

static int sec_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sec_ts_data *data;
	struct sec_ts_plat_data *pdata = dev_get_platdata(&client->dev);
	struct input_dev *input;
	bool force_update = false;
	u8 status;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		input_err(true, &client->dev, "I2C Functionality not supported\n");
		return -EIO;
	}

	if (!pdata) {
		pdata = sec_ts_parse_dt(&client->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	input = devm_input_allocate_device(&client->dev);
	if (!data || !input)
		return -ENOMEM;

	i2c_set_clientdata(client, data);

	data->client = client;
	data->pdata = pdata;
	data->input = input;

	data->irq = gpio_to_irq(pdata->gpio);
	if (data->irq < 0) {
		input_err(true, &client->dev, "failed to obtain irq number\n");
		return data->irq;
	}

	mutex_init(&data->device_mutex);
	mutex_init(&data->i2c_mutex);

	/* in case TDDI of, tsp does not control power */
	data->power_status = SEC_TS_STATE_POWER_ON;

	ret = sec_ts_wait_for_ready(data);
	if (ret)
		goto err;

	ret = sec_ts_i2c_read_reg(data, SEC_TS_READ_BOOT_STATUS, &status, 1);
	if (ret < 0) {
		input_err(true, &client->dev, "failed to read IC status");
	} else {
		input_info(true, &client->dev, "device status 0x%x", status);
		if (status == SEC_TS_STATUS_BOOT_MODE)
			force_update = true;
	}

	ret = sec_ts_firmware_update(data, force_update);
	if (ret)
		goto err;

	sec_ts_set_input_values(data, input);

	ret = input_register_device(input);
	if (ret) {
		input_err(true, &client->dev, "failed to register %s input device\n",
				input->name);
		goto err;
	}

	ret = devm_request_threaded_irq(&client->dev, data->irq, NULL,
					sec_ts_irq_thread, pdata->irq_type,
					SEC_TS_I2C_NAME, data);

	if (ret) {
		input_err(true, &client->dev, "failed to request irq\n");
		goto err;
	}

#ifdef CONFIG_FB
	//data->fb_ready = true;
	data->fb_notifier.notifier_call = sec_ts_fb_notifier_cb;
	ret = fb_register_client(&data->fb_notifier);
	if (ret) {
		input_err(true, &client->dev, "failed to register fb notifier client\n");
		goto err;
	}
#endif

	ret = sec_ts_fn_init(data);
	if (ret < 0)
		goto err;

	return 0;

err:
	mutex_destroy(&data->device_mutex);
	mutex_destroy(&data->i2c_mutex);

	return ret;
}

static int sec_ts_remove(struct i2c_client *client)
{
	struct sec_ts_data *data = i2c_get_clientdata(client);

	input_info(true, &client->dev, "%s\n", __func__);

	sec_ts_stop_device(data);

	mutex_destroy(&data->device_mutex);
	mutex_destroy(&data->i2c_mutex);

	return 0;
}

static void sec_ts_shutdown(struct i2c_client *client)
{
	struct sec_ts_data *data = i2c_get_clientdata(client);

	input_info(true, &client->dev, "%s\n", __func__);

	sec_ts_stop_device(data);
}

#ifdef CONFIG_PM
static int sec_ts_suspend(struct device *dev)
{
#ifndef USE_OPEN_CLOSE
	struct sec_ts_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->input->mutex);
	if (data->input->users)
		sec_ts_stop_device(data);
	mutex_unlock(&data->input->mutex);
#endif
	return 0;
}

static int sec_ts_resume(struct device *dev)
{
#ifndef USE_OPEN_CLOSE
	struct sec_ts_data *data = dev_get_drvdata(dev);

	mutex_lock(&input->mutex);
	if (input->users)
		sec_ts_start_device(data);
	mutex_unlock(&input->mutex);
#endif
	return 0;
}

static SIMPLE_DEV_PM_OPS(sec_ts_pm_ops, sec_ts_suspend, sec_ts_resume);
#endif

static const struct i2c_device_id sec_ts_i2c_ids[] = {
	{ SEC_TS_I2C_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sec_ts_i2c_ids);

static struct of_device_id sec_ts_dt_ids[] = {
	{ .compatible = "sec,sec_ts",},
	{ },
};
MODULE_DEVICE_TABLE(of, sec_ts_dt_ids);

static struct i2c_driver sec_ts_driver = {
	.probe		= sec_ts_probe,
	.remove		= sec_ts_remove,
	.shutdown	= sec_ts_shutdown,
	.id_table	= sec_ts_i2c_ids,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SEC_TS_I2C_NAME,
		.of_match_table	= of_match_ptr(sec_ts_dt_ids),
#ifdef CONFIG_PM
		.pm	= &sec_ts_pm_ops,
#endif
	},
};

static int __init sec_ts_init(void)
{
	int ret;

#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge) {
		pr_info("%s: %s: Do not load driver due to : lpm %d\n", SECLOG,
			__func__, lpcharge);
		return -ENODEV;
	}
#endif
	ret = i2c_add_driver(&sec_ts_driver);
	if (ret)
		pr_err("%s: %s: failed to add i2c driver\n", SECLOG, __func__);

	return ret;
}

static void __exit sec_ts_exit(void)
{
	i2c_del_driver(&sec_ts_driver);
}

late_initcall_sync(sec_ts_init);
module_exit(sec_ts_exit);

MODULE_DESCRIPTION("Samsung Electronics TouchScreen driver");
MODULE_LICENSE("GPL");
