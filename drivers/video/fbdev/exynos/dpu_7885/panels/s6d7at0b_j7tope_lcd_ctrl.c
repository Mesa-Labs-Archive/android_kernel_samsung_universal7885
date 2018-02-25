/* linux/drivers/video/fbdev/exynos/decon_7885/panels/s6d7at0b_j7tope_lcd_ctrl.c
 *
 * Samsung SoC MIPI LCD CONTROL functions
 *
 * Copyright (c) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/of_device.h>
#include <video/mipi_display.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include "../dsim.h"
#include "../decon.h"
#include "dsim_panel.h"

#include "s6d7at0b_j7tope_param.h"
//#include "backlight_tuning.h"
#include "../decon_board.h"
//#include "../decon_helper.h"

#if defined(CONFIG_EXYNOS_DECON_MDNIE_LITE)
#include "mdnie.h"
#include "mdnie_lite_table_j7tope.h"
#endif

#define PANEL_STATE_SUSPENED	0
#define PANEL_STATE_RESUMED	1
#define PANEL_STATE_SUSPENDING	2

#define S6D7AT0B_ID_REG			0xDA	/* LCD ID1,ID2,ID3 */
#define S6D7AT0B_ID_LEN			3
#define BRIGHTNESS_REG			0x51

#if defined(CONFIG_SEC_INCELL)
#include <linux/sec_incell.h>
#endif

#define DSI_WRITE(cmd, size)		do {				\
	ret |= dsim_write_hl_data(lcd, cmd, size);			\
	if (ret < 0) {							\
		dev_err(&lcd->ld->dev, "%s: failed to write %s\n", __func__, #cmd);	\
		ret = -EPERM;						\
		goto exit;						\
	}								\
} while (0)

struct lcd_info {
	unsigned int			bl;
	unsigned int			brightness;
	unsigned int			current_bl;
	unsigned int			state;

	struct lcd_device		*ld;
	struct backlight_device		*bd;

	union {
		u32			value;
		unsigned char		id[S6D7AT0B_ID_LEN];
	} id_info;
	unsigned char			dump_info[3];
	unsigned int 			data_type;

	int						lux;
	struct class			*mdnie_class;

	struct dsim_device		*dsim;
	struct mutex			lock;

	struct kobject			*dsi_access;
	struct kobj_attribute		dsi_access_r;
	struct kobj_attribute		dsi_access_w;

	struct notifier_block		fb_notif_panel;
	struct i2c_client		*backlight_client;
};


static int dsim_write_hl_data(struct lcd_info *lcd, const u8 *cmd, u32 cmdSize)
{
	int ret = 0;
	int retry = 5;
	struct panel_private *priv = &lcd->dsim->priv;

	if (!priv->lcdConnected)
		return ret;

try_write:
	if (cmdSize == 1)
		ret = dsim_write_data(lcd->dsim, MIPI_DSI_DCS_SHORT_WRITE, cmd[0], 0);
	else if (cmdSize == 2)
		ret = dsim_write_data(lcd->dsim, MIPI_DSI_DCS_SHORT_WRITE_PARAM, cmd[0], cmd[1]);
	else
		ret = dsim_write_data(lcd->dsim, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)cmd, cmdSize);

	if (ret != 0) {
		if (--retry)
			goto try_write;
		else
			dev_err(&lcd->ld->dev, "%s: fail. cmd: %x, ret: %d\n", __func__, cmd[0], ret);
	}

	return ret;
}

static int dsim_read_hl_data(struct lcd_info *lcd, u8 addr, u32 size, u8 *buf)
{
	int ret = 0;
	int retry = 2;
	struct panel_private *priv = &lcd->dsim->priv;

	if (!priv->lcdConnected)
		return size;

try_read:
	ret = dsim_read_data(lcd->dsim, MIPI_DSI_DCS_READ, (u32)addr, size, buf);
	dev_info(&lcd->ld->dev, "%s: addr: %x, ret: %d\n", __func__, addr, ret);
	if (ret != size) {
		if (--retry)
			goto try_read;
		else
			dev_err(&lcd->ld->dev, "%s: fail. addr: %x\n", __func__, addr);
	}

	return ret;
}

#if 0

static int dsim_read_hl_data_offset(struct lcd_info *lcd, u8 addr, u32 size, u8 *buf, u32 offset)
{
	unsigned char wbuf[] = {0xB0, 0};
	int ret = 0;

	wbuf[1] = offset;

	DSI_WRITE(wbuf, ARRAY_SIZE(wbuf));

	ret = dsim_read_hl_data(lcd, addr, size, buf);

	if (ret < 1)
		dev_err(&lcd->ld->dev, "%s: fail\n", __func__);

exit:
	return ret;
}
#endif

static int LM3632_array_write(struct lcd_info *lcd, const struct i2c_rom_data *eprom_ptr, int eprom_size)
{
	int i = 0;
	int ret = 0;
	char temp = 0;

	if (!lcd->backlight_client)
		return 0;

	for (i = 0; i < eprom_size; i++) {
		ret = i2c_smbus_write_byte_data(lcd->backlight_client, eprom_ptr[i].addr, eprom_ptr[i].val);
		if (ret < 0)
			dev_err(&lcd->ld->dev, "%s: fail. %d, %2x, %2x\n", __func__, ret, eprom_ptr[i].addr, eprom_ptr[i].val);
	}

	for (i = 0; i < eprom_size; i++) {
		temp = i2c_smbus_read_byte_data(lcd->backlight_client, eprom_ptr[i].addr);
		if (temp < 0)
			dsim_err("%s: error : BL DEVICE_CTRL read fail\n", __func__);
		else
			dsim_info("%s: addr:[%02X] write:[%02X] read:[%02X]\n", __func__,
			eprom_ptr[i].addr, eprom_ptr[i].val, temp);
	}

	return ret;
}

static int dsim_panel_set_brightness(struct lcd_info *lcd, int force)
{
	int ret = 0;
	unsigned char bl_reg[2];

	mutex_lock(&lcd->lock);

	lcd->brightness = lcd->bd->props.brightness;

	lcd->bl = lcd->brightness;
	lcd->bl = (lcd->bl > UI_MAX_BRIGHTNESS) ? UI_MAX_BRIGHTNESS : lcd->bl;

	if (!force && lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active state\n", __func__);
		goto exit;
	}
	bl_reg[0] = BRIGHTNESS_REG;
	bl_reg[1] = brightness_table[lcd->brightness];

	dev_info(&lcd->ld->dev, "%s: platform BL : %d panel BL reg : %d\n", __func__, lcd->bd->props.brightness, bl_reg[1]);

	DSI_WRITE(SEQ_TEST_KEY_ON_9F, ARRAY_SIZE(SEQ_TEST_KEY_ON_9F));

	if (dsim_write_hl_data(lcd, bl_reg, ARRAY_SIZE(bl_reg)) < 0)
		dev_err(&lcd->ld->dev, "%s: failed to write brightness cmd.\n", __func__);

	DSI_WRITE(SEQ_TEST_KEY_OFF_9F, ARRAY_SIZE(SEQ_TEST_KEY_OFF_9F));

exit:
	mutex_unlock(&lcd->lock);

	return ret;
}

static int panel_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int panel_set_brightness(struct backlight_device *bd)
{
	int ret = 0;
	struct lcd_info *lcd = bl_get_data(bd);

	if (lcd->state == PANEL_STATE_RESUMED) {
		ret = dsim_panel_set_brightness(lcd, 0);
		if (ret) {
			dev_err(&lcd->ld->dev, "%s: failed to set brightness\n", __func__);
			goto exit;
		}
	}

exit:
	return ret;
}

static const struct backlight_ops panel_backlight_ops = {
	.get_brightness = panel_get_brightness,
	.update_status = panel_set_brightness,
};


static int s6d7at0b_read_init_info(struct lcd_info *lcd)
{
	struct panel_private *priv = &lcd->dsim->priv;
	int i = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	if (lcdtype == 0) {
		priv->lcdConnected = 0;
		goto read_exit;
	}

	lcd->id_info.id[0] = (lcdtype & 0xFF0000) >> 16;
	lcd->id_info.id[1] = (lcdtype & 0x00FF00) >> 8;
	lcd->id_info.id[2] = (lcdtype & 0x0000FF) >> 0;

	dev_info(&lcd->ld->dev, "READ ID : ");
	for (i = 0; i < S6D7AT0B_ID_LEN; i++)
		dev_info(&lcd->ld->dev, "%02x, ", lcd->id_info.id[i]);
	dev_info(&lcd->ld->dev, "\n");

read_exit:
	return 0;
}

static int s6d7at0b_read_id(struct lcd_info *lcd)
{
	int ret = 0;
	u8 buf[3] = {0, };
	struct panel_private *priv = &lcd->dsim->priv;

	priv->lcdConnected = 1;

	ret = dsim_read_hl_data(lcd, S6D7AT0B_ID_REG, S6D7AT0B_ID_LEN, (u8 *)&buf[0]);
	ret = dsim_read_hl_data(lcd, S6D7AT0B_ID_REG + 1, S6D7AT0B_ID_LEN, (u8 *)&buf[1]);
	ret = dsim_read_hl_data(lcd, S6D7AT0B_ID_REG + 2, S6D7AT0B_ID_LEN, (u8 *)&buf[2]);

	if (ret <= 0)
		priv->lcdConnected = 0;

	lcd->id_info.id[0] = buf[0];
	lcd->id_info.id[1] = buf[1];
	lcd->id_info.id[2] = buf[2];

	dev_info(&lcd->ld->dev, "%s: %d, 0x%02x 0x%02x 0x%02x\n", __func__, ret, lcd->id_info.id[0], lcd->id_info.id[1], lcd->id_info.id[2]);

	return ret;
}

static int s6d7at0b_displayon(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	return ret;
}

static int s6d7at0b_displayon_late(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	DSI_WRITE(SEQ_TEST_KEY_ON_9F, ARRAY_SIZE(SEQ_TEST_KEY_ON_9F));

	DSI_WRITE(SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));

	DSI_WRITE(SEQ_TEST_KEY_OFF_9F, ARRAY_SIZE(SEQ_TEST_KEY_OFF_9F));

	dsim_panel_set_brightness(lcd, 1);

exit:
	return ret;
}

static int s6d7at0b_exit(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	DSI_WRITE(SEQ_TEST_KEY_ON_9F, ARRAY_SIZE(SEQ_TEST_KEY_ON_9F));

	DSI_WRITE(SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));
	msleep(20);

	DSI_WRITE(SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));
	DSI_WRITE(SEQ_TEST_KEY_OFF_9F, ARRAY_SIZE(SEQ_TEST_KEY_OFF_9F));

	msleep(100);

exit:
	return ret;
}

static int s6d7at0b_init(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s: ++\n", __func__);

	s6d7at0b_read_id(lcd);

	DSI_WRITE(SEQ_TEST_KEY_ON_9F, ARRAY_SIZE(SEQ_TEST_KEY_ON_9F));
	DSI_WRITE(SEQ_TEST_KEY_ON_F0, ARRAY_SIZE(SEQ_TEST_KEY_ON_F0));
	DSI_WRITE(SEQ_TEST_KEY_ON_FC, ARRAY_SIZE(SEQ_TEST_KEY_ON_FC));

	DSI_WRITE(SEQ_S6D7AT0B_36, ARRAY_SIZE(SEQ_S6D7AT0B_36));
	DSI_WRITE(SEQ_S6D7AT0B_71, ARRAY_SIZE(SEQ_S6D7AT0B_71));
	DSI_WRITE(SEQ_S6D7AT0B_73, ARRAY_SIZE(SEQ_S6D7AT0B_73));
	DSI_WRITE(SEQ_S6D7AT0B_B1, ARRAY_SIZE(SEQ_S6D7AT0B_B1));
	DSI_WRITE(SEQ_S6D7AT0B_B3, ARRAY_SIZE(SEQ_S6D7AT0B_B3));
	DSI_WRITE(SEQ_S6D7AT0B_B6, ARRAY_SIZE(SEQ_S6D7AT0B_B6));
	DSI_WRITE(SEQ_S6D7AT0B_BB, ARRAY_SIZE(SEQ_S6D7AT0B_BB));
	DSI_WRITE(SEQ_S6D7AT0B_BA, ARRAY_SIZE(SEQ_S6D7AT0B_BA));
	DSI_WRITE(SEQ_S6D7AT0B_C8, ARRAY_SIZE(SEQ_S6D7AT0B_C8));
	DSI_WRITE(SEQ_S6D7AT0B_EC, ARRAY_SIZE(SEQ_S6D7AT0B_EC));
	DSI_WRITE(SEQ_S6D7AT0B_ED, ARRAY_SIZE(SEQ_S6D7AT0B_ED));
	DSI_WRITE(SEQ_S6D7AT0B_EE, ARRAY_SIZE(SEQ_S6D7AT0B_EE));
	DSI_WRITE(SEQ_S6D7AT0B_EF, ARRAY_SIZE(SEQ_S6D7AT0B_EF));
	DSI_WRITE(SEQ_S6D7AT0B_F2, ARRAY_SIZE(SEQ_S6D7AT0B_F2));
	DSI_WRITE(SEQ_S6D7AT0B_B0, ARRAY_SIZE(SEQ_S6D7AT0B_B0));
	DSI_WRITE(SEQ_S6D7AT0B_F4, ARRAY_SIZE(SEQ_S6D7AT0B_F4));
	DSI_WRITE(SEQ_S6D7AT0B_F9, ARRAY_SIZE(SEQ_S6D7AT0B_F9));
	DSI_WRITE(SEQ_S6D7AT0B_FE, ARRAY_SIZE(SEQ_S6D7AT0B_FE));
	DSI_WRITE(SEQ_S6D7AT0B_E8, ARRAY_SIZE(SEQ_S6D7AT0B_E8));
	DSI_WRITE(SEQ_S6D7AT0B_E9, ARRAY_SIZE(SEQ_S6D7AT0B_E9));
	DSI_WRITE(SEQ_S6D7AT0B_EA, ARRAY_SIZE(SEQ_S6D7AT0B_EA));
	DSI_WRITE(SEQ_S6D7AT0B_EB, ARRAY_SIZE(SEQ_S6D7AT0B_EB));

	DSI_WRITE(SEQ_S6D7AT0B_51, ARRAY_SIZE(SEQ_S6D7AT0B_51));
	DSI_WRITE(SEQ_S6D7AT0B_53, ARRAY_SIZE(SEQ_S6D7AT0B_53));
	DSI_WRITE(SEQ_S6D7AT0B_55, ARRAY_SIZE(SEQ_S6D7AT0B_55));
	DSI_WRITE(SEQ_S6D7AT0B_C0, ARRAY_SIZE(SEQ_S6D7AT0B_C0));

	DSI_WRITE(SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));

	msleep(120);

	dev_info(&lcd->ld->dev, "%s: --\n", __func__);

exit:
	return ret;
}

static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct lcd_info *lcd = NULL;
	int fb_blank;

	switch (event) {
	case FB_EVENT_BLANK:
		break;
	default:
		return 0;
	}

	lcd = container_of(self, struct lcd_info, fb_notif_panel);

	fb_blank = *(int *)evdata->data;

	dev_info(&lcd->ld->dev, "%s: %d\n", __func__, fb_blank);

	if (evdata->info->node != 0)
		return 0;

	if (fb_blank == FB_BLANK_UNBLANK)
		s6d7at0b_displayon_late(lcd);

	return 0;
}

#if defined(CONFIG_SEC_INCELL)
void incell_blank_unblank(void *drv_data)
{
	struct fb_info *info = registered_fb[0];
	struct decon_device *decon = get_decon_drvdata(0);
	struct decon_psr_info psr;

	decon_to_psr_info(decon, &psr);

	dsim_info("+%s\n", __func__);

	if (!lock_fb_info(info))
	    return;

	if (decon->state == DECON_STATE_OFF) {
		dsim_info("decon status is inactive\n");
		goto exit;
	}

	info->flags |= FBINFO_MISC_USEREVENT;
	decon->esd_recovery = 1;
	decon->ignore_vsync = 1;
	decon_reg_wait_for_update_timeout(DECON_INT, 300 * 1000);
	decon_reg_stop(DECON_INT, decon->pdata->dsi_mode, &psr);
	fb_blank(info, FB_BLANK_POWERDOWN);
	fb_blank(info, FB_BLANK_UNBLANK);
	decon->esd_recovery = 0;
	decon->ignore_vsync = 0;
	info->flags &= ~FBINFO_MISC_USEREVENT;
exit:
	unlock_fb_info(info);

	dsim_info("-%s\n", __func__);
}
#endif

static int lm3632_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct lcd_info *lcd = NULL;
	int ret = 0;

	if (id && id->driver_data)
		lcd = (struct lcd_info *)id->driver_data;

	if (!lcd) {
		dsim_err("%s: failed to find driver_data for lcd\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&lcd->ld->dev, "%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	i2c_set_clientdata(client, lcd);

	lcd->backlight_client = client;

	dev_info(&lcd->ld->dev, "%s: %s %s\n", __func__, dev_name(&client->adapter->dev), of_node_full_name(client->dev.of_node));

exit:
	return ret;
}

static struct i2c_device_id lm3632_id[] = {
	{"lm3632", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lm3632_id);

static struct of_device_id lm3632_i2c_dt_ids[] = {
	{ .compatible = "i2c,lm3632" },
	{ }
};

MODULE_DEVICE_TABLE(of, lm3632_i2c_dt_ids);

static struct i2c_driver lm3632_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "lm3632",
		.of_match_table	= of_match_ptr(lm3632_i2c_dt_ids),
	},
	.id_table = lm3632_id,
	.probe = lm3632_probe,
};

static int s6d7at0b_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct panel_private *priv = &dsim->priv;
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "%s: was called\n", __func__);

	priv->lcdConnected = 1;

	lcd->bd->props.max_brightness = EXTEND_BRIGHTNESS;
	lcd->bd->props.brightness = UI_DEFAULT_BRIGHTNESS;

	lcd->dsim = dsim;
	lcd->state = PANEL_STATE_RESUMED;
	lcd->lux = -1;

	s6d7at0b_read_init_info(lcd);
	if (!priv->lcdConnected) {
		dev_err(&lcd->ld->dev, "%s: lcd was not connected\n", __func__);
		goto exit;
	}

	memset(&lcd->fb_notif_panel, 0, sizeof(lcd->fb_notif_panel));
	lcd->fb_notif_panel.notifier_call = fb_notifier_callback;
	fb_register_client(&lcd->fb_notif_panel);

	lcd->backlight_client = NULL;

#if defined(CONFIG_SEC_INCELL)
	incell_data.blank_unblank = incell_blank_unblank;
#endif

	lm3632_id->driver_data = (kernel_ulong_t)lcd;
	i2c_add_driver(&lm3632_i2c_driver);

	dev_info(&lcd->ld->dev, "%s: done\n", __func__);
exit:
	return ret;
}


static ssize_t lcd_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	sprintf(buf, "BOE_%02X%02X%02X\n", lcd->id_info.id[0], lcd->id_info.id[1], lcd->id_info.id[2]);

	return strlen(buf);
}

static ssize_t window_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	sprintf(buf, "%x %x %x\n", lcd->id_info.id[0], lcd->id_info.id[1], lcd->id_info.id[2]);

	return strlen(buf);
}

static ssize_t lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	sprintf(buf, "%d\n", lcd->lux);

	return strlen(buf);
}

static ssize_t lux_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &value);

	if (rc < 0)
		return rc;

	if (lcd->lux != value) {
		mutex_lock(&lcd->lock);
		lcd->lux = value;
		mutex_unlock(&lcd->lock);

#if defined(CONFIG_EXYNOS_DECON_MDNIE_LITE)
		attr_store_for_each(lcd->mdnie_class, attr->attr.name, buf, size);
#endif
	}

	return size;
}

static DEVICE_ATTR(lcd_type, 0444, lcd_type_show, NULL);
static DEVICE_ATTR(window_type, 0444, window_type_show, NULL);
static DEVICE_ATTR(lux, 0644, lux_show, lux_store);

static struct attribute *lcd_sysfs_attributes[] = {
	&dev_attr_lcd_type.attr,
	&dev_attr_window_type.attr,
	&dev_attr_lux.attr,
	NULL,
};

static const struct attribute_group lcd_sysfs_attr_group = {
	.attrs = lcd_sysfs_attributes,
};

static ssize_t read_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct lcd_info *lcd = container_of(attr, struct lcd_info, dsi_access_r);
	char *pos = buf;
	u8 reg, len, param;
	int i;
	u8 *dump = NULL;
	unsigned int data_type;

	reg = lcd->dump_info[0];
	len = lcd->dump_info[1];
	param = lcd->dump_info[2];
	data_type = lcd->data_type;

	if (!reg || !len || reg > 0xff || len > 255 || param > 0xff)
		goto exit;

	dump = kcalloc(len, sizeof(u8), GFP_KERNEL);

	if (lcd->state == PANEL_STATE_RESUMED) {
		dsim_read_data(lcd->dsim, data_type, reg, len, dump);
	}

	for (i = 0; i < len; i++)
		pos += sprintf(pos, "%02x ", dump[i]);
	pos += sprintf(pos, "\n");

	dev_info(&lcd->ld->dev, "+ [%02X]\n", reg);
	for (i = 0; i < len; i++)
		dev_info(&lcd->ld->dev, "%2d: %02x\n", i + 1, dump[i]);
	dev_info(&lcd->ld->dev, "- [%02X]\n", reg);

	kfree(dump);
exit:
	return pos - buf;
}

static ssize_t read_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct lcd_info *lcd = container_of(attr, struct lcd_info, dsi_access_r);
	unsigned int reg, len, param;
	unsigned int data_type, return_packet_type;
	int ret;

	ret = sscanf(buf, "%8x %8x %8x %8x %8x", &data_type, &reg, &param, &return_packet_type, &len);

	if (ret != 5)
		return -EINVAL;

	dev_info(&lcd->ld->dev, "%s: %x %x %x %x %x", __func__, data_type, reg, param, return_packet_type, len);

	if (!reg || !len || reg > 0xff || len > 255 || param > 255)
		return -EINVAL;

	lcd->data_type = data_type;
	lcd->dump_info[0] = reg;
	lcd->dump_info[1] = len;
	lcd->dump_info[2] = param;

	return size;
}

static ssize_t write_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct lcd_info *lcd = container_of(attr, struct lcd_info, dsi_access_w);
	int ret, i, val, len = 0;
	unsigned char seqbuf[255] = {0,};
	unsigned char *printbuf = NULL;
	char *pos, *token;

	pos = (char *)buf;
	while ((token = strsep(&pos, " ")) != NULL) {
		ret = kstrtouint(token, 16, &val);
		if (!ret) {
			seqbuf[len] = val;
			len++;
		}
		if (len == ARRAY_SIZE(seqbuf))
			break;
	}

	pos = printbuf = kcalloc(size, sizeof(u8), GFP_KERNEL);
	for (i = 0; i < len; i++)
		pos += sprintf(pos, "%02x ", seqbuf[i]);
	pos += sprintf(pos, "\n");

	len--;
	if (len < 1) {
		dev_info(&lcd->ld->dev, "%s: invalid input, %s\n", __func__, printbuf);
		goto exit;
	} else
		dev_info(&lcd->ld->dev, "%s: %d, %s\n", __func__, len, printbuf);

	if (lcd->state == PANEL_STATE_RESUMED) {
		if ((seqbuf[0] == 0x29) || (seqbuf[0] == 0x39))
			ret = dsim_write_data(lcd->dsim, (unsigned int)seqbuf[0], (unsigned long)&seqbuf[1], len);
		else if (len == 1)
			ret = dsim_write_data(lcd->dsim, (unsigned int)seqbuf[0], seqbuf[1], len);
		else if (len == 2)
			ret = dsim_write_data(lcd->dsim, (unsigned int)seqbuf[0], seqbuf[1], seqbuf[2]);
		else
			ret = dsim_write_data(lcd->dsim, (unsigned int)seqbuf[0], (unsigned long)&seqbuf[1], len);
	}

exit:
	kfree(printbuf);

	return size;
}

static void lcd_init_dsi_access(struct lcd_info *lcd)
{
	int ret = 0;

	lcd->dsi_access = kobject_create_and_add("dsi_access", NULL);
	if (!lcd->dsi_access)
		return;

	sysfs_attr_init(&lcd->dsi_access_r.attr);
	lcd->dsi_access_r.attr.name = "read";
	lcd->dsi_access_r.attr.mode = 0644;
	lcd->dsi_access_r.store = read_store;
	lcd->dsi_access_r.show = read_show;
	ret = sysfs_create_file(lcd->dsi_access, &lcd->dsi_access_r.attr);
	if (ret < 0)
		dev_err(&lcd->ld->dev, "failed to add kobj_attribute_r\n");

	sysfs_attr_init(&lcd->dsi_access_w.attr);
	lcd->dsi_access_w.attr.name = "write";
	lcd->dsi_access_w.attr.mode = 0220;
	lcd->dsi_access_w.store = write_store;
	ret = sysfs_create_file(lcd->dsi_access, &lcd->dsi_access_w.attr);
	if (ret < 0)
		dev_err(&lcd->ld->dev, "failed to add kobj_attribute_w\n");
}

static void lcd_init_sysfs(struct lcd_info *lcd)
{
	int ret = 0;

	ret = sysfs_create_group(&lcd->ld->dev.kobj, &lcd_sysfs_attr_group);
	if (ret < 0)
		dev_err(&lcd->ld->dev, "failed to add lcd sysfs\n");

	lcd_init_dsi_access(lcd);
}

#if defined(CONFIG_EXYNOS_DECON_MDNIE_LITE)
static int mdnie_lite_write_set(struct lcd_info *lcd, struct lcd_seq_info *seq, u32 num)
{
	int ret = 0, i;

	for (i = 0; i < num; i++) {
		if (seq[i].cmd) {
			ret = dsim_write_hl_data(lcd, seq[i].cmd, seq[i].len);
			if (ret != 0) {
				dev_info(&lcd->ld->dev, "%s: %dth fail\n", __func__, i);
				return ret;
			}
		}
		if (seq[i].sleep)
			usleep_range(seq[i].sleep * 1000, seq[i].sleep * 1000);
	}
	return ret;
}

static int mdnie_lite_send_seq(struct lcd_info *lcd, struct lcd_seq_info *seq, u32 num)
{
	int ret = 0;

	if (lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active\n", __func__);
		return -EIO;
	}

	mutex_lock(&lcd->lock);
	ret = mdnie_lite_write_set(lcd, seq, num);
	mutex_unlock(&lcd->lock);

	return ret;
}

static int mdnie_lite_read(struct lcd_info *lcd, u8 addr, u8 *buf, u32 size)
{
	int ret = 0;

	return ret;
}
#endif

static int dsim_panel_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd;
	printk("%s: %s: start\n", kbasename(__FILE__), __func__);

	dsim->priv.par = lcd = kzalloc(sizeof(struct lcd_info), GFP_KERNEL);
	if (!lcd) {
		pr_err("%s: failed to allocate for lcd\n", __func__);
		ret = -ENOMEM;
		goto probe_err;
	}

	lcd->ld = lcd_device_register("panel", dsim->dev, lcd, NULL);
	if (IS_ERR(lcd->ld)) {
		pr_err("%s: failed to register lcd device\n", __func__);
		ret = PTR_ERR(lcd->ld);
		goto probe_err;
	}

	lcd->bd = backlight_device_register("panel", dsim->dev, lcd, &panel_backlight_ops, NULL);
	if (IS_ERR(lcd->bd)) {
		pr_err("%s: failed to register backlight device\n", __func__);
		ret = PTR_ERR(lcd->bd);
		goto probe_err;
	}

	mutex_init(&lcd->lock);

	ret = s6d7at0b_probe(dsim);
	if (ret < 0) {
		dev_err(&lcd->ld->dev, "%s: failed to probe panel\n", __func__);
		goto probe_err;
	}

	lcd_init_sysfs(lcd);

#if defined(CONFIG_EXYNOS_DECON_MDNIE_LITE)
	mdnie_register(&lcd->ld->dev, lcd, (mdnie_w)mdnie_lite_send_seq, (mdnie_r)mdnie_lite_read, NULL, &tune_info);
	lcd->mdnie_class = get_mdnie_class();
#endif

	dev_info(&lcd->ld->dev, "%s: %s: done\n", kbasename(__FILE__), __func__);
probe_err:
	return ret;
}

static int dsim_panel_resume_early(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;
	struct panel_private *priv = &lcd->dsim->priv;
	int ret = 0;

	if (lcd == NULL)
		return 0;

	dev_info(&lcd->ld->dev, "+%s\n", __func__);

	/* VSP VSN setting, So, It should be called before power enabling */

	ret = LM3632_array_write(lcd, LM3632_INIT, ARRAY_SIZE(LM3632_INIT));
	if (ret < 0)
		dev_err(&lcd->ld->dev, "%s: error : BL DEVICE_CTRL setting fail\n", __func__);

	dev_info(&lcd->ld->dev, "-%s: %d\n", __func__, priv->lcdConnected);

	return ret;
}

static int dsim_panel_displayon(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;
	struct panel_private *priv = &lcd->dsim->priv;
	int ret = 0;

	dev_info(&lcd->ld->dev, "+%s\n", __func__);

	if (lcd->state == PANEL_STATE_SUSPENED) {
		ret = s6d7at0b_init(lcd);
		if (ret) {
			dev_info(&lcd->ld->dev, "%s: failed to panel init\n", __func__);
			goto displayon_err;
		}
	}

	ret = s6d7at0b_displayon(lcd);
	if (ret) {
		dev_info(&lcd->ld->dev, "%s: failed to panel display on\n", __func__);
		goto displayon_err;
	}

displayon_err:
	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_RESUMED;
	mutex_unlock(&lcd->lock);

	dev_info(&lcd->ld->dev, "-%s: %d\n", __func__, priv->lcdConnected);

	return ret;
}

static int dsim_panel_suspend(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;
	struct panel_private *priv = &lcd->dsim->priv;
	int ret = 0;

	dev_info(&lcd->ld->dev, "+%s\n", __func__);

	if (lcd->state == PANEL_STATE_SUSPENED)
		goto suspend_err;

	lcd->state = PANEL_STATE_SUSPENDING;

	ret = s6d7at0b_exit(lcd);
	if (ret) {
		dev_info(&lcd->ld->dev, "%s: failed to panel exit\n", __func__);
		goto suspend_err;
	}

suspend_err:
	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_SUSPENED;
	mutex_unlock(&lcd->lock);

	dev_info(&lcd->ld->dev, "-%s: %d\n", __func__, priv->lcdConnected);

	return ret;
}

struct dsim_lcd_driver s6d7at0b_mipi_lcd_driver = {
	.name		= "s6d7at0b",
	.probe		= dsim_panel_probe,
	.resume_early	= dsim_panel_resume_early,
	.displayon	= dsim_panel_displayon,
	.suspend	= dsim_panel_suspend,
};
