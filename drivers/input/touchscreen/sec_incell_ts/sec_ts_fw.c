/* drivers/input/touchscreen/sec_ts_fw.c
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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#include "sec_ts.h"


#undef CONFIG_CMD_PP
//#define CONFIG_CMD_PP
#undef CONFIG_USE_CE
//#define CONFIG_USE_CE
/*
static int sec_ts_enter_fw_update_mode(struct sec_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 cmd_param[] = {0x55, 0xAC};
	u8 status;
	int ret;

	ret = sec_ts_i2c_write_reg(data, SEC_TS_CMD_ENTER_FW_MODE, cmd_param,
					sizeof(cmd_param));
	if (ret < 0) {
		input_err(true, &client->dev, "%s: failed to write cmd of enter fw update\n",
				__func__);
		return ret;
	}
	sec_ts_delay(1000);

	ret = sec_ts_i2c_read_reg(data, SEC_TS_READ_BOOT_STATUS, &status, 1);
	if (ret < 0) {
		input_err(true, &client->dev, "%s: failed to read IC status",
				__func__);
		return ret;
	}

	if (status != SEC_TS_STATUS_BOOT_MODE) {
		input_err(true, &client->dev, "%s: failed to enter fw update mode\n",
				__func__);
		return -EPERM;
	}

	return 0;
}
*/
int sec_ts_sw_reset(struct sec_ts_data *data)
{
	int ret;

	ret = sec_ts_i2c_write_reg(data, SEC_TS_CMD_SW_RESET, NULL, 0);
	if (ret < 0) {
		input_err(true, &data->client->dev, "%s: write fail, sw_reset\n", __func__);
		return 0;
	}

	sec_ts_delay(500);
	input_info(true, &data->client->dev, "%s: sw_reset\n", __func__);

	return 1;
}

static int sec_ts_check_firmware_version(struct sec_ts_data *data, const u8 *fw_info)
{
	struct sec_ts_plat_data *pdata = data->pdata;
	fw_header *fw_hd;
	u8 buff[13] = { 0 };
	u8 ic_fw_ver[4] = { 0 };
	u8 bin_fw_ver[4] = { 0 };
	int ret;

	fw_hd = (fw_header *)fw_info;

	bin_fw_ver[0] = (fw_hd->version & 0xFF);
	bin_fw_ver[1] = (fw_hd->version >> 8 & 0xFF);
	bin_fw_ver[2] = (fw_hd->version >> 16 & 0xFF);
	bin_fw_ver[3] = (fw_hd->version >> 24 & 0xFF);

	/* fw version in BIN */
	pdata->bin_fw_ver[0] = bin_fw_ver[0];
	pdata->bin_fw_ver[1] = bin_fw_ver[1];
	pdata->bin_fw_ver[2] = bin_fw_ver[2];
	pdata->bin_fw_ver[3] = bin_fw_ver[3];

	if (fw_hd->signature != SEC_TS_FW_HEADER_SIGN) {
		input_err(true, &data->client->dev, "%s firmware header error = %08X\n",
				__func__, fw_hd->signature);
		return -EPERM;
	}

	ret = sec_ts_i2c_read_reg(data, SEC_TS_READ_SUB_ID, buff, 13);
	if (ret < 0) {
		input_info(true, &data->client->dev, "%s: firmware version read error\n ",
				__func__);
		goto err;
	}

	ic_fw_ver[0] = buff[9];
	ic_fw_ver[1] = buff[10];
	ic_fw_ver[2] = buff[11];
	ic_fw_ver[3] = buff[12];

	/* fw version in IC */
	pdata->ic_fw_ver[0] = ic_fw_ver[0];
	pdata->ic_fw_ver[1] = ic_fw_ver[1];
	pdata->ic_fw_ver[2] = ic_fw_ver[2];
	pdata->ic_fw_ver[3] = ic_fw_ver[3];

	input_info(true, &data->client->dev, "%s: [IC] version: %x.%x.%x.%x, [BIN] version %x.%x.%x.%x\n",
			__func__, ic_fw_ver[0], ic_fw_ver[1], ic_fw_ver[2],
			ic_fw_ver[3], bin_fw_ver[0], bin_fw_ver[1], bin_fw_ver[2],
			bin_fw_ver[3]);

	if (bin_fw_ver[0] != ic_fw_ver[0] || bin_fw_ver[1] != ic_fw_ver[1]) {
		input_err(true, &data->client->dev, "%s: f/w product ID is not equal: %x.%x\n",
				__func__, ic_fw_ver[0], ic_fw_ver[1]);
		return -EPERM;
	}

err:
	if (bin_fw_ver[3] > ic_fw_ver[3] || bin_fw_ver[2] > ic_fw_ver[2]) {
		input_info(true, &data->client->dev, "%s: need to update\n ",
				__func__);
		ret = 0;
	} else {
		input_info(true, &data->client->dev, "%s: do not need to update\n ",
				__func__);
		ret = -EPERM;
	}

	return ret;
}

#ifndef CONFIG_CMD_PP
static u8 sec_ts_checksum(u8 *data, int offset, int size)
{
	int i;
	u8 checksum = 0;

	for (i = 0; i < size; i++)
		checksum += data[i + offset];

	return checksum;
}
#endif

/***********************/
/** Ext-flash control **/
/***********************/
#define SEC_TS_CMD_CS_CONTROL	0x8B

#define SEC_TS_CMD_FLASH_READ_ADDR	0xD0
#define SEC_TS_CMD_SET_DATA_NUM		0xD1
#define SEC_TS_CMD_FLASH_READ_MEM	0xDC
#define SEC_TS_CMD_ECHO			0xE1

#define FLASH_CMD_RDSR		0x05
#define FLASH_CMD_WREN		0x06
#define FLASH_CMD_SE		0x20
#define FLASH_CMD_CE		0x60
#define FLASH_CMD_PP		0x02
#define SEC_TS_CMD_FLASH_SEND_DATA	0xEB
#define SEC_TS_CMD_FLASH_READ_DATA	0xEC

#define CS_LOW			0
#define CS_HIGH			1

#define BYTE_PER_SECTOR		4096
#define BYTE_PER_PAGE		256
#define PAGE_DATA_HEADER_SIZE	4

#define SEC_TS_FLASH_WIP_MASK   0x01
#define SEC_TS_FLASH_SIZE_256   256

#define BYTE_PER_SECTOR	4096
#define BYTE_PER_PAGE	256
#define PAGE_PER_SECTOR	16

static int sec_ts_flash_set_datanum(struct sec_ts_data *ts, u16 num)
{
	u8 tData[2];
	int ret;

	tData[0] = (num >> 8) & 0xFF;
	tData[1] = num & 0xFF;

	ret = sec_ts_i2c_write_reg(ts, SEC_TS_CMD_SET_DATA_NUM, tData, 2);
	if( ret < 0)
		input_err(true, &ts->client->dev, "%s: Set datanum Fail %d\n",
				__func__, num);

	return ret;
}

static int sec_ts_flash_cs_control(struct sec_ts_data *ts, bool cs_level)
{
	u8 tData;
	int ret;

	tData = cs_level ? 1 : 0;

	ret = sec_ts_i2c_write_reg(ts, SEC_TS_CMD_CS_CONTROL, &tData, 1);
	if( ret < 0)
		input_err(true, &ts->client->dev, "%s: %s control Fail!\n",
				__func__, cs_level ? "CS High" : "CS Low");
	return ret;
}

static int  sec_ts_wren(struct sec_ts_data *ts)
{
	u8 tData[2];
	int ret;

	sec_ts_flash_cs_control(ts, CS_LOW);

	sec_ts_flash_set_datanum(ts, 6);

	tData[0] = FLASH_CMD_WREN;
	ret = sec_ts_i2c_write_reg(ts, SEC_TS_CMD_FLASH_SEND_DATA, &tData[0], 1);
	if( ret < 0)
		input_err(true, &ts->client->dev, "%s: Send WREN fail!\n", __func__);

	sec_ts_flash_cs_control(ts, CS_HIGH);

	return ret;
}

static u8 sec_ts_rdsr(struct sec_ts_data *ts)
{
	u8 tData[2];

	sec_ts_flash_cs_control(ts, CS_LOW);

	sec_ts_flash_set_datanum(ts, 2);

	tData[0] = FLASH_CMD_RDSR;
	sec_ts_i2c_write_reg(ts, SEC_TS_CMD_FLASH_SEND_DATA, tData, 1);

	sec_ts_flash_set_datanum(ts, 1);

	sec_ts_i2c_read_reg(ts, SEC_TS_CMD_FLASH_READ_DATA, tData, 1);

	sec_ts_flash_cs_control(ts, CS_HIGH);

	return tData[0];
}

static bool IsFlashBusy(struct sec_ts_data *ts)
{
	u8 tBuf;
	sec_ts_wren(ts);
	tBuf = sec_ts_rdsr(ts);
	if ( (tBuf & SEC_TS_FLASH_WIP_MASK) == SEC_TS_FLASH_WIP_MASK )
		return true;
	else	return false;
}

static int sec_ts_wait_for_flash_busy(struct sec_ts_data *ts)
{
	int retry_cnt = 0;
	int ret = 0 ;

	while( IsFlashBusy(ts) )
	{
		sec_ts_delay(10);

		if(retry_cnt++ > SEC_TS_WAIT_RETRY_CNT )
		{
			input_err(true, &ts->client->dev, "%s: Retry Cnt over!\n", __func__);
			ret = -1;
		}
	}

	return ret;
}
#ifndef CONFIG_USE_CE
static int sec_ts_cmd_flash_se(struct sec_ts_data *ts, u32 flash_addr)
{
	int ret;
	u8 tBuf[5];

	if( IsFlashBusy(ts) )
	{
		input_err(true, &ts->client->dev, "%s: flash busy, flash_addr = %X\n", __func__, flash_addr);
		return false;
	}

	sec_ts_wren(ts);

	sec_ts_flash_cs_control(ts, CS_LOW);

	sec_ts_flash_set_datanum(ts, 5);

	tBuf[0] = SEC_TS_CMD_FLASH_SEND_DATA;
	tBuf[1] = FLASH_CMD_SE;
	tBuf[2] = (flash_addr >> 16) & 0xFF;
	tBuf[3] = (flash_addr >>  8) & 0xFF;
	tBuf[4] = (flash_addr >>  0) & 0xFF;
	ret = sec_ts_i2c_write(ts, tBuf, 5);
	sec_ts_flash_cs_control(ts, CS_HIGH);
	if( ret < 0)
	{
		input_err(true, &ts->client->dev, "%s: Send sector erase cmd fail!\n", __func__);
		return ret;
	}

	ret = sec_ts_wait_for_flash_busy(ts);
	if( ret < 0 )
		input_err(true, &ts->client->dev, "%s: Time out! - flash busy wait\n", __func__);

	return ret;
}
#else
static int sec_ts_cmd_flash_ce(struct sec_ts_data *ts)
{
	u8 data[1];
	if (IsFlashBusy(ts)) {
		input_err(true, &ts->client->dev, "%s: Fail\n",
				__func__);
		return false;
	}

	data[0] = FLASH_CMD_CE;
	sec_ts_wren(ts);
	sec_ts_flash_cs_control(ts, CS_LOW);
	sec_ts_flash_set_datanum(ts, 0x2);
	sec_ts_i2c_write_reg(ts, SEC_TS_CMD_FLASH_SEND_DATA, data, 1);

	sec_ts_flash_cs_control(ts, CS_HIGH);

	sec_ts_wait_for_flash_busy(ts);
	input_info(true, &ts->client->dev, "%s: Chip Erase Success\n", __func__);
	return true;
}
#endif

#ifdef CONFIG_CMD_PP
bool sec_ts_cmd_pp (struct sec_ts_data *ts, int flash_address, u8* source_data, int byte_length)
{
	int data_byte_total_length;
	u8* tCmd;
	int ret, i;

	if (IsFlashBusy(ts)) return false;

	sec_ts_wren(ts);

	data_byte_total_length = 1 + 3 + byte_length + 1;
	tCmd = kzalloc(data_byte_total_length, GFP_KERNEL);

	sec_ts_flash_cs_control(ts, CS_LOW);
	sec_ts_flash_set_datanum(ts, 0x104);

	tCmd[0] = SEC_TS_CMD_FLASH_SEND_DATA;
	tCmd[1] = FLASH_CMD_PP;
	tCmd[2] = (flash_address >> 16) & 0xFF;
	tCmd[3] = (flash_address >> 8) & 0xFF;
	tCmd[4] = (flash_address >> 0) & 0xFF;

	for (i = 0; i < byte_length; i++) tCmd[5 + i] = source_data[i];

	ret = sec_ts_i2c_write(ts, tCmd, data_byte_total_length);
	sec_ts_flash_cs_control(ts, CS_HIGH);

	if( ret < 0)
	{
		input_err(true, &ts->client->dev, "%s: PP cmd fail!\n", __func__);
		return false;
	}
	input_info(true, &ts->client->dev, "%s : addr = %X%X%X\n", __func__, tCmd[2], tCmd[3], tCmd[4]);

	kfree(tCmd);

	while (IsFlashBusy(ts)) { sec_ts_delay(10); };
	return true;
}
#endif

#ifndef CONFIG_USE_CE
static int sec_ts_FlashSectorErase(struct sec_ts_data *ts, u32 page_idx)
{
	u32 addr;
	int ret;

	//addr = sector_idx * BYTE_PER_SECTOR;
	addr = page_idx * BYTE_PER_PAGE;

	ret = sec_ts_cmd_flash_se(ts, addr);
	if( ret < 0)
		input_err(true, &ts->client->dev, "%s: Fail!\n", __func__);
	//input_info(true, &ts->client->dev, "%s : ret = %d, sector_idx = %d\n", __func__, ret, sector_idx);
	sec_ts_delay(10);

	return ret;
}
#endif

static bool sec_ts_flashpagewrite(struct sec_ts_data *ts, u32 page_idx, u8* page_data)
{
#ifndef CONFIG_CMD_PP
	int ret;
	int i,j;
	u8* tCmd;
	u8 copy_data[3 + 256];
	int copy_left = 256 + 3;
	int copy_size = 0;
	//int copy_max = ts->i2c_burstmax - 1;
	int copy_max = 200;

	copy_data[0] = (u8)((page_idx >> 8) & 0xFF);
	copy_data[1] = (u8)((page_idx >> 0) & 0xFF);
	for (i = 0; i < 256; i++) copy_data[2 + i] = page_data[i];
	copy_data[2 + 256] = sec_ts_checksum(copy_data, 0, 2 + 256);

	sec_ts_flash_cs_control(ts, CS_LOW);
	while (copy_left > 0) {
		int copy_cur = (copy_left > copy_max) ? copy_max: copy_left;
		tCmd = (u8 *)kzalloc(copy_cur + 1, GFP_KERNEL);
		if( copy_size == 0 ) tCmd[0] = 0xD9;
		else tCmd[0] = 0xDA;

		for (j=0;j<copy_cur;j++)tCmd[j+1] = copy_data[copy_size + j];
		ret = sec_ts_i2c_write(ts,tCmd,1+copy_cur);
		copy_size += copy_cur;
		copy_left -= copy_cur;
		kfree(tCmd);
	}

	//sec_ts_delay(10);
	sec_ts_flash_cs_control(ts, CS_HIGH);

	return ret;
#else
	int size;
	int addr;
	size = BYTE_PER_PAGE;
	addr = page_idx * BYTE_PER_PAGE;
	input_info(true, &ts->client->dev, "%s: addr = %d\n", __func__, addr);
	sec_ts_cmd_pp(ts, addr, page_data, size);
	return true;
#endif
}

static int sec_ts_flashwrite(struct sec_ts_data *ts, u32 mem_addr, const u8 *mem_data, u32 mem_size)
{
	int ret;
	int page_idx;
	int size_left;
	int size_copy;
	u32 flash_page_size;
	u32 page_idx_start;
	u32 page_idx_end;
	u32 page_num;
	u8 page_buf[SEC_TS_FLASH_SIZE_256];

	if (0 == mem_size)
		return 0;

	flash_page_size = SEC_TS_FLASH_SIZE_256;
	page_idx_start = mem_addr / flash_page_size;
	page_idx_end = (mem_addr + mem_size - 1) / flash_page_size;
	page_num = page_idx_end - page_idx_start + 1;

	input_info(true, &ts->client->dev, "%s: page_idx_start=%X, page_idx_end=%X\n",
		__func__, page_idx_start, page_idx_end);

#ifndef CONFIG_USE_CE
	for (page_idx = (int)((page_num - 1)/16); page_idx >= 0; page_idx--) {
		ret = sec_ts_FlashSectorErase(ts, (u32)(page_idx_start + page_idx * 16));
		if (ret < 0) {
			input_err(true, &ts->client->dev, "%s: Sector erase fail! sector_idx = %08X\n",
					__func__, page_idx_start + page_idx * 16);
			return -EIO;
		}
	}

	input_info(true, &ts->client->dev, "%s flash sector erase done\n", __func__);
#else
	ret = sec_ts_cmd_flash_ce(ts);
#endif
	sec_ts_delay(page_num + 10);

	size_left = (int)mem_size;
	size_copy = (int)(mem_size % flash_page_size);
	if (size_copy == 0)
		size_copy = (int)flash_page_size;

	memset(page_buf, 0, SEC_TS_FLASH_SIZE_256);

	for (page_idx = (int)page_num - 1; page_idx >= 0; page_idx--) {
		memcpy(page_buf, mem_data + (page_idx * flash_page_size), size_copy);

		ret = sec_ts_flashpagewrite(ts, (u32)(page_idx + page_idx_start), page_buf);
		if (ret < 0) {
			input_err(true, &ts->client->dev, "%s fw write failed, page_idx = %d\n", __func__, page_idx);
			goto err;
		}

		size_copy = (int)flash_page_size;
		sec_ts_delay(5);
	}
	input_info(true, &ts->client->dev, "%s flash page write done\n", __func__);

	return mem_size;
err:
	return -EIO;
}

static int sec_ts_memoryblockread(struct sec_ts_data *ts, u32 mem_addr, u32 mem_size, u8* buf)
{
	int ret;
	u8 cmd[5];

	if (mem_size >= 64 * 1024) {
		input_err(true, &ts->client->dev,
					"%s mem size over 64K\n", __func__);
		return -1;
	}

	cmd[0] = (u8)SEC_TS_CMD_FLASH_READ_ADDR;
	cmd[1] = (u8)((mem_addr >> 24) & 0xff);
	cmd[2] = (u8)((mem_addr >> 16) & 0xff);
	cmd[3] = (u8)((mem_addr >> 8) & 0xff);
	cmd[4] = (u8)((mem_addr >> 0) & 0xff);

	ret = sec_ts_i2c_write(ts, cmd, 5);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s send command failed, %02X\n", __func__, cmd[0]);
		return -1;
	}

	udelay(10);
	cmd[0] = (u8)SEC_TS_CMD_SET_DATA_NUM;
	cmd[1] = (u8)((mem_size >> 8) & 0xff);
	cmd[2] = (u8)((mem_size >> 0) & 0xff);

	ret = sec_ts_i2c_write(ts, cmd, 3);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s send command failed, %02X\n", __func__, cmd[0]);
		return -1;
	}

	udelay(10);
	cmd[0] = (u8)SEC_TS_CMD_FLASH_READ_MEM;

	ret = sec_ts_i2c_read_reg(ts, cmd[0], buf, mem_size);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s memory read failed\n", __func__);
		return -1;
	}

	sec_ts_delay(1);
	return 0;
}

static int sec_ts_memoryread(struct sec_ts_data *ts, u32 mem_addr, u8 *mem_data, u32 mem_size)
{
	int ret;
	int retry = 3;
	int read_size = 0;
	int unit_size;
	//int max_size = 1024;
	int max_size = 256;
	int read_left = (int)mem_size;

	while (read_left > 0) {
		unit_size = (read_left > max_size) ? max_size : read_left;
		retry = 3;
		do {
			ret = sec_ts_memoryblockread(ts, mem_addr, (u32)unit_size, mem_data + read_size);
			if (retry-- == 0) {
				input_err(true, &ts->client->dev,
							"%s fw read fail mem_addr=%08X,unit_size=%d\n",
							__func__, mem_addr, unit_size);
				return -1;
			}
		} while (ret < 0);

		mem_addr += unit_size;
		read_size += unit_size;
		read_left -= unit_size;
	}

	return read_size;
}

static int sec_ts_chunk_update(struct sec_ts_data *ts, u32 addr, u32 size, const u8 *data)
{
	u32 fw_size;
	u32 write_size;
	u8* mem_rb;
	int ret = 0;

	fw_size = size;

	write_size = sec_ts_flashwrite(ts, addr, data, fw_size);
	if (write_size != fw_size) {
		input_err(true, &ts->client->dev, "%s fw write failed\n", __func__);
		ret = -1;
		goto err_write_fail;
	}

	input_info(true, &ts->client->dev, "%s flash write done, write size = %X\n", __func__, write_size);

	//mem_rb = kzalloc(fw_size, GFP_KERNEL);
	mem_rb = (u8 *)vzalloc(fw_size);
	if (!mem_rb) {
		input_err(true, &ts->client->dev, "%s kmalloc failed\n", __func__);
		ret = -1;
		goto err_write_fail;
	}

	if (sec_ts_memoryread(ts, addr, mem_rb, fw_size) >= 0) {
		u32 ii;
		for (ii = 0; ii < fw_size; ii++) {
			if (data[ii] != mem_rb[ii])
			{
				input_info(true, &ts->client->dev, "%s data[ii] = %X, mem_rb[ii] = %X\n", __func__,
						data[ii], mem_rb[ii]);
				break;
			}
		}
		if (fw_size != ii) {
			input_err(true, &ts->client->dev, "%s fw verify fail, ii = %d\n", __func__, ii);
			ret = -1;
			goto out;
		}
	} else {
		ret = -1;
		goto out;
	}

	input_info(true, &ts->client->dev, "%s: fw verify done\n", __func__);
out:
	//kfree(mem_rb);
	vfree(mem_rb);

err_write_fail:
	sec_ts_delay(10);
	return ret;
}

static int sec_ts_flash_firmware(struct sec_ts_data *ts_data, const u8 *data, size_t size)
{
	fw_header *fw_hd;
	u8 num_chunk;
	int ret;
	int i;

	fw_hd = (fw_header *)data;

	/* Enter Firmware Update Mode */
/*
	ret = sec_ts_enter_fw_update_mode(ts_data);
	if (ret) {
		goto err;
	}
*/
	num_chunk = fw_hd->chunk_num[0] && 0xFF;
	input_info(true, &ts_data->client->dev, "%s num_chunk : %d\n", __func__,
			num_chunk);
	input_info(true, &ts_data->client->dev, "%s 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
			__func__, fw_hd->signature, fw_hd->flag, size,
			fw_hd->setting);


	for (i = 0; i < num_chunk; i++) {
		ret = sec_ts_chunk_update(ts_data, 0x00, size, data);
		if (ret < 0) {
			input_err(true, &ts_data->client->dev, "%s firmware chunk write failed\n",
					__func__);
			return -1;
		}
	}

//err:
	if (sec_ts_sw_reset(ts_data) < 0) {
		input_err(true, &ts_data->client->dev, "%s: sw reset fail!\n",
				__func__);
	}

	input_info(true, &ts_data->client->dev, "%s: fw update Success!\n",
			__func__);

	return 0;
}

int sec_ts_firmware_update(struct sec_ts_data *data, bool force)
{
	const struct firmware *fw_entry;
	char fw_path[SEC_TS_MAX_FW_PATH];
	int ret;

	if (data->pdata->bringup) {
		input_info(true, &data->client->dev, "bringup, skip firmware update\n");
		return 0;
	}

	if (data->pdata->firmware_name) {
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s",
				data->pdata->firmware_name);
	} else {
		input_err(true, &data->client->dev, "do not exist fw\n");
		return 0;
	}

	input_info(true, &data->client->dev, "initial firmware update %s\n", fw_path);

	/* Loading Firmware */
	ret = request_firmware(&fw_entry, fw_path, &data->client->dev);
	if (ret) {
		input_err(true, &data->client->dev, "firmware is not available\n");
		return 0;
	}

	input_info(true, &data->client->dev, "request firmware done! size = %d\n",
			fw_entry->size);

	disable_irq(data->irq);

	ret = sec_ts_check_firmware_version(data, fw_entry->data);
	if (ret) {
		if (force) {
			input_info(true, &data->client->dev, "%s: need to update by force\n ",
				__func__);
		} else {
			ret = 0;
			goto err;
		}
	}

	if (sec_ts_flash_firmware(data, fw_entry->data, fw_entry->size) < 0)
		ret = -EPERM;
	else
		ret = 0;

err:
	enable_irq(data->irq);
	release_firmware(fw_entry);

	return ret;
}

static int sec_ts_load_fw_from_ums(struct sec_ts_data *data)
{
	fw_header *fw_hd;
	struct file *fp;
	mm_segment_t old_fs;
	long fw_size, nread;
	int error = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(SEC_TS_DEFAULT_UMS_FW, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		input_err(true, &data->client->dev, "%s: failed to open %s.\n",
				__func__, SEC_TS_DEFAULT_UMS_FW);
		error = -ENOENT;
		goto open_err;
	}

	fw_size = fp->f_path.dentry->d_inode->i_size;

	if (0 < fw_size) {
		unsigned char *fw_data;

		fw_data = kzalloc(fw_size, GFP_KERNEL);
		nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);

		input_info(true, &data->client->dev, "%s: start, file path %s, size %ld Bytes\n",
				__func__, SEC_TS_DEFAULT_UMS_FW, fw_size);

		if (nread != fw_size) {
			input_err(true, &data->client->dev, "%s: failed to read firmware file, nread %ld Bytes\n",
					__func__, nread);
			error = -EIO;
		} else {
			fw_hd = (fw_header *)fw_data;

			input_info(true, &data->client->dev, "%s: IMG version %08X\n",
					__func__, fw_hd->version);

			if (data->irq)
				disable_irq(data->irq);
			if (sec_ts_flash_firmware(data, fw_data, fw_size) < 0)
				goto done;
			if (data->irq)
				enable_irq(data->irq);
		}

		if (error < 0) {
			input_err(true, &data->client->dev, "%s: failed update firmware\n",
					__func__);
		}

done:
		kfree(fw_data);
	}

	filp_close(fp, NULL);

open_err:
	set_fs(old_fs);
	return error;
}

int sec_ts_firmware_update_on_hidden_menu(struct sec_ts_data *data, int update_type)
{
	int ret = 0;

	/* Factory cmd for firmware update
	 * argument represent what is source of firmware like below.
	 *
	 * 0 : [BUILT_IN] Getting firmware which is for user.
	 * 1 : [UMS] Getting firmware from sd card.
	 * 2 : none
	 */

	switch (update_type) {
	case BUILT_IN:
		ret = sec_ts_firmware_update(data, true);
		break;
	case UMS:
		ret = sec_ts_load_fw_from_ums(data);
		break;
	default:
		input_err(true, &data->client->dev, "%s: Not support command[%d]\n",
				__func__, update_type);
		break;
	}
	return ret;
}
