/*
 * This file is part of the libusbhost library
 * hosted at http://github.com/libusbhost/libusbhost
 *
 * Copyright (C) 2015 Amir Hammad <amir.hammad@hotmail.com>
 *
 *
 * libusbhost is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "msc.h"

#include "usbh_config.h"
#include "usbh_driver_msc.h"
#include "usart_helpers.h"
#include "mini_endian.h"
#include "ff.h"
#include "diskio.h"

#include <stdint.h>
#include <string.h>

struct _partition {
	uint8_t Boot_Flag;
	uint8_t CHS_Begin[3];
	uint8_t Type_Code;
	uint8_t CHS_End[3];
	uint32_t LBA_Begin;
	uint32_t Number_Of_Sectors;
};

struct _private_data {
	int state_next;
	bool busy;
	uint8_t buffer[512];
	struct _partition partitions[4];

	struct _volume_id_info {
		uint16_t bytes_per_sector;
		uint8_t sectors_per_cluster;
		uint16_t sectors_reserved;
		uint8_t number_of_fat;
		uint32_t sectors_per_fat;
		uint32_t root_dir;
	} volume_id_info;
};
struct _private_data private_data[USBH_MSC_MAX_DEVICES];

static void parse_mbr(struct _private_data *priv)
{
	uint8_t *data = priv->buffer;

	const struct _partition *part = (void*)&data[512 - 4*16 - 2];
	memcpy(priv->partitions, part, 4*sizeof(struct _partition));

	int i;
	for (i = 0; i < 16; i++) {
		LOG_PRINTF("%02X ", ((char *)part)[i]);
	}
	LOG_PRINTF("\n");
	LOG_PRINTF("LBA Begin: %d, Sectors: %d, Type: 0x%02X\n", le32toh(part->LBA_Begin), le32toh(part->Number_Of_Sectors), part->Type_Code);
}

static void parse_volume_id(struct _private_data *priv)
{
	uint8_t *data = priv->buffer;

	priv->volume_id_info.bytes_per_sector = *((uint32_t *)&data[0x0B]);
	priv->volume_id_info.sectors_per_cluster = data[0x0D];
	priv->volume_id_info.sectors_reserved = *((uint16_t *)&data[0x0E]);
	priv->volume_id_info.number_of_fat = data[0x10];
	priv->volume_id_info.sectors_per_fat = *((uint32_t *)&data[0x24]);
	priv->volume_id_info.root_dir = *((uint32_t *)&data[0x2C]);
}

static void callback(int arg)
{
	struct _private_data *priv = (struct _private_data*)arg;

	switch (priv->state_next) {
	case 1:
		parse_mbr(priv);
		priv->busy = false;
		break;

	case 2:
		parse_volume_id(priv);
		priv->busy = false;
		break;
	}
}

static void run(int device_id)
{
	struct _private_data *priv = &private_data[device_id];
	if (priv->busy) {
		return;
	}
	switch (priv->state_next) {
	case 0:
		if (msc_idle(device_id)) {
			msc_read10(device_id, priv->buffer, 1, 0, callback, (int)priv);
			priv->busy = true;
			priv->state_next++;
		}
		break;

	case 1:
		if (msc_idle(device_id)) {
			uint32_t lba_begin = priv->partitions[0].LBA_Begin;

			msc_read10(device_id, priv->buffer, 1, lba_begin, callback, (int)priv);
			priv->busy = true;
			priv->state_next++;
		}
		break;
	case 2:
		if (msc_idle(device_id)) {

		}
	}
}

void fat_msc_init(void)
{
	size_t i;
	for (i = 0; i < sizeof(private_data)/sizeof(private_data[0]); i++) {
		struct _private_data *priv = &private_data[i];

		priv->busy = false;
		priv->state_next = 0;
	}
}

void fat_msc_device_init(uint8_t device_id)
{
	private_data[device_id].state_next = 0;
}

void fat_msc_poll(uint8_t device_id)
{
	if (msc_idle(device_id)) {
		run(device_id);
	}
}

static bool read_finished = true;

static void callback_ff(int arg)
{
	read_finished = true;
}

extern uint32_t tim6_get_time_us(void);

DRESULT USB_disk_read(BYTE *buffer, DWORD sector, UINT count)
{
	LOG_PRINTF("read\n");
	msc_read10(0, buffer, count, sector, callback_ff, 0);
	read_finished = false;
	while (!read_finished) {
		usbh_poll(tim6_get_time_us());
		LOG_FLUSH();
	}
	return RES_OK;
}

DRESULT USB_disk_write(const BYTE *buffer, DWORD sector, UINT count)
{
	LOG_PRINTF("read\n");
	msc_write10(0, buffer, count, sector, callback_ff, 0);
	read_finished = false;
	while (!read_finished) {
		usbh_poll(tim6_get_time_us());
		LOG_FLUSH();
	}
	return RES_OK;
}

DSTATUS USB_disk_status(void)
{
	if (!msc_present(0)) {
		return STA_NODISK;
	} else if (!msc_initialized()) {
		return STA_NOINIT;
	}
	return 0;
}

static const msc_config_t msc_config = {
	.update = 0,
	.notify_connected = 0,
	.notify_disconnected = 0
};

DSTATUS USB_disk_initialize(void)
{
	return 0;
}

DWORD get_fattime (void)
{
	return 0;
}
