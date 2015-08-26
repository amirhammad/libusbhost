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

#ifndef MSC_H_
#define MSC_H_

#include <stdint.h>
#include "diskio.h"

void fat_msc_init(void);
void fat_msc_device_init(uint8_t device_id);
void fat_msc_poll(uint8_t device_id);

DRESULT USB_disk_read(BYTE *buffer, DWORD sector, UINT count);
DRESULT USB_disk_write(const BYTE *buffer, DWORD sector, UINT count);
DSTATUS USB_disk_status(void);
DSTATUS USB_disk_initialize(void);


#endif
