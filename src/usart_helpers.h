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

#ifndef USBH_USART_HELPERS_H
#define USBH_USART_HELPERS_H

#include "usbh_hubbed.h"
#include <stdint.h>
#include <stdarg.h>

BEGIN_DECLS

struct usart_commands{
	const char * cmd;
	void (*callback)(const char * arg);
};


void usart_init(uint32_t usart, uint32_t baudrate);
void usart_printf(const char *str, ...);
void usart_vprintf(const char *str, va_list va);
void usart_fifo_send(void);

void usart_call_cmd(struct usart_commands * commands);
void usart_interrupt(void);

#ifdef USART_DEBUG
#define LOG_PRINTF(format, ...) usart_printf(format, ##__VA_ARGS__);
#define LOG_PRINTF_S(format, ...) do { usart_printf(format, ##__VA_ARGS__);usart_fifo_send();} while (0);
#define LOG_FLUSH() usart_fifo_send()
#else
#define LOG_PRINTF(dummy, ...) ((void)dummy)
#define LOG_FLUSH()
#endif
#define ERROR(arg) LOG_PRINTF("UNHANDLED_ERROR %d: file: %s, line: %d",\
							arg, __FILE__, __LINE__)
#define ERROR_S(arg) LOG_PRINTF("UNHANDLED ERROR_S '%s': file: '%s', line: %d",\
							arg, __FILE__, __LINE__)

END_DECLS

#endif
