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


#ifndef MINI_ENDIAN_H_
#define MINI_ENDIAN_H_

#include <machine/endian.h>

/* Mini endian  BEGIN */
# if BYTE_ORDER == LITTLE_ENDIAN
#  define htobe16(x) byteswap_16 (x)
#  define htole16(x) (x)
#  define be16toh(x) byteswap_16 (x)
#  define le16toh(x) (x)

#  define htobe32(x) byteswap_32 (x)
#  define htole32(x) (x)
#  define be32toh(x) byteswap_32 (x)
#  define le32toh(x) (x)

#  define htobe64(x) byteswap_64 (x)
#  define htole64(x) (x)
#  define be64toh(x) byteswap_64 (x)
#  define le64toh(x) (x)

# else
#  define htobe16(x) (x)
#  define htole16(x) byteswap_16 (x)
#  define be16toh(x) (x)
#  define le16toh(x) byteswap_16 (x)

#  define htobe32(x) (x)
#  define htole32(x) byteswap_32 (x)
#  define be32toh(x) (x)
#  define le32toh(x) byteswap_32 (x)

#  define htobe64(x) (x)
#  define htole64(x) byteswap_64 (x)
#  define be64toh(x) (x)
#  define le64toh(x) byteswap_64 (x)
# endif
static inline uint16_t byteswap_16(uint16_t val)
{
	union {
		uint16_t num;
		uint8_t	arr[2];
	} in, ret;
	in.num = val;

	ret.arr[0] = in.arr[1];
	ret.arr[1] = in.arr[0];
	return ret.num;
}
static inline uint32_t byteswap_32(uint32_t val)
{
	union {
		uint32_t num;
		uint8_t	arr[4];
	} in, ret;
	in.num = val;

	ret.arr[0] = in.arr[3];
	ret.arr[1] = in.arr[2];
	ret.arr[2] = in.arr[1];
	ret.arr[3] = in.arr[0];
	return ret.num;
}


#endif
