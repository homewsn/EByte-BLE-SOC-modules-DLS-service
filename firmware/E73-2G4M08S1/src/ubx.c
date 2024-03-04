/*
* Copyright (c) 2020 Vladimir Alemasov
* All rights reserved
*
* This program and the accompanying materials are distributed under
* the terms of GNU General Public License version 2
* as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <stdint.h>		/* uint8_t */
#include <stddef.h>     /* size_t */
#include "geo_location.h"
#include "ubx.h"

//------------------------------------------------------------------
static int ubx_checksum(const uint8_t *buf, const size_t size)
{
	uint8_t ck_a;
	uint8_t ck_b;
	size_t cnt;

	for (cnt = 2, ck_a = 0, ck_b = 0; cnt < size - 2; cnt++)
	{
		ck_a += buf[cnt];
		ck_b += ck_a;
	}
	if (ck_a == buf[size - 2] && ck_b == buf[size - 1])
	{
		return 0;
	}
	return -1;
}

//------------------------------------------------------------------
int ubx_nav_pvt_parse(const uint8_t *buf, const size_t size, geo_location_t *loc)
{
	if (ubx_checksum(buf, size) < 0)
	{
		return -1;
	}
	if (size != UBX_NAV_PVT_MSG_LENGTH)
	{
		return -1;
	}

	if (buf[UBX_HEADER_LENGTH + 11] & 0x03)
	{
		loc->utc_time.year = buf[UBX_HEADER_LENGTH + 4] | (buf[UBX_HEADER_LENGTH + 5] << 8);
		loc->utc_time.month = buf[UBX_HEADER_LENGTH + 6];
		loc->utc_time.day = buf[UBX_HEADER_LENGTH + 7];
		loc->utc_time.hours = buf[UBX_HEADER_LENGTH + 8];
		loc->utc_time.minutes = buf[UBX_HEADER_LENGTH + 9];
		loc->utc_time.seconds = buf[UBX_HEADER_LENGTH + 10];
	}
	else
	{
		loc->utc_time.year = 0;
	}

	if (buf[UBX_HEADER_LENGTH + 21] & 0x01)
	{
		loc->satellites = buf[UBX_HEADER_LENGTH + 23];
		loc->longitude = buf[UBX_HEADER_LENGTH + 24] |
			(buf[UBX_HEADER_LENGTH + 25] << 8) |
			(buf[UBX_HEADER_LENGTH + 26] << 16) |
			(buf[UBX_HEADER_LENGTH + 27] << 24); // 1e-7
		loc->latitude = buf[UBX_HEADER_LENGTH + 28] |
			(buf[UBX_HEADER_LENGTH + 29] << 8) |
			(buf[UBX_HEADER_LENGTH + 30] << 16) |
			(buf[UBX_HEADER_LENGTH + 31] << 24); // 1e-7
		loc->horizontal_accuracy = buf[UBX_HEADER_LENGTH + 40] |
			(buf[UBX_HEADER_LENGTH + 41] << 8) |
			(buf[UBX_HEADER_LENGTH + 42] << 16) |
			(buf[UBX_HEADER_LENGTH + 43] << 24);
		loc->horizontal_accuracy /= 10; // 1e-3 => 1e-2
		loc->altitude = buf[UBX_HEADER_LENGTH + 32] |
			(buf[UBX_HEADER_LENGTH + 33] << 8) |
			(buf[UBX_HEADER_LENGTH + 34] << 16) |
			(buf[UBX_HEADER_LENGTH + 35] << 24);
		loc->altitude /= 10; // 1e-3 => 1e-2
		loc->vertical_accuracy = buf[UBX_HEADER_LENGTH + 44] |
			(buf[UBX_HEADER_LENGTH + 45] << 8) |
			(buf[UBX_HEADER_LENGTH + 46] << 16) |
			(buf[UBX_HEADER_LENGTH + 47] << 24);
		loc->vertical_accuracy /= 10; // 1e-3 => 1e-2
		loc->bearing = buf[UBX_HEADER_LENGTH + 64] |
			(buf[UBX_HEADER_LENGTH + 65] << 8) |
			(buf[UBX_HEADER_LENGTH + 66] << 16) |
			(buf[UBX_HEADER_LENGTH + 67] << 24);
		loc->bearing /= 1000; // 1e-5 => 1e-2
		loc->bearing_accuracy = buf[UBX_HEADER_LENGTH + 72] |
			(buf[UBX_HEADER_LENGTH + 73] << 8) |
			(buf[UBX_HEADER_LENGTH + 74] << 16) |
			(buf[UBX_HEADER_LENGTH + 75] << 24);
		loc->bearing_accuracy /= 1000; // 1e-5 => 1e-2
		loc->speed = buf[UBX_HEADER_LENGTH + 60] |
			(buf[UBX_HEADER_LENGTH + 61] << 8) |
			(buf[UBX_HEADER_LENGTH + 62] << 16) |
			(buf[UBX_HEADER_LENGTH + 63] << 24);
		loc->speed /= 100; // 1e-3 => 1e-1
		loc->speed_accuracy = buf[UBX_HEADER_LENGTH + 68] |
			(buf[UBX_HEADER_LENGTH + 69] << 8) |
			(buf[UBX_HEADER_LENGTH + 70] << 16) |
			(buf[UBX_HEADER_LENGTH + 71] << 24);
		loc->speed_accuracy /= 100; // 1e-3 => 1e-1
	}
	else
	{
		loc->satellites = 0;
	}

	return 0;
}

//------------------------------------------------------------------
int ubx_message_check(const uint8_t *buf, const size_t size)
{
	uint16_t len;

	if (size >= 1 && buf[0] != UBX_SYNC_CHAR1)
	{
		return -1;
	}
	if (size >= 2 && buf[1] != UBX_SYNC_CHAR2)
	{
		return -1;
	}
	if (size < UBX_HEADER_LENGTH)
	{
		return 0;
	}
	len = buf[4] | (buf[5] << 8);
	if (size < UBX_HEADER_LENGTH + len + UBX_CHECKSUM_LENGTH)
	{
		return 0;
	}
	return UBX_HEADER_LENGTH + len + UBX_CHECKSUM_LENGTH;
}
