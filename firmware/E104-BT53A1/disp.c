/*
* Copyright (c) 2023 Vladimir Alemasov
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
#include "disp.h"

//--------------------------------------------
int disp_message_check(const uint8_t *buf, const size_t size)
{
	size_t len;

	if (!size || buf[0] != DISP_SYNC_CHAR1)
	{
		return -1;
	}
	if (size < DISP_HEADER_LENGTH)
	{
		return 0;
	}
	if (buf[1] != DISP_SYNC_CHAR2)
	{
		return -1;
	}
	len = buf[3];
	if (size < DISP_HEADER_LENGTH + len)
	{
		return 0;
	}
	return (int)(DISP_HEADER_LENGTH + len);
}

//--------------------------------------------
size_t disp_set_info_pkt_cmd(uint8_t *disp_buf, disp_info_t *info)
{
	*(disp_buf++) = DISP_SYNC_CHAR1;
	*(disp_buf++) = DISP_SYNC_CHAR2;
	*(disp_buf++) = DISP_INFO_PKT_CMD;
	*(disp_buf++) = sizeof(disp_info_t);
	*(disp_buf++) = info->declination;
	*(disp_buf++) = info->declination >> 8;
	*(disp_buf++) = info->declination >> 16;
	*(disp_buf++) = info->declination >> 24;
	*(disp_buf++) = info->local_latitude;
	*(disp_buf++) = info->local_latitude >> 8;
	*(disp_buf++) = info->local_latitude >> 16;
	*(disp_buf++) = info->local_latitude >> 24;
	*(disp_buf++) = info->local_longitude;
	*(disp_buf++) = info->local_longitude >> 8;
	*(disp_buf++) = info->local_longitude >> 16;
	*(disp_buf++) = info->local_longitude >> 24;
	*(disp_buf++) = info->remote_latitude;
	*(disp_buf++) = info->remote_latitude >> 8;
	*(disp_buf++) = info->remote_latitude >> 16;
	*(disp_buf++) = info->remote_latitude >> 24;
	*(disp_buf++) = info->remote_longitude;
	*(disp_buf++) = info->remote_longitude >> 8;
	*(disp_buf++) = info->remote_longitude >> 16;
	*(disp_buf++) = info->remote_longitude >> 24;
	return (DISP_HEADER_LENGTH + sizeof(disp_info_t));
}
