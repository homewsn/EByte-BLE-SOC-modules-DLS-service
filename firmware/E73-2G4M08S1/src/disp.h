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

#ifndef DISP_H_
#define DISP_H_

#pragma pack(push,1)

//--------------------------------------------
typedef struct disp_info
{
	int32_t declination;             // 1e-7
	int32_t local_latitude;          // 1e-7
	int32_t local_longitude;         // 1e-7
	int32_t remote_latitude;         // 1e-7
	int32_t remote_longitude;        // 1e-7
} disp_info_t;

#pragma pack(pop)

//--------------------------------------------
#define DISP_SYNC_CHAR1               0x40
#define DISP_SYNC_CHAR2               0x53
#define DISP_INFO_PKT_CMD             0x01
#define DISP_HEADER_LENGTH            4
#define DISP_INFO_PKT_LENGTH         (sizeof(disp_info_t))
#define DISP_MAX_MSG_LENGTH          (DISP_HEADER_LENGTH + DISP_INFO_PKT_LENGTH)
#define DISP_BUF_LENGTH              (DISP_MAX_MSG_LENGTH)

//--------------------------------------------
int disp_message_check(const uint8_t *buf, const size_t size);
size_t disp_set_info_pkt_cmd(uint8_t *disp_buf, disp_info_t *info);

#endif // DISP_H_


