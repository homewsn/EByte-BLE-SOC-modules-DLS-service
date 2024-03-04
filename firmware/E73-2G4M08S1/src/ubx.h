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

#ifndef UBX_H_
#define UBX_H_

#define UBX_SYNC_CHAR1            0xB5
#define UBX_SYNC_CHAR2            0x62
#define UBX_CLASS_NAV             0x01
#define UBX_ID_NAV_PVT            0x07
#define UBX_NAV_PVT_LENGTH        92
#define UBX_HEADER_LENGTH         6
#define UBX_CHECKSUM_LENGTH       2
#define UBX_NAV_PVT_MSG_LENGTH    (UBX_HEADER_LENGTH + UBX_NAV_PVT_LENGTH + UBX_CHECKSUM_LENGTH)
#define UBX_BUF_LENGTH            (UBX_NAV_PVT_MSG_LENGTH * 2)

int ubx_message_check(const uint8_t *buf, const size_t size);
int ubx_nav_pvt_parse(const uint8_t *buf, const size_t size, geo_location_t *loc);

#endif // UBX_H_
