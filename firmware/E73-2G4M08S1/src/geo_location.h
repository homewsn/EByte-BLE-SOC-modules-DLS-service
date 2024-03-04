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

#ifndef GEO_LOCATION_
#define GEO_LOCATION_

#pragma pack(push,1)

//--------------------------------------------
typedef struct
{
	uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
} geo_date_time_t;

typedef struct
{
	uint8_t          satellites;
	int32_t          latitude;             // 1e-7
	int32_t          longitude;            // 1e-7
	uint32_t         horizontal_accuracy;  // 1e-2
	int32_t          altitude;             // 1e-2
	uint32_t         vertical_accuracy;    // 1e-2
	int32_t          bearing;              // 1e-2
	uint32_t         bearing_accuracy;     // 1e-2
	int32_t          speed;                // 1e-1
	uint32_t         speed_accuracy;       // 1e-1
    geo_date_time_t  utc_time;
} geo_location_t;

#pragma pack(pop)

#endif /* GEO_LOCATION_ */