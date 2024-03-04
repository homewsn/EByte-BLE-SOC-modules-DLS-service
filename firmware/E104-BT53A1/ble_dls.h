/*
* Copyright (c) 2020, 2021, 2023 Vladimir Alemasov
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

#ifndef BLE_DLS_
#define BLE_DLS_

//--------------------------------------------
// RemoteSettings characteristic
#pragma pack(push,1)
typedef struct
{
	uint32_t  frequency;
	uint8_t   bandwidth;
	uint8_t   spreading_factor;
	uint8_t   coding_rate;
	uint8_t   interval;
	int16_t   rssi_channel_busy;
	int32_t   magnetic_declination;
} ble_dls_remote_settings_t;
#pragma pack(pop)

//--------------------------------------------
// RemoteAesKey characteristic
typedef struct
{
	uint8_t   aes_key[16];
} ble_dls_remote_aes_key_t;

// Default aes key
static const uint8_t DEFAULT_AES_KEY[16] = { 0 };

//--------------------------------------------
// DeviceLocation and RemoteLocation characteristics
typedef geo_location_t ble_dls_location_t;

//--------------------------------------------
// LoraModuleName and LoraCoreName characteristic
typedef struct
{
	uint8_t   name[30];
} ble_dls_lora_name_t;

//--------------------------------------------
// RssiNoise and RssiLastPacket characteristic
typedef struct
{
	int16_t   rssi;
} ble_dls_rssi_t;


#endif /* BLE_DLS_ */
