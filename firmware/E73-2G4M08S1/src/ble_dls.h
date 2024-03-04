/*
* Copyright (c) 2020, 2023 Vladimir Alemasov
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
#define BLE_UUID_SERVICE_DOUBLE_LOCATION_BASE    {0x13, 0xD6, 0x12, 0x50, 0xFC, 0x2D, 0x03, 0x87, 0x5E, 0x46, 0xCA, 0x94, 0xB9, 0x91, 0x82, 0x7B}

#define BLE_UUID_SERVICE_DOUBLE_LOCATION         0x1400
#define BLE_UUID_CHAR_DEVICE_LOCATION            0x1401
#define BLE_UUID_CHAR_REMOTE_LOCATION            0x1402
#define BLE_UUID_CHAR_REMOTE_SETTINGS            0x1403
#define BLE_UUID_CHAR_REMOTE_AES_KEY             0x1404
#define BLE_UUID_CHAR_LORA_MODULE_NAME           0x1405
#define BLE_UUID_CHAR_LORA_CORE_NAME             0x1406
#define BLE_UUID_CHAR_RSSI_NOISE                 0x1407
#define BLE_UUID_CHAR_RSSI_LAST_PACKET           0x1408

//--------------------------------------------
// Macro for defining a ble_dls instance.
#define BLE_DLS_DEF(_name)                       \
static ble_dls_t _name;                          \
NRF_SDH_BLE_OBSERVER(_name ## _obs,              \
                     BLE_HRS_BLE_OBSERVER_PRIO,  \
                     ble_dls_on_ble_evt, &_name)

#pragma pack(push,1)
//--------------------------------------------
// RemoteSettings characteristic
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


//--------------------------------------------
// Service init structure. This contains all options and data needed for initialization of the service.
typedef struct
{
	security_req_t char_device_location_security_req_cccd_write_perm;
	security_req_t char_remote_location_security_req_cccd_write_perm;
	security_req_t char_remote_settings_security_req_read_perm;
	security_req_t char_remote_settings_security_req_write_perm;
	security_req_t char_remote_aes_key_security_req_read_perm;
	security_req_t char_remote_aes_key_security_req_write_perm;
	security_req_t char_lora_module_name_security_req_read_perm;
	security_req_t char_lora_module_core_security_req_read_perm;
	security_req_t char_rssi_noise_security_req_cccd_write_perm;
	security_req_t char_rssi_last_packet_security_req_cccd_write_perm;
} ble_dls_init_t;

//--------------------------------------------
// Double Location Service event handler type.
typedef struct ble_dls ble_dls_t;
typedef enum
{
	BLE_DLS_DEVICE_LOCATION_NOTIFICATION_ENABLED,
	BLE_DLS_DEVICE_LOCATION_NOTIFICATION_DISABLED,
	BLE_DLS_REMOTE_LOCATION_NOTIFICATION_ENABLED,
	BLE_DLS_REMOTE_LOCATION_NOTIFICATION_DISABLED,
	BLE_DLS_REMOTE_SETTINGS_WRITE,
	BLE_DLS_REMOTE_LORA_SETTINGS_CHANGED,
	BLE_DLS_REMOTE_RSSI_SETTINGS_CHANGED,
	BLE_DLS_REMOTE_DISP_SETTINGS_CHANGED,
	BLE_DLS_REMOTE_INTV_SETTINGS_CHANGED,
	BLE_DLS_REMOTE_AES_KEY_WRITE,
	BLE_DLS_RSSI_NOISE_NOTIFICATION_ENABLED,
	BLE_DLS_RSSI_NOISE_NOTIFICATION_DISABLED,
	BLE_DLS_RSSI_LAST_PACKET_NOTIFICATION_ENABLED,
	BLE_DLS_RSSI_LAST_PACKET_NOTIFICATION_DISABLED
} ble_dls_evt_type_t;
typedef struct
{
	ble_dls_evt_type_t evt_type;
} ble_dls_evt_t;
typedef void (*ble_dls_evt_handler_t) (ble_dls_t const *p_dls, ble_dls_evt_t const *p_evt);

//--------------------------------------------
// Double Location Service structure. This contains various status information for the service.
struct ble_dls
{
	uint16_t                      service_handle;                 /**< Handle of Double Location Service (as provided by the BLE stack). */
	ble_gatts_char_handles_t      char_device_location_handles;   /**< Handles related to the DeviceLocation characteristic. */
	ble_gatts_char_handles_t      char_remote_location_handles;   /**< Handles related to the RemoteLocation characteristic. */
	ble_gatts_char_handles_t      char_remote_settings_handles;   /**< Handles related to the RemoteSettings characteristic. */
	ble_gatts_char_handles_t      char_remote_aes_key_handles;    /**< Handles related to the RemoteAesKey characteristic. */
	ble_gatts_char_handles_t      char_lora_module_name_handles;  /**< Handles related to the LoraModuleName characteristic. */
	ble_gatts_char_handles_t      char_lora_core_name_handles;    /**< Handles related to the LoraCoreName characteristic. */
	ble_gatts_char_handles_t      char_rssi_noise_handles;        /**< Handles related to the RssiNoise characteristic. */
	ble_gatts_char_handles_t      char_rssi_last_packet_handles;  /**< Handles related to the RssiLastPacket characteristic. */
	uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
	uint8_t                       uuid_type; 
	ble_dls_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Double Location Service. */
	ble_dls_location_t            *char_device_location;
	ble_dls_location_t            *char_remote_location;
	ble_dls_remote_settings_t     *char_remote_settings;
	ble_dls_remote_aes_key_t      *char_remote_aes_key;
	ble_dls_lora_name_t           *char_lora_module_name;
	ble_dls_lora_name_t           *char_lora_core_name;
	ble_dls_rssi_t                *char_rssi_noise;
	ble_dls_rssi_t                *char_rssi_last_packet;
	bool                          is_device_location_notification_enabled;
	bool                          is_remote_location_notification_enabled;
	bool                          is_rssi_noise_notification_enabled;
	bool                          is_rssi_last_packet_notification_enabled;
};

//--------------------------------------------
// Function for initializing the Double Location Service.
ret_code_t ble_dls_init(ble_dls_t *dls, const ble_dls_init_t *dls_init);

//--------------------------------------------
// Function for handling Double Location Service BLE stack events.
void ble_dls_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

//--------------------------------------------
// Functions for update characteristic.
ret_code_t ble_dls_device_location_update(ble_dls_t *p_dls, ble_dls_location_t *device_location_data);
ret_code_t ble_dls_remote_location_update(ble_dls_t *p_dls, ble_dls_location_t *remote_location_data);
//ret_code_t ble_dls_remote_settings_update(ble_dls_t *p_dls, ble_dls_remote_settings_t *remote_settings_data);
//ret_code_t ble_dls_remote_aes_key_update(ble_dls_t *p_dls, ble_dls_remote_aes_key_t *remote_aes_key_data);

ret_code_t ble_dls_rssi_noise_update(ble_dls_t *p_dls, ble_dls_rssi_t *rssi_noise_data);
ret_code_t ble_dls_rssi_last_packet_update(ble_dls_t *p_dls, ble_dls_rssi_t *rssi_last_packet_data);

ret_code_t ble_dls_lora_module_name_update(ble_dls_t *p_dls);
ret_code_t ble_dls_lora_core_name_update(ble_dls_t *p_dls);

#endif /* BLE_DLS_ */