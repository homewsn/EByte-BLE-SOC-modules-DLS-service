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

#include <stdint.h>
#include "sdk_common.h"
#include "ble_srv_common.h"
#include "geo_location.h"
#include "ble_dls.h"

//------------------------------------------------------------------
static ret_code_t char_add_device_location(ble_dls_t *p_dls, const ble_dls_init_t *p_dls_init)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid              = BLE_UUID_CHAR_DEVICE_LOCATION;
	add_char_params.uuid_type         = p_dls->uuid_type;
	add_char_params.max_len           = sizeof(*p_dls->char_device_location);
	add_char_params.init_len          = sizeof(*p_dls->char_device_location);
	add_char_params.p_init_value      = (uint8_t *)p_dls->char_device_location;
	add_char_params.char_props.notify = true;
	add_char_params.cccd_write_access = p_dls_init->char_device_location_security_req_cccd_write_perm;
	add_char_params.is_value_user     = true;

	return characteristic_add(p_dls->service_handle,
	                          &add_char_params,
	                          &p_dls->char_device_location_handles);
}

//------------------------------------------------------------------
static ret_code_t char_add_remote_location(ble_dls_t *p_dls, const ble_dls_init_t *p_dls_init)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid              = BLE_UUID_CHAR_REMOTE_LOCATION;
	add_char_params.uuid_type         = p_dls->uuid_type;
	add_char_params.max_len           = sizeof(*p_dls->char_remote_location);
	add_char_params.init_len          = sizeof(*p_dls->char_remote_location);
	add_char_params.p_init_value      = (uint8_t *)p_dls->char_remote_location;
	add_char_params.char_props.notify = true;
	add_char_params.cccd_write_access = p_dls_init->char_remote_location_security_req_cccd_write_perm;
	add_char_params.is_value_user     = true;

	return characteristic_add(p_dls->service_handle,
	                          &add_char_params,
	                          &p_dls->char_remote_location_handles);
}

//------------------------------------------------------------------
static ret_code_t char_add_remote_settings(ble_dls_t *p_dls, const ble_dls_init_t *p_dls_init)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid              = BLE_UUID_CHAR_REMOTE_SETTINGS;
	add_char_params.uuid_type         = p_dls->uuid_type;
	add_char_params.max_len           = sizeof(*p_dls->char_remote_settings);
	add_char_params.init_len          = sizeof(*p_dls->char_remote_settings);
	add_char_params.p_init_value      = (uint8_t *)p_dls->char_remote_settings;
	add_char_params.char_props.read   = true;
	add_char_params.read_access       = p_dls_init->char_remote_settings_security_req_read_perm;
	add_char_params.char_props.write  = true;
	add_char_params.write_access      = p_dls_init->char_remote_location_security_req_cccd_write_perm;
	add_char_params.is_value_user     = false;  // don't change the characteristics value automatically

	return characteristic_add(p_dls->service_handle,
	                          &add_char_params,
	                          &p_dls->char_remote_settings_handles);
}

//------------------------------------------------------------------
static ret_code_t char_add_remote_aes_key(ble_dls_t *p_dls, const ble_dls_init_t *p_dls_init)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid              = BLE_UUID_CHAR_REMOTE_AES_KEY;
	add_char_params.uuid_type         = p_dls->uuid_type;
	add_char_params.max_len           = sizeof(*p_dls->char_remote_aes_key);
	add_char_params.init_len          = sizeof(*p_dls->char_remote_aes_key);
	add_char_params.p_init_value      = (uint8_t *)p_dls->char_remote_aes_key;
	add_char_params.char_props.read   = true;
	add_char_params.read_access       = p_dls_init->char_remote_aes_key_security_req_read_perm;
	add_char_params.char_props.write  = true;
	add_char_params.write_access      = p_dls_init->char_remote_location_security_req_cccd_write_perm;
	add_char_params.is_value_user     = false;  // don't change the characteristics value automatically

	return characteristic_add(p_dls->service_handle,
	                          &add_char_params,
	                          &p_dls->char_remote_aes_key_handles);
}

//------------------------------------------------------------------
static ret_code_t char_add_lora_module_name(ble_dls_t *p_dls, const ble_dls_init_t *p_dls_init)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid              = BLE_UUID_CHAR_LORA_MODULE_NAME;
	add_char_params.uuid_type         = p_dls->uuid_type;
	add_char_params.max_len           = sizeof(*p_dls->char_lora_module_name);
	add_char_params.init_len          = strlen((const char *)(p_dls->char_lora_module_name)->name);
	add_char_params.is_var_len        = true;
	add_char_params.p_init_value      = (uint8_t *)p_dls->char_lora_module_name;
	add_char_params.char_props.read   = true;
	add_char_params.read_access       = p_dls_init->char_lora_module_name_security_req_read_perm;
	add_char_params.is_value_user     = true;

	return characteristic_add(p_dls->service_handle,
	                          &add_char_params,
	                          &p_dls->char_lora_module_name_handles);
}

//------------------------------------------------------------------
static ret_code_t char_add_lora_core_name(ble_dls_t *p_dls, const ble_dls_init_t *p_dls_init)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid              = BLE_UUID_CHAR_LORA_CORE_NAME;
	add_char_params.uuid_type         = p_dls->uuid_type;
	add_char_params.max_len           = sizeof(*p_dls->char_lora_core_name);
	add_char_params.init_len          = strlen((const char *)(p_dls->char_lora_core_name)->name);
	add_char_params.is_var_len        = true;
	add_char_params.p_init_value      = (uint8_t *)p_dls->char_lora_core_name;
	add_char_params.char_props.read   = true;
	add_char_params.read_access       = p_dls_init->char_lora_module_core_security_req_read_perm;
	add_char_params.is_value_user     = true;

	return characteristic_add(p_dls->service_handle,
	                          &add_char_params,
	                          &p_dls->char_lora_core_name_handles);
}

//------------------------------------------------------------------
static ret_code_t char_add_rssi_noise(ble_dls_t *p_dls, const ble_dls_init_t *p_dls_init)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid              = BLE_UUID_CHAR_RSSI_NOISE;
	add_char_params.uuid_type         = p_dls->uuid_type;
	add_char_params.max_len           = sizeof(*p_dls->char_rssi_noise);
	add_char_params.init_len          = sizeof(*p_dls->char_rssi_noise);
	add_char_params.p_init_value      = (uint8_t *)p_dls->char_rssi_noise;
	add_char_params.char_props.notify = true;
	add_char_params.cccd_write_access = p_dls_init->char_rssi_noise_security_req_cccd_write_perm;
	add_char_params.is_value_user     = true;

	return characteristic_add(p_dls->service_handle,
	                          &add_char_params,
	                          &p_dls->char_rssi_noise_handles);
}

//------------------------------------------------------------------
static ret_code_t char_add_rssi_last_packet(ble_dls_t *p_dls, const ble_dls_init_t *p_dls_init)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid              = BLE_UUID_CHAR_RSSI_LAST_PACKET;
	add_char_params.uuid_type         = p_dls->uuid_type;
	add_char_params.max_len           = sizeof(*p_dls->char_rssi_last_packet);
	add_char_params.init_len          = sizeof(*p_dls->char_rssi_last_packet);
	add_char_params.p_init_value      = (uint8_t *)p_dls->char_rssi_last_packet;
	add_char_params.char_props.notify = true;
	add_char_params.cccd_write_access = p_dls_init->char_rssi_last_packet_security_req_cccd_write_perm;
	add_char_params.is_value_user     = true;

	return characteristic_add(p_dls->service_handle,
	                          &add_char_params,
	                          &p_dls->char_rssi_last_packet_handles);
}

//------------------------------------------------------------------
ret_code_t ble_dls_init(ble_dls_t *p_dls, const ble_dls_init_t *p_dls_init)
{
	VERIFY_PARAM_NOT_NULL(p_dls);
	VERIFY_PARAM_NOT_NULL(p_dls_init);

	ret_code_t err_code;
	ble_uuid_t ble_uuid;

	p_dls->conn_handle = BLE_CONN_HANDLE_INVALID;

	// Add Location Source Service UUID
	ble_uuid128_t base_uuid = {BLE_UUID_SERVICE_DOUBLE_LOCATION_BASE};
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p_dls->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p_dls->uuid_type;
	ble_uuid.uuid = BLE_UUID_SERVICE_DOUBLE_LOCATION;

	// Add the Location Source Service
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_dls->service_handle);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	// Add characteristics
	err_code = char_add_device_location(p_dls, p_dls_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	err_code = char_add_remote_location(p_dls, p_dls_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	err_code = char_add_remote_settings(p_dls, p_dls_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	err_code = char_add_remote_aes_key(p_dls, p_dls_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	err_code = char_add_lora_module_name(p_dls, p_dls_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	err_code = char_add_lora_core_name(p_dls, p_dls_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	err_code = char_add_rssi_noise(p_dls, p_dls_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	err_code = char_add_rssi_last_packet(p_dls, p_dls_init);
	return err_code;
}

//------------------------------------------------------------------
static void on_connect(ble_dls_t *p_dls, ble_evt_t const *p_ble_evt)
{
	p_dls->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

//------------------------------------------------------------------
static void on_disconnect(ble_dls_t *p_dls, ble_evt_t const *p_ble_evt)
{
	p_dls->conn_handle = BLE_CONN_HANDLE_INVALID;
}

//------------------------------------------------------------------
static void on_write(ble_dls_t *p_dls, ble_evt_t const *p_ble_evt)
{
	ble_dls_evt_t evt;
	ble_gatts_evt_write_t *p_evt_write = (ble_gatts_evt_write_t *)&p_ble_evt->evt.gatts_evt.params.write;
	
	if (p_evt_write->handle == p_dls->char_device_location_handles.cccd_handle)
	{
		p_dls->is_device_location_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
		if (p_dls->is_device_location_notification_enabled)
		{
			evt.evt_type = BLE_DLS_DEVICE_LOCATION_NOTIFICATION_ENABLED;
		}
		else
		{
			evt.evt_type = BLE_DLS_DEVICE_LOCATION_NOTIFICATION_DISABLED;
		}
		p_dls->evt_handler(p_dls, &evt);
	}
	else if (p_evt_write->handle == p_dls->char_remote_location_handles.cccd_handle)
	{
		p_dls->is_remote_location_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
		if (p_dls->is_remote_location_notification_enabled)
		{
			evt.evt_type = BLE_DLS_REMOTE_LOCATION_NOTIFICATION_ENABLED;
		}
		else
		{
			evt.evt_type = BLE_DLS_REMOTE_LOCATION_NOTIFICATION_DISABLED;
		}
		p_dls->evt_handler(p_dls, &evt);
	}
	else if (p_evt_write->handle == p_dls->char_remote_settings_handles.value_handle)
	{
		if (memcmp(p_dls->char_remote_settings, p_evt_write->data, p_evt_write->len))
		{
			if (p_dls->char_remote_settings->rssi_channel_busy != ((ble_dls_remote_settings_t *)(p_evt_write->data))->rssi_channel_busy)
			{
				evt.evt_type = BLE_DLS_REMOTE_RSSI_SETTINGS_CHANGED;
			}
			else if (p_dls->char_remote_settings->magnetic_declination != ((ble_dls_remote_settings_t *)(p_evt_write->data))->magnetic_declination)
			{
				evt.evt_type = BLE_DLS_REMOTE_DISP_SETTINGS_CHANGED;
			}
			else if (p_dls->char_remote_settings->interval != ((ble_dls_remote_settings_t *)(p_evt_write->data))->interval)
			{
				evt.evt_type = BLE_DLS_REMOTE_INTV_SETTINGS_CHANGED;
			}
			else
			{
				evt.evt_type = BLE_DLS_REMOTE_LORA_SETTINGS_CHANGED;
			}
			memcpy(p_dls->char_remote_settings, p_evt_write->data, p_evt_write->len);
			p_dls->evt_handler(p_dls, &evt);
			evt.evt_type = BLE_DLS_REMOTE_SETTINGS_WRITE;
			p_dls->evt_handler(p_dls, &evt);
		}
	}
	else if (p_evt_write->handle == p_dls->char_remote_aes_key_handles.value_handle)
	{
		if (memcmp(p_dls->char_remote_aes_key, p_evt_write->data, p_evt_write->len))
		{
			memcpy(p_dls->char_remote_aes_key, p_evt_write->data, p_evt_write->len);
			evt.evt_type = BLE_DLS_REMOTE_AES_KEY_WRITE;
			p_dls->evt_handler(p_dls, &evt);
		}
	}
	else if (p_evt_write->handle == p_dls->char_rssi_noise_handles.cccd_handle)
	{
		p_dls->is_rssi_noise_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
		if (p_dls->is_rssi_noise_notification_enabled)
		{
			evt.evt_type = BLE_DLS_RSSI_NOISE_NOTIFICATION_ENABLED;
		}
		else
		{
			evt.evt_type = BLE_DLS_RSSI_NOISE_NOTIFICATION_DISABLED;
		}
		p_dls->evt_handler(p_dls, &evt);
	}
	else if (p_evt_write->handle == p_dls->char_rssi_last_packet_handles.cccd_handle)
	{
		p_dls->is_rssi_last_packet_notification_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
		if (p_dls->is_rssi_last_packet_notification_enabled)
		{
			evt.evt_type = BLE_DLS_RSSI_LAST_PACKET_NOTIFICATION_ENABLED;
		}
		else
		{
			evt.evt_type = BLE_DLS_RSSI_LAST_PACKET_NOTIFICATION_DISABLED;
		}
		p_dls->evt_handler(p_dls, &evt);
	}
}

//------------------------------------------------------------------
void ble_dls_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
	VERIFY_PARAM_NOT_NULL_VOID(p_context);
	VERIFY_PARAM_NOT_NULL_VOID(p_ble_evt);

	ble_dls_t *p_dls = (ble_dls_t *)p_context;

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		on_connect(p_dls, p_ble_evt);
		break;
	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnect(p_dls, p_ble_evt);
		break;
	case BLE_GATTS_EVT_WRITE:
		on_write(p_dls, p_ble_evt);
		break;
	default:
		break;
	}
}

//--------------------------------------------
ret_code_t ble_dls_device_location_update(ble_dls_t *p_dls, ble_dls_location_t *device_location_data)
{
	VERIFY_PARAM_NOT_NULL(p_dls);
	VERIFY_PARAM_NOT_NULL(device_location_data);

	ret_code_t err_code;
	ble_gatts_value_t gatts_value;

	memset(&gatts_value, 0, sizeof(gatts_value));
	gatts_value.len     = sizeof(ble_dls_location_t);
	gatts_value.offset  = 0;
	gatts_value.p_value = (uint8_t *)device_location_data;

	err_code = sd_ble_gatts_value_set(p_dls->conn_handle,
	                                  p_dls->char_device_location_handles.value_handle,
	                                  &gatts_value);

	// Send value if connected and notifying
	if ((p_dls->conn_handle != BLE_CONN_HANDLE_INVALID)) 
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));
		hvx_params.handle = p_dls->char_device_location_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_dls->conn_handle, &hvx_params);
		if (err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
		{
			err_code = sd_ble_gatts_sys_attr_set(p_dls->conn_handle, NULL, 0, 0);
			err_code = sd_ble_gatts_hvx(p_dls->conn_handle, &hvx_params);
		}
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}

//--------------------------------------------
ret_code_t ble_dls_remote_location_update(ble_dls_t *p_dls, ble_dls_location_t *remote_location_data)
{
	VERIFY_PARAM_NOT_NULL(p_dls);
	VERIFY_PARAM_NOT_NULL(remote_location_data);

	ret_code_t err_code;
	ble_gatts_value_t gatts_value;

	memset(&gatts_value, 0, sizeof(gatts_value));
	gatts_value.len     = sizeof(ble_dls_location_t);
	gatts_value.offset  = 0;
	gatts_value.p_value = (uint8_t *)remote_location_data;

	err_code = sd_ble_gatts_value_set(p_dls->conn_handle,
	                                  p_dls->char_remote_location_handles.value_handle,
	                                  &gatts_value);

	// Send value if connected and notifying
	if ((p_dls->conn_handle != BLE_CONN_HANDLE_INVALID))
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));
		hvx_params.handle = p_dls->char_remote_location_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_dls->conn_handle, &hvx_params);
		if (err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
		{
			err_code = sd_ble_gatts_sys_attr_set(p_dls->conn_handle, NULL, 0, 0);
			err_code = sd_ble_gatts_hvx(p_dls->conn_handle, &hvx_params);
		}
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}

//--------------------------------------------
ret_code_t ble_dls_rssi_noise_update(ble_dls_t *p_dls, ble_dls_rssi_t *rssi_noise_data)
{
	VERIFY_PARAM_NOT_NULL(p_dls);
	VERIFY_PARAM_NOT_NULL(rssi_noise_data);

	ret_code_t err_code;
	ble_gatts_value_t gatts_value;

	memset(&gatts_value, 0, sizeof(gatts_value));
	gatts_value.len     = sizeof(ble_dls_rssi_t);
	gatts_value.offset  = 0;
	gatts_value.p_value = (uint8_t *)rssi_noise_data;

	err_code = sd_ble_gatts_value_set(p_dls->conn_handle,
	                                  p_dls->char_rssi_noise_handles.value_handle,
	                                  &gatts_value);

	// Send value if connected and notifying
	if ((p_dls->conn_handle != BLE_CONN_HANDLE_INVALID))
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));
		hvx_params.handle = p_dls->char_rssi_noise_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_dls->conn_handle, &hvx_params);
		if (err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
		{
			err_code = sd_ble_gatts_sys_attr_set(p_dls->conn_handle, NULL, 0, 0);
			err_code = sd_ble_gatts_hvx(p_dls->conn_handle, &hvx_params);
		}
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}

//--------------------------------------------
ret_code_t ble_dls_rssi_last_packet_update(ble_dls_t *p_dls, ble_dls_rssi_t *rssi_last_packet_data)
{
	VERIFY_PARAM_NOT_NULL(p_dls);
	VERIFY_PARAM_NOT_NULL(rssi_last_packet_data);

	ret_code_t err_code;
	ble_gatts_value_t gatts_value;

	memset(&gatts_value, 0, sizeof(gatts_value));
	gatts_value.len     = sizeof(ble_dls_rssi_t);
	gatts_value.offset  = 0;
	gatts_value.p_value = (uint8_t *)rssi_last_packet_data;

	err_code = sd_ble_gatts_value_set(p_dls->conn_handle,
	                                  p_dls->char_rssi_last_packet_handles.value_handle,
	                                  &gatts_value);

	// Send value if connected and notifying
	if ((p_dls->conn_handle != BLE_CONN_HANDLE_INVALID))
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));
		hvx_params.handle = p_dls->char_rssi_last_packet_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_dls->conn_handle, &hvx_params);
		if (err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
		{
			err_code = sd_ble_gatts_sys_attr_set(p_dls->conn_handle, NULL, 0, 0);
			err_code = sd_ble_gatts_hvx(p_dls->conn_handle, &hvx_params);
		}
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}

//--------------------------------------------
ret_code_t ble_dls_lora_module_name_update(ble_dls_t *p_dls)
{
	VERIFY_PARAM_NOT_NULL(p_dls);

	ret_code_t err_code;
	ble_gatts_value_t gatts_value;

	memset(&gatts_value, 0, sizeof(gatts_value));
	gatts_value.len     = strlen((const char *)(p_dls->char_lora_module_name)->name);
	gatts_value.offset  = 0;
	gatts_value.p_value = (uint8_t *)p_dls->char_lora_module_name;

	err_code = sd_ble_gatts_value_set(p_dls->conn_handle,
	                                  p_dls->char_lora_module_name_handles.value_handle,
	                                  &gatts_value);
	return err_code;
}

//--------------------------------------------
ret_code_t ble_dls_lora_core_name_update(ble_dls_t *p_dls)
{
	VERIFY_PARAM_NOT_NULL(p_dls);

	ret_code_t err_code;
	ble_gatts_value_t gatts_value;

	memset(&gatts_value, 0, sizeof(gatts_value));
	gatts_value.len     = strlen((const char *)(p_dls->char_lora_core_name)->name);
	gatts_value.offset  = 0;
	gatts_value.p_value = (uint8_t *)p_dls->char_lora_core_name;

	err_code = sd_ble_gatts_value_set(p_dls->conn_handle,
	                                  p_dls->char_lora_core_name_handles.value_handle,
	                                  &gatts_value);
	return err_code;
}
