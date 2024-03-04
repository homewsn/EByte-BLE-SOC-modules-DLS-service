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

/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "geo_location.h"
#include "ble_dls.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "ble_conn_params.h"
#include "app_timer.h"
#include "boards.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrfx_uarte.h"
#include "ubx.h"
#include "nrf_crypto_aead.h"
#include "nrf_crypto_init.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "crc16.h"
#include "lora-defs.h"
#include "isoc.h"
#include "disp.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"

//------------------------------------------------------------------
#define DEVICE_NAME                     "HomeWSN_DLS"                      /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "HomeWSN"                          /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                  /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_ADV_INTERVAL                300                                /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#if DEBUG
#define APP_ADV_DURATION                0                                  /**< The advertising duration (indefinitely) in units of 10 milliseconds. */
#else
#define APP_ADV_DURATION                18000                              /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#endif

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)   /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)   /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                  /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)              /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)             /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                  /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                  /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                  /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  1                                  /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS              0                                  /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE               /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                  /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                  /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                 /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */



//------------------------------------------------------------------
BLE_DLS_DEF(m_dls);                                        /**< Double location service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                  /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                    /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                        /**< Advertising module instance. */

//------------------------------------------------------------------
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

//------------------------------------------------------------------
static ble_dls_location_t m_device_location;
static ble_dls_location_t m_remote_location;
static __ALIGN(4) ble_dls_remote_settings_t m_remote_settings;
static __ALIGN(4) ble_dls_remote_aes_key_t m_remote_aes_key;
static ble_dls_lora_name_t m_lora_module_name = { .name = "Unknown" };
static ble_dls_lora_name_t m_lora_core_name = { .name = "Unknown" };
static ble_dls_rssi_t m_rssi_noise;
static ble_dls_rssi_t m_rssi_last_packet;

//------------------------------------------------------------------
static void advertising_start(bool erase_bonds);
static void idle_state_handle(void);

//------------------------------------------------------------------
#define FSTORAGE_START_ADDR         0x50000
#define FSTORAGE_END_ADDR           0x50FFF  // one 4-byte page
#define FSTORAGE_SETTINGS_ADDR      FSTORAGE_START_ADDR
#define FSTORAGE_AES_KEY_ADDR       FSTORAGE_START_ADDR + 0x10
#define FSTORAGE_CRC16_ADDR         FSTORAGE_START_ADDR + 0x20
typedef struct crc_remote
{
	uint16_t settings;
	uint16_t aes_key;
} crc_remote_t;
static bool flash;
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = FSTORAGE_START_ADDR,
    .end_addr   = FSTORAGE_END_ADDR,
};

//------------------------------------------------------------------
// UARTE0 for ISOC communication
#define ISOC_AIRPKT_LENGTH    (sizeof(ble_dls_location_t) + (16U - sizeof(ble_dls_location_t) % 16U))
static nrfx_uarte_t isoc_uart = NRFX_UARTE_INSTANCE(0);
static uint8_t isoc_rx_byte;
static uint8_t isoc_tx_buf[ISOC_MAX_MSG_LENGTH];
static uint8_t isoc_rx_buf[ISOC_BUF_LENGTH];
static uint8_t airpkt_rx_buf[ISOC_MAX_AIRPKT_LENGTH];
static uint8_t disp_tx_buf[DISP_BUF_LENGTH];
static bool remset;
static bool airpkt_tx;
static bool airpkt_rx;
static bool disp_tx;
static bool lm_name = true;
static bool lc_name;
static bool set_rssi = true;
static bool get_rssi;

//------------------------------------------------------------------
// Timer for ISOC communication
#define TIMER_1SEC_INTERVAL     APP_TIMER_TICKS(1000)
APP_TIMER_DEF(isoc_timer);

//------------------------------------------------------------------
// MAC length
#define AES_CCM_MAC_LENGTH      4

//------------------------------------------------------------------
// UART1 for GNSS/DISP communication
static nrfx_uarte_t gnss_uart = NRFX_UARTE_INSTANCE(1);
static uint8_t gnss_rx_byte;
static uint8_t gnss_buf[UBX_BUF_LENGTH];

//------------------------------------------------------------------
// UICR REGOUT0 section for using LDO0 with VOUT = 5 (3.3v)
#pragma location = "UICR_REGOUT0"
const uint32_t uicr_regout0 = 0xFFFFFFFD;


//------------------------------------------------------------------
// function to prevent the compiler from excluding the variable uicr_regout0
void uicr_regout0_init(void)
{
	if ((uicr_regout0 & UICR_REGOUT0_VOUT_Msk) != UICR_REGOUT0_VOUT_3V3)
	{
		// dummy line below
		NRF_UICR->REGOUT0 = UICR_REGOUT0_VOUT_3V3;
	}
}


//------------------------------------------------------------------
#define FLASH_BUF_SIZE sizeof(ble_dls_remote_settings_t) + sizeof(ble_dls_remote_settings_t) % 4
typedef union
{
	ble_dls_remote_settings_t remote_settings;
	uint8_t buf[FLASH_BUF_SIZE];
} flash_buf_t;

//------------------------------------------------------------------
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
	if (p_evt->result != NRF_SUCCESS)
	{
		NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
		return;
	}
	switch (p_evt->id)
	{
	case NRF_FSTORAGE_EVT_WRITE_RESULT:
		NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.", p_evt->len, p_evt->addr);
		break;
	case NRF_FSTORAGE_EVT_ERASE_RESULT:
		NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.", p_evt->len, p_evt->addr);
		break;
	default:
		break;
	}
}

//------------------------------------------------------------------
void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    // While fstorage is busy call idle_state_handle
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        idle_state_handle();
    }
}

//------------------------------------------------------------------
static void fstorage_update(void)
{
	ret_code_t err_code;
	crc_remote_t crc_remote;
	flash_buf_t flash_buf = { 0 };

	err_code = nrf_fstorage_erase(&fstorage, FSTORAGE_START_ADDR, 1,  NULL);
    APP_ERROR_CHECK(err_code);
	wait_for_flash_ready(&fstorage);

	memcpy(&flash_buf.remote_settings, &m_remote_settings, sizeof(ble_dls_remote_settings_t));
	err_code = nrf_fstorage_write(&fstorage, FSTORAGE_SETTINGS_ADDR, &flash_buf.buf, sizeof(flash_buf.buf), NULL);
	APP_ERROR_CHECK(err_code);
	wait_for_flash_ready(&fstorage);

	err_code = nrf_fstorage_write(&fstorage, FSTORAGE_AES_KEY_ADDR, &m_remote_aes_key, sizeof(m_remote_aes_key), NULL);
	APP_ERROR_CHECK(err_code);
	wait_for_flash_ready(&fstorage);

	crc_remote.settings = crc16_compute((uint8_t *)&m_remote_settings, sizeof(m_remote_settings), NULL);
	crc_remote.aes_key = crc16_compute((uint8_t *)&m_remote_aes_key, sizeof(m_remote_aes_key), NULL);
	err_code = nrf_fstorage_write(&fstorage, FSTORAGE_CRC16_ADDR, &crc_remote, sizeof(crc_remote), NULL);
	APP_ERROR_CHECK(err_code);
	wait_for_flash_ready(&fstorage);
}

//------------------------------------------------------------------
static void fstorage_init(void)
{
	ret_code_t err_code;
	nrf_fstorage_api_t *p_fs_api;
	flash_buf_t flash_buf = { 0 };
	ble_dls_remote_aes_key_t remote_aes_key;
	crc_remote_t crc_remote;
	uint16_t crc;

    p_fs_api = &nrf_fstorage_sd;
	err_code = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(err_code);

	err_code = nrf_fstorage_read(&fstorage, FSTORAGE_SETTINGS_ADDR, &flash_buf.buf, sizeof(flash_buf.buf));
    APP_ERROR_CHECK(err_code);
	err_code = nrf_fstorage_read(&fstorage, FSTORAGE_AES_KEY_ADDR, &remote_aes_key, sizeof(remote_aes_key));
    APP_ERROR_CHECK(err_code);
	err_code = nrf_fstorage_read(&fstorage, FSTORAGE_CRC16_ADDR, &crc_remote, sizeof(crc_remote));
    APP_ERROR_CHECK(err_code);
	crc = crc16_compute((uint8_t *)&flash_buf.remote_settings, sizeof(ble_dls_remote_settings_t), NULL);
	if (crc != crc_remote.settings)
	{
		m_remote_settings.frequency            = 434000000;
		m_remote_settings.bandwidth            = LORA_BW_125;
		m_remote_settings.spreading_factor     = LORA_SF9;
		m_remote_settings.coding_rate          = LORA_CR_4_8;
		m_remote_settings.interval             = 10;
		m_remote_settings.rssi_channel_busy    = 0;
		m_remote_settings.magnetic_declination = 0x6FE5D55; // = +11°44' * 10000000
		flash = true;
	}
	else
	{
		memcpy(&m_remote_settings, &flash_buf.remote_settings, sizeof(ble_dls_remote_settings_t));
	}
	crc = crc16_compute((uint8_t *)&remote_aes_key, sizeof(remote_aes_key), NULL);
	if (crc != crc_remote.aes_key)
	{
		memcpy(&m_remote_aes_key, DEFAULT_AES_KEY, sizeof(DEFAULT_AES_KEY));
		flash = true;
	}
	else
	{
		memcpy(&m_remote_aes_key, &remote_aes_key, sizeof(remote_aes_key));
	}
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
	case PM_EVT_PEERS_DELETE_SUCCEEDED:
		// Bonds are deleted. Start advertising.
        advertising_start(false);
        break;
	default:
		break;
    }
}


/**@brief Function for starting timers.
 */
static void app_timers_start(void)
{
    ret_code_t err_code;

	if (m_remote_settings.interval)
	{
		err_code = app_timer_start(isoc_timer, m_remote_settings.interval * TIMER_1SEC_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
	}
}


/**@brief Double Location event handler.
 *
 * @details This function will be called for all events of the Double Location Module that
 *          are passed to the application.
 *
 * @param[in]   p_evt   Event received from the Double Location Module.
 */
static void on_dls_evt(ble_dls_t const *p_dls, ble_dls_evt_t const *p_evt)
{
    ret_code_t err_code;

	switch (p_evt->evt_type)
	{
	case BLE_DLS_REMOTE_RSSI_SETTINGS_CHANGED:
		set_rssi = true;
		NRF_LOG_INFO("DLS: RemoteRssiSettings changed event");
		break;
	case BLE_DLS_REMOTE_DISP_SETTINGS_CHANGED:
		disp_tx = true;
		NRF_LOG_INFO("DLS: RemoteDispSettings changed event");
		break;
	case BLE_DLS_REMOTE_LORA_SETTINGS_CHANGED:
		remset = true;
		NRF_LOG_INFO("DLS: RemoteLoraSettings changed event");
		break;
	case BLE_DLS_REMOTE_INTV_SETTINGS_CHANGED:
		err_code = app_timer_stop(isoc_timer);
    	APP_ERROR_CHECK(err_code);
		app_timers_start();
		NRF_LOG_INFO("DLS: RemoteIntvSettings changed event");
		break;
	case BLE_DLS_REMOTE_SETTINGS_WRITE:
		flash = true;
		NRF_LOG_INFO("DLS: RemoteSettings write event");
		break;
	case BLE_DLS_REMOTE_AES_KEY_WRITE:
		flash = true;
		NRF_LOG_INFO("DLS: RemoteAesKey write event");
		break;
	default:
		break;
	}
}


/**@brief Function for handling the Remote Location event timer time-out.
 *
 * @details This function will be called each time the Remote Location event timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the time-out handler.
 */
static void isoc_timer_timeout_handler(void *p_context)
{
	airpkt_tx = true;
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
	ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

	// Create timers.
    err_code = app_timer_create(&isoc_timer,
                                APP_TIMER_MODE_REPEATED,
                                isoc_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = { 0 };
	ble_dls_init_t     dls_init;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

	// Initialize Double Location Service structure
	memset(&dls_init, 0, sizeof(dls_init));
	m_dls.evt_handler      = on_dls_evt;
//	m_dls.error_handler    = on_dls_error;
	m_dls.char_device_location = &m_device_location;
	m_dls.char_remote_location = &m_remote_location;
	m_dls.char_remote_settings = &m_remote_settings;
	m_dls.char_remote_aes_key = &m_remote_aes_key;
	m_dls.char_lora_module_name = &m_lora_module_name;
	m_dls.char_lora_core_name = &m_lora_core_name;
	m_dls.char_rssi_noise = &m_rssi_noise;
	m_dls.char_rssi_last_packet = &m_rssi_last_packet;

	// Initialize Double Location Service characteristic structure
	dls_init.char_device_location_security_req_cccd_write_perm = SEC_OPEN;
	dls_init.char_remote_location_security_req_cccd_write_perm = SEC_OPEN;
	dls_init.char_remote_settings_security_req_read_perm = SEC_OPEN;
	dls_init.char_remote_settings_security_req_write_perm = SEC_OPEN;
	dls_init.char_remote_aes_key_security_req_read_perm = SEC_OPEN;
	dls_init.char_remote_aes_key_security_req_write_perm = SEC_OPEN;
	dls_init.char_lora_module_name_security_req_read_perm = SEC_OPEN;
	dls_init.char_lora_module_core_security_req_read_perm = SEC_OPEN;
	dls_init.char_rssi_noise_security_req_cccd_write_perm = SEC_OPEN;
	dls_init.char_rssi_last_packet_security_req_cccd_write_perm = SEC_OPEN;

	// Initialize Double Location Service.
	err_code = ble_dls_init(&m_dls, &dls_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
        NRF_LOG_INFO("Fast advertising.");
        break;
    case BLE_ADV_EVT_IDLE:
		NRF_LOG_INFO("Sleep mode.");
        sleep_mode_enter();
        break;
    default:
        break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected.");
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected.");
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
	        NRF_LOG_DEBUG("PHY update request.");
    	    ble_gap_phys_t const phys =
        	{
            	.rx_phys = BLE_GAP_PHY_AUTO,
            	.tx_phys = BLE_GAP_PHY_AUTO,
        	};
        	err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        	APP_ERROR_CHECK(err_code);
    	}
		break;
	case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    	{
        	ble_gap_data_length_params_t dl_params;

        	// Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
        	memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
	        APP_ERROR_CHECK(err_code);
    	}
		break;
	case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
		NRF_LOG_DEBUG("DHKey request.");
        break;
	default:
        // No implementation needed.
        break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ret_code_t           err_code;
    ble_gap_sec_params_t sec_param;

    memset(&sec_param, 0, sizeof(sec_param));

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type            = BLE_ADVDATA_FULL_NAME;
    init.advdata.flags                = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
	ret_code_t err_code;

	err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;
	static uint8_t nonce[13] = { 0 };
	static uint8_t aes_ccm_buf[sizeof(m_device_location) + AES_CCM_MAC_LENGTH] = { 0 };

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

	if (lm_name)
	{
		size_t len;

		lm_name = false;
		len = isoc_get_module_name_cmd(isoc_tx_buf);
		if (set_rssi)
		{
			set_rssi = false;
			len += isoc_set_rssi_cmd(isoc_tx_buf + len, m_remote_settings.rssi_channel_busy);
		}
		err_code = nrfx_uarte_tx(&isoc_uart, isoc_tx_buf, len);
	}

	if (lc_name)
	{
		size_t len;

		lc_name = false;
		len = isoc_get_loracore_name_cmd(isoc_tx_buf);
		err_code = nrfx_uarte_tx(&isoc_uart, isoc_tx_buf, len);
	}

	if (flash)
	{
		flash = false;
		fstorage_update();
	}

	if (remset)
	{
		size_t len;
		lora_params_t params = { 0 };

		remset = false;
		params.freq_in_hz = m_remote_settings.frequency;
		params.bw = (lora_bw_t)m_remote_settings.bandwidth;
		params.cr = (lora_cr_t)m_remote_settings.coding_rate;
		params.sf = (lora_sf_t)m_remote_settings.spreading_factor;
		len = isoc_set_lora_params_cmd(isoc_tx_buf, &params);
		if (!get_rssi)
		{
			get_rssi = true;
			len += isoc_set_rssi_cmd(isoc_tx_buf + len, m_remote_settings.rssi_channel_busy);
		}
		err_code = nrfx_uarte_tx(&isoc_uart, isoc_tx_buf, len);
	}

	if (airpkt_tx)
	{
		// encoding
		size_t len;
		nrf_crypto_aead_context_t ccm_ctx;
		nrf_crypto_aead_info_t const *p_ccm_k128_info = &g_nrf_crypto_aes_ccm_128_info;

		err_code = nrf_crypto_aead_init(&ccm_ctx,
			p_ccm_k128_info,
			m_remote_aes_key.aes_key);
		err_code = nrf_crypto_aead_crypt(&ccm_ctx,
			NRF_CRYPTO_ENCRYPT,
			nonce,
			sizeof(nonce),
			NULL,
			0,
			(uint8_t *)&m_device_location,
			sizeof(m_device_location),
			aes_ccm_buf,
			&aes_ccm_buf[sizeof(m_device_location)],
			AES_CCM_MAC_LENGTH);
		err_code = nrf_crypto_aead_uninit(&ccm_ctx);

		len = isoc_air_pkt_cmd(isoc_tx_buf, aes_ccm_buf, ISOC_AIRPKT_LENGTH);
		err_code = nrfx_uarte_tx(&isoc_uart, isoc_tx_buf, len);
		airpkt_tx = false;
	}

	if (airpkt_rx)
	{
		// decoding
		nrf_crypto_aead_context_t ccm_ctx;
		nrf_crypto_aead_info_t const *p_ccm_k128_info = &g_nrf_crypto_aes_ccm_128_info;

		err_code = nrf_crypto_aead_init(&ccm_ctx,
			p_ccm_k128_info,
			m_remote_aes_key.aes_key);
		err_code = nrf_crypto_aead_crypt(&ccm_ctx,
			NRF_CRYPTO_DECRYPT,
			nonce,
			sizeof(nonce),
			NULL,
			0,
			airpkt_rx_buf,
			sizeof(m_device_location),
			aes_ccm_buf,
			&airpkt_rx_buf[sizeof(m_device_location)],
			AES_CCM_MAC_LENGTH);
		if (!err_code)
		{
			memcpy(&m_remote_location, aes_ccm_buf, sizeof(m_device_location));
		    err_code = ble_dls_remote_location_update(&m_dls, &m_remote_location);
			if (err_code != NRF_ERROR_INVALID_STATE)
			{
    			APP_ERROR_CHECK(err_code);
			}
		}
		err_code = nrf_crypto_aead_uninit(&ccm_ctx);
		airpkt_rx = false;
	}

	if (disp_tx)
	{
		size_t len;
		disp_info_t info;

		disp_tx = false;
		info.declination = m_remote_settings.magnetic_declination;
		info.local_latitude = m_device_location.latitude;
		info.local_longitude = m_device_location.longitude;
		info.remote_latitude = m_remote_location.latitude;
		info.remote_longitude = m_remote_location.longitude;
		len = disp_set_info_pkt_cmd(disp_tx_buf, &info);
		err_code = nrfx_uarte_tx(&gnss_uart, disp_tx_buf, len);

		if (get_rssi)
		{
			len = isoc_get_rssi_cmd(isoc_tx_buf);
			err_code = nrfx_uarte_tx(&isoc_uart, isoc_tx_buf, len);
		}
	}

	if (set_rssi)
	{
		size_t len;

		set_rssi = false;
		len = isoc_set_rssi_cmd(isoc_tx_buf, m_remote_settings.rssi_channel_busy);
		err_code = nrfx_uarte_tx(&isoc_uart, isoc_tx_buf, len);
	}

	if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event
		// in idle_state_handle() => nrf_ble_lesc_request_handler()
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


//------------------------------------------------------------------
static void uart_isoc_event_handler(nrfx_uarte_event_t const *p_event, void *p_context)
{
	static size_t cnt;
	ret_code_t err_code;
	int len;

	switch (p_event->type)
    {
	case NRFX_UARTE_EVT_RX_DONE:
		isoc_rx_buf[cnt++] = isoc_rx_byte;
		err_code = nrfx_uarte_rx(&isoc_uart, &isoc_rx_byte, 1);
		if (err_code != NRFX_SUCCESS)
		{
			break;
		}
		len = isoc_message_check(isoc_rx_buf, cnt);
		if (len < 0)
		{
			cnt = 0;
		}
		else if (len > 0)
		{
			switch (isoc_get_cmd(isoc_rx_buf))
			{
			case ISOC_SET_RSSI_CMD:
				m_rssi_noise.rssi = isoc_get_rssi(isoc_rx_buf);
	    		err_code = ble_dls_rssi_noise_update(&m_dls, &m_rssi_noise);
				if (err_code != NRF_ERROR_INVALID_STATE)
				{
   					APP_ERROR_CHECK(err_code);
				}
				break;
			case ISOC_REQ_LORA_PARAMS_CMD:
				remset = true;
				break;
			case ISOC_GET_MODULE_NAME_RSP:
				memset(m_lora_module_name.name, 0, sizeof(m_lora_module_name.name));
				strncpy((char *)m_lora_module_name.name, (const char *)&isoc_rx_buf[ISOC_HEADER_LENGTH], isoc_get_payload_length(isoc_rx_buf));
			    err_code = ble_dls_lora_module_name_update(&m_dls);
				if (err_code != NRF_ERROR_INVALID_STATE)
				{
    				APP_ERROR_CHECK(err_code);
				}
				lc_name = true;
				break;
			case ISOC_GET_LORACORE_NAME_RSP:
				memset(m_lora_core_name.name, 0, sizeof(m_lora_core_name.name));
				strncpy((char *)m_lora_core_name.name, (const char *)&isoc_rx_buf[ISOC_HEADER_LENGTH], isoc_get_payload_length(isoc_rx_buf));
			    err_code = ble_dls_lora_core_name_update(&m_dls);
				if (err_code != NRF_ERROR_INVALID_STATE)
				{
    				APP_ERROR_CHECK(err_code);
				}
#if DEBUG
				// if remset equals false the program waits for ISOC_REQ_LORA_PARAMS_CMD command from LORA module
				// so the yellow LED blinks a few seconds to let us know the modules work
//				remset = true;
#endif
				break;
			case ISOC_REQ_AIR_PKT_CMD:
				if (m_remote_settings.interval)
				{
					app_timers_start();
					airpkt_tx = true;
				}
				break;
			case ISOC_AIR_PKT_CMD:
				m_rssi_last_packet.rssi = isoc_get_air_pkt_rssi(isoc_rx_buf);
	    		err_code = ble_dls_rssi_last_packet_update(&m_dls, &m_rssi_last_packet);
				if (err_code != NRF_ERROR_INVALID_STATE)
				{
   					APP_ERROR_CHECK(err_code);
				}
				memcpy(airpkt_rx_buf, &isoc_rx_buf[ISOC_HEADER_LENGTH], ISOC_AIRPKT_LENGTH);
				airpkt_rx = true;
				disp_tx = true;
				break;
			}
			cnt = 0;
		}
		break;
    case NRFX_UARTE_EVT_ERROR:
		err_code = nrfx_uarte_rx(&isoc_uart, &isoc_rx_byte, 1);
		cnt = 0;
		break;
    case NRFX_UARTE_EVT_TX_DONE:
		break;
	}
}

//------------------------------------------------------------------
static void uart_isoc_init(void)
{
	ret_code_t err_code;
	nrfx_uarte_config_t m_uart_isoc_drv_config = NRFX_UARTE_DEFAULT_CONFIG;

    m_uart_isoc_drv_config.pseltxd = UART_ISOC_TX_PIN_NUMBER;
    m_uart_isoc_drv_config.pselrxd = UART_ISOC_RX_PIN_NUMBER;
    m_uart_isoc_drv_config.pselcts = UART_ISOC_CTS_PIN_NUMBER;
    m_uart_isoc_drv_config.pselrts = UART_ISOC_RTS_PIN_NUMBER;

	err_code = nrfx_uarte_init(&isoc_uart, &m_uart_isoc_drv_config, uart_isoc_event_handler);
	err_code = nrfx_uarte_rx(&isoc_uart, &isoc_rx_byte, 1);
}

//------------------------------------------------------------------
static void uart_gnss_event_handler(nrfx_uarte_event_t const *p_event, void *p_context)
{
	static size_t cnt;
	ret_code_t err_code;
	int len;

	switch (p_event->type)
    {
	case NRFX_UARTE_EVT_RX_DONE:
		gnss_buf[cnt++] = gnss_rx_byte;
		err_code = nrfx_uarte_rx(&gnss_uart, &gnss_rx_byte, 1);
		if (err_code != NRFX_SUCCESS)
		{
			break;
		}

		len = ubx_message_check(gnss_buf, cnt);
		if (len < 0)
		{
			cnt = 0;
		}
		else if (len > 0)
		{
			if (!ubx_nav_pvt_parse(gnss_buf, cnt, &m_device_location))
			{
				ret_code_t err_code;

				err_code = ble_dls_device_location_update(&m_dls, &m_device_location);
				if (err_code != NRF_ERROR_INVALID_STATE)
				{
    				APP_ERROR_CHECK(err_code);
				}
				disp_tx = true;
			}
			cnt = 0;
		}
		break;
    case NRFX_UARTE_EVT_ERROR:
		err_code = nrfx_uarte_rx(&gnss_uart, &gnss_rx_byte, 1);
		cnt = 0;
		break;
    case NRFX_UARTE_EVT_TX_DONE:
		break;
	}
}

//------------------------------------------------------------------
static void uart_gnss_init(void)
{
	ret_code_t err_code;
	nrfx_uarte_config_t m_uart_gnss_drv_config = NRFX_UARTE_DEFAULT_CONFIG;

    m_uart_gnss_drv_config.pseltxd = UART_GNSS_TX_PIN_NUMBER;
    m_uart_gnss_drv_config.pselrxd = UART_GNSS_RX_PIN_NUMBER;
    m_uart_gnss_drv_config.pselcts = UART_GNSS_CTS_PIN_NUMBER;
    m_uart_gnss_drv_config.pselrts = UART_GNSS_RTS_PIN_NUMBER;

	err_code = nrfx_uarte_init(&gnss_uart, &m_uart_gnss_drv_config, uart_gnss_event_handler);
	err_code = nrfx_uarte_rx(&gnss_uart, &gnss_rx_byte, 1);
}


//------------------------------------------------------------------
int main(void)
{
#if DEBUG
	// Debug Monitor Mode
	NVIC_SetPriority(DebugMonitor_IRQn, _PRIO_SD_LOW);
#endif

#if NRF_CRYPTO_BACKEND_CC310_BL_ENABLED || NRF_CRYPTO_BACKEND_CC310_ENABLED
	// SDK 17.1.0
	// https://devzone.nordicsemi.com/f/nordic-q-a/80129/crys_rndinit-returns-crys_rnd_instantiation_error
	ret_code_t err_code;
	err_code = nrf_crypto_init();
#endif

	// Initialize.
	uicr_regout0_init();
	uart_gnss_init();
	uart_isoc_init();
    log_init();
    timers_init();
    power_management_init();
    ble_stack_init();
	fstorage_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

//	app_timers_start();

#if DEBUG
	// enable new pairing each time
	advertising_start(true);
#else
	advertising_start(false);
#endif

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
