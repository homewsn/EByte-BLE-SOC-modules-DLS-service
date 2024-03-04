/*
* Copyright (c) 2021-2023 Vladimir Alemasov
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
/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_timer.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "psa/crypto.h"

#include "geo_location.h"
#include "ble_dls.h"
#include "ubx.h"
#include "lora-defs.h"
#include "isoc.h"
#include "disp.h"

void HardFault_Handler(void)
{
	asm("BKPT #0");
	while (1);
}


//------------------------------------------------------------------
// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

//------------------------------------------------------------------
static ble_dls_location_t char_device_location;
static ble_dls_location_t char_remote_location;
static ble_dls_remote_settings_t char_remote_settings;
static ble_dls_remote_aes_key_t char_remote_aes_key;
static ble_dls_lora_name_t char_lora_module_name = { .name = "Unknown" };
static ble_dls_lora_name_t char_lora_core_name = { .name = "Unknown" };
static ble_dls_rssi_t char_rssi_noise;
static ble_dls_rssi_t char_rssi_last_packet;

//------------------------------------------------------------------
// USART0 for ISOC communication
#define ISOC_AIRPKT_LENGTH    (sizeof(ble_dls_location_t) + (16U - sizeof(ble_dls_location_t) % 16U))
static uint8_t isoc_tx_buf[ISOC_MAX_MSG_LENGTH];
static uint8_t isoc_rx_buf[ISOC_BUF_LENGTH];
static uint8_t airpkt_rx_buf[ISOC_MAX_AIRPKT_LENGTH];
static uint8_t disp_tx_buf[DISP_BUF_LENGTH];
static bool remset;
static bool airpkt_tx;
static bool airpkt_rx;
static bool disp_tx;
static bool lm_name;
static bool lc_name;
static bool set_rssi;
static bool get_rssi;
static bool nf_rssi;

//------------------------------------------------------------------
// USART1 for GNSS/DISP communication
static uint8_t gnss_buf[UBX_BUF_LENGTH];
static bool gnss_rx;

//------------------------------------------------------------------
// Timer
static uint32_t timerFreq;
#define INIT_DELAY_10MS         1

//------------------------------------------------------------------
// MAC length
#define AES_CCM_MAC_LENGTH      4

//------------------------------------------------------------------
// NVM storage
#define NVM_KEY_SETTINGS      0x4000
#define NVM_KEY_AES_KEY       0x4001
static bool flash;

//------------------------------------------------------------------
// Init application timer
static void app_timer_init(void)
{
	uint32_t cnt_val;

	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	// Initialize TIMER0
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.enable = false;
	timerInit.prescale = timerPrescale1024;
	TIMER_Init(TIMER0, &timerInit);
	// Initialize TIMER1
	TIMER_Init_TypeDef timerInit1 = TIMER_INIT_DEFAULT;
	timerInit1.enable = false;
	timerInit1.prescale = timerPrescale1024;
	TIMER_Init(TIMER1, &timerInit1);
	// Configure TIMER0 Compare/Capture for output compare
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModeCompare;
	TIMER_InitCC(TIMER0, 0, &timerCCInit);
	// Configure TIMER1 Compare/Capture for output compare
	TIMER_InitCC_TypeDef timerCCInit1 = TIMER_INITCC_DEFAULT;
	timerCCInit1.mode = timerCCModeCompare;
	TIMER_InitCC(TIMER1, 0, &timerCCInit1);

	// timerFreq for TIMER1
	timerFreq = CMU_ClockFreqGet(cmuClock_TIMER1) / (timerInit.prescale + 1) / 100;

	cnt_val = timerFreq * INIT_DELAY_10MS;
	// set top count value
	TIMER_TopSet(TIMER1, cnt_val);
	// Set compare value to the first compare value
	TIMER_CompareSet(TIMER1, 0, cnt_val);

	// replacement timerFreq with the necessary one in the future for TIMER0
	timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);

	// Enable TIMER0 interrupts
	TIMER_IntEnable(TIMER0, TIMER_IEN_CC0);
	NVIC_EnableIRQ(TIMER0_IRQn);
	// Enable TIMER1 interrupts
	TIMER_IntEnable(TIMER1, TIMER_IEN_CC0);
	NVIC_EnableIRQ(TIMER1_IRQn);
}

//------------------------------------------------------------------
// Start application timer
static void app_timer_start(void)
{
	if (char_remote_settings.interval)
	{
		uint32_t cnt_val = timerFreq * char_remote_settings.interval;
		// Set top count value
		TIMER_TopSet(TIMER0, cnt_val);
		// Set compare value to the first compare value
		TIMER_CompareSet(TIMER0, 0, cnt_val);
		// Start timer
		TIMER_Enable(TIMER0, true);
	}
}

//------------------------------------------------------------------
// Stop application timer
static void app_timer_stop(void)
{
	// Stop timer
	TIMER_Enable(TIMER0, false);
}

//------------------------------------------------------------------
void TIMER0_IRQHandler(void)
{
	// Acknowledge the interrupt
	uint32_t flags = TIMER_IntGet(TIMER0);
	TIMER_IntClear(TIMER0, flags);

	airpkt_tx = true;
}

//------------------------------------------------------------------
void TIMER1_IRQHandler(void)
{
	// Acknowledge the interrupt
	uint32_t flags = TIMER_IntGet(TIMER1);
	TIMER_IntClear(TIMER1, flags);
	// Stop timer
	TIMER_Enable(TIMER1, false);
	// Enable receive data valid interrupt
	USART_IntEnable(USART0, USART_IEN_RXDATAV /* | USART_IEN_RXOF*/);
	USART_IntEnable(USART1, USART_IEN_RXDATAV /*| USART_IEN_RXOF*/);
	// Initial initialization of the LORA module
	lm_name = true;
	set_rssi = true;
}

//------------------------------------------------------------------
// Init ISOC and GNSS USARTs (Use Peripheral API)
static void uart_isoc_gnss_init(void)
{
	// Enable clock to GPIO, USART0, USART1
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART0, true);
	CMU_ClockEnable(cmuClock_USART1, true);
	// Configure the USART RX pin
	GPIO_PinModeSet(gpioPortC, 0, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortB, 0, gpioModeInput, 0);
	// Configure the USART TX pin
	GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortB, 1, gpioModePushPull, 0);

	// Route USART0 TX and RX to the board controller TX and RX pins
	GPIO->USARTROUTE[0].TXROUTE = (gpioPortC << _GPIO_USART_TXROUTE_PORT_SHIFT) | (1 << _GPIO_USART_TXROUTE_PIN_SHIFT);
	GPIO->USARTROUTE[0].RXROUTE = (gpioPortC << _GPIO_USART_RXROUTE_PORT_SHIFT) | (0 << _GPIO_USART_RXROUTE_PIN_SHIFT);
	// Route USART1 TX and RX to the board controller TX and RX pins
	GPIO->USARTROUTE[1].TXROUTE = (gpioPortB << _GPIO_USART_TXROUTE_PORT_SHIFT) | (1 << _GPIO_USART_TXROUTE_PIN_SHIFT);
	GPIO->USARTROUTE[1].RXROUTE = (gpioPortB << _GPIO_USART_RXROUTE_PORT_SHIFT) | (0 << _GPIO_USART_RXROUTE_PIN_SHIFT);

	// Enable RX and TX signals now that they have been routed
	GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN | GPIO_USART_ROUTEEN_TXPEN;
	GPIO->USARTROUTE[1].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN | GPIO_USART_ROUTEEN_TXPEN;

	// Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
	USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
	// Configure and enable USART0 and USART1
	USART_InitAsync(USART0, &init);
	USART_InitAsync(USART1, &init);

	// Enable NVIC USART sources
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);
	NVIC_EnableIRQ(USART0_RX_IRQn);
	NVIC_ClearPendingIRQ(USART1_RX_IRQn);
	NVIC_EnableIRQ(USART1_RX_IRQn);
}

//------------------------------------------------------------------
// Send buffer to the ISOC USART
static void uart_isoc_tx(uint8_t *buf, size_t len)
{
	size_t cnt;
	for (cnt = 0; cnt < len; cnt++)
	{
		while (!(USART0->STATUS & USART_STATUS_TXBL));
		USART0->TXDATA = buf[cnt];
	}
}

//------------------------------------------------------------------
// Send buffer to the DISP USART
static void uart_disp_tx(uint8_t *buf, size_t len)
{
	size_t cnt;
	for (cnt = 0; cnt < len; cnt++)
	{
		while (!(USART1->STATUS & USART_STATUS_TXBL));
		USART1->TXDATA = buf[cnt];
	}
}

//------------------------------------------------------------------
// ISOC UART IRQ handler
void USART0_RX_IRQHandler(void)
{
	sl_status_t sc;
	static size_t cnt;
	int len;

	if (USART0->IF & USART_IF_RXOF)
	{
		USART0->CMD |= USART_CMD_CLEARRX;
		USART0->IF_CLR = _USART_IF_MASK;
		cnt = 0;
		return;
	}
	if (USART0->STATUS & USART_STATUS_RXDATAV)
	{
		isoc_rx_buf[cnt++] = USART0->RXDATA;
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
				char_rssi_noise.rssi = isoc_get_rssi(isoc_rx_buf);
				sc = sl_bt_gatt_server_write_attribute_value(gattdb_rssi_noise,
				                                             0,
				                                             sizeof(char_rssi_noise),
				                                             (uint8_t const *)&char_rssi_noise);
				app_assert_status(sc);
				nf_rssi = true;
				break;
			case ISOC_REQ_LORA_PARAMS_CMD:
				remset = true;
				break;
			case ISOC_GET_MODULE_NAME_RSP:
				memset(char_lora_module_name.name, 0, sizeof(char_lora_module_name.name));
				strncpy((char *)char_lora_module_name.name, (const char *)&isoc_rx_buf[ISOC_HEADER_LENGTH], isoc_get_payload_length(isoc_rx_buf));
				sc = sl_bt_gatt_server_write_attribute_value(gattdb_lora_module_name,
				                                             0,
				                                             /*sizeof(char_lora_module_name)*/isoc_get_payload_length(isoc_rx_buf),
				                                             (uint8_t const *)&char_lora_module_name);
				app_assert_status(sc);
				lc_name = true;
				break;
			case ISOC_GET_LORACORE_NAME_RSP:
				memset(char_lora_core_name.name, 0, sizeof(char_lora_core_name.name));
				strncpy((char *)char_lora_core_name.name, (const char *)&isoc_rx_buf[ISOC_HEADER_LENGTH], isoc_get_payload_length(isoc_rx_buf));
				sc = sl_bt_gatt_server_write_attribute_value(gattdb_lora_core_name,
				                                             0,
				                                             /*sizeof(char_lora_core_name)*/isoc_get_payload_length(isoc_rx_buf),
				                                             (uint8_t const *)&char_lora_core_name);
				app_assert_status(sc);
				// if remset equals false the program waits for ISOC_REQ_LORA_PARAMS_CMD command from LORA module
				// so the yellow LED blinks a few seconds to let us know the modules work
				remset = true;
				break;
			case ISOC_REQ_AIR_PKT_CMD:
				if (char_remote_settings.interval)
				{
					app_timer_start();
					airpkt_tx = true;
				}
				break;
			case ISOC_AIR_PKT_CMD:
				char_rssi_last_packet.rssi = isoc_get_air_pkt_rssi(isoc_rx_buf);
				sc = sl_bt_gatt_server_write_attribute_value(gattdb_rssi_last_packet,
				                                             0,
				                                             sizeof(char_rssi_last_packet),
				                                             (uint8_t const *)&char_rssi_last_packet);
				app_assert_status(sc);
				memcpy(airpkt_rx_buf, &isoc_rx_buf[ISOC_HEADER_LENGTH], ISOC_AIRPKT_LENGTH);
				airpkt_rx = true;
				disp_tx = true;
				break;
			}
			cnt = 0;
		}
	}
}

//------------------------------------------------------------------
// GNSS UART IRQ handler
void USART1_RX_IRQHandler(void)
{
	sl_status_t sc;
	static size_t cnt;
	int len;

	if (USART1->IF & USART_IF_RXOF)
	{
		USART1->CMD |= USART_CMD_CLEARRX;
		USART1->IF_CLR = _USART_IF_MASK;
		cnt = 0;
		return;
	}
	if (USART1->STATUS & USART_STATUS_RXDATAV)
	{
		gnss_buf[cnt++] = USART1->RXDATA;
		len = ubx_message_check(gnss_buf, cnt);
		if (len < 0)
		{
			cnt = 0;
		}
		else if (len > 0)
		{
			if (!ubx_nav_pvt_parse(gnss_buf, cnt, &char_device_location))
			{
				// Disable receive data valid interrupt
				USART_IntDisable(USART1, USART_IEN_RXDATAV);
				sc = sl_bt_gatt_server_write_attribute_value(gattdb_device_location,
				                                             0,
				                                             sizeof(char_device_location),
				                                             (uint8_t const *)&char_device_location);
				app_assert_status(sc);
				gnss_rx = true;
			}
			cnt = 0;
		}
		if (cnt == sizeof(gnss_buf))
		{
			cnt = 0;
		}
	}
}

//------------------------------------------------------------------
// Application Init
void app_init(void)
{
	uart_isoc_gnss_init();
	app_timer_init();

	// PSA Crypto
	psa_crypto_init();

	// NVM storage
	sl_status_t sc;
	size_t nvm_len = 0;
	// read the characteristics from the NV memory
	sc = sl_bt_nvm_load(NVM_KEY_SETTINGS, sizeof(char_remote_settings), &nvm_len, (uint8_t *)&char_remote_settings);
	if (sc != SL_STATUS_OK || nvm_len != sizeof(char_remote_settings))
	{
		char_remote_settings.frequency            = 434000000;
		char_remote_settings.bandwidth            = LORA_BW_125;
		char_remote_settings.spreading_factor     = LORA_SF9;
		char_remote_settings.coding_rate          = LORA_CR_4_8;
		char_remote_settings.interval             = 10;
		char_remote_settings.rssi_channel_busy    = 0;
		char_remote_settings.magnetic_declination = 0x6FE5D55; // = +11°44' * 10000000
		sl_bt_nvm_save(NVM_KEY_SETTINGS, sizeof(char_remote_settings), (const uint8_t *)&char_remote_settings);
	}
	sc = sl_bt_nvm_load(NVM_KEY_AES_KEY, sizeof(char_remote_aes_key), &nvm_len, (uint8_t *)&char_remote_aes_key);
	if (sc != SL_STATUS_OK || nvm_len != sizeof(char_remote_aes_key))
	{
		memcpy(&char_remote_aes_key, DEFAULT_AES_KEY, sizeof(DEFAULT_AES_KEY));
		sl_bt_nvm_save(NVM_KEY_AES_KEY, sizeof(char_remote_aes_key), (uint8_t *)&char_remote_aes_key);
	}
}

//------------------------------------------------------------------
// Application Process Action
void app_process_action(void)
{
	static psa_key_attributes_t key_attr;
	static psa_key_id_t key_id;
	static psa_algorithm_t alg = PSA_ALG_AEAD_WITH_SHORTENED_TAG(PSA_ALG_CCM, AES_CCM_MAC_LENGTH);
	static uint8_t nonce[13] = { 0 };
	static uint8_t aes_ccm_buf[sizeof(char_device_location) + AES_CCM_MAC_LENGTH] = { 0 };
	static size_t len;

	if (lm_name)
	{
		size_t len;

		lm_name = false;
		len = isoc_get_module_name_cmd(isoc_tx_buf);
		if (set_rssi)
		{
			set_rssi = false;
			len += isoc_set_rssi_cmd(isoc_tx_buf + len, char_remote_settings.rssi_channel_busy);
		}
		uart_isoc_tx(isoc_tx_buf, len);
	}

	if (lc_name)
	{
		size_t len;

		lc_name = false;
		len = isoc_get_loracore_name_cmd(isoc_tx_buf);
		uart_isoc_tx(isoc_tx_buf, len);
	}

	if (flash)
	{
		flash = false;
		// Save the characteristics to the NV memory
		sl_bt_nvm_save(NVM_KEY_SETTINGS, sizeof(char_remote_settings), (const uint8_t *)&char_remote_settings);
		sl_bt_nvm_save(NVM_KEY_AES_KEY, sizeof(char_remote_aes_key), (uint8_t *)&char_remote_aes_key);
	}

	if (remset)
	{
		size_t len;
		lora_params_t params = { 0 };

		remset = false;
		// Send remote settings command to the long range module
		params.freq_in_hz = char_remote_settings.frequency;
		params.bw = (lora_bw_t)char_remote_settings.bandwidth;
		params.cr = (lora_cr_t)char_remote_settings.coding_rate;
		params.sf = (lora_sf_t)char_remote_settings.spreading_factor;
		len = isoc_set_lora_params_cmd(isoc_tx_buf, &params);
		if (!get_rssi)
		{
			get_rssi = true;
			len += isoc_set_rssi_cmd(isoc_tx_buf + len, char_remote_settings.rssi_channel_busy);
		}
		uart_isoc_tx(isoc_tx_buf, len);
	}

	if (gnss_rx)
	{
		sl_status_t sc;

		sc = sl_bt_gatt_server_notify_all(gattdb_device_location,
		                                  sizeof(char_device_location),
		                                  (uint8_t const *)&char_device_location);
		app_assert_status(sc);
		gnss_rx = false;
		// Enable receive data valid interrupt
		USART_IntEnable(USART1, USART_IEN_RXDATAV);
		disp_tx = true;
	}

	if (airpkt_tx)
	{
		// encoding
		key_attr = psa_key_attributes_init();
		psa_set_key_type(&key_attr, PSA_KEY_TYPE_AES);
		psa_set_key_bits(&key_attr, 128);
		psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
		psa_set_key_algorithm(&key_attr, alg);
		psa_set_key_lifetime(&key_attr, PSA_KEY_LIFETIME_VOLATILE);

		psa_import_key(&key_attr, (const uint8_t *)&char_remote_aes_key, sizeof(char_remote_aes_key), &key_id);
		psa_reset_key_attributes(&key_attr);

		psa_aead_encrypt(key_id,
						 alg,
						 nonce,
						 sizeof(nonce),
						 NULL,
						 0,
						 (uint8_t *)&char_device_location,
						 sizeof(char_device_location),
						 aes_ccm_buf,
						 sizeof(aes_ccm_buf),
						 &len);

		psa_destroy_key(key_id);

		// Send air packet command to the long range module
		len = isoc_air_pkt_cmd(isoc_tx_buf, aes_ccm_buf, ISOC_AIRPKT_LENGTH);
		uart_isoc_tx(isoc_tx_buf, len);
		airpkt_tx = false;
	}

	if (airpkt_rx)
	{
		// decoding
		psa_status_t status;

		key_attr = psa_key_attributes_init();
		psa_set_key_type(&key_attr, PSA_KEY_TYPE_AES);
		psa_set_key_bits(&key_attr, 128);
		psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
		psa_set_key_algorithm(&key_attr, alg);
		psa_set_key_lifetime(&key_attr, PSA_KEY_LIFETIME_VOLATILE);

		psa_import_key(&key_attr, (const uint8_t *)&char_remote_aes_key, sizeof(char_remote_aes_key), &key_id);
		psa_reset_key_attributes(&key_attr);

		status = psa_aead_decrypt(key_id,
								  alg,
								  nonce,
								  sizeof(nonce),
								  NULL,
								  0,
								  airpkt_rx_buf,
								  sizeof(char_remote_location) + AES_CCM_MAC_LENGTH,
								  (uint8_t *)&char_remote_location,
								  sizeof(char_remote_location),
								  &len);

		psa_destroy_key(key_id);

		if (status == PSA_SUCCESS)
		{
			sl_status_t sc;

			sc = sl_bt_gatt_server_notify_all(gattdb_remote_location,
			                                  sizeof(char_remote_location),
			                                  (uint8_t const *)&char_remote_location);
			app_assert_status(sc);
			sc = sl_bt_gatt_server_notify_all(gattdb_rssi_last_packet,
			                                  sizeof(char_rssi_last_packet),
			                                  (uint8_t const *)&char_rssi_last_packet);
			app_assert_status(sc);
		}
		airpkt_rx = false;
	}

	if (disp_tx)
	{
		size_t len;
		disp_info_t info;

		disp_tx = false;
		info.declination = char_remote_settings.magnetic_declination;
		info.local_latitude = char_device_location.latitude;
		info.local_longitude = char_device_location.longitude;
		info.remote_latitude = char_remote_location.latitude;
		info.remote_longitude = char_remote_location.longitude;
		len = disp_set_info_pkt_cmd(disp_tx_buf, &info);
		uart_disp_tx(disp_tx_buf, len);

		if (get_rssi)
		{
			len = isoc_get_rssi_cmd(isoc_tx_buf);
			uart_isoc_tx(isoc_tx_buf, len);
		}
	}

	if (set_rssi)
	{
		size_t len;

		set_rssi = false;
		len = isoc_set_rssi_cmd(isoc_tx_buf, char_remote_settings.rssi_channel_busy);
		uart_isoc_tx(isoc_tx_buf, len);
	}

	if (nf_rssi)
	{
		sl_status_t sc;

		nf_rssi = false;
		sc = sl_bt_gatt_server_notify_all(gattdb_rssi_noise,
		                                  sizeof(char_rssi_noise),
		                                  (uint8_t const *)&char_rssi_noise);
		app_assert_status(sc);
	}
}

//------------------------------------------------------------------
// Bluetooth stack event handler (This overrides the dummy weak implementation)
void sl_bt_on_event(sl_bt_msg_t *evt)
{
	sl_status_t sc;

	switch (SL_BT_MSG_ID(evt->header))
	{
	case sl_bt_evt_system_boot_id:
		// This event indicates the device has started and the radio is ready.
		// Do not call any stack command before receiving this boot event!

#if 0
		// Set Security Manager in debug mode. In this mode, the secure connections bonding uses known debug keys,
		// so that the encrypted packet can be opened by Bluetooth protocol analyzer.
		// Bondings made in debug mode are unsecure.
		sl_bt_sm_set_debug_mode();
#endif

		// Write attribute values restored from the NV memory
		sc = sl_bt_gatt_server_write_attribute_value(gattdb_remote_settings,
													 0,
													 sizeof(char_remote_settings),
													 (uint8_t const *)&char_remote_settings);
		app_assert_status(sc);
		sc = sl_bt_gatt_server_write_attribute_value(gattdb_remote_aes_key,
													 0,
													 sizeof(char_remote_aes_key),
													 (uint8_t const *)&char_remote_aes_key);
		app_assert_status(sc);
		// Write default values
		sc = sl_bt_gatt_server_write_attribute_value(gattdb_lora_module_name,
													 0,
													 strlen((const char *)char_lora_module_name.name),
													 (uint8_t const *)&char_lora_module_name);
		app_assert_status(sc);
		sc = sl_bt_gatt_server_write_attribute_value(gattdb_lora_core_name,
													 0,
													 strlen((const char *)char_lora_core_name.name),
													 (uint8_t const *)&char_lora_core_name);
		app_assert_status(sc);

		// Create an advertising set.
		sc = sl_bt_advertiser_create_set(&advertising_set_handle);
		app_assert_status(sc);
		// Set advertising interval to 100ms.
		sc = sl_bt_advertiser_set_timing(advertising_set_handle,
		                                 160, // min. adv. interval (milliseconds * 1.6)
		                                 160, // max. adv. interval (milliseconds * 1.6)
		                                 0,   // adv. duration
		                                 0);  // max. num. adv. events
		app_assert_status(sc);
		// Start general advertising and enable connections.
		sc = sl_bt_advertiser_start(advertising_set_handle,
		                            sl_bt_advertiser_general_discoverable,
		                            sl_bt_advertiser_connectable_scannable);
		app_assert_status(sc);

		// Start the UART initialization timer
		TIMER_Enable(TIMER1, true);
		break;

	case sl_bt_evt_connection_opened_id:
		// This event indicates that a new connection was opened.
		break;

	case sl_bt_evt_connection_closed_id:
		// This event indicates that a connection was closed.
		// Restart advertising after client has disconnected.
		sc = sl_bt_advertiser_start(advertising_set_handle,
                                    sl_bt_advertiser_general_discoverable,
                                    sl_bt_advertiser_connectable_scannable);
		app_assert_status(sc);
		break;

	case sl_bt_evt_gatt_server_attribute_value_id:
		// This event indicates that the value of an attribute in the local GATT
		// database was changed by a remote GATT client.
		if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_remote_settings)
		{
			size_t data_recv_len;
			ble_dls_remote_settings_t remote_settings;

			sc = sl_bt_gatt_server_read_attribute_value(gattdb_remote_settings,
														0,
														sizeof(remote_settings),
														&data_recv_len,
														(uint8_t *)&remote_settings);
			app_assert_status(sc);
			if (sc != SL_STATUS_OK)
			{
				break;
			}
			if (memcmp(&char_remote_settings, &remote_settings, sizeof(remote_settings)))
			{
				if (char_remote_settings.rssi_channel_busy != remote_settings.rssi_channel_busy)
				{
					set_rssi = true;
				}
				if (char_remote_settings.magnetic_declination != remote_settings.magnetic_declination)
				{
					disp_tx = true;
				}
				if (char_remote_settings.bandwidth != remote_settings.bandwidth ||
					char_remote_settings.coding_rate != remote_settings.coding_rate ||
					char_remote_settings.frequency != remote_settings.frequency ||
					char_remote_settings.spreading_factor != remote_settings.spreading_factor)
				{
					remset = true;
				}
				if (char_remote_settings.interval != remote_settings.interval)
				{
					char_remote_settings.interval = remote_settings.interval;
					app_timer_stop();
					app_timer_start();
				}
				memcpy((uint8_t *)&char_remote_settings, (uint8_t *)&remote_settings, sizeof(remote_settings));
				flash = true;
			}
		}
		else if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_remote_aes_key)
		{
			size_t data_recv_len;
			ble_dls_remote_aes_key_t remote_aes_key;

			sc = sl_bt_gatt_server_read_attribute_value(gattdb_remote_aes_key,
														0,
														sizeof(remote_aes_key),
														&data_recv_len,
														(uint8_t *)&remote_aes_key);
			app_assert_status(sc);
			if (sc != SL_STATUS_OK)
			{
				break;
			}
			memcpy((uint8_t *)&char_remote_aes_key, (uint8_t *)&remote_aes_key, sizeof(remote_aes_key));
			flash = true;
		}
		break;

	default:
		break;
	}
}
