/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
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
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

//------------------------------------------------------------------
#ifdef NRF52833_XXAA
// EBYTE board E73_2G4M08S1E_H
#define UART_ISOC_RX_PIN_NUMBER      8   // P0.08
#define UART_ISOC_TX_PIN_NUMBER      6   // P0.06
#define UART_ISOC_CTS_PIN_NUMBER     7   // P0.07
#define UART_ISOC_RTS_PIN_NUMBER     5   // P0.05

#define UART_GNSS_RX_PIN_NUMBER      28  // P0.28
#define UART_GNSS_TX_PIN_NUMBER      25  // P0.25
#define UART_GNSS_CTS_PIN_NUMBER     36  // P1.04
#define UART_GNSS_RTS_PIN_NUMBER     38  // P1.06
#endif
	
//------------------------------------------------------------------
#ifdef NRF52840_XXAA
// EBYTE board E73_2G4M08S1C_H
#define UART_ISOC_RX_PIN_NUMBER      8   // P0.08
#define UART_ISOC_TX_PIN_NUMBER      6   // P0.06
#define UART_ISOC_CTS_PIN_NUMBER     7   // P0.07
#define UART_ISOC_RTS_PIN_NUMBER     5   // P0.05

#define UART_GNSS_RX_PIN_NUMBER      42  // P1.10
#define UART_GNSS_TX_PIN_NUMBER      43  // P1.11
#define UART_GNSS_CTS_PIN_NUMBER     36  // P1.04
#define UART_GNSS_RTS_PIN_NUMBER     38  // P1.06
#endif

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
