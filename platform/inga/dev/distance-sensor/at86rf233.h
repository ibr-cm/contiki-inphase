/*
 * Copyright (c) 2014, TU Braunschweig.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *      AT86RF233 interface definition
 * \author
 *      Yannic Schröder <yschroed@ibr.cs.tu-bs.de>
 */

/**
 * \addtogroup avr_sensors
 * @{
 */

/**
 * \defgroup avr_radio_driver AT86RF233 interface
 *
 * This interface allows to measure the distance between AT86RF233 radios.
 *
 */

#ifndef __AT86RF233_H__
#define __AT86RF233_H__

#include "stdint.h"
#include "at86rf233-network.h"
#include "distance-sensor.h"

#ifndef AT86RF233_NETWORK
#ifdef AT86RF233_CONF_NETWORK
#define AT86RF233_NETWORK AT86RF233_CONF_NETWORK
#else /* AT86RF233_CONF_NETWORK */
#define AT86RF233_NETWORK at86rf233_rime_network
#endif /* AT86RF233_CONF_NETWORK */
#endif /* AT86RF233_NETWORK */

#define RANGE_REQUEST_START      0x00 // this is a normal range request packet, but the one the initiator should send to the reflector
#define RANGE_REQUEST            0x01
#define RANGE_ACCEPT             0x02
#define TIME_SYNC_REQUEST        0x11
#define PMU_START                0x12
#define RESULT_REQUEST           0x21
#define RESULT_CONFIRM           0x22

#define RANGING_METHOD_PMU       0x01

#define RANGE_ACCEPT_STATUS_SUCCESS 0x10
#define RANGE_ACCEPT_STATUS_REJECT  0x12

#define REJECT_REASON_NORF233    0x01 // node has no compatible radio
#define REJECT_REASON_BUSY       0x02 // node is currently busy with other ranging request
#define REJECT_REASON_NOTALLOWED 0x03 // node is blocking ranging due to data transmissions
#define REJECT_REASON_NOINIT     0x04 // ranging system is not initialized by user

#define RESULT_TYPE_PMU          0x00
#define RESULT_TYPE_RSSI         0x01

#define RESULT_DATA_LENGTH       100  // number of byte to send in one result frame

typedef struct {
	uint8_t  ranging_method;
	uint16_t f_start[DISTANCE_FREQUENCY_BANDS];
	uint16_t f_stop[DISTANCE_FREQUENCY_BANDS];
	uint8_t  f_step;
	uint8_t  capabilities;
} frame_range_request_t;

typedef struct {
	uint8_t ranging_accept;
	uint8_t reject_reason;
	uint8_t accepted_ranging_method;
	uint8_t accepted_capabilities;
} frame_range_accept_t;

typedef struct {
	uint8_t result_data_type;
	uint16_t result_start_address;
} frame_result_request_t;

typedef struct {
	uint8_t  result_data_type;
	uint16_t result_start_address;
	uint8_t result_length;
	uint8_t result_data[RESULT_DATA_LENGTH];
} frame_result_confirm_t;

typedef union {
	frame_range_request_t range_request;
	frame_range_accept_t range_accept;
	frame_result_request_t result_request;
	frame_result_confirm_t result_confirm;
} frame_subframe_t;

typedef struct {
	uint8_t frame_type;
	frame_subframe_t content;
} frame_range_basic_t;

extern const struct at86rf233_network AT86RF233_NETWORK;

uint8_t at86rf233_available(void);

uint8_t at86rf233_init(void);

uint8_t at86rf233_deinit(void);

int8_t at86rf233_get_status();

uint8_t at86rf233_start_ranging(void);

uint8_t at86rf233_set_target(linkaddr_t* addr);

int8_t at86rf233_set_frequencies(frequency_bands_t *f);

uint8_t at86rf233_set_fstep(uint8_t fstep);

uint8_t at86rf233_set_raw_output(uint8_t raw);

/**
 * \brief coap_input
 *        this function must be called by the network convergence layer for all incoming packages
 * \param src pointer to source node
 * \param netstack pointer to netstack which is calling this function
 * \param msg_len length of the incoming package
 * \param msg_type type of the incoming message
 * \param msg pointer to the message
 */
void at86rf233_input(const linkaddr_t* src, uint16_t msg_len, void *msg);

#endif /* __AT86RF233_H__ */

/** @} */ // avr_radio_driver
