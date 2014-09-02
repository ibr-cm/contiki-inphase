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
 *      AT86RF233 interface implementation
 * \author
 *      Yannic Schröder <yschroed@ibr.cs.tu-bs.de>
 *
 */

/**
 * \addtogroup avr_sensors
 * @{
 */

#include "at86rf233.h"

#include "radio/rf230bb/hal.h"
#include "dev/radio.h"
#include "radio/rf230bb/rf230bb.h"
#include "leds.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <string.h>

linkaddr_t target;
linkaddr_t initiator_requested;

enum fsm_states {
	IDLE,

	// Initiator States
	RANGING_REQUESTED,
	RESULT_REQUESTED,

	// Reflector States
	RANGING_ACCEPTED,
	WAIT_FOR_RESULT_REQ,
};

enum fsm_states fsm_state = IDLE;

uint8_t at86rf233_available(void) {
	if (RF233 == hal_register_read(RG_PART_NUM)) {
		return 1;
	}
	return 0; // AT86RF233 is not available
}

uint8_t at86rf233_init(void) {
	// initialize the measurement system

	// set up a network connection to communicate with other nodes
	AT86RF233_NETWORK.init();

	// initialize needed data structures

	return 0; // sensor successfully initialized
}

uint8_t at86rf233_deinit(void) {

	// close network connection
	AT86RF233_NETWORK.close();

	return 1; // sensor successfully deinitialized
}

uint8_t at86rf233_startRanging(void) {
	// start a range measurement

	frame_subframe_t reqframe;

	reqframe.range_request.ranging_method = RANGING_METHOD_PMU;
	reqframe.range_request.f0_start = 25000;
	reqframe.range_request.f0_stop = 28000;
	reqframe.range_request.f1_start = 0;
	reqframe.range_request.f1_stop = 0;
	reqframe.range_request.f2_start = 0;
	reqframe.range_request.f2_stop = 0;
	reqframe.range_request.f3_start = 0;
	reqframe.range_request.f3_stop = 0;
	reqframe.range_request.f4_start = 0;
	reqframe.range_request.f4_stop = 0;
	reqframe.range_request.f_step = 5;
	reqframe.range_request.capabilities = 0x00;

	fsm_state = IDLE;

	at86rf233_statemachine(RANGE_REQUEST_START, &reqframe);

	return 1;
}

uint8_t at86rf233_setTarget(linkaddr_t *addr) {
	printf("DISTANCE: Set target to %d.%d\n", addr->u8[0], addr->u8[1]); // debug message
	linkaddr_copy(&target, addr);
	return 1;
}

void at86rf233_statemachine(uint8_t frame_type, frame_subframe_t *frame) {

	printf("state: frame_type: %u, fsm_state: %u\n", frame_type, fsm_state);

	switch (fsm_state) {
	case IDLE:
	{
		if (frame_type == RANGE_REQUEST_START) {
			// TODO: check if ranging is possible

			// send RANGE_REQUEST
			frame_range_basic_t f;
			f.frame_type = RANGE_REQUEST;
			memcpy(&f.content.range_request, frame, sizeof(frame_range_request_t));
			AT86RF233_NETWORK.send(target, sizeof(frame_range_request_t)+1, &f);
			fsm_state = RANGING_REQUESTED;
		}
		else if (frame_type == RANGE_REQUEST) {
			// TODO: check if ranging is possible

			// TODO: save sent configuration data

			// send RANGE_ACCEPT
			frame_range_basic_t f;
			f.frame_type = RANGE_ACCEPT;
			f.content.range_accept.ranging_accept = RANGE_ACCEPT_STATUS_SUCCESS;
			f.content.range_accept.reject_reason = 0;
			f.content.range_accept.accepted_ranging_method = RANGING_METHOD_PMU;
			f.content.range_accept.accepted_capabilities = 0x00;
			AT86RF233_NETWORK.send(initiator_requested, sizeof(frame_range_accept_t)+1, &f);
			fsm_state = RANGING_ACCEPTED;
		} else {
			// all other frames are invalid here
		}
		break;
	}

	// initiator states
	case RANGING_REQUESTED:
	{
		if (frame_type == RANGE_ACCEPT) {
			// send TIME_SYNC_REQUEST
			frame_range_basic_t f;
			f.frame_type = TIME_SYNC_REQUEST;
			AT86RF233_NETWORK.send(target, 1, &f);

			at86rf233_pmuMagicInitiator();

			// send RESULT_REQUEST
			fsm_state = RESULT_REQUESTED;
		} else {
			// all other frames are invalid here
		}
		break;
	}
	case RESULT_REQUESTED:
	{
		if (frame_type == RESULT_CONFIRM) {
			// send RESULT_REQUEST
			fsm_state = RESULT_REQUESTED;
		} else {
			// all other frames are invalid here
		}
		break;
	}

	// reflector states
	case RANGING_ACCEPTED:
	{
		if (frame_type == TIME_SYNC_REQUEST) {
			at86rf233_pmuMagicReflector();
			fsm_state = WAIT_FOR_RESULT_REQ;
		} else {
			// all other frames are invalid here
		}
		break;
	}
	case WAIT_FOR_RESULT_REQ:
	{
		if (frame_type == RESULT_REQUEST) {
			// send RESULT_CONFIRM
			fsm_state = WAIT_FOR_RESULT_REQ; // wait for more result requests
		} else {
			// all other frames are invalid here
		}
		break;
	}

	default:
		break;
	}
}

void at86rf233_input(const linkaddr_t *src, uint16_t msg_len, void *msg) {
	frame_range_basic_t *frame_basic = msg;
	uint8_t msg_accepted = 0;
	switch (frame_basic->frame_type) {
	case RANGE_REQUEST:
	{
		if (msg_len == sizeof(frame_range_request_t)+1) {
			// correct message length
			msg_accepted = 1;
			linkaddr_copy(&initiator_requested, src);
		}
		break;
	}
	case RANGE_ACCEPT:
	{
		if (msg_len == sizeof(frame_range_accept_t)+1) {
			// correct message length
			msg_accepted = 1;
		}
		break;
	}
	case TIME_SYNC_REQUEST:
	{
		if (msg_len == 1) {
			// correct message length
			msg_accepted = 1;
		}
		break;
	}
	case PMU_START:
	{
		if (msg_len == 1) {
			// correct message length
			msg_accepted = 1;
		}
		break;
	}
	case RESULT_REQUEST:
	{
		if (msg_len == sizeof(frame_result_request_t)+1) {
			// correct message length
			msg_accepted = 1;
		}
		break;
	}
	case RESULT_CONFIRM:
	{
		frame_result_confirm_t *frame_result = &frame_basic->content.result_confirm;
		if (msg_len == (frame_result->result_length + 3)) {
			// correct message length
			msg_accepted = 1;
		}
		break;
	}
	default:
		// message type unknown!
		break;
	}
	if (msg_accepted) {
		at86rf233_statemachine(frame_basic->frame_type, &frame_basic->content);
	}

	printf("message received!\n");
}

void at86rf233_pmuMagicInitiator() {
	printf("entered PMU Initiator\n");

	cli();
	watchdog_stop();

	hal_subregister_write(SR_TX_PWR, 0xF);			// set TX output power to -17dBm to avoid reflections
	hal_register_read(RG_IRQ_STATUS);				// clear all pending interrupts
	hal_subregister_write(SR_ARET_TX_TS_EN, 0x1);	// signal frame transmission via DIG2
	hal_subregister_write(SR_IRQ_2_EXT_EN, 0x1);	// enable time stamping via DIG2

	// wait for DIG2
	leds_on(2);
	while (PINB & PB0 == 0) {}
	leds_off(2);

	while (1) {
		leds_on(1);
		_delay_ms(1000);
		leds_off(1);
		_delay_ms(1000);
	}

	watchdog_start();
	sei();
}

void at86rf233_pmuMagicReflector() {
	printf("entered PMU Reflector\n");

	cli();
	watchdog_stop();

	hal_subregister_write(SR_TX_PWR, 0xF);			// set TX output power to -17dBm to avoid reflections
	hal_register_read(RG_IRQ_STATUS);				// clear all pending interrupts
	hal_subregister_write(SR_ARET_TX_TS_EN, 0x1);	// signal frame transmission via DIG2
	hal_subregister_write(SR_IRQ_2_EXT_EN, 0x1);	// enable time stamping via DIG2
	hal_subregister_write(SR_TOM_EN, 0x0);			// disable TOM mode (unclear why this is done here)

	// send PMU start
	frame_range_basic_t f;
	f.frame_type = PMU_START;
	AT86RF233_NETWORK.send(target, 1, &f);

	// wait for DIG2
	leds_on(2);
	while (PINB & PB0 == 0) {}
	leds_off(2);

	while (1) {
		leds_on(1);
		_delay_ms(1000);
		leds_off(1);
		_delay_ms(1000);
	}

	watchdog_start();
	sei();
}
