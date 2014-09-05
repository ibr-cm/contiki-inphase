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
#include "net/packetbuf.h"
#include "dev/watchdog.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <string.h>

#define PMU_SAMPLES 4	// number of samples that are taken for each frequency by both nodes
#define PMU_MEASUREMENTS 200 // number of frequencies to measure
#define PMU_STEP 1 // frequency step for measurement

#define AT86RF233_ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )
#define AT86RF233_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}

linkaddr_t target;
linkaddr_t initiator_requested;

uint8_t local_pmu_values[PMU_MEASUREMENTS*PMU_SAMPLES];
uint8_t remote_pmu_values[PMU_MEASUREMENTS*PMU_SAMPLES];

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
			frame_range_basic_t f2;
			f2.frame_type = RESULT_REQUEST;
			f2.content.result_request.result_data_type = RESULT_TYPE_PMU;
			f2.content.result_request.result_start_address = 0;
			AT86RF233_NETWORK.send(target, sizeof(frame_result_request_t)+1, &f2);
			fsm_state = RESULT_REQUESTED;
		} else {
			// all other frames are invalid here
		}
		break;
	}
	case RESULT_REQUESTED:
	{
		if (frame_type == RESULT_CONFIRM) {
			// get last results from frame
			uint16_t last_start = frame->result_confirm.result_start_address;
			uint16_t result_length = frame->result_confirm.result_length;
			uint16_t i;
			for (i = 0; i < result_length; i++) {
				remote_pmu_values[i+last_start] = frame->result_confirm.result_data[i];
			}

			uint16_t next_start = last_start + result_length;

			// send RESULT_REQUEST if more results are needed
			frame_range_basic_t f;
			f.frame_type = RESULT_REQUEST;
			f.content.result_request.result_data_type = RESULT_TYPE_PMU;
			f.content.result_request.result_start_address = next_start;
			AT86RF233_NETWORK.send(target, sizeof(frame_result_request_t)+1, &f);
			if (next_start < PMU_MEASUREMENTS*PMU_SAMPLES) {
				fsm_state = RESULT_REQUESTED;
			} else {
				fsm_state = IDLE;
			}
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
			if (frame->result_request.result_data_type == RESULT_TYPE_PMU) {
				// send a pmu result frame
				uint16_t start_address = frame->result_request.result_start_address;
				if (start_address >= PMU_MEASUREMENTS*PMU_SAMPLES) {
					// start address points outside of the pmu data, this indicates that the initiator does not need more data
					fsm_state = IDLE;
				} else {
					uint8_t result_length;
					if (((PMU_MEASUREMENTS*PMU_SAMPLES) - start_address) > RESULT_DATA_LENGTH) {
						result_length = RESULT_DATA_LENGTH;
					} else {
						result_length = (PMU_MEASUREMENTS*PMU_SAMPLES) - start_address;
					}

					frame_range_basic_t f;
					f.frame_type = RESULT_CONFIRM;
					f.content.result_confirm.result_data_type = RESULT_TYPE_PMU;
					f.content.result_confirm.result_start_address = start_address;
					f.content.result_confirm.result_length = result_length;
					memcpy(&f.content.result_confirm.result_data, local_pmu_values[start_address], result_length);
					AT86RF233_NETWORK.send(initiator_requested, result_length+5, &f);

					fsm_state = WAIT_FOR_RESULT_REQ; // wait for more result requests
				}
			} else {
				// this can be an RSSI result...
				fsm_state = IDLE; // no other results allowed, return to idle
			}
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
	printf("AT86RF233_input: message received!\n");
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
		if (msg_len == (frame_result->result_length + 5)) {
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
}

/*---------------------------------------------------------------------------*/
/* These functions are used to control TIMER2. The TIMER2 is clocked by an
 * external 32.768 kHz crystal. By dividing the instructions needed to do a
 * measurement into smaller chunks, we are able to wait for the timer to expire
 * before the next chunk is executed. This way we can guarantee that both nodes
 * are always at the same point in their code without the need to synchronize
 * the clocks of the CPUs. Every time wait_for_timer2() returns both nodes are
 * at the same point in time. */

uint8_t ASSR_s, TIMSK2_s, OCR2A_s, OCR2B_s, TCCR2A_s, TCCR2B_s;

void init_timer2(void)
{
	// save TIMER2 configuration
	ASSR_s = ASSR;
	TIMSK2_s = TIMSK2;
	OCR2A_s = OCR2A;
	OCR2B_s = OCR2B;
	TCCR2A_s = TCCR2A;
	TCCR2B_s = TCCR2B;

	// setup TIMER2 to run at 32.768 kHz from external oscillator
	ASSR = 0b00100000; 	// asynchronous clock
	TIMSK2 = 0x00;		// disable interrupts for timer
	TIFR2 = 0xFF;		// clear flags
	TCNT2 = 0x00;		// set counter to 0
	TCCR2A = 0b00000010;// CTC mode, TOP = OCR2A
	TCCR2B = 0;			// default, stop timer
}

void restore_timer2(void)
{
	ASSR = ASSR_s;
	TIMSK2 = TIMSK2_s;
	OCR2A = OCR2A_s;
	OCR2B = OCR2B_s;
	TCCR2A = TCCR2A_s;
	TCCR2B = TCCR2B_s;
}

void start_timer2(uint8_t max) {
	TCCR2B = 0;			// ensure that the timer is stopped, otherwise setting TCNT2 may go wrong!
	TCNT2 = 0;
	OCR2A = max;
	TIFR2 = 0xFF;		// clear flags
	TCCR2B = 0x01;		// default, prescaler 1x
}

void wait_for_timer2(uint8_t id) {
	if (TIFR2 & (1 << OCF2A)) {
		printf("ERROR on id %u: TIMER2 compare match missed, timing can be corrupted!\n", id);
	}
	while (!(TIFR2 & (1 << OCF2A))){}	// wait for compare match
	TIFR2 = 0xFF;
	leds_toggle(1);						// show that the timer ran out (debugging on oscilloscope)
}
/*---------------------------------------------------------------------------*/

int8_t wait_for_dig2(void) {
	// wait for DIG2
	leds_on(2);
	uint16_t cnt0 = 1;	// this counts up while waiting for the signal, when it overflows the measurement is aborted
	uint16_t cnt1 = 1;	// this counts up while waiting for the signal, when it overflows the measurement is aborted
	while ((PINB & (1<<PB0)) == 0) {	// TODO: define this input pin in the platform
		cnt0++;
		if (cnt0 == 0) {
			return -1;	// signal never went low, abort measurement
		}
	}
	while ((PINB & (1<<PB0)) == 1) {	// wait for falling edge, these are closer together than the rising edges
		cnt1++;
		if (cnt1 == 0) {
			return -1;	// signal never went high, abort measurement
		}
	}
	start_timer2(18);					// timer counts to 18, we have 580us between synchronization points
	leds_off(2);

	return 0;	// everything worked as expected
}

uint8_t rf230_state;
uint8_t rg_phy_tx_pwr;
uint8_t rg_xah_ctrl_1;
uint8_t rg_trx_ctrl_1;
uint8_t rg_tst_sdm;
uint8_t rg_trx_ctrl_0;
uint8_t rg_rx_syn;
uint8_t rg_tst_agc;
uint8_t rg_cc_ctrl_1;
uint8_t rg_cc_ctrl_0;

void backup_registers(void) {
	// backup radio state
	rf230_state = radio_get_trx_state();

	rg_phy_tx_pwr = hal_register_read(RG_PHY_TX_PWR);
	rg_xah_ctrl_1 = hal_register_read(RG_XAH_CTRL_1);
	rg_trx_ctrl_1 = hal_register_read(RG_TRX_CTRL_1);
	rg_tst_sdm = hal_register_read(RG_TST_SDM);
	rg_trx_ctrl_0 = hal_register_read(RG_TRX_CTRL_0);
	rg_rx_syn = hal_register_read(RG_RX_SYN);
	rg_tst_agc = hal_register_read(RG_TST_AGC);
	rg_cc_ctrl_1 = hal_register_read(RG_CC_CTRL_1);
	rg_cc_ctrl_0 = hal_register_read(RG_CC_CTRL_0);
}

void restore_registers(void) {
	// restore radio state
	hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
	hal_subregister_write(SR_TRX_CMD, rf230_state);

	hal_register_write(RG_PHY_TX_PWR, rg_phy_tx_pwr);
	hal_register_write(RG_XAH_CTRL_1, rg_xah_ctrl_1);
	hal_register_write(RG_TRX_CTRL_1, rg_trx_ctrl_1);
	hal_register_write(RG_TST_SDM, rg_tst_sdm);
	hal_register_write(RG_TRX_CTRL_0, rg_trx_ctrl_0);
	hal_register_write(RG_RX_SYN, rg_rx_syn);
	hal_register_write(RG_TST_AGC, rg_tst_agc);
	hal_register_write(RG_CC_CTRL_1, rg_cc_ctrl_1);
	hal_register_write(RG_CC_CTRL_0, rg_cc_ctrl_0);

	hal_register_read(RG_IRQ_STATUS);				// clear all pending interrupts
}

void restore_initial_status(void) {
	// restore the system and run normally again
	restore_timer2();
	restore_registers();		// reset and initialize the radio
	watchdog_start();
}

// set the frequency in MHz to send on
// if offset == 1, the frequency is 0.5 Mhz higher
// frequency must be between 2322 MHz and 2527 MHz
void at86rf233_setFrequency(uint16_t f, uint8_t offset) {
	if (f < 2322) {
		// frequency is not supported
	} else if (f < 2434) {
		// CC_BAND is 0x8
		hal_register_write(RG_CC_CTRL_1, 0x08);
		f -= 2306;			// f is now between 0x10 and 0x7F
	} else if (f < 2528) {
		// CC_BAND is 0x9
		hal_register_write(RG_CC_CTRL_1, 0x09);
		f -= 2434;			// f is now between 0x00 and 0x5D
	} else {
		// frequency is not supported
	}
	f = f << 1;				// f is now between 0x00 and 0xFE
	if (offset) {
		f += 1;				// f is chosen 0.5 MHz higher (0x01 to 0xFF)
	}

	hal_register_write(RG_CC_CTRL_0, (uint8_t)f);
}

void at86rf233_senderPMU(void) {
	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
	hal_register_write(RG_TRX_STATE, CMD_TX_START);
	_delay_us(60 + 15 * PMU_SAMPLES);	// wait for receiver to measure
}

void at86rf233_receiverPMU(uint8_t* pmu_values) {
	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
	hal_register_write(RG_TRX_STATE, CMD_RX_ON);

	_delay_us(50); // wait for sender to be ready

	uint8_t i;

	for (i = 0; i < PMU_SAMPLES; i++) {
		pmu_values[i] = hal_register_read(RG_PHY_PMU_VALUE);
		//_delay_us(8); // value is updated every 8us but we are slower with reading the value anyway
	}
}

void at86rf233_pmuMagicInitiator() {
	printf("entered PMU Initiator\n");

	leds_off(2);
	leds_off(1);

	AT86RF233_ENTER_CRITICAL_REGION();
	watchdog_stop();
	backup_registers();

	init_timer2();

	hal_subregister_write(SR_TX_PWR, 0xF);			// set TX output power to -17dBm to avoid reflections
	hal_register_read(RG_IRQ_STATUS);				// clear all pending interrupts
	hal_subregister_write(SR_ARET_TX_TS_EN, 0x1);	// signal frame transmission via DIG2
	hal_subregister_write(SR_IRQ_2_EXT_EN, 0x1);	// enable time stamping via DIG2

	// wait for sync signal
	wait_for_dig2();

	// now in sync with the reflector

	hal_subregister_write(SR_TX_RX, 0);			// RX PLL frequency is selected
	hal_subregister_write(SR_RX_PDT_DIS, 1);	// RX Path is disabled

	hal_subregister_write(SR_PMU_EN, 1);		// enable PMU
	hal_subregister_write(SR_MOD_SEL, 1);		// manual control of modulation data
	hal_subregister_write(SR_MOD, 0);			// continuous 0 chips for modulation
	hal_subregister_write(SR_TX_RX_SEL, 1);		// manual control of PLL frequency mode

	wait_for_timer2(1);

	// TODO: maybe add a function to HAL that writes only zeros to FB to save memory?
	uint8_t fb_data[127] = {0};					// setup a framebuffer with all zeroes
	// TODO: setting the FB to zeros does not seem toimpact the measurement, maybe remove it completely?
	hal_frame_write(fb_data, 127);				// copy data to sram, framebuffer section

	// antenna diversity control is skipped, we only have one antenna

	wait_for_timer2(2);

	// measure RSSI
	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
	hal_register_write(RG_TRX_STATE, CMD_RX_ON);

	_delay_us(50); // wait some time for sender to be ready...

	uint8_t rssi = hal_subregister_read(SR_RSSI);
	hal_register_write(RG_TST_AGC, 0x09);		// TODO: set gain according to rssi

	wait_for_timer2(3);

	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
	hal_register_write(RG_TRX_STATE, CMD_TX_START);

	wait_for_timer2(4);
	start_timer2(10);					// timer counts to 10, we have 336us between synchronization points

	uint8_t i;
	for (i=0; i < PMU_MEASUREMENTS; i++) {
		at86rf233_setFrequency(2322 + (i * PMU_STEP), 0);
		at86rf233_receiverPMU(&local_pmu_values[i*PMU_SAMPLES]);
		at86rf233_senderPMU();
		wait_for_timer2(5);
	}

//	wait_for_timer2(6);
//
//	printf("rssi: %u\n", rssi);
//
//	uint8_t j;
//	for (j = 0; j < PMU_MEASUREMENTS; j++) {
//		for (i = 0; i < PMU_SAMPLES; i++) {
//			printf("pmu[%u][%u]: %u\n", j, i, local_pmu_values[j][i]);
//		}
//	}

	restore_initial_status();
	AT86RF233_LEAVE_CRITICAL_REGION();
}

void at86rf233_pmuMagicReflector() {
	printf("entered PMU Reflector\n");

	leds_off(2);
	leds_off(1);

	AT86RF233_ENTER_CRITICAL_REGION();
	watchdog_stop();
	backup_registers();

	init_timer2();

	hal_subregister_write(SR_TX_PWR, 0xF);			// set TX output power to -17dBm to avoid reflections
	hal_register_read(RG_IRQ_STATUS);				// clear all pending interrupts
	hal_subregister_write(SR_ARET_TX_TS_EN, 0x1);	// signal frame transmission via DIG2
	hal_subregister_write(SR_IRQ_2_EXT_EN, 0x1);	// enable time stamping via DIG2
	hal_subregister_write(SR_TOM_EN, 0x0);			// disable TOM mode (unclear why this is done here)

	// send PMU start
	frame_range_basic_t f;
	f.frame_type = PMU_START;
	//AT86RF233_NETWORK.send(initiator_requested, 1, &f);

	// send PMU start on bare metal, as the normal protocol stack is too slow for us to be able to see the DIG2 signal
	//packetbuf_copyfrom(&f, 1);
	//packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &initiator_requested);
	//packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
	//packetbuf_compact();
	hal_subregister_write(SR_TRX_CMD, CMD_TX_ARET_ON);
	hal_set_slptr_low();
	hal_set_slptr_high(); // send the packet at the latest time possible


	// wait for sync signal
	wait_for_dig2();

	// now in sync with the initiator

	hal_subregister_write(SR_PMU_IF_INVERSE, 1);// Inverse IF position
	hal_subregister_write(SR_TX_RX, 1);			// TX PLL frequency is selected
	hal_subregister_write(SR_RX_PDT_DIS, 1);	// RX Path is disabled

	hal_subregister_write(SR_PMU_EN, 1);		// enable PMU
	hal_subregister_write(SR_MOD_SEL, 1);		// manual control of modulation data
	hal_subregister_write(SR_MOD, 0);			// continuous 0 chips for modulation
	hal_subregister_write(SR_TX_RX_SEL, 1);		// manual control of PLL frequency mode

	wait_for_timer2(1);

	// TODO: maybe add a function to HAL that writes only zeros to FB to save memory?
	uint8_t fb_data[127] = {0};					// setup a framebuffer with all zeroes
	// TODO: setting the FB to zeros does not seem toimpact the measurement, maybe remove it completely?
	hal_frame_write(fb_data, 127);				// copy data to sram, framebuffer section

	// antenna diversity control is skipped, we only have one antenna

	wait_for_timer2(2);

	// measure RSSI
	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
	hal_register_write(RG_TRX_STATE, CMD_TX_START);

	wait_for_timer2(3);

	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
	hal_register_write(RG_TRX_STATE, CMD_RX_ON);

	_delay_us(50); // wait some time for sender to be ready...

	uint8_t rssi = hal_subregister_read(SR_RSSI);
	hal_register_write(RG_TST_AGC, 0x09);		// TODO: set gain according to rssi

	wait_for_timer2(4);
	start_timer2(10);					// timer counts to 10, we have 336us between synchronization points

	uint8_t i;
	for (i=0; i < PMU_MEASUREMENTS; i++) {
		at86rf233_setFrequency(2322 + (i * PMU_STEP), 1);
		at86rf233_senderPMU();
		at86rf233_receiverPMU(&local_pmu_values[i*PMU_SAMPLES]);
		wait_for_timer2(5);
	}

//	wait_for_timer2(6);
//
//	printf("rssi: %u\n", rssi);
//
//	uint8_t j;
//	for (j = 0; j < PMU_MEASUREMENTS; j++) {
//		for (i = 0; i < PMU_SAMPLES; i++) {
//			printf("pmu[%u][%u]: %u\n", j, i, local_pmu_values[j][i]);
//		}
//	}

	restore_initial_status();
	AT86RF233_LEAVE_CRITICAL_REGION();
}
