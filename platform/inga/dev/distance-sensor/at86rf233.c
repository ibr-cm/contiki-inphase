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

#include "contiki.h"

#include "at86rf233.h"
#include "../distance-sensor.h"
#include "fft/ffft.h"

#include "radio/rf230bb/hal.h"
#include "dev/radio.h"
#include "radio/rf230bb/rf230bb.h"
#include "leds.h"
#include "net/packetbuf.h"
#include "dev/watchdog.h"
#include "platform-conf.h"
#include "binary-uart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include <stdio.h>
#include <string.h>
#include <stdfix.h>

#define PMU_MINIMUM_FREQUENCY 2322		// minimum frequency from AT86RF233 data sheet
#define PMU_MAXIMUM_FREQUENCY 2527		// maximum frequency from AT86RF233 data sheet (maybe 2543.5 MHz will work)

#define PMU_START_FREQUENCY 2322 // start frequency for measurement
#define PMU_SAMPLES 4	// number of samples that are taken for each frequency by both nodes
#define PMU_MEASUREMENTS 200 // number of frequencies to measure
#define PMU_STEP 1 // frequency step for measurement

// special symbols for serial output of sensor data
#define SERIAL_FRAME_START     0x3C
#define SERIAL_FRAME_END       0x3E
#define SERIAL_ESCAPE_BYTE     0x40
#define SERIAL_ESCAPE_ADD      0x10

#define AT86RF233_ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )
#define AT86RF233_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}

#define DEBUG 0
#if DEBUG
#define PRINTF(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#else
#define PRINTF(...)
#endif

#ifndef DIG2_PIN
#error "DIG2_PIN not defined, please define it in platform-conf.h"
#endif

#ifndef DIG2_PIN_BIT
#error "DIG2_PIN_BIT not defined, please define it in platform-conf.h"
#endif

PROCESS(ranging_process, "AT86RF233 Ranging process");

static int8_t pmu_magic(uint8_t type);

uint8_t status_code;

struct {
	linkaddr_t reflector;						// address of the reflector
	linkaddr_t initiator;						// address of the initiator that issued the current measurement
	uint16_t f_start[DISTANCE_FREQUENCY_BANDS];	// frequency bands to measure
	uint16_t f_stop[DISTANCE_FREQUENCY_BANDS];  // data sheet says 2322 - 2527 MHz (it seems up to 2543 MHz will work)
	uint8_t f_step;								// frequency step in MHz, 0 = 500 kHz
	uint8_t raw_output;							// if 1, output PMU data to serial port
} settings;

// TODO: allocate these buffers at runtime depending on settings (Contiki mmem)
// allocate two arrays with 824 bytes each, this works for full spectrum at 1 MHz steps
uint8_t local_pmu_values[(PMU_MAXIMUM_FREQUENCY-PMU_MINIMUM_FREQUENCY+1)*PMU_SAMPLES];
// TODO: only allocate this buffer when running as initiator to store received values
uint8_t remote_pmu_values[(PMU_MAXIMUM_FREQUENCY-PMU_MINIMUM_FREQUENCY+1)*PMU_SAMPLES];

#define MEASUREMENT_TIMEOUT (0.5 * CLOCK_SECOND)
struct ctimer timeout_timer;  	// if this timer runs out, the measurement has
								// failed due to a timeout in the network

uint8_t next_status_code;

enum fsm_states {
	IDLE,

	// Initiator States
	RANGING_REQUESTED,
	RESULT_REQUESTED,

	// Reflector States
	RANGING_ACCEPTED,
	WAIT_FOR_RESULT_REQ,
};

volatile enum fsm_states fsm_state = IDLE;

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
	linkaddr_copy(&settings.reflector, &linkaddr_null); // invalidate reflector address
	linkaddr_copy(&settings.initiator, &linkaddr_null); // invalidate initiator_requested address

	// TODO: init some fool proof settings here that measure rather okayish...

	// init frequency bands
	uint8_t i;
	for (i = 0; i < DISTANCE_FREQUENCY_BANDS; i++) {
		settings.f_start[i] = 0;
		settings.f_stop[i] = 0;
	}
	settings.f_step = 1;

	settings.raw_output = 0;

	if (!at86rf233_available()) {
		status_code = DISTANCE_NO_RF233;
	} else {
		status_code = DISTANCE_IDLE;
	}

	return 0; // sensor successfully initialized
}

uint8_t at86rf233_deinit(void) {

	// close network connection
	AT86RF233_NETWORK.close();

	return 1; // sensor successfully deinitialized
}

int8_t at86rf233_get_status() {
	return status_code;
}

uint8_t at86rf233_start_ranging(void) {

	if (process_is_running(&ranging_process)) {
		return 1;
	}

	// ranging process is not running, ranging is possible
	process_start(&ranging_process, NULL);
	return 0;
}

uint8_t at86rf233_set_target(linkaddr_t *addr) {
	PRINTF("DISTANCE: Set reflector to %d.%d\n", addr->u8[0], addr->u8[1]); // debug message
	linkaddr_copy(&settings.reflector, addr);
	return 1;
}

int8_t at86rf233_set_frequencies(frequency_bands_t *f) {
	// check frequency bands

	uint8_t i;
	// check if all frequencies are in the defined range
	for (i = 0; i < DISTANCE_FREQUENCY_BANDS; i++) {
		if (f->f_start[i] == 0) {	// 0 is allowed
			continue;
		}
		if (f->f_stop[i] == 0) {	// 0 is allowed
			continue;
		}

		if (f->f_start[i] < PMU_MINIMUM_FREQUENCY) {
			PRINTF("DISTANCE: frequency failed[%d]: 1\n", i);
			return 1; // error!
		}

		if (f->f_start[i] > PMU_MAXIMUM_FREQUENCY) {
			PRINTF("DISTANCE: frequency failed[%d]: 2\n", i);
			return 1; // error!
		}

		if (f->f_stop[i] < PMU_MINIMUM_FREQUENCY) {
			PRINTF("DISTANCE: frequency failed[%d]: 3\n", i);
			return 1; // error!
		}

		if (f->f_stop[i] > PMU_MAXIMUM_FREQUENCY) {
			PRINTF("DISTANCE: frequency failed[%d]: 4\n", i);
			return 1; // error!
		}
	}


	// first frequency band can never be 0
	if (f->f_start[0] == 0) {
		PRINTF("DISTANCE: frequency failed[%d]: 5\n", 0);
		return 1; // error!
	}
	if (f->f_stop[0] == 0) {
		PRINTF("DISTANCE: frequency failed[%d]: 6\n", 0);
		return 1; // error!
	}

	// it is allowed that frequency bands are 0 but all following ones must be 0 too
	uint8_t zero_detected = 0;
	for (i = 1; i < DISTANCE_FREQUENCY_BANDS; i++) {
		if (zero_detected) { // everything must be 0 now
			if (f->f_start[i]) {
				PRINTF("DISTANCE: frequency failed[%d]: 7\n", i);
				return 1; // error!
			}
			if (f->f_stop[i]) {
				PRINTF("DISTANCE: frequency failed[%d]: 8\n", i);
				return 1; // error!
			}
		} else {			// find first 0
			if (f->f_start[i] == 0) {
				zero_detected = 1;
				if (f->f_stop[i]) {
					PRINTF("DISTANCE: frequency failed[%d]: 9\n", i);
					return 1; // error!
				}
			}
		}
	}

	// every start of a frequency band must be greater than the end of the last one
	for (i = 1; i < DISTANCE_FREQUENCY_BANDS; i++) {
		if (f->f_start[i] == 0) {
			break;
		}
		if (f->f_stop[i-1] > f->f_start[i]) {
			PRINTF("DISTANCE: frequency failed[%d]: 10\n", i);
			return 1; // error!
		}
	}

	// every stop must be greater or equal to the start frequency
	for (i = 0; i < DISTANCE_FREQUENCY_BANDS; i++) {
		if (f->f_start[i] == 0) {
			break;
		}
		if (f->f_stop[i] < f->f_start[i]) {
			PRINTF("DISTANCE: frequency failed[%d]: 11\n", i);
			return 1; // error!
		}
	}

	// well done, the frequency settings are valid!

	// copy frequency bands
	for (i = 0; i < DISTANCE_FREQUENCY_BANDS; i++) {
		settings.f_start[i] = f->f_start[i];
		settings.f_stop[i] = f->f_stop[i];

		PRINTF("DISTANCE: settings.f_start[%d]: %d, settings.f_stop[%d]: %d\n",
				i, settings.f_start[i], i, settings.f_stop[i]);
	}
	return 0;
}

uint8_t at86rf233_set_fstep(uint8_t fstep) {
	// TODO: check if steps are valid and return error
	// return 0 on error
	settings.f_step = fstep;
	PRINTF("DISTANCE: settings.f_step: %d\n", settings.f_step);
	return 1;
}

uint8_t at86rf233_set_raw_output(uint8_t raw) {
	if (raw > 0) {
		settings.raw_output = 1;
	} else {
		settings.raw_output = 0;
	}
	return 0;
}

// this gets call from the timeout_timer when it expires
static void reset_statemachine() {
	fsm_state = IDLE;
	status_code = next_status_code;
}

static void send_serial(void) {
    //rs232_send(RS232_PORT_0, SERIAL_FRAME_START);
	binary_start_frame();

    // send number of samples per frequency
    binary_send_byte(PMU_SAMPLES);

    // send step size
    binary_send_byte(PMU_STEP);

    // send number of frequency blocks
    binary_send_byte(DISTANCE_FREQUENCY_BANDS);

	// send frequency bands
	uint8_t i;
	for (i = 0; i < DISTANCE_FREQUENCY_BANDS; i++) {
		binary_send_short(settings.f_start[i]);
		binary_send_short(settings.f_stop[i]);
	}

    uint16_t j;
    for (j = 0; j < PMU_MEASUREMENTS*PMU_SAMPLES; j++)
    {
    	// do basic calculations to make transmission via serial faster
    	int16_t v = local_pmu_values[j]-remote_pmu_values[j];

    	if (v > 127) {
    		v -= 256;
    	} else if (v < -128) {
    		v += 256;
    	}

    	// transmit data
    	binary_send_byte((v & 0xFF));
    }

    //rs232_send(RS232_PORT_0, SERIAL_FRAME_END);
    binary_end_frame();
}

static void statemachine(uint8_t frame_type, frame_subframe_t *frame) {

	PRINTF("DISTANCE: state: frame_type: %u, fsm_state: %u\n", frame_type, fsm_state);

	_delay_us(25); // FIXME: this is some race condition

	switch (fsm_state) {
	case IDLE:
	{
		if (frame_type == RANGE_REQUEST_START) {
			// TODO: check if ranging is possible

			status_code = DISTANCE_RUNNING;
			next_status_code = DISTANCE_NO_REFLECTOR;
			ctimer_set(&timeout_timer, MEASUREMENT_TIMEOUT, reset_statemachine, NULL); // setup timeout_timer

			// send RANGE_REQUEST
			frame_range_basic_t f;
			f.frame_type = RANGE_REQUEST;
			memcpy(&f.content.range_request, frame, sizeof(frame_range_request_t));
			AT86RF233_NETWORK.send(settings.reflector, sizeof(frame_range_request_t)+1, &f);
			fsm_state = RANGING_REQUESTED;
		}
		else if (frame_type == RANGE_REQUEST) {
			// TODO: check if ranging is possible

			// TODO: save sent configuration data

			status_code = DISTANCE_RUNNING;
			next_status_code = DISTANCE_TIMEOUT;
			ctimer_set(&timeout_timer, MEASUREMENT_TIMEOUT, reset_statemachine, NULL); // setup timeout_timer

			// send RANGE_ACCEPT
			frame_range_basic_t f;
			f.frame_type = RANGE_ACCEPT;
			f.content.range_accept.ranging_accept = RANGE_ACCEPT_STATUS_SUCCESS;
			f.content.range_accept.reject_reason = 0;
			f.content.range_accept.accepted_ranging_method = RANGING_METHOD_PMU;
			f.content.range_accept.accepted_capabilities = 0x00;
			AT86RF233_NETWORK.send(settings.initiator, sizeof(frame_range_accept_t)+1, &f);
			fsm_state = RANGING_ACCEPTED;
		} else {
			// all other frames are invalid here
			status_code = DISTANCE_IDLE;
		}
		break;
	}

	// initiator states
	case RANGING_REQUESTED:
	{
		if (frame_type == RANGE_ACCEPT) {
			next_status_code = DISTANCE_TIMEOUT;
			ctimer_restart(&timeout_timer); // partner sent valid next frame, reset timer

			// send TIME_SYNC_REQUEST
			frame_range_basic_t f;
			f.frame_type = TIME_SYNC_REQUEST;
			AT86RF233_NETWORK.send(settings.reflector, 1, &f);

			ctimer_stop(&timeout_timer); // stop timer for pmu_magic
			if (pmu_magic(0)) {
				next_status_code = DISTANCE_NO_SYNC;
				reset_statemachine(); // DIG2 timed out, abort!

			} else {
				ctimer_restart(&timeout_timer); // magic finished, restart timer

				// send RESULT_REQUEST
				frame_range_basic_t f2;
				f2.frame_type = RESULT_REQUEST;
				f2.content.result_request.result_data_type = RESULT_TYPE_PMU;
				f2.content.result_request.result_start_address = 0;
				AT86RF233_NETWORK.send(settings.reflector, sizeof(frame_result_request_t)+1, &f2);
				fsm_state = RESULT_REQUESTED;
			}
		} else {
			// all other frames are invalid here
		}
		break;
	}
	case RESULT_REQUESTED:
	{
		if (frame_type == RESULT_CONFIRM) {
			next_status_code = DISTANCE_TIMEOUT;
			ctimer_restart(&timeout_timer); // partner sent valid next frame, reset timer

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
			AT86RF233_NETWORK.send(settings.reflector, sizeof(frame_result_request_t)+1, &f);
			if (next_start < PMU_MEASUREMENTS*PMU_SAMPLES) {
				fsm_state = RESULT_REQUESTED;
			} else {
				if (settings.raw_output) {
					send_serial(); // all results gathered, send via serial port
				}
				fsm_state = IDLE;
				status_code = DISTANCE_IDLE;
				ctimer_stop(&timeout_timer); // done, stop timer
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
			next_status_code = DISTANCE_TIMEOUT;
			ctimer_stop(&timeout_timer); // stop timer for pmu_magic
			if (pmu_magic(1)) {
				next_status_code = DISTANCE_NO_SYNC;
				reset_statemachine(); // DIG2 timed out, abort!
			} else {
				ctimer_restart(&timeout_timer); // magic finished, restart timer
				fsm_state = WAIT_FOR_RESULT_REQ;
			}
		} else {
			// all other frames are invalid here
		}
		break;
	}
	case WAIT_FOR_RESULT_REQ:
	{
		if (frame_type == RESULT_REQUEST) {
			next_status_code = DISTANCE_TIMEOUT;
			ctimer_restart(&timeout_timer); // partner sent valid next frame, reset timer

			if (frame->result_request.result_data_type == RESULT_TYPE_PMU) {
				// send a pmu result frame
				uint16_t start_address = frame->result_request.result_start_address;
				if (start_address >= PMU_MEASUREMENTS*PMU_SAMPLES) {
					// start address points outside of the pmu data, this indicates that the initiator does not need more data
					fsm_state = IDLE;
					status_code = DISTANCE_IDLE;
					ctimer_stop(&timeout_timer); // done, stop timer
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
					memcpy(&f.content.result_confirm.result_data, &local_pmu_values[start_address], result_length);
					AT86RF233_NETWORK.send(settings.initiator, result_length+5, &f);

					fsm_state = WAIT_FOR_RESULT_REQ; // wait for more result requests
				}
			} else {
				// this can be an RSSI result...
				fsm_state = IDLE; // no other results allowed, return to idle
				status_code = DISTANCE_IDLE;
				ctimer_stop(&timeout_timer); // done, stop timer
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
	PRINTF("DISTANCE: AT86RF233_input: message received!\n");
	frame_range_basic_t *frame_basic = msg;
	uint8_t msg_accepted = 0;
	switch (frame_basic->frame_type) {
	case RANGE_REQUEST:
	{
		if (msg_len == sizeof(frame_range_request_t)+1) {
			// correct message length
			msg_accepted = 1;
			linkaddr_copy(&settings.initiator, src);
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
		statemachine(frame_basic->frame_type, &frame_basic->content);
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

static void init_timer2(void)
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

static void restore_timer2(void)
{
	ASSR = ASSR_s;
	TIMSK2 = TIMSK2_s;
	OCR2A = OCR2A_s;
	OCR2B = OCR2B_s;
	TCCR2A = TCCR2A_s;
	TCCR2B = TCCR2B_s;
}

static void start_timer2(uint8_t max) {
	TCCR2B = 0;			// ensure that the timer is stopped, otherwise setting TCNT2 may go wrong!
	TCNT2 = 0;
	OCR2A = max;
	TIFR2 = 0xFF;		// clear flags
	TCCR2B = 0x01;		// default, prescaler 1x
}

static void wait_for_timer2(uint8_t id) {
	if (TIFR2 & (1 << OCF2A)) {
		printf_P(PSTR("DISTANCE: ERROR on id %u: TIMER2 compare match missed, timing can be corrupted!\n"), id);
	}
	while (!(TIFR2 & (1 << OCF2A))){}	// wait for compare match
	TIFR2 = 0xFF;
	leds_toggle(1);						// show that the timer ran out (debugging on oscilloscope)
}
/*---------------------------------------------------------------------------*/

static int8_t wait_for_dig2(void) {
	leds_on(2);
	leds_off(1);
	// wait for DIG2
	uint16_t cnt0 = 1;	// this counts up while waiting for the signal, when it overflows the measurement is aborted
	uint16_t cnt1 = 1;	// this counts up while waiting for the signal, when it overflows the measurement is aborted
	while ((DIG2_PIN & (1<<DIG2_PIN_BIT)) == 0) {
		cnt0++;
		if (cnt0 == 0) {
			leds_on(1);
			return -1;	// signal never went low, abort measurement
		}
	}
	while ((DIG2_PIN & (1<<DIG2_PIN_BIT)) == 1) {	// wait for falling edge, these are closer together than the rising edges
		cnt1++;
		if (cnt1 == 0) {
			return -1;	// signal never went high, abort measurement
		}
	}
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

static void backup_registers(void) {
	// backup radio state
	rf230_state = hal_subregister_read(SR_TRX_STATUS);

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

static void restore_registers(void) {
	// restore radio state
	hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);

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


	// TODO: set this to RX_AACK_ON or just RX_ON depending on previous state
	// the current state read out from the radio may be wrong though...
	//hal_subregister_write(SR_TRX_CMD, rf230_state);
	hal_subregister_write(SR_TRX_CMD, RX_AACK_ON);
}

static void restore_initial_status(void) {
	// restore the system and run normally again
	restore_timer2();
	restore_registers();		// reset and initialize the radio
	watchdog_start();
}

// set the frequency in MHz to send on
// if offset == 1, the frequency is 0.5 Mhz higher
// frequency must be between 2322 MHz and 2527 MHz
static void setFrequency(uint16_t f, uint8_t offset) {
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

static void sender_pmu(void) {
	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
	hal_register_write(RG_TRX_STATE, CMD_TX_START);
	_delay_us(60 + 15 * PMU_SAMPLES);	// wait for receiver to measure
}

static void receiver_pmu(uint8_t* pmu_values) {
	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
	hal_register_write(RG_TRX_STATE, CMD_RX_ON);

	_delay_us(50); // wait for sender to be ready

	uint8_t i;

	for (i = 0; i < PMU_SAMPLES; i++) {
		pmu_values[i] = hal_register_read(RG_PHY_PMU_VALUE);
		//_delay_us(8); // value is updated every 8us but we are slower with reading the value anyway
	}
}

// unified version of pmu magic
// type == 0: initiator
// else: reflector
static int8_t pmu_magic(uint8_t type) {
	int8_t ret_val;

	switch (type) {
	case 0:		// initiator
		PRINTF("DISTANCE: entered PMU Initiator\n");
		break;
	default:	// reflector
		PRINTF("DISTANCE: entered PMU Reflector\n");
		break;
	}

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

	// this line is normally only done at the reflector:
	hal_subregister_write(SR_TOM_EN, 0x0);			// disable TOM mode (unclear why this is done here)

	// reflector sends the synchronization frame
	if (type) {
		// send PMU start
		frame_range_basic_t f;
		f.frame_type = PMU_START;

		// TODO: send the correct frame here, just to sleep well...

		//AT86RF233_NETWORK.send(initiator_requested, 1, &f);

		// send PMU start on bare metal, as the normal protocol stack is too slow for us to be able to see the DIG2 signal
		//packetbuf_copyfrom(&f, 1);
		//packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &initiator_requested);
		//packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
		//packetbuf_compact();

		_delay_ms(2); // wait for initiator, it needs more time before it listens to DIG2

		hal_subregister_write(SR_TRX_CMD, CMD_TX_ARET_ON);
		hal_set_slptr_low();
		hal_set_slptr_high(); // send the packet at the latest time possible
	}

	// wait for sync signal
	if (!wait_for_dig2()) {
		// DIG2 signal not found, abort measurement
		// to be honest: if the reflector does not get the DIG2 from its own sending
		// there must be something horribly wrong...
		if (type) {
			_delay_us(9.5243);	// DIG2 signal is on average 9.5243 us delayed on the initiator, reflector waits
		}
		start_timer2(18);		// timer counts to 18, we have 580us between synchronization points
		leds_off(2);

		// now in sync with the other node

		switch (type) {
		case 0:		// initiator
			hal_subregister_write(SR_TX_RX, 0);				// RX PLL frequency is selected
			break;
		default:	// reflector
			hal_subregister_write(SR_PMU_IF_INVERSE, 1);	// Inverse IF position
			hal_subregister_write(SR_TX_RX, 1);				// TX PLL frequency is selected
			break;
		}

		hal_subregister_write(SR_RX_PDT_DIS, 1);	// RX Path is disabled
		hal_subregister_write(SR_PMU_EN, 1);		// enable PMU
		hal_subregister_write(SR_MOD_SEL, 1);		// manual control of modulation data
		hal_subregister_write(SR_MOD, 0);			// continuous 0 chips for modulation
		hal_subregister_write(SR_TX_RX_SEL, 1);		// manual control of PLL frequency mode


		wait_for_timer2(1);
#if 0    // don't do this, it seems to not affect the measurement
		// TODO: maybe add a function to HAL that writes only zeros to FB to save memory?
		uint8_t fb_data[127] = {0};					// setup a framebuffer with all zeroes
		// TODO: setting the FB to zeros does not seem toimpact the measurement, maybe remove it completely?
		hal_frame_write(fb_data, 127);				// copy data to sram, framebuffer section

		// antenna diversity control is skipped, we only have one antenna

		wait_for_timer2(2);
#endif
		// measure RSSI
		uint8_t rssi;
		hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
		switch (type) {
		case 0:		// initiator
			hal_register_write(RG_TRX_STATE, CMD_RX_ON);
			_delay_us(50); // wait some time for sender to be ready...
			rssi = hal_subregister_read(SR_RSSI);
			break;
		default:	// reflector
			hal_register_write(RG_TRX_STATE, CMD_TX_START);
			break;
		}

		wait_for_timer2(3);

		hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
		switch (type) {
		case 0:		// initiator
			hal_register_write(RG_TRX_STATE, CMD_TX_START);
			break;
		default:	// reflector
			hal_register_write(RG_TRX_STATE, CMD_RX_ON);
			_delay_us(50); // wait some time for sender to be ready...
			rssi = hal_subregister_read(SR_RSSI);
			break;
		}

		// TODO: set gain according to rssi
		hal_register_write(RG_TST_AGC, 0x09);

		wait_for_timer2(4);
		start_timer2(10);					// timer counts to 10, we have 336us between synchronization points

		uint8_t i;
		for (i=0; i < PMU_MEASUREMENTS; i++) {
			switch (type) {
			case 0:		// initiator
				setFrequency(2322 + (i * PMU_STEP), 0);
				receiver_pmu(&local_pmu_values[i*PMU_SAMPLES]);
				sender_pmu();
				break;
			default:	// reflector
				setFrequency(2322 + (i * PMU_STEP), 1);
				sender_pmu();
				receiver_pmu(&local_pmu_values[i*PMU_SAMPLES]);
				break;
			}
			wait_for_timer2(5);
		}
		ret_val = 0;
	} else {
		ret_val = -1; // DIG2 signal not seen, abort!
	}

	restore_initial_status();
	watchdog_start();
	AT86RF233_LEAVE_CRITICAL_REGION();

	return ret_val;
}

/*---------------------------------------------------------------------------*/
void autocorr(int8_t* in, int16_t* out, uint16_t length) {
	uint16_t i;
	for (i = 0; i < length; i++) { // for every output data element
		wdt_reset();
		int64_t fieldval = 0;
		uint16_t j;
		for (j = i; j < length; j++) { // for every remaining element in the input data
			int16_t x = (int16_t)in[j-i];
			int16_t y = (int16_t)in[j];
			fieldval += (x * y);
		}
		out[i] = fieldval / length; // fieldval must fit into int16_t
	}
}
/*---------------------------------------------------------------------------*/
/* This process manages a range measurement.
 * It is run at the initiator node and ensures that the measurement runs
 * asynchronously while the user process that started the ranging can do other tasks.
 *
 * NOTE: the processor is completely blocked when the actual ranging happens,
 * only during negotiation phases between initiator and reflector other processes
 * can run.
 */
PROCESS_THREAD(ranging_process, ev, data)
{
	PROCESS_BEGIN();

	// check if the reflector address is set
	if (linkaddr_cmp(&settings.reflector, &linkaddr_null)) {
		// no reflector address is set, ranging not possible
		return 0;
	}

	status_code = DISTANCE_RUNNING;

	// start a range measurement
	frame_subframe_t reqframe;

	reqframe.range_request.ranging_method = RANGING_METHOD_PMU;

	// copy frequency bands
	uint8_t i;
	for (i = 0; i < DISTANCE_FREQUENCY_BANDS; i++) {
		reqframe.range_request.f_start[i] = settings.f_start[i];
		reqframe.range_request.f_stop[i] = settings.f_stop[i];
	}

	reqframe.range_request.f_step = settings.f_step;
	reqframe.range_request.capabilities = 0x00;

	fsm_state = IDLE; // hard reset the statemachine, maybe it is stuck in some timed out measurement

	statemachine(RANGE_REQUEST_START, &reqframe);

	while (fsm_state != IDLE) {
		// TODO: use event to avoid busy waiting
		PROCESS_PAUSE(); // wait while measurement is running
	}

	// start calculation of distance
	static uint16_t j; // 16 bit counter for loops

	// calculate average value
	static int8_t pmu_values[PMU_MEASUREMENTS];
	for (j = 0; j < PMU_MEASUREMENTS; j++) {
		int16_t v;
		v  = local_pmu_values[j*4+0]-remote_pmu_values[j*4+0];
		v += local_pmu_values[j*4+1]-remote_pmu_values[j*4+1];
		v += local_pmu_values[j*4+2]-remote_pmu_values[j*4+2];
		v += local_pmu_values[j*4+3]-remote_pmu_values[j*4+3];

		v /= 4;

		if (v > 127) {
			v -= 256;
		} else if (v < -128) {
			v += 256;
		}

		pmu_values[j] = (int8_t)v; // lets hope the compiler does this in a good way...
		//printf("pmu_values: %d\n", v);
	}
	PROCESS_PAUSE();


	// calculate autocorrelation
#if FFT_N < PMU_MEASUREMENTS
	#error FFT size too small!
#endif
	static int16_t autocorr_values[FFT_N];
	autocorr(pmu_values, autocorr_values, PMU_MEASUREMENTS);
	PROCESS_PAUSE();

	// fill rest of window with zeroes
	for (j = PMU_MEASUREMENTS; j < FFT_N; j++) {
		autocorr_values[j] = 0;
	}
	PROCESS_PAUSE();

	for (j = 0; j < FFT_N; j++) {
		//printf("autocorr: %d\n", autocorr_values[j]);
	}

	// run fft
	static complex_t fft_buff[FFT_N];							// FTT buffer
	static uint16_t* fft_result = (uint16_t*)autocorr_values;	// reuse buffer to save memory
	fft_input(autocorr_values, fft_buff);
	PROCESS_PAUSE();
	fft_execute(fft_buff);
	PROCESS_PAUSE();
	fft_output(fft_buff, fft_result);
	PROCESS_PAUSE();

	// find peak in fft result
	uint16_t peak_idx = 0;
	uint16_t peak_val = 0;
	for (j = 0; j < FFT_N/2; j++) {
		if (peak_val < fft_result[j]) {
			peak_val = fft_result[j];
			peak_idx = j;
		}
		//printf("fft: %u\n", fft_result[j]);
	}

	printf("peak_idx: %u\n", peak_idx);

	// calculate distance from peak
	accum dist = (77.515678989k / (FFT_N) / (FFT_N) * peak_idx * peak_idx + 132.76889372k / (FFT_N) * peak_idx - 0.59048228802k);
	printf("distance: %f\n", (float)dist);


	// check if measurement was successful

	PROCESS_END();
}
