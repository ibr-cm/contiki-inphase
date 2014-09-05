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

#define PMU_START_FREQUENCY 2400		// start frequency for measurement
#define PMU_SAMPLES 1					// number of samples that are taken for each frequency by both nodes
#define PMU_SAMPLES_SHIFT 0				// bitshift to substitute the division by the PMU_SAMPLES
#define PMU_MEASUREMENTS 200			// number of frequencies to measure

// calibration values for quadratic
#define PMU_CALIB_X2 77.515678989k		// quadratic part
#define PMU_CALIB_X  132.76889372k		// linear part
#define PMU_CALIB_B  0.59048228802k		// constant part

//#define PMU_DIST_OFFSET 1.092k			// distance offset from hardware (meter)
#define PMU_DIST_OFFSET 3.342k			// distance offset from hardware (meter)
#define PMU_DIST_SLOPE	149.896229k		// slope calculated from sample distance

// configuration of signaling on the LEDs
// LEDs can be disabled to enabled measurement of power consumption
#ifndef PMU_GREEN_LED
#define PMU_GREEN_LED	(PMU_LED_ON_WHILE_RANGING | PMU_LED_ON_WHILE_CALC)
#endif
#ifndef PMU_YELLOW_LED
#define PMU_YELLOW_LED	PMU_LED_TIMER_TOGGLE
#endif

// possibilities to display on LEDs
#define PMU_LED_NONE				0	// do not change LEDs
#define PMU_LED_TIMER_TOGGLE		1	// toggle LED when sync timer runs out (display this on oscilloscope)
#define PMU_LED_ON_WHILE_RANGING	2	// enable LED during ranging
#define PMU_LED_ON_WHILE_CALC		4	// enable LED during distance calculation
#define PMU_LED_ON_WHILE_DIG2		8	// enable LED while waiting for DIG2 signal
#define PMU_LED_ENABLE_ON_ERROR		16	// turn LED on on error, disable it on next measurement
#define PMU_LED_ON_WHILE_PMU_READ	32	// turn LED on while the PMU values are read

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
uint8_t next_status_code;

uint8_t dist_last_meter = 0;
uint8_t dist_last_centimeter = 0;
uint8_t dist_last_quality = 0;

struct {
	linkaddr_t reflector;						// address of the reflector
	linkaddr_t initiator;						// address of the initiator that issued the current measurement
	uint8_t raw_output;							// if 1, output PMU data to serial port
	uint8_t allow_ranging;
} settings;

// allocate an array with 206 bytes, this works for full spectrum at 1 MHz steps
#define PMU_VALUES_LEN PMU_MAXIMUM_FREQUENCY-PMU_MINIMUM_FREQUENCY+1
uint8_t local_pmu_values[PMU_VALUES_LEN];
int8_t* signed_local_pmu_values = (int8_t*)local_pmu_values;	// reuse buffer to save memory

#define MEASUREMENT_TIMEOUT (0.02 * CLOCK_SECOND)
struct ctimer timeout_timer;  	// if this timer runs out, the measurement has
								// failed due to a timeout in the network

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

	settings.raw_output = 0;
	settings.allow_ranging = 0; // ranging is disabled by default

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

uint8_t at86rf233_get_dist_meter() {
	return dist_last_meter;
}

uint8_t at86rf233_get_dist_centimeter() {
	return dist_last_centimeter;
}

uint8_t at86rf233_get_quality() {
	return dist_last_quality;
}

uint16_t at86rf233_get_raw_len() {
	return PMU_VALUES_LEN;
}

int8_t* at86rf233_get_raw_ptr() {
	return signed_local_pmu_values;
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

uint8_t at86rf233_set_raw_output(uint8_t raw) {
	if (raw > 0) {
		settings.raw_output = 1;
	} else {
		settings.raw_output = 0;
	}
	return 0;
}

uint8_t at86rf233_set_allow_ranging(uint8_t allow) {
	if (allow > 0) {
		settings.allow_ranging = 1;
	} else {
		settings.allow_ranging = 0;
	}
	return 0;
}

// this gets call from the timeout_timer when it expires
static void reset_statemachine() {
	fsm_state = IDLE;
	status_code = next_status_code;

	// indicate errors
	#if PMU_GREEN_LED & PMU_LED_ENABLE_ON_ERROR
		leds_on(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ENABLE_ON_ERROR
		leds_on(LEDS_YELLOW);
	#endif

}

static void send_serial(void) {
	binary_start_frame();

    // send number of samples per frequency
    binary_send_byte(1); // only one sample is transmitted it is already averaged

    // send step size
    binary_send_byte(0); // 0 is parsed as 500 kHz

	// send start frequency
	binary_send_short(PMU_START_FREQUENCY);

	// send total amount of samples
	binary_send_short(PMU_MEASUREMENTS);

	// send reflector address
	binary_send_short(settings.reflector.u16);

	// send calculated distance meter
	binary_send_byte(dist_last_meter);

	// send calculated distance centimeter
	binary_send_byte(dist_last_centimeter);

	// send last quality
	binary_send_byte(dist_last_quality);

	// send system status
	binary_send_byte(status_code);

	// transmit data
	binary_send_data(&local_pmu_values, PMU_MEASUREMENTS);

    binary_end_frame();
}

static void statemachine(uint8_t frame_type, frame_subframe_t *frame) {

	PRINTF("DISTANCE: state: frame_type: %u, fsm_state: %u\n", frame_type, fsm_state);

	switch (fsm_state) {
	case IDLE:
	{
		if (frame_type == RANGE_REQUEST_START) {
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
			// check if ranging is allowed
			if (!settings.allow_ranging) {
				PRINTF("DISTANCE: ranging request ignored (ranging not allowed)\n");
			} else {
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
			}
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
				// do basic calculations to save memory
				int16_t v = local_pmu_values[i+last_start]-frame->result_confirm.result_data[i];

				//v /= 2;	// active reflector measured phase twice


				if (v > 127) {
					v -= 256;
				} else if (v < -128) {
					v += 256;
				}



				// overwrite data in local array
				signed_local_pmu_values[i+last_start] = (int8_t)v;
			}

			uint16_t next_start = last_start + result_length;

			// send RESULT_REQUEST if more results are needed
			frame_range_basic_t f;
			f.frame_type = RESULT_REQUEST;
			f.content.result_request.result_data_type = RESULT_TYPE_PMU;
			f.content.result_request.result_start_address = next_start;
			AT86RF233_NETWORK.send(settings.reflector, sizeof(frame_result_request_t)+1, &f);
			if (next_start < PMU_MEASUREMENTS) {
				fsm_state = RESULT_REQUESTED;
			} else {
				fsm_state = IDLE;
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
				if (start_address >= PMU_MEASUREMENTS) {
					// start address points outside of the pmu data, this indicates that the initiator does not need more data
					fsm_state = IDLE;
					status_code = DISTANCE_IDLE;
					ctimer_stop(&timeout_timer); // done, stop timer
				} else {
					uint8_t result_length;
					if ((PMU_MEASUREMENTS - start_address) > RESULT_DATA_LENGTH) {
						result_length = RESULT_DATA_LENGTH;
					} else {
						result_length = PMU_MEASUREMENTS - start_address;
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

	// show that the timer ran out (debugging on oscilloscope)
	#if PMU_GREEN_LED & PMU_LED_TIMER_TOGGLE
		leds_toggle(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_TIMER_TOGGLE
		leds_toggle(LEDS_YELLOW);
	#endif
}
/*---------------------------------------------------------------------------*/

static int8_t wait_for_dig2(void) {
	// turn LED on while waiting
	#if PMU_GREEN_LED & PMU_LED_ON_WHILE_DIG2
		leds_on(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ON_WHILE_DIG2
		leds_on(LEDS_YELLOW);
	#endif
	// wait for DIG2
	uint16_t cnt0 = 1;	// this counts up while waiting for the signal, when it overflows the measurement is aborted
	uint16_t cnt1 = 1;	// this counts up while waiting for the signal, when it overflows the measurement is aborted
	while ((DIG2_PIN & (1<<DIG2_PIN_BIT)) == 0) {
		cnt0++;
		if (cnt0 == 0) {
			#if PMU_GREEN_LED & PMU_LED_ON_WHILE_DIG2
				leds_off(LEDS_GREEN);
			#endif
			#if PMU_YELLOW_LED & PMU_LED_ON_WHILE_DIG2
				leds_off(LEDS_YELLOW);
			#endif
			return -1;	// signal never went low, abort measurement
		}
	}
	while ((DIG2_PIN & (1<<DIG2_PIN_BIT)) == 1) {	// wait for falling edge, these are closer together than the rising edges
		cnt1++;
		if (cnt1 == 0) {
			#if PMU_GREEN_LED & PMU_LED_ON_WHILE_DIG2
				leds_off(LEDS_GREEN);
			#endif
			#if PMU_YELLOW_LED & PMU_LED_ON_WHILE_DIG2
				leds_off(LEDS_YELLOW);
			#endif
			return -1;	// signal never went high, abort measurement
		}
	}
	#if PMU_GREEN_LED & PMU_LED_ON_WHILE_DIG2
		leds_off(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ON_WHILE_DIG2
		leds_off(LEDS_YELLOW);
	#endif
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

	#if PMU_GREEN_LED & PMU_LED_ON_WHILE_PMU_READ
		leds_on(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ON_WHILE_PMU_READ
		leds_on(LEDS_YELLOW);
	#endif

	uint8_t i;
	uint16_t accumulator = 0; // this hold all sampled pmu values
	for (i = 0; i < PMU_SAMPLES; i++) {
		accumulator += hal_register_read(RG_PHY_PMU_VALUE);
		//_delay_us(8); // value is updated every 8us but we are slower with reading the value anyway
	}
	pmu_values[0] = (accumulator>>PMU_SAMPLES_SHIFT); // divide by the number of samples

	#if PMU_GREEN_LED & PMU_LED_ON_WHILE_PMU_READ
		leds_off(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ON_WHILE_PMU_READ
		leds_off(LEDS_YELLOW);
	#endif
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

	// disable LED if timer toggle is indicated to make sure it always toggles the same
	// enable LED if it indicates ranging
	#if PMU_GREEN_LED & PMU_LED_TIMER_TOGGLE
		leds_off(LEDS_GREEN);
	#elif PMU_GREEN_LED & PMU_LED_ON_WHILE_RANGING
		leds_on(LEDS_GREEN);
	#endif

	#if PMU_YELLOW_LED & PMU_LED_TIMER_TOGGLE
		leds_off(LEDS_YELLOW);
	#elif PMU_YELLOW_LED & PMU_LED_ON_WHILE_RANGING
		leds_on(LEDS_YELLOW);
	#endif

	AT86RF233_ENTER_CRITICAL_REGION();
	watchdog_stop();
	backup_registers();

	init_timer2();

	//hal_subregister_write(SR_TX_PWR, 0xF);			// set TX output power to -17dBm to avoid reflections
	hal_subregister_write(SR_TX_PWR, 0x0);			// set TX output power to +4dBm, MAXIMUM POWER
	hal_register_read(RG_IRQ_STATUS);				// clear all pending interrupts
	hal_subregister_write(SR_ARET_TX_TS_EN, 0x1);	// signal frame transmission via DIG2
	hal_subregister_write(SR_IRQ_2_EXT_EN, 0x1);	// enable time stamping via DIG2

	// this line is normally only done at the reflector:
	hal_subregister_write(SR_TOM_EN, 0x0);			// disable TOM mode (unclear why this is done here)

	// reflector sends the synchronization frame
	if (type) {
		// send PMU start

		// TODO: send the correct frame here, just to sleep well...

		//frame_range_basic_t f;
		//f.frame_type = PMU_START;

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
		if (type) {
			_delay_us(9.5243);	// DIG2 signal is on average 9.5243 us delayed on the initiator, reflector waits
		}
		start_timer2(18);		// timer counts to 18, we have 580us between synchronization points
		//leds_off(2);

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
			/*switch (type) {
			case 0:		// initiator
				setFrequency(PMU_START_FREQUENCY + (i * PMU_STEP), 0);
				receiver_pmu(&local_pmu_values[i]);
				sender_pmu();
				break;
			default:	// reflector
				setFrequency(PMU_START_FREQUENCY + (i * PMU_STEP), 1);
				sender_pmu();
				receiver_pmu(&local_pmu_values[i]);
				break;
			}*/
			// use 500 kHz spacing
			uint8_t f, f_full, f_half;
			switch (type) {
			case 0:		// initiator
				f = i;
				f_full = f >> 1;
				f_half = f & 0x01;
				setFrequency(PMU_START_FREQUENCY + f_full, f_half);
				receiver_pmu(&local_pmu_values[i]);
				sender_pmu();
				break;
			default:	// reflector
				f = i + 1; // reflector is 500 kHz higher
				f_full = f >> 1;
				f_half = f & 0x01;
				setFrequency(PMU_START_FREQUENCY + f_full, f_half);
				sender_pmu();
				receiver_pmu(&local_pmu_values[i]);
				break;
			}
			wait_for_timer2(5);
		}
		ret_val = 0;
	} else {
		// DIG2 signal not found, abort measurement
		// to be honest: if the reflector does not get the DIG2 from its own sending
		// there must be something horribly wrong...
		ret_val = -1; // DIG2 signal not seen, abort!
	}

	restore_initial_status();
	watchdog_start();
	AT86RF233_LEAVE_CRITICAL_REGION();

	#if PMU_GREEN_LED & PMU_LED_ON_WHILE_RANGING
		leds_off(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ON_WHILE_RANGING
		leds_off(LEDS_YELLOW);
	#endif

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

	static uint8_t calc_status;
	calc_status = DISTANCE_INVALID;

	// new measurement start, no error so far
	#if PMU_GREEN_LED & PMU_LED_ENABLE_ON_ERROR
		leds_off(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ENABLE_ON_ERROR
		leds_off(LEDS_YELLOW);
	#endif

	// check if the reflector address is set
	if (linkaddr_cmp(&settings.reflector, &linkaddr_null)) {
		// no reflector address is set, ranging not possible
		#if PMU_GREEN_LED & PMU_LED_ENABLE_ON_ERROR
				leds_on(LEDS_GREEN);
		#endif
		#if PMU_YELLOW_LED & PMU_LED_ENABLE_ON_ERROR
				leds_on(LEDS_YELLOW);
		#endif
		return 0;
	}

	status_code = DISTANCE_RUNNING;

	// start a range measurement
	frame_subframe_t reqframe;

	reqframe.range_request.ranging_method = RANGING_METHOD_PMU;
	reqframe.range_request.capabilities = 0x00;

	fsm_state = IDLE; // hard reset the statemachine, maybe it is stuck in some timed out measurement

	statemachine(RANGE_REQUEST_START, &reqframe);

	while (fsm_state != IDLE) {
		// TODO: use event to avoid busy waiting
		PRINTF("DISTANCE_PROCESS: FSM_STATE: %u\n", fsm_state);
		PROCESS_PAUSE(); // wait while measurement is running
	}

	if (status_code != DISTANCE_RUNNING) {
		// measurement was not successful, do not calculate a distance
	#if PMU_GREEN_LED & PMU_LED_ENABLE_ON_ERROR
			leds_on(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ENABLE_ON_ERROR
			leds_on(LEDS_YELLOW);
	#endif
		PRINTF("DISTANCE_PROCESS: measurement not successful\n");
		PROCESS_EXIT();
	}

	// start calculation of distance

	#if PMU_GREEN_LED & PMU_LED_ON_WHILE_CALC
		leds_on(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ON_WHILE_CALC
		leds_on(LEDS_YELLOW);
	#endif
/*
	static uint16_t j; // 16 bit counter for loops

	// calculate autocorrelation
	#if FFT_N < PMU_MEASUREMENTS
		#error FFT size too small!
	#endif
	static int16_t autocorr_values[FFT_N];
	autocorr(signed_local_pmu_values, autocorr_values, PMU_MEASUREMENTS);
	PROCESS_PAUSE();

	// fill rest of window with zeroes
	for (j = PMU_MEASUREMENTS; j < FFT_N; j++) {
		autocorr_values[j] = 0;
	}
	PROCESS_PAUSE();

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

	//printf("peak_idx: %u\n", peak_idx);

	// calculate distance from peak
	fract m = (2.0k * peak_idx) / (1.0k * FFT_N); // take care of 500 kHz spacing
	PRINTF("DISTANCE_PROCESS: m: %f\n", (float)m);
	accum dist = (PMU_DIST_SLOPE * m - PMU_DIST_OFFSET);


	// check if measurement was successful
	if (dist < 0) {
		// the distance must always be positive, otherwise the measurement is useless
		calc_status = DISTANCE_VALUE_ERROR;
	} else {
		// measurement was ok,
		dist_last_meter = (uint8_t)dist;
		dist_last_centimeter = (uint8_t)((dist-dist_last_meter)*100);

		// calculate quality
		// just use the peak value from the fft
		if (peak_val > 255) {
			dist_last_quality = 255;
		} else {
			dist_last_quality = (uint8_t)peak_val;
		}

	}

	PRINTF("DISTANCE_PROCESS: ");

	if (peak_val > 40) {
		if (dist > 0) {
			PRINTF("GOOD ");
		}
	}

	PRINTF("distance: %u.%u meter\n", (uint8_t)dist, (uint8_t)((dist-((uint8_t)dist))*100));
*/
	#if PMU_GREEN_LED & PMU_LED_ON_WHILE_CALC
		leds_off(LEDS_GREEN);
	#endif
	#if PMU_YELLOW_LED & PMU_LED_ON_WHILE_CALC
		leds_off(LEDS_YELLOW);
	#endif

	// everything is done
	if (calc_status != DISTANCE_INVALID) {
		status_code = calc_status;
	} else {
		status_code = DISTANCE_IDLE;
	}

	// output result data if configured
	if (settings.raw_output) {
		send_serial(); // all results gathered, send via serial port
	}

	PROCESS_END();
}
