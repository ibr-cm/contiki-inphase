/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"

#include "net/rime/rime.h"          // for transmision tests
#include "radio/rf230bb/hal.h"      // for SPI tests
#include "dev/radio.h"              // for radio tests
#include "radio/rf230bb/rf230bb.h"  // for SPI tests
#include "leds.h"                   // for LED tests

// pin definitions for INGA and InPhaseBase
#define DIG2_PIN                PINB
#define DIG2_PIN_BIT            PB0

/** Access parameters for sub-register ARET_TX_TS_EN in register @ref RG_XAH_CTRL_1 */
#define SR_ARET_TX_TS_EN             0x17, 0x80, 7

/** Access parameters for sub-register IRQ_2_EXT_EN in register @ref RG_TRX_CTRL_1 */
#define SR_IRQ_2_EXT_EN              0x04, 0x40, 6

/** Constant RF230 for sub-register @ref SR_PART_NUM */
#define RF230_ID                 (0x02)
/** Constant RF231 for sub-register @ref SR_PART_NUM */
#define RF231_ID                 (0x03)
/** Constant RF232 for sub-register @ref SR_PART_NUM */
#define RF232_ID                 (0x0a)
/** Constant RF233 for sub-register @ref SR_PART_NUM */
#define RF233_ID                 (0x0b)

#define FAIL_STRING "[\e[0;31m FAIL \e[0m] "
#define PASS_STRING "[\e[0;32m PASS \e[0m] "
#define WARN_STRING "[\e[1;33m WARN \e[0m] "
#define INFO_STRING "[\e[0;34m INFO \e[0m] "

/*---------------------------------------------------------------------------*/
// copied from halbb.c to use it here
#include <avr/io.h>
#include <avr/interrupt.h>

#define HAL_SPI_TRANSFER_OPEN() { \
  HAL_ENTER_CRITICAL_REGION();    \
  HAL_SS_LOW(); /* Start the SPI transaction by pulling the Slave Select low. */
#define HAL_SPI_TRANSFER_WRITE(to_write) (SPDR = (to_write))
#define HAL_SPI_TRANSFER_WAIT() ({while ((SPSR & (1 << SPIF)) == 0) {;}}) /* gcc extension, alternative inline function */
#define HAL_SPI_TRANSFER_READ() (SPDR)
#define HAL_SPI_TRANSFER_CLOSE() \
    HAL_SS_HIGH(); /* End the transaction by pulling the Slave Select High. */ \
    HAL_LEAVE_CRITICAL_REGION(); \
    }
#define HAL_SPI_TRANSFER(to_write) (      \
                    HAL_SPI_TRANSFER_WRITE(to_write),   \
                    HAL_SPI_TRANSFER_WAIT(),        \
                    HAL_SPI_TRANSFER_READ() )

/*---------------------------------------------------------------------------*/

#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
// copied from InPhase driver to test DIG2 pin
int8_t wait_for_dig2(void) {
    // wait for DIG2
    uint16_t cnt0 = 1;  // this counts up while waiting for the signal, when it overflows the measurement is aborted
    uint16_t cnt1 = 1;  // this counts up while waiting for the signal, when it overflows the measurement is aborted
    while ((DIG2_PIN & (1<<DIG2_PIN_BIT)) == 0) {
        cnt0++;
        if (cnt0 == 0) {
            return -1;  // signal never went low, abort measurement
        }
    }
    while ((DIG2_PIN & (1<<DIG2_PIN_BIT)) == 1) {   // wait for falling edge, these are closer together than the rising edges
        cnt1++;
        if (cnt1 == 0) {
            return -1;  // signal never went high, abort measurement
        }
    }
    return 0;   // everything worked as expected
}
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
static uint16_t receiving_indicator = 0;
static uint8_t sending_indicator = 0;
char answer_key[] = "InPhase Hardware Send Test";
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  printf(INFO_STRING);
  printf("Broadcast message received from %d.%d: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
  receiving_indicator++; // something was received

  if (strncmp(answer_key,(char *)packetbuf_dataptr(), 26) == 0) {
    sending_indicator++;
  }
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
static struct etimer timer;
static uint8_t spi_read_data;
static uint8_t tests_failed = 0;
PROCESS_THREAD(hello_world_process, ev, data)
{
    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

    PROCESS_BEGIN();

    leds_init();
    leds_on(LEDS_GREEN | LEDS_YELLOW);

    printf(INFO_STRING);
    printf("Starting InPhase hardware test...\n");

    printf(INFO_STRING);
    printf("Testing SPI communication...\n");
    printf(INFO_STRING);
    printf("Reading radio part number...\n");

    hal_init();
    HAL_SPI_TRANSFER_OPEN();
    HAL_SPI_TRANSFER(0x80 | RG_PART_NUM);
    spi_read_data = HAL_SPI_TRANSFER(0x80);
    HAL_SPI_TRANSFER_CLOSE();

    switch (spi_read_data) {
        case 0x00:
            printf(FAIL_STRING);
            printf("SPI returned 0x00, check if MISO is tied to GND!\n");
            tests_failed++;
            break;
        case 0xFF:
            printf(FAIL_STRING);
            printf("SPI returned 0xFF, check if MISO is tied to VCC!\n");
            tests_failed++;
            break;
        case 0x80:
            printf(FAIL_STRING);
            printf("SPI returned 0x80, check if MISO is bridged to MOSI!\n");
            tests_failed++;
            break;
        case RF230_ID:
            printf(FAIL_STRING);
            printf("AT86RF230 detected! InPhase needs AT86RF233.\n");
            tests_failed++;
            break;
        case RF231_ID:
            printf(FAIL_STRING);
            printf("AT86RF231 detected! InPhase needs AT86RF233.\n");
            tests_failed++;
            break;
        case RF232_ID:
            printf(FAIL_STRING);
            printf("AT86RF232 detected! InPhase needs AT86RF233.\n");
            tests_failed++;
            break;
        case RF233_ID:
            printf(PASS_STRING);
            printf("AT86RF233 detected. SPI is working.\n");
            break;
        default:
            printf(FAIL_STRING);
            printf("Uncaught SPI error. SPI returned %02x\n", spi_read_data);
            tests_failed++;
            break;
    }

    printf(INFO_STRING);
    printf("Testing Radio transmission...\n");

    broadcast_open(&broadcast, 129, &broadcast_call);

    printf(INFO_STRING);
    printf("Testing DIG2 and SLPTR pins...\n");
    packetbuf_copyfrom("InPhase Hardware Test DIG2 SLPTR\0", 33);
    broadcast_send(&broadcast);

    leds_off(LEDS_GREEN);
    hal_subregister_write(SR_ARET_TX_TS_EN, 0x1);   // signal frame transmission via DIG2
    hal_subregister_write(SR_IRQ_2_EXT_EN, 0x1);    // enable time stamping via DIG2
    hal_subregister_write(SR_TRX_CMD, CMD_TX_ARET_ON);
    hal_set_slptr_low();
    hal_set_slptr_high(); // send the packet at the latest time possible
    hal_set_slptr_low();
    leds_on(LEDS_GREEN);

    switch (wait_for_dig2()) {
        case -1:
            printf(FAIL_STRING);
            printf("DIG2 signal not seen, check connection of DIG2 and SLPTR!\n");
            tests_failed++;
            break;
        case 0:
            printf(PASS_STRING);
            printf("DIG2 and SLPTR signals working.\n");
            break;
        default:
            printf(FAIL_STRING);
            printf("Uncaught error when testing DIG2 and SLPTR!\n");
            tests_failed++;
            break;
    }

    printf(INFO_STRING);
    printf("Trying to receive any broadcast message (trying 10 seconds)...\n");
    hal_subregister_write(SR_TRX_CMD, RX_AACK_ON);

    static uint8_t loop_cnt = 0;
    while (loop_cnt < 10) {
        if (receiving_indicator > 0) {
            break;
        }
        etimer_set(&timer,  CLOCK_SECOND);
        PROCESS_YIELD();
        loop_cnt++;
    }

    if (receiving_indicator > 0) {
        printf(PASS_STRING);
        printf("Receiver is working.\n");
    } else {
        printf(FAIL_STRING);
        printf("No packet received, check IRQ pin!\n");
        tests_failed++;
    }

    printf(INFO_STRING);
    printf("Trying to send a broadcast message and receive the answer (10 tries in 10 seconds)...\n");

    loop_cnt = 0;
    while (loop_cnt < 10) {
        if (sending_indicator > 0) {
            break;
        }
        packetbuf_copyfrom("InPhase Hardware Send Test\0", 27);
        broadcast_send(&broadcast);
        etimer_set(&timer,  CLOCK_SECOND);
        PROCESS_YIELD();
        loop_cnt++;
    }
    if (sending_indicator > 0) {
        printf(PASS_STRING);
        printf("Answer to sent message received.\n");
    } else {
        printf(FAIL_STRING);
        printf("No answer received, check SLPTR and IRQ pins.\n");
        tests_failed++;
    }

    printf(INFO_STRING);
    printf("Automated tests finished.\n");
    if (tests_failed == 0) {
        printf(PASS_STRING);
        printf("\e[0;32mAll automated tests passed.\e[0m\n");
    } else {
        printf(FAIL_STRING);
        printf("\e[0;31mAutomated tests failed!\e[0m\n");
    }

    printf(INFO_STRING);
    printf("Testing LEDs...\n");
    printf(INFO_STRING);
    printf("User interaction: check if yellow LED is on 4 times as long as the green one.\n");
    printf(INFO_STRING);
    printf("User interaction: check if LEDs are green and yellow.\n");
    printf(INFO_STRING);
    printf("User interaction: check if LEDs are alternating.\n");
    printf(INFO_STRING);
    printf("User interaction: check if power LED (LED1) is red.\n");
    printf(INFO_STRING);
    printf("User interaction: check if LED on radio module is blue.\n");

    leds_off(LEDS_GREEN);
    leds_on(LEDS_YELLOW);

    while(1) {
        etimer_set(&timer,  CLOCK_SECOND);
        leds_toggle(LEDS_GREEN | LEDS_YELLOW); // toggle both LEDs at once using bitwise or
        PROCESS_YIELD();
        etimer_set(&timer,  4*CLOCK_SECOND);
        leds_toggle(LEDS_GREEN | LEDS_YELLOW); // toggle both LEDs at once using bitwise or
        PROCESS_YIELD();
    }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
