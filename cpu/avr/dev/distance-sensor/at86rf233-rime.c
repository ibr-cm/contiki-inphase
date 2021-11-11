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
 *      AT86RF233 rime network implementation
 * \author
 *      Yannic Schröder <yschroed@ibr.cs.tu-bs.de>
 */

/**
 * \addtogroup avr_sensors
 * @{
 */

/**
 * \defgroup avr_radio_driver AT86RF233 rime network
 *
 * This is the implementation for a rime network to use with the AT86RF233 distance sensor.
 *
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "at86rf233-network.h"
#include "at86rf233.h"

#define AT86RF233_RIME_CHANNEL 142

static void recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
  printf("unicast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *) packetbuf_dataptr());
  at86rf233_input(from, packetbuf_datalen(), packetbuf_dataptr());
}
static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;

uint8_t at86rf233_rime_init(void) {
	unicast_open(&uc, AT86RF233_RIME_CHANNEL, &unicast_callbacks);
	return 0;
}

uint8_t at86rf233_rime_send(linkaddr_t dest, uint8_t msg_len, void *msg) {
	packetbuf_copyfrom(msg, msg_len);
	printf("DISTANCE: Message sent to %d.%d\n", dest.u8[0], dest.u8[1]); // debug message
	unicast_send(&uc, &dest);
	return 0;
}

uint8_t at86rf233_rime_close(void) {
	unicast_close(&uc);
	return 0;
}

const struct at86rf233_network at86rf233_rime_network =  {at86rf233_rime_init, at86rf233_rime_send, at86rf233_rime_close};
