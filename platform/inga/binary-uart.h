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
 *      Binary stdout transmission of arbitrary data
 * \author
 *      Yannic Schröder <yschroed@ibr.cs.tu-bs.de>
 */

#ifndef __BINARY_STDOUT_H__
#define __BINARY_STDOUT_H__

#include <stdint.h>

// special symbols for serial output
#define BINARY_FRAME_START     0x3C		// this byte marks a new frame
#define BINARY_FRAME_END       0x3E		// this byte marks the end of a frame
#define BINARY_ESCAPE_BYTE     0x40		// this byte shows that the next byte's value will be changed
#define BINARY_ESCAPE_ADD      0x10		// this is the amount the next byte is changed

// start a new frame transmission
void binary_start_frame();

// finish a frame transmission
void binary_end_frame();

// send a single byte
void binary_send_byte(uint8_t data);

// send a 2 byte value
void binary_send_short(uint16_t data);

// sends arbitrary data
void binary_send_data(uint8_t* data, uint8_t length);

// send a complete frame at once (no need to start and end frame manually)
void binary_send_frame(uint8_t* frame, uint8_t length);

#endif /* __BINARY_STDOUT_H__ */
