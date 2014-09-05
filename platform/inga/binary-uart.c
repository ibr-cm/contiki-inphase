#include "binary-uart.h"

#include "dev/rs232_atmega1284.h"
#include "dev/rs232.h"

#include <stdio.h>

static void send_escaped(uint8_t data) {
	switch(data) {
    case BINARY_FRAME_START:
    case BINARY_FRAME_END:
    case BINARY_ESCAPE_BYTE:
    	rs232_send(RS232_PORT_0, BINARY_ESCAPE_BYTE);
    	data -= BINARY_ESCAPE_ADD;
    	// no break here
    default:
    	rs232_send(RS232_PORT_0, data);
    	break;
    }
}

void binary_start_frame() {
	rs232_send(RS232_PORT_0, BINARY_FRAME_START);
}

void binary_end_frame() {
	rs232_send(RS232_PORT_0, BINARY_FRAME_END);
}

void binary_send_byte(uint8_t data) {
	send_escaped(data);
}

void binary_send_short(uint16_t data) {
	send_escaped((data >> 8) & 0xFF);
	send_escaped(data & 0xFF);
}

void binary_send_data(uint8_t* data, uint8_t length) {
	uint8_t i;
	for (i = 0; i < length; i++) {
		send_escaped(data[i]);
	}
}

void binary_send_frame(uint8_t* frame, uint8_t length) {
	binary_start_frame();
	binary_send_data(frame, length);
	binary_end_frame();
}
