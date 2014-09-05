/**
 * \file
 *      InPhase Ranging example
 * \author
 *      Yannic Schröder <yschroed@ibr.cs.tu-bs.de>
 *
 */

#include "contiki.h"
#include "net/linkaddr.h"
#include "dev/distance-sensor.h"
#include "leds.h"

#include <stdio.h>
#include <stddef.h>
#include <avr/pgmspace.h>

#ifndef INITIATOR_ADDR
 #error "INITIATOR_ADDR not defined. Supply it to make: INITIATOR_ADDR=0xXXXX"
#endif

#ifndef REFLECTOR_ADDR
 #error "REFLECTOR_ADDR not defined. Supply it to make: REFLECTOR_ADDR=0xXXXX"
#endif

#define PRINTF_P(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
/*---------------------------------------------------------------------------*/
void print_status(uint8_t status) {
	PRINTF_P("distance_status: ");
	switch (status) {
	case DISTANCE_INVALID:
		PRINTF_P("DISTANCE_INVALID");
		break;
	case DISTANCE_IDLE:
		PRINTF_P("DISTANCE_IDLE");
		break;
	case DISTANCE_RUNNING:
		PRINTF_P("DISTANCE_RUNNING");
		break;
	case DISTANCE_FAILED:
		PRINTF_P("DISTANCE_FAILED");
		break;
	case DISTANCE_NO_RF233:
		PRINTF_P("DISTANCE_NO_RF233");
		break;
	case DISTANCE_TIMEOUT:
		PRINTF_P("DISTANCE_TIMEOUT");
		break;
	case DISTANCE_NO_REFLECTOR:
		PRINTF_P("DISTANCE_NO_REFLECTOR");
		break;
	case DISTANCE_NO_SYNC:
		PRINTF_P("DISTANCE_NO_SYNC");
		break;
	case DISTANCE_VALUE_ERROR:
		PRINTF_P("DISTANCE_VALUE_ERROR");
		break;
	default:
		PRINTF_P("status code unknown");
		break;
	}
	PRINTF_P("\n");
}
/*---------------------------------------------------------------------------*/
void print_status_nospam(uint8_t status) {
	static uint8_t last_status = DISTANCE_INVALID; // default invalid status

	if (last_status != status) {
		print_status(status);
		last_status = status;
	}
}
/*---------------------------------------------------------------------------*/
PROCESS(node_chooser, "Node Choosing Process");
PROCESS(initiator_process, "Process running on node 0");
PROCESS(reflector_process, "Process running on node 1");
AUTOSTART_PROCESSES(&node_chooser);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_chooser, ev, data)
{
	PROCESS_BEGIN();

	leds_on(LEDS_YELLOW); // enable LED to see system is running

	linkaddr_t addr;
	addr.u8[0] = INITIATOR_ADDR >> 8;
	addr.u8[1] = INITIATOR_ADDR & 0x00FF;

	PRINTF_P("Local Address %x.%x\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]); // debug message
	PRINTF_P("Initiator Address %x.%x\n", addr.u8[0], addr.u8[1]); // debug message
	PRINTF_P("Reflector Address %x.%x\n", REFLECTOR_ADDR >> 8, REFLECTOR_ADDR & 0x00FF); // debug message

	if (linkaddr_cmp(&addr, &linkaddr_node_addr)) {
		process_start(&initiator_process, NULL);
	} else {
		process_start(&reflector_process, NULL);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
// This node is the initiator, it starts a range measurement
PROCESS_THREAD(initiator_process, ev, data)
{
	PROCESS_BEGIN();

	PRINTF_P("I am the initiator, find me!\n");

	// get pointer to sensor
	static const struct sensors_sensor *distance_sensor;

	distance_sensor = sensors_find("Distance");

	// activate and check status
	uint8_t status = SENSORS_ACTIVATE(*distance_sensor);
	if (status == 0) {
		PRINTF_P("Error: Failed to init distance sensor, aborting...\n");
		PROCESS_EXIT();
	}
	PRINTF_P("distance sensor activated!\n");

	// set up the list of reflector addresses
	static linkaddr_t reflector;

	reflector.u8[0] = REFLECTOR_ADDR >> 8;
	reflector.u8[1] = REFLECTOR_ADDR & 0x00FF;

	distance_sensor->configure(DISTANCE_RAW_OUTPUT, 0); // output raw data
	PRINTF_P("Reflector %x.%x\n", reflector.u8[0], reflector.u8[1]); // debug message
	// set reflector
	distance_sensor->configure(DISTANCE_TARGET, (int)&reflector);

	while(1) {
		// start measurement
		distance_sensor->configure(DISTANCE_START, 0);
		// get status from sensor
		static uint8_t status;
		status = distance_sensor->status(SENSORS_READY);
		// wait for ranging to start
		while (status != DISTANCE_RUNNING) {
			// start measurement
			distance_sensor->configure(DISTANCE_START, 0);
			status = distance_sensor->status(SENSORS_READY);
			PROCESS_PAUSE();
		}
		// wait for ranging to finish
		while (status == DISTANCE_RUNNING) {
			status = distance_sensor->status(SENSORS_READY);
			PROCESS_PAUSE();
		}
		//print_status(status);
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
// This node is the reflector, it does not start a measurement,
// but waits passively for incoming measurement requests
PROCESS_THREAD(reflector_process, ev, data)
{
	PROCESS_BEGIN();

	PRINTF_P("I am a reflector, I'll answer him!\n");

	// get pointer to sensor
	static const struct sensors_sensor *distance_sensor;

	distance_sensor = sensors_find("Distance");

	// activate and check status
	uint8_t status = SENSORS_ACTIVATE(*distance_sensor);
	if (status == 0) {
		PRINTF_P("Error: Failed to init distance sensor, aborting...\n");
		PROCESS_EXIT();
	}
	PRINTF_P("distance sensor activated!\n");

	distance_sensor->configure(DISTANCE_ALLOW_RANGING, 1);

	while (1) {
		// not much to do here...
		//print_status_nospam(distance_sensor->status(SENSORS_READY));
		PROCESS_PAUSE();
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
