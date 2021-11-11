/**
 * \file
 *      InPhase Ranging example
 * \author
 *      Yannic Schröder <schroeder@ibr.cs.tu-bs.de>
 *
 */

#include "contiki.h"
#include "net/linkaddr.h"
#include "dev/distance-sensor.h"
#include "leds.h"

#include <stdio.h>
#include <stddef.h>
#include <avr/pgmspace.h>

// the initiator is the sensor node that want to measure the distance
// it is often connected to a PC and transfers the measurement data to it
// define its network address here
// network address is displayed in contiki boot screen
#define INITIATOR_ADDR 0x05AF

// the initiator uses this node as measurement partner
// it is called reflector
// define its network address here
// network address is displayed in contiki boot screen
#define REFLECTOR_ADDR 0x066F

// function to print strings from flash memory instead of RAM (saves memory on static strings)
#define PRINTF_P(FORMAT,args...) printf_P(PSTR(FORMAT),##args)

/*---------------------------------------------------------------------------*/

// prints the status of the measurement to the console
// very useful for debugging
void print_status(uint8_t status) {
    PRINTF_P("distance_status: ");
    switch (status) {
    case DISTANCE_INVALID:
        // default error message
        PRINTF_P("DISTANCE_INVALID");
        break;
    case DISTANCE_IDLE:
        // measurement finished successfully
        // a new measurement can be started
        PRINTF_P("DISTANCE_IDLE");
        break;
    case DISTANCE_RUNNING:
        // measurement is currently running
        // you have to wait until it is finished
        PRINTF_P("DISTANCE_RUNNING");
        break;
    case DISTANCE_FAILED:
        PRINTF_P("DISTANCE_FAILED");
        break;
    case DISTANCE_NO_RF233:
        // your sensor node has no AT86RF233 radio
        // you need the correct radio transceiver for InPhase to work
        PRINTF_P("DISTANCE_NO_RF233");
        break;
    case DISTANCE_TIMEOUT:
        // the other node did not answer in time
        // this happens when the radio link is very lossy
        PRINTF_P("DISTANCE_TIMEOUT");
        break;
    case DISTANCE_NO_REFLECTOR:
        // the reflector did not answer at all
        // check the reflector address
        // check if reflector allows ranging
        PRINTF_P("DISTANCE_NO_REFLECTOR");
        break;
    case DISTANCE_NO_SYNC:
        // time sync did not work, the measurement was aborted
        // the reflector did not send a timesync packet
        // this happens due to packet loss on the radio link
        PRINTF_P("DISTANCE_NO_SYNC");
        break;
    case DISTANCE_VALUE_ERROR:
        // distance computation result is implausible
        // for example the distance is negative after subtracting the antenna offset
        PRINTF_P("DISTANCE_VALUE_ERROR");
        break;
    case DISTANCE_WRONG_SYNC:
        // initiator synced to the wrong packet
        // this can happen if antoher sensor node sent a frame over the wireless link
        // the initiator can check if it received the correct frame
        // the reflector cannot do this, it will execute the measurement anyway
        PRINTF_P("DISTANCE_WRONG_SYNC");
        break;
    default:
        PRINTF_P("status code unknown");
        break;
    }
    PRINTF_P("\n");
}
/*---------------------------------------------------------------------------*/
// this function only prints the status message if it changed since the last print
// avoids spamming the serial console
void print_status_nospam(uint8_t status) {
    static uint8_t last_status = DISTANCE_INVALID; // default invalid status

    if (last_status != status) {
        print_status(status);
        last_status = status;
    }
}
/*---------------------------------------------------------------------------*/
// Contiki process definitions
PROCESS(node_chooser, "Node Choosing Process");
PROCESS(initiator_process, "Process running on initiator");
PROCESS(reflector_process, "Process running on reflector");
AUTOSTART_PROCESSES(&node_chooser);
/*---------------------------------------------------------------------------*/
// The node chooser process determines which node is the initiator and which is the reflector
// Basically all nodes that are not the initiator will be configured as reflector
PROCESS_THREAD(node_chooser, ev, data)
{
    PROCESS_BEGIN();

    leds_on(LEDS_YELLOW); // enable LED to see system is running

    // construct a linkaddr from the defined address above
    linkaddr_t initiator_addr;
    uint16_t addr = INITIATOR_ADDR;
    addr = (addr << 8) | (addr >> 8); // swap the address bytes
    initiator_addr.u16 = addr;

    // check if the local address matches the initiator address
    if (linkaddr_cmp(&initiator_addr, &linkaddr_node_addr)) {
        // if it matches, start the initiator process
        process_start(&initiator_process, NULL);
    } else {
        // if it does not match, start the reflector process
        process_start(&reflector_process, NULL);
    }

    // process exits here, one of the processes started above takes over
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
// the initoator process starts range measurements with the reflector in an infinite loop
PROCESS_THREAD(initiator_process, ev, data)
{
    PROCESS_BEGIN();

    PRINTF_P("initiator process started\n");

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

    // set up the reflector address
    static linkaddr_t reflector;
    uint16_t addr = REFLECTOR_ADDR;
    addr = (addr << 8) | (addr >> 8); // swap the address bytes
    reflector.u16 = addr;

    // you can enable the output of raw measurement data here
    // it is printed to the serial console and can be parsed by the InPhase Python library
    // set parameter to 1 to output raw data
    distance_sensor->configure(DISTANCE_RAW_OUTPUT, 0);

    // enable distance computation on the sensor node
    // this takes quite some time to complete
    // you can disable this if you only need the raw data to speed up the measurement
    distance_sensor->configure(DISTANCE_COMPUTE, 1);

    // enable interpolation
    // this increases the resolution of the computed distance
    // however, interpolation needs extra CPU time...
    distance_sensor->configure(DISTANCE_INTERPOLATE, 1);

    // offset originating from the used hardware
    // each sensor node adds a specific additional distance to the measured result
    // to obtain the real distance, it needs to be subtracted from the measured distance
    // the distance sensor can do this automatically before reporting the distance
    // the offset for a pair of INGA 1.6 sensor nodes is 768 millimeters (384 millimeters per INGA)
    distance_sensor->configure(DISTANCE_OFFSET, 768);

    // infinite main loop
    while(1) {
        // set reflector address for the next measurement
        // if you have multiple reflectors, you could iterate through them here
        distance_sensor->configure(DISTANCE_TARGET, (int)&reflector);

        // allow start of next range measurement
        if( distance_sensor->configure(DISTANCE_START, 0) ) {
            PRINTF_P("distance: distance process is still running\n");
            continue;
        }

        // get status from sensor
        static uint8_t status;
        status = distance_sensor->status(SENSORS_READY);

        // wait for ranging be actually started
        while (status != DISTANCE_RUNNING) {
            status = distance_sensor->status(SENSORS_READY);
            PROCESS_PAUSE();
        }
        // wait for ranging to finish
        while (status == DISTANCE_RUNNING) {
            status = distance_sensor->status(SENSORS_READY);
            PROCESS_PAUSE();
        }

        // print status of the last measurement
        print_status(status);

        // print the distance if measurement was successful
        if (status == DISTANCE_IDLE) {
            // obtain distance from sensor
            uint8_t meter = distance_sensor->value(DISTANCE_LAST_DIST_METER);
            uint8_t centimeter = distance_sensor->value(DISTANCE_LAST_DIST_CENTIMETER);
            // obtain dqi from sensor
            // it indicates the quality of the measurement
            // values can range from 0 to 255, higher value is better
            uint16_t dqi = distance_sensor->value(DISTANCE_LAST_DIST_QUALITY);

            // print values to console
            PRINTF_P("distance: %u.%02u meter, DQI: %u\n", meter, centimeter, dqi);
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
// this process runs at the reflector, it does not start a measurement,
// but waits passively for incoming measurement requests
PROCESS_THREAD(reflector_process, ev, data)
{
    PROCESS_BEGIN();

    PRINTF_P("reflector process started\n");

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

    // allow ranging
    // if ranging is not allowed, the node would not answer to ranging requests
    distance_sensor->configure(DISTANCE_ALLOW_RANGING, 1);

    // this loop is not strctly needed, it just displays the status
    while (1) {
        // not much to do here...
        print_status_nospam(distance_sensor->status(SENSORS_READY));
        PROCESS_PAUSE();
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
