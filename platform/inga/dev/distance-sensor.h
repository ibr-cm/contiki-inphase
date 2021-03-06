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
 *      Distance sensor definition
 * \author
 *      Yannic Schröder <yschroed@ibr.cs.tu-bs.de>
 */

/**
 * \addtogroup avr_sensors
 * @{
 */

/**
 * \defgroup avr_radio_driver Distance Sensor
 *
 * This sensor interface allows to measure the distance between AT86RF233 radios.
 *
 */

#ifndef __DISTANCE_SENSOR_H__
#define __DISTANCE_SENSOR_H__

#include "lib/sensors.h"

extern const struct sensors_sensor distance_sensor;

#define DISTANCE_SENSOR "Distance"

// Values to read
// this always reads data from the last successful measurement
#define DISTANCE_LAST_DIST_METER        0
#define DISTANCE_LAST_DIST_CENTIMETER   1
#define DISTANCE_LAST_DIST_QUALITY      2
#define DISTANCE_LAST_RAW_LEN			3	// length of the buffer with the raw value
#define DISTANCE_LAST_RAW_PTR			4	// pointer to the raw distance values

// Configuration parameters

// target node to do ranging with
#define DISTANCE_TARGET        0
// start a range measurement
#define DISTANCE_START         1
// allow the sensor to answer ranging requests from other nodes
#define DISTANCE_ALLOW_RANGING 2
// output raw measurement data on stdout at initiator node for post-processing
#define DISTANCE_RAW_OUTPUT    3
// compute distance on sensor node
#define DISTANCE_COMPUTE       4
// use interpolation for distance computation (slower but more accurate)
#define DISTANCE_INTERPOLATE   5
// subtract this offset (in millimeters) from the computed distance (to take care of hardware offsets)
#define DISTANCE_OFFSET        6

// Status codes for SENSORS_READY
#define DISTANCE_INVALID      0
// no measurement running, idle, ready for measurement
#define DISTANCE_IDLE         1
// measurement is currently running
#define DISTANCE_RUNNING      2
// measurement failed to unknown reasons
#define DISTANCE_FAILED       3
// this sensor node has no AT86RF233 radio, measurement not possible
#define DISTANCE_NO_RF233     4
// communication with reflector timed out
#define DISTANCE_TIMEOUT      5
// reflector doesn't answer to ranging request
#define DISTANCE_NO_REFLECTOR 6
// timing synchronization failed
#define DISTANCE_NO_SYNC      7
// distance calculation returned wrong value
#define DISTANCE_VALUE_ERROR  8
// synced to wrong frame (not the one send by reflector)
#define DISTANCE_WRONG_SYNC   9

#endif /* __DISTANCE_SENSOR_H__ */

/** @} */ // avr_radio_driver
