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
 *      Distance sensor implementation
 * \author
 *      Yannic Schröder <yschroed@ibr.cs.tu-bs.de>
 */

#include "contiki.h"
#include "lib/sensors.h"
#include "dev/distance-sensor.h"
#include "at86rf233.h"
const struct sensors_sensor distance_sensor;
static uint8_t initialized = 0;
static uint8_t ready = 0;
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  switch (type) {
    case DISTANCE_LAST_DIST_METER:
      return at86rf233_get_dist_meter();
    case DISTANCE_LAST_DIST_CENTIMETER:
      return at86rf233_get_dist_centimeter();
    case DISTANCE_LAST_DIST_QUALITY:
      return at86rf233_get_quality();
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch (type) {
    case SENSORS_ACTIVE:
      return initialized;
    case SENSORS_READY:
      return at86rf233_get_status();
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{
  uint8_t value = 0;
  switch (type) {

    case SENSORS_HW_INIT:
      if (at86rf233_available()) {
        ready = 1;
        return 1;
      } else {
        ready = 0;
        return 0;
      }
      break;

    case SENSORS_ACTIVE:
      if (c) {
        if (at86rf233_init() == 0) {
          initialized = 1;
          return 1;
        }
      } else {
        if (at86rf233_deinit() == 0) {
          initialized = 0;
          return 1;
        }
      }
      break;

    case DISTANCE_TARGET:
    	at86rf233_set_target(c);
    	return 0;

    case DISTANCE_RAW_OUTPUT:
		at86rf233_set_raw_output(c);
		return 0;

    case DISTANCE_START:
    	return at86rf233_start_ranging();

    case DISTANCE_ALLOW_RANGING:
    	at86rf233_set_allow_ranging(c);
    	return 0;

  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(distance_sensor, DISTANCE_SENSOR, value, configure, status);
