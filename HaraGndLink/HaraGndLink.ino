/*
  HaraGndLink - Haraki Ground Link
  (c) Bamfax 21.06.2015
  Not for commercial use
	
	This is about: Getting a decent and mostly complete telemetry dataset from harakiri down to earth to Taranis, using FrSky RX. 
	And to allow having custom sensors in the airverhicle, where needed.
  
  Written for and tested on:
	- Harakiri TestCode 3, commit 147
	- OpenTX 2.0.99 on Taranis, X4R-sb
	- Teensy 3.1
	
	Would not be here without the work from:
	- Crashpilot1000 (https://github.com/Crashpilot1000) and his Harakiri Firmware
	- Pawelsky (http://www.rcgroups.com/forums/member.php?u=393936) and his FrSky S-Port telemetry library
	- The OpenTX project
	- ScottFlys MAVLink to FrSky Telemetry Firmware (http://www.rcgroups.com/forums/showthread.php?t=2274401)
	- Many others of the Multiwii and FPV community
	
	Compiling:
	- Has a bug with Arduino 1.6.4. Hangs while compiling. Use Arduino 1.6.5 and Teensyduino 1.24 beta3 instead.
	
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  A copy of the GNU General Public License is available at <http://www.gnu.org/licenses/>.
*/
#pragma GCC diagnostic ignored "-Wwrite-strings"
#include <GCS_MAVLink.h>
#include <Time.h>
#include "HaraGndLink.h"
#include "Logger.h"
#include "MavLink.h"
#include "FrSkySPort.h"
#include "TelemetryData.h"

uint32_t next_1000_loop = 0L;
uint32_t next_200_loop = 0L;
uint32_t next_100_loop = 0L;

void console_print(char* fmt, ...) {
    char formatted_string[256];
    va_list argptr;
    va_start(argptr, fmt);
    vsprintf(formatted_string, fmt, argptr);
    va_end(argptr);
    DEBUG_SERIAL.print(formatted_string);
}

void setup() {
  debug_init();
  delay(5000);

  pinMode(LEDPIN, OUTPUT);
  console_print("%s\r\nStarting\r\n", PRODUCT_STRING);
	
	telem_data_init();
	frsky_init();
	mavlink_init();
	
	console_print("] ");
}

void loop() {
//  uint16_t len;
  uint32_t current_milli;
	
	current_milli = millis();

  if (current_milli >= next_1000_loop) {
    next_1000_loop = current_milli + 1000;
		digitalWrite(LEDPIN, !digitalRead(LEDPIN));
//    diags_update_led();
  }
	
	process_mavlink_packets();						// Get data from mavlink and process it
	frsky_process();								// Prepare and send FrSky S.Port data
	check_for_console_command();
}
