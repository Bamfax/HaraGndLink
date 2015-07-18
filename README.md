# HaraGndLink
Custom Telemetry converter adapted for Harakiri TestCode3 and OpenTX 2.1. Needs a Teensy 3.1.

HaraGndLink - Haraki Ground Link
by Bamfax 21.06.2015
v0.1 - first ugly draft, still work in progress. Do not use.
Not for commercial use

This is about: Getting a decent and mostly complete telemetry dataset from harakiri down to earth to Taranis, using FrSky RX. 
And to allow having custom sensors in the airverhicle, where needed.

Written for and tested on:
- Harakiri TestCode 3, commit 147
- OpenTX 2.0.99 on Taranis, X4R-sb
- Teensy 3.1

Would not be here without the work from:
- ScottFlys MAVLink to FrSky Telemetry Firmware (http://www.rcgroups.com/forums/showthread.php?t=2274401)
- Pawelsky (http://www.rcgroups.com/forums/member.php?u=393936) and his FrSky S-Port telemetry library
- Crashpilot1000 (https://github.com/Crashpilot1000) and his Harakiri Firmware
- The OpenTX project
- Many others of the Multiwii and FPV community

Compiling:
- Has a bug with Arduino 1.6.4. Hangs while compiling. Use Arduino 1.6.5 and Teensyduino 1.24 instead.



The main program is licensed under the GPL: (exluding the FrSky library, which is copyright Pawelsky)
-----------------------------------------------------------------------------------------------------
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

A copy of the GNU General Public License is available at <http://www.gnu.org/licenses/>.
