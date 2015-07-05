/*
	Custom Sensor "BaseVars", by Bamfax, based upon:
	
	FrSky RPM sensor class for Teensy 3.x and 328P based boards (e.g. Pro Mini, Nano, Uno)
	(c) Pawelsky 20141120
	Not for commercial use
*/

#include "FrSkySportSensorBaseVars.h" 

FrSkySportSensorBaseVars::FrSkySportSensorBaseVars(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorBaseVars::setData(uint8_t base_mode, uint16_t custom_mode, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag)
{
	FrSkySportSensorBaseVars::base_mode = base_mode;
	FrSkySportSensorBaseVars::custom_mode = custom_mode;
	FrSkySportSensorBaseVars::xacc = xacc;
	FrSkySportSensorBaseVars::yacc = yacc;
	FrSkySportSensorBaseVars::zacc = zacc;
	FrSkySportSensorBaseVars::xgyro = xgyro;
	FrSkySportSensorBaseVars::ygyro = ygyro;
	FrSkySportSensorBaseVars::zgyro = zgyro;
	FrSkySportSensorBaseVars::xmag = xmag;
	FrSkySportSensorBaseVars::ymag = ymag;
	FrSkySportSensorBaseVars::zmag = zmag;
}

void FrSkySportSensorBaseVars::send(FrSkySportSingleWireSerial& serial, uint8_t id)
{
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
		case 0:
			serial.sendData(BASEVARS_BASEMODE_DATA_ID1, base_mode);
			break;
		case 1:
			serial.sendData(BASEVARS_CUSTOMMODE_DATA_ID2, custom_mode);
			break;
		case 2:
			serial.sendData(BASEVARS_XACC_DATA_ID3, xacc);
			break;
		case 3:
			serial.sendData(BASEVARS_YACC_DATA_ID4, yacc);
			break;
		case 4:
			serial.sendData(BASEVARS_ZACC_DATA_ID5, zacc);
			break;
		case 5:
			serial.sendData(BASEVARS_XGYRO_DATA_ID6, xgyro);
			break;
		case 6:
			serial.sendData(BASEVARS_YGYRO_DATA_ID7, ygyro);
			break;
		case 7:
			serial.sendData(BASEVARS_ZGYRO_DATA_ID8, zgyro);
			break;
		case 8:
			serial.sendData(BASEVARS_XMAG_DATA_ID9, xmag);
			break;
		case 9:
			serial.sendData(BASEVARS_YMAG_DATA_ID10, ymag);
			break;
		case 10:
			serial.sendData(BASEVARS_ZMAG_DATA_ID11, zmag);
			break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= BASEVARS_DATA_COUNT) sensorDataIdx = 0;
  }
}