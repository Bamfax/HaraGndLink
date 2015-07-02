/*
	Custom Sensor "BaseVars", by Bamfax, based upon:
	
	FrSky RPM sensor class for Teensy 3.x and 328P based boards (e.g. Pro Mini, Nano, Uno)
	(c) Pawelsky 20141120
	Not for commercial use
*/

#include "FrSkySportSensorBaseVars.h" 

FrSkySportSensorBaseVars::FrSkySportSensorBaseVars(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorBaseVars::setData(uint8_t base_mode, uint16_t custom_mode)
{
	FrSkySportSensorBaseVars::base_mode = (uint32_t)base_mode;
	FrSkySportSensorBaseVars::custom_mode = (uint32_t)custom_mode;
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
    }
    sensorDataIdx++;
    if(sensorDataIdx >= BASEVARS_DATA_COUNT) sensorDataIdx = 0;
  }
}