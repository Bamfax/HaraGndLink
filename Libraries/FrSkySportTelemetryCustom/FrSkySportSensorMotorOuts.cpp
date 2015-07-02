/*
  FrSky RPM sensor class for Teensy 3.x and 328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20141120
  Not for commercial use
*/

#include "FrSkySportSensorMotorOuts.h" 

FrSkySportSensorMotorOuts::FrSkySportSensorMotorOuts(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorMotorOuts::setData(float motorout1, float motorout2, float motorout3, float motorout4)
{
	FrSkySportSensorMotorOuts::motorout1 = (uint32_t)motorout1;
	FrSkySportSensorMotorOuts::motorout2 = (uint32_t)motorout2;
	FrSkySportSensorMotorOuts::motorout3 = (uint32_t)motorout3;
	FrSkySportSensorMotorOuts::motorout4 = (uint32_t)motorout4;
}

void FrSkySportSensorMotorOuts::send(FrSkySportSingleWireSerial& serial, uint8_t id)
{
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        serial.sendData(RPM_ROT_DATA_ID1, motorout1);
        break;
      case 1:
        serial.sendData(RPM_ROT_DATA_ID2, motorout2);
        break;
      case 2:
        serial.sendData(RPM_ROT_DATA_ID3, motorout3);
        break;
			case 3:
        serial.sendData(RPM_ROT_DATA_ID4, motorout4);
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= RPM_DATA_COUNT) sensorDataIdx = 0;
  }
}

