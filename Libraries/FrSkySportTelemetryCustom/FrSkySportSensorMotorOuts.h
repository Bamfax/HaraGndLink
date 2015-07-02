/*
  FrSky RPM sensor class for Teensy 3.x and 328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20141120
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_MOTOROUT_H_
#define _FRSKY_SPORT_SENSOR_MOTOROUT_H_

#include "FrSkySportSensor.h"

#define RPM_DEFAULT_ID ID6
#define RPM_DATA_COUNT 4
#define RPM_ROT_DATA_ID1 0x0500
#define RPM_ROT_DATA_ID2 0x0501
#define RPM_ROT_DATA_ID3 0x0502
#define RPM_ROT_DATA_ID4 0x0503

class FrSkySportSensorMotorOuts : public FrSkySportSensor
{
  public:
    FrSkySportSensorMotorOuts(SensorId id = RPM_DEFAULT_ID);
    void setData(float motorout1, float motorout2, float motorout3, float motorout4);
    virtual void send(FrSkySportSingleWireSerial& serial, uint8_t id);

  private:
    uint32_t motorout1;
    uint32_t motorout2;
    uint32_t motorout3;
	uint32_t motorout4;
};

#endif // _FRSKY_SPORT_SENSOR_MOTOROUT_H_
