/*
	Custom Sensor "BaseVars", by Bamfax, based upon:
	
	FrSky RPM sensor class for Teensy 3.x and 328P based boards (e.g. Pro Mini, Nano, Uno)
	(c) Pawelsky 20141120
	Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_BASEVARS_H_
#define _FRSKY_SPORT_SENSOR_BASEVARS_H_

#include "FrSkySportSensor.h"

#define BASEVARS_DEFAULT_ID ID7
#define BASEVARS_DATA_COUNT 2
#define BASEVARS_BASEMODE_DATA_ID1 0x0410		// T2_FIRST_ID, as in lvale. There is no perfect fit for this FrSky AppIDs.
#define BASEVARS_CUSTOMMODE_DATA_ID2 0x0600		// FUEL_FIRST_ID, as in lvale. There is no perfect fit for this FrSky AppIDs.

class FrSkySportSensorBaseVars : public FrSkySportSensor
{
  public:
    FrSkySportSensorBaseVars(SensorId id = BASEVARS_DEFAULT_ID);
    void setData(uint8_t base_mode, uint16_t custom_mode);
    virtual void send(FrSkySportSingleWireSerial& serial, uint8_t id);

  private:
    uint8_t base_mode;
    uint16_t custom_mode;
};

#endif // _FRSKY_SPORT_SENSOR_MOTOROUT_H_
