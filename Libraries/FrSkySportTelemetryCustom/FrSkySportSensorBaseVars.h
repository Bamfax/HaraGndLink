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
#define BASEVARS_DATA_COUNT 12
#define BASEVARS_BASEMODE_DATA_ID1 0x0410		// T2_FIRST_ID, as in lvale. There is no perfect fit for this FrSky AppIDs.
#define BASEVARS_CUSTOMMODE_DATA_ID2 0x0600		// FUEL_FIRST_ID, as in lvale. There is no perfect fit for this FrSky AppIDs.
#define BASEVARS_HEADING_DATA_ID3 0x0841		
#define BASEVARS_XACC_DATA_ID4 0x0700			// Raw IMU ACC
#define BASEVARS_YACC_DATA_ID5 0x0710
#define BASEVARS_ZACC_DATA_ID6 0x0720
#define BASEVARS_XGYRO_DATA_ID7 0x0701			// Raw IMU Gyro
#define BASEVARS_YGYRO_DATA_ID8 0x0711
#define BASEVARS_ZGYRO_DATA_ID9 0x0721
#define BASEVARS_XMAG_DATA_ID10 0x0702			// Raw IMU Mag
#define BASEVARS_YMAG_DATA_ID11 0x0712
#define BASEVARS_ZMAG_DATA_ID12 0x0722

class FrSkySportSensorBaseVars : public FrSkySportSensor
{
  public:
    FrSkySportSensorBaseVars(SensorId id = BASEVARS_DEFAULT_ID);
    void setData(uint8_t base_mode, uint16_t custom_mode, int16_t heading, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag);
    virtual void send(FrSkySportSingleWireSerial& serial, uint8_t id);

  private:
    uint8_t base_mode;
    uint16_t custom_mode;
	int16_t heading;
	int16_t xacc;
	int16_t yacc;
	int16_t zacc;
	int16_t xgyro;
	int16_t ygyro;
	int16_t zgyro;
	int16_t xmag;
	int16_t ymag;
	int16_t zmag;
};

#endif // _FRSKY_SPORT_SENSOR_MOTOROUT_H_
