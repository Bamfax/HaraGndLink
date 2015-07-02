/*
	Some adaptions to Pawelsky code, as it was expecting true human float lat / lon values, but mavlink is throwing centidegrees
	
	FrSky GPS sensor class for Teensy 3.x and 328P based boards (e.g. Pro Mini, Nano, Uno)
	(c) Pawelsky 20141120
	Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_GPS_H_
#define _FRSKY_SPORT_SENSOR_GPS_H_

#include "FrSkySportSensor.h"

#define GPS_DEFAULT_ID ID4
#define GPS_DATA_COUNT 10
#define GPS_FIX_DATA_ID			0x0411		// T2_FIRST_ID + 1 ...hopefully an int, we will see
#define GPS_SATCOUNT_DATA_ID	0x0412		// T2_FIRST_ID + 2 ...hopefully an int, we will see
#define GPS_LAT_LON_DATA_ID		0x0800
#define GPS_ALT_DATA_ID			0x0820
#define GPS_SPEED_DATA_ID		0x0830
#define GPS_COG_DATA_ID			0x0840
#define GPS_HDOP_DATA_ID		0x0413		// T2_FIRST_ID + 3 ...hopefully a float, we will see
#define GPS_DATE_TIME_DATA_ID	0x0850


class FrSkySportSensorGps : public FrSkySportSensor
{
  public:
    FrSkySportSensorGps(SensorId id = GPS_DEFAULT_ID);
    void setData(uint8_t fixtype, uint8_t satellites_visible, int32_t lat, int32_t lon, float alt, float speed, float cog, uint16_t hdop, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
    virtual void send(FrSkySportSingleWireSerial& serial, uint8_t id);

  private:
    static uint32_t setLatLon(int32_t latLon, bool isLat);
    static uint32_t setDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate);
    uint8_t fixtype;
	uint8_t satellites_visible;
	uint32_t lat;
    uint32_t lon;
    int32_t alt;
    uint32_t speed;
    uint32_t cog;
	uint16_t hdop;
    uint32_t date;
    uint32_t time;
};

#endif // _FRSKY_SPORT_SENSOR_GPS_H_
