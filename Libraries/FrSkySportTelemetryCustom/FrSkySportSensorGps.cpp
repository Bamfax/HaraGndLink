/*
	Some adaptions to Pawelsky code, as it was expecting true human float lat / lon values, but mavlink is throwing centidegrees

	FrSky GPS sensor class for Teensy 3.x and 328P based boards (e.g. Pro Mini, Nano, Uno)
	(c) Pawelsky 20141129
	Not for commercial use
*/

#include "FrSkySportSensorGps.h" 

FrSkySportSensorGps::FrSkySportSensorGps(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorGps::setData(uint8_t fixtype, uint8_t satellites_visible, int32_t lat, int32_t lon, float alt, float speed, float cog, uint16_t hdop, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
  FrSkySportSensorGps::fixtype = fixtype;
  FrSkySportSensorGps::satellites_visible = satellites_visible;
  FrSkySportSensorGps::lat = setLatLon(lat, true);
  FrSkySportSensorGps::lon = setLatLon(lon, false);
  FrSkySportSensorGps::cog = cog;
  FrSkySportSensorGps::speed = speed * 1944; // Convert m/s to knots
  FrSkySportSensorGps::alt = alt * 100;
  FrSkySportSensorGps::hdop = hdop;
  FrSkySportSensorGps::date = setDateTime(year, month, day, true);
  FrSkySportSensorGps::time = setDateTime(hour, minute, second, false);
}

uint32_t FrSkySportSensorGps::setLatLon(int32_t latLon, bool isLat)
{
// removed Pawelsky code, as it is expecting true human float lat / lon values, but mavlink is throwing int32_t with centidegrees
//  uint32_t data = (uint32_t)((latLon < 0 ? -latLon : latLon) * 60 * 10000) & 0x3FFFFFFF;
//  if(isLat == false) data |= 0x80000000;
//  if(latLon < 0) data |= 0x40000000;

//	Taken from Wolkstein
	if(isLat==true) {
		if(isLat < 0 )
		latLon = ((abs(latLon)/100)*6) | 0x40000000;
		else
		latLon = ((abs(latLon)/100)*6);
	} else {
		if(isLat < 0)
		latLon = ((abs(latLon)/100)*6)  | 0xC0000000;
		else
		latLon = ((abs(latLon)/100)*6)  | 0x80000000;
	}
	uint32_t data = (uint32_t) latLon;

	return data;
}

uint32_t FrSkySportSensorGps::setDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate)
{
  uint32_t data = yearOrHour;
  data <<= 8;
  data |= monthOrMinute;
  data <<= 8;
  data |= dayOrSecond;
  data <<= 8;
  if(isDate == true) data |= 0xFF;

  return data;
}

void FrSkySportSensorGps::send(FrSkySportSingleWireSerial& serial, uint8_t id)
{
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
		case 0:
			serial.sendData(GPS_FIX_DATA_ID, fixtype);
			break;
		case 1:
			serial.sendData(GPS_SATCOUNT_DATA_ID, satellites_visible);
			break;
		case 2:
			serial.sendData(GPS_LAT_LON_DATA_ID, lat);
			break;
		case 3:
			serial.sendData(GPS_LAT_LON_DATA_ID, lon);
			break;
		case 4:
			serial.sendData(GPS_ALT_DATA_ID, alt);
			break;
		case 5:
			serial.sendData(GPS_SPEED_DATA_ID, speed);
			break;
		case 6:
			serial.sendData(GPS_COG_DATA_ID, cog);
			break;
		case 7:
			serial.sendData(GPS_HDOP_DATA_ID, hdop);
			break;
		case 8:        
			serial.sendData(GPS_DATE_TIME_DATA_ID, date);
			break;
		case 9:
			serial.sendData(GPS_DATE_TIME_DATA_ID, time);
			break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= GPS_DATA_COUNT) sensorDataIdx = 0;
  }
}
