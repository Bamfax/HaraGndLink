
// Pawelsky FrSky Library
#include "FrSkySportSensorBaseVars.h"
#include "FrSkySportSensorMotorOuts.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorFcs.h"
#include "FrSkySportSensor.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"

// FrSkySportSensorFlvss flvss1;                          // Create FLVSS sensor with default ID
// FrSkySportSensorFlvss flvss2(FrSkySportSensor::ID15);  // Create FLVSS sensor with given ID
// FrSkySportSensorRpm rpm;                               // Create RPM sensor with default ID
//FrSkySportSensorFcs fcs;								// Create FCS sensor with default ID
FrSkySportSensorGps gps;								// Create GPS sensor with default ID
FrSkySportSensorVario vario;							// Create Variometer sensor with default ID
FrSkySportSensorMotorOuts motorouts;					// Create MotorOuts sensor with default ID
FrSkySportSensorBaseVars basevars;						// Create BaseVars sensor with default ID
FrSkySportTelemetry telemetry;							// Create Variometer telemetry object

#define EXPIRY_MILLIS_FRSKY_REQUEST   1200
#define EXPIRY_MILLIS_FRSKY_VARIO     1200
#define EXPIRY_MILLIS_FRSKY_FAS       1200
#define EXPIRY_MILLIS_FRSKY_GPS       1200
#define EXPIRY_MILLIS_FRSKY_RPM       1200
#define EXPIRY_MILLIS_FRSKY_OTHER     1200

short crc;                      
boolean waitingForSensorId = false;
uint8_t next_fas = 0;
uint8_t next_vario = 0;
uint8_t next_gps = 0;
uint8_t next_default = 0;
uint8_t gps_first_position_good = 0;

// Scale factor for roll/pitch:
// We need to scale down 360 deg to fit when max value is 256, and 256 equals 362 deg
float scalefactor = 360.0/((362.0/360.0)*256.0);

void frsky_init(void)  {
	// Configure the telemetry serial port and sensors (remember to use & to specify a pointer to sensor)
	// telemetry.begin(FRSKY_SERIAL, &fcs, &flvss1, &flvss2, &gps, &rpm, &vario);
	//telemetry.begin(FRSKY_SERIAL, &vario, &fcs, &gps, &motorouts, &basevars);
	telemetry.begin(FRSKY_SERIAL, &vario, &gps, &motorouts, &basevars);
}

int frsky_online() {  
  if(get_timestamp_age(TIMESTAMP_FRSKY_REQUEST) < EXPIRY_MILLIS_FRSKY_REQUEST) {
    return 1;
  } else {
    return 0;
  }
}

void frsky_process(void) {
	time_t t = 0;
//	uint8_t hdop_threshold;

/*
  if(!gps_first_position_good) {
    if(mav.gps_fixtype == 3 && mav.gps_hdop <= (EEPROM.read(EEPROM_ADDR_HDOP_THRESHOLD) * 100)) {
      gps_first_position_good = 1;
    }
  }
*/

	// Vario Sensor
	add_timestamp(TIMESTAMP_FRSKY_VARIO);
	if(mavlink_vfr_data_valid()) {
		// Set variometer data
		// (set Variometer source to VSpd in menu to use the vertical speed data from this sensor for variometer).
		vario.setData(	mav.bar_altitude,  // Altitude in Meters (can be negative)
						mav.ap_climb_rate);  // Vertical speed in m/s (positive - up, negative - down)
	}
	
	/*
	fcs.setData(	current,							// Current consumption in amps
					12.6);								// Battery voltage in volts
	debug_print(LOG_FRSKY_MOTOROUTS, "FRSKY MOTOROUTS: Current: %f, Voltage: %f", current, 12.6); 
*/

	// GPS Sensor
	add_timestamp(TIMESTAMP_FRSKY_GPS);
	t = (uint64_t)(mav.gps_time_usec/1000000ULL);
//	t = mav.gps_time_usec;
	//		if(mavlink_gps_data_valid() && gps_first_position_good) {
	// Set GPS data
	gps.setData(	
					// 48.858289, 2.294502,	// Latitude and longitude in degrees decimal (positive for N/E, negative for S/W) // removed Pawelsky code, as it is expecting true human float lat / lon values, but mavlink is throwing int32_t with centidegrees
					mav.gps_fixtype,
					mav.gps_satellites_visible,
					mav.gps_latitude,						// lat / lon from mavlink, which is int32_t in centidegrees
					mav.gps_longitude,						// lat / lon from mavlink, which is int32_t in centidegrees
					mav.gps_altitude,						// Altitude in m (can be negative)
					mav.gps_speed,							// Speed in m/s
					mav.gps_cog/100,						// Course over ground in degrees
					mav.gps_pdop,							// PDOP, goood :)
					year(t)-2000, month(t), day(t),			// Date (year - 2000, month, day)
					hour(t), minute(t), second(t));			// Time (hour, minute, second) - will be affected by timezone settings in your radio
	// }
	
	// Custom MotorOuts Sensor (based on RPM Sensor)
	add_timestamp(TIMESTAMP_FRSKY_RPM);
	motorouts.setData(	mav.motor1,    
						mav.motor2,   
						mav.motor3,
						mav.motor4);
	
	// Custom BaseVars Sensor
	basevars.setData(	mav.base_mode,
						mav.custom_mode,
						mav.heading,
						mav.imu_xacc,
						mav.imu_yacc,
						mav.imu_zacc,
						mav.imu_xgyro,
						mav.imu_ygyro,
						mav.imu_zgyro,
						mav.imu_xmag,
						mav.imu_ymag,
						mav.imu_zmag
						);
	
  // Send the telemetry data, note that the data will only be sent for sensors
  // that are being polled at given moment
  telemetry.send();
}
