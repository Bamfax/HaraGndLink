
// a1                                           // dont know how to set this one 
// a2             <- gps_hdop / 4
// a3             <- roll
// a4             <- pitch   
// accx           <- average_xacc, xacc         //                               *configurable
// accy           <- average_yacc, yacc         //                               *configurable
// accz           <- average_zacc, zacc         //                               *configurable
// air-speed      <-                            // todo - dont know how to set this
// altitude       <- bar_altitude               // 100 = 1m     from barometer
// current        <- average_current, current   // 10  = 10ma                    *configurable
// fuel           <- custom_mode                //
// gps-altitude   <- gps_altitude               // 100 = 1m     from gps
// gps-speed      <- gps_speed                  // 100 = 1m/s   setting gps_speed sets air-speed too           
// heading        <- gps_heading                // 10000 = 100 degrees         
// latitude       <- gps_latitude
// longitude      <- gps_longitude
// rpm            <- used for text message
// temp1          <- gps_sats_visible * 10 + gps_fixtype
// temp2          <- battery_remaining, mission_current_seq, temperature, wp_dist  // *configurable
// vertical-speed <- ap_climb_rate              // 100 = 1m/s
// fcs           <- average_voltage, voltage   // 10  = 1mv                     *configurable

#include "FrSkySPort.h"
#include "TelemetryData.h"

#define FRSKY_SERIAL    FrSkySportSingleWireSerial::SERIAL_1

// FrSkySportSensorFcs fcs;                               // Create FCS sensor with default ID
// FrSkySportSensorFlvss flvss1;                          // Create FLVSS sensor with default ID
// FrSkySportSensorFlvss flvss2(FrSkySportSensor::ID15);  // Create FLVSS sensor with given ID
FrSkySportSensorGps gps;                               // Create GPS sensor with default ID
FrSkySportSensorRpm rpm;                               // Create RPM sensor with default ID
FrSkySportSensorVario vario;                           // Create Variometer sensor with default ID
FrSkySportTelemetry telemetry;                         // Create Variometer telemetry object

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
//  telemetry.begin(FRSKY_SERIAL, &fcs, &flvss1, &flvss2, &gps, &rpm, &vario);
  telemetry.begin(FRSKY_SERIAL, &gps, &rpm, &vario);
}

int frsky_online() {  
  if(get_timestamp_age(TIMESTAMP_FRSKY_REQUEST) < EXPIRY_MILLIS_FRSKY_REQUEST) {
    return 1;
  } else {
    return 0;
  }
}

void frsky_process(void) {
  uint32_t latlong = 0;
  uint8_t hdop_threshold;

	/*
  if(!gps_first_position_good) {
    if(mav.gps_fixtype == 3 && mav.gps_hdop <= (EEPROM.read(EEPROM_ADDR_HDOP_THRESHOLD) * 100)) {
      gps_first_position_good = 1;
    }
  }
	*/
	
  if (&vario) {
		add_timestamp(TIMESTAMP_FRSKY_VARIO);
		if(mavlink_vfr_data_valid()) {
			// Set variometer data
			// (set Variometer source to VSpd in menu to use the vertical speed data from this sensor for variometer).
			vario.setData(	mav.bar_altitude,  // Altitude in Meters (can be negative)
							mav.ap_climb_rate);  // Vertical speed in m/s (positive - up, negative - down)
		}
}

/*
// disabled as long as FrSky Sensors are still used.	      
	if (&fcs) {
		add_timestamp(TIMESTAMP_FRSKY_FCS);
		if(mavlink_sys_status_data_valid() && EEPROM.read(EEPROM_ADDR_FRSKY_FCS_ENABLE)) {
			// Set current/voltage sensor (FCS) data
			// (set Voltage source to FAS in menu to use this data for battery voltage,
			//  set Current source to FAS in menu to use this data for current readins)
			fcs.setData(	25.3,   // Current consumption in amps
							12.6);  // Battery voltage in volts
		}
  }
*/
	
  if (&gps) {
		add_timestamp(TIMESTAMP_FRSKY_GPS);
//		if(mavlink_gps_data_valid() && gps_first_position_good) {
			// Set GPS data
			gps.setData(	48.858289, 2.294502,   // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
							mav.gps_altitude,      // Altitude in m (can be nevative)
							mav.gps_speed,         // Speed in m/s
							mav.heading,           // Course over ground in degrees
							14, 9, 14,             // Date (year - 2000, month, day)
							12, 00, 00);           // Time (hour, minute, second) - will be affected by timezone setings in your radio
//			}
	}
  
  if (&rpm) {
		add_timestamp(TIMESTAMP_FRSKY_RPM);
		// Set RPM/temperature sensor data
		// (set number of blades to 2 in telemetry menu to get correct rpm value)
		rpm.setData(200,    // Rotations per minute
								25.6,   // Temperature #1 in degrees Celsuis (can be negative)
								-7.8);  // Temperature #1 in degrees Celsuis (can be negative)
	}

/*
// disabled as long as FrSky Sensors are still used.	
	if (&flvss1) {
		add_timestamp(TIMESTAMP_FRSKY_FLVSS);
		// Set LiPo voltage sensor (FLVSS) data (we use two sensors to simulate 8S battery 
		// (set Voltage source to Cells in menu to use this data for battery voltage)
		flvss1.setData(4.07, 4.08, 4.09, 4.10, 4.11, 4.12);  // Cell voltages in volts (cells 1-6)
		flvss2.setData(4.13, 4.14);                          // Cell voltages in volts (cells 7-8)
  }
*/	
	
  // Send the telemetry data, note that the data will only be sent for sensors
  // that are being polled at given moment
  telemetry.send();
}
