
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

// GPS		float lat, float lon, float alt, float speed, float cog, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second
// Vario	float altitude, float vsi
// Rpm		float rpm, float t1 = 0.0, float t2 = 0.0
// Flvss	float cell1, float cell2 = 0.0, float cell3 = 0.0, float cell4 = 0.0, float cell5 = 0.0, float cell6 = 0.0
// Fcs		float current, float voltage

// FrSky new DATA IDs (2 bytes)
// #define RSSI_ID                 0xf101
// #define ADC1_ID                 0xf102
// #define ADC2_ID                 0xf103
// #define BATT_ID                 0xf104
// #define SWR_ID                  0xf105
// #define T1_FIRST_ID             0x0400 - 0x040f
// #define T2_FIRST_ID             0x0410 - 0x041f
// #define RPM_FIRST_ID            0x0500 - 0x050f
// #define FUEL_FIRST_ID           0x0600 - 0x060f
// #define ALT_FIRST_ID            0x0100 - 0x010f
// #define VARIO_FIRST_ID          0x0110 - 0x011f
// #define ACCX_FIRST_ID           0x0700 - 0x070f
// #define ACCY_FIRST_ID           0x0710 - 0x071f
// #define ACCZ_FIRST_ID           0x0720 - 0x072f
// #define CURR_FIRST_ID           0x0200 - 0x020f
// #define VFAS_FIRST_ID           0x0210 - 0x021f
// #define CELLS_FIRST_ID          0x0300 - 0x030f
// #define GPS_LONG_LATI_FIRST_ID  0x0800 - 0x080f
// #define GPS_ALT_FIRST_ID        0x0820 - 0x082f
// #define GPS_SPEED_FIRST_ID      0x0830 - 0x083f
// #define GPS_COURS_FIRST_ID      0x0840 - 0x084f
// #define GPS_TIME_DATE_FIRST_ID  0x0850 - 0x085f
// https://github.com/opentx/opentx/blob/260da330c468f0c0e0090dc0ad11930046f3cc45/radio/src/telemetry/frsky.h#L206

// Pawelsky Lib
#include "FrSkySportSensorMotorOuts.h"
#include "FrSkySportSensor.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"

// FrSkySportSensorFcs fcs;                               // Create FCS sensor with default ID
// FrSkySportSensorFlvss flvss1;                          // Create FLVSS sensor with default ID
// FrSkySportSensorFlvss flvss2(FrSkySportSensor::ID15);  // Create FLVSS sensor with given ID
// FrSkySportSensorGps gps;                               // Create GPS sensor with default ID
// FrSkySportSensorRpm rpm;                               // Create RPM sensor with default ID
// FrSkySportSensorVario vario;                           // Create Variometer sensor with default ID
FrSkySportSensorMotorOuts motorouts;                               // Create RPM sensor with default ID
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
	// telemetry.begin(FRSKY_SERIAL, &fcs, &flvss1, &flvss2, &gps, &rpm, &vario);
  telemetry.begin(FRSKY_SERIAL, &motorouts);
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

	if (&motorouts) {
		add_timestamp(TIMESTAMP_FRSKY_RPM);
		// Set RPM/temperature sensor data
		// (set number of blades to 2 in telemetry menu to get correct rpm value)
		motorouts.setData(	200,    
							25.6,   
							-7.8,
							-7.8);
	}

  // Send the telemetry data, note that the data will only be sent for sensors
  // that are being polled at given moment
  telemetry.send();
}
