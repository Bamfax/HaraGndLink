
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

// Standard Sensor IDs as in Pawelsky
//	Vario:		ID1, 0x00
//	Flvss:		ID2, 0xA1
//	FCS:		ID3, 0x22
//	GPS:		ID4, 0x83
//	RPM:		ID5, 0xE4
//	MotorOuts:	ID6, 0x45
//	BaseVars:	ID7, 0xC6
//     enum SensorId {	ID1 = 0x00,  ID2 = 0xA1,  ID3 = 0x22,  ID4 = 0x83,  ID5 = 0xE4,  ID6 = 0x45,  ID7 = 0xC6,
// 						ID8 = 0x67,  ID9 = 0x48,  ID10 = 0xE9, ID11 = 0x6A, ID12 = 0xCB, ID13 = 0xAC, ID14 = 0x0D,
// 						ID15 = 0x8E, ID16 = 0x2F, ID17 = 0xD0, ID18 = 0x71, ID19 = 0xF2, ID20 = 0x53, ID21 = 0x34,
//						ID22 = 0x95, ID23 = 0x16, ID24 = 0xB7, ID25 = 0x98, ID26 = 0x39, ID27 = 0xBA, ID28 = 0x1b };
//
// Standard Sensor IDs as in OpenTX 2.1
//	#define DATA_ID_VARIO            0x00 // 0
//	#define DATA_ID_FLVSS            0xA1 // 1
//	#define DATA_ID_FAS              0x22 // 2
//	#define DATA_ID_GPS              0x83 // 3
//	#define DATA_ID_RPM              0xE4 // 4
//	#define DATA_ID_SP2UH            0x45 // 5
//	#define DATA_ID_SP2UR            0xC6 // 6

// FrSky new DATA IDs (2 bytes) as in OpenTX 2.0
//	#define RSSI_ID                 0xf101
//	#define ADC1_ID                 0xf102
//	#define ADC2_ID                 0xf103
//	#define BATT_ID                 0xf104
//	#define SWR_ID                  0xf105
//	#define T1_FIRST_ID             0x0400 - 0x040f
//	#define T2_FIRST_ID             0x0410 - 0x041f
//	#define RPM_FIRST_ID            0x0500 - 0x050f
//	#define FUEL_FIRST_ID           0x0600 - 0x060f
//	#define ALT_FIRST_ID            0x0100 - 0x010f
//	#define VARIO_FIRST_ID          0x0110 - 0x011f
//	#define ACCX_FIRST_ID           0x0700 - 0x070f
//	#define ACCY_FIRST_ID           0x0710 - 0x071f
//	#define ACCZ_FIRST_ID           0x0720 - 0x072f
//	#define CURR_FIRST_ID           0x0200 - 0x020f
//	#define VFAS_FIRST_ID           0x0210 - 0x021f
//	#define CELLS_FIRST_ID          0x0300 - 0x030f
//	#define GPS_LONG_LATI_FIRST_ID  0x0800 - 0x080f
//	#define GPS_ALT_FIRST_ID        0x0820 - 0x082f
//	#define GPS_SPEED_FIRST_ID      0x0830 - 0x083f
//	#define GPS_COURS_FIRST_ID      0x0840 - 0x084f
//	#define GPS_TIME_DATE_FIRST_ID  0x0850 - 0x085f
//	https://github.com/opentx/opentx/blob/260da330c468f0c0e0090dc0ad11930046f3cc45/radio/src/telemetry/frsky.h#L206
//
// FrSky new DATA IDs (2 bytes) as in OpenTX 2.1
//	#define ALT_FIRST_ID            0x0100
//	#define ALT_LAST_ID             0x010f
//	#define VARIO_FIRST_ID          0x0110
//	#define VARIO_LAST_ID           0x011f
//	#define CURR_FIRST_ID           0x0200
//	#define CURR_LAST_ID            0x020f
//	#define VFAS_FIRST_ID           0x0210
//	#define VFAS_LAST_ID            0x021f
//	#define CELLS_FIRST_ID          0x0300
//	#define CELLS_LAST_ID           0x030f
//	#define T1_FIRST_ID             0x0400
//	#define T1_LAST_ID              0x040f
//	#define T2_FIRST_ID             0x0410
//	#define T2_LAST_ID              0x041f
//	#define RPM_FIRST_ID            0x0500
//	#define RPM_LAST_ID             0x050f
//	#define FUEL_FIRST_ID           0x0600
//	#define FUEL_LAST_ID            0x060f
//	#define ACCX_FIRST_ID           0x0700
//	#define ACCX_LAST_ID            0x070f
//	#define ACCY_FIRST_ID           0x0710
//	#define ACCY_LAST_ID            0x071f
//	#define ACCZ_FIRST_ID           0x0720
//	#define ACCZ_LAST_ID            0x072f
//	#define GPS_LONG_LATI_FIRST_ID  0x0800
//	#define GPS_LONG_LATI_LAST_ID   0x080f
//	#define GPS_ALT_FIRST_ID        0x0820
//	#define GPS_ALT_LAST_ID         0x082f
//	#define GPS_SPEED_FIRST_ID      0x0830
//	#define GPS_SPEED_LAST_ID       0x083f
//	#define GPS_COURS_FIRST_ID      0x0840
//	#define GPS_COURS_LAST_ID       0x084f
//	#define GPS_TIME_DATE_FIRST_ID  0x0850
//	#define GPS_TIME_DATE_LAST_ID   0x085f
//	#define A3_FIRST_ID             0x0900
//	#define A3_LAST_ID              0x090f
//	#define A4_FIRST_ID             0x0910
//	#define A4_LAST_ID              0x091f
//	#define AIR_SPEED_FIRST_ID      0x0a00
//	#define AIR_SPEED_LAST_ID       0x0a0f
//	#define RSSI_ID                 0xf101
//	#define ADC1_ID                 0xf102
//	#define ADC2_ID                 0xf103
//	#define SP2UART_A_ID            0xfd00
//	#define SP2UART_B_ID            0xfd01
//	#define BATT_ID                 0xf104
//	#define SWR_ID                  0xf105
//	#define XJT_VERSION_ID          0xf106
//	#define FUEL_QTY_FIRST_ID       0x0a10
//	#define FUEL_QTY_LAST_ID        0x0a1f
// https://github.com/opentx/opentx/blob/260da330c468f0c0e0090dc0ad11930046f3cc45/radio/src/telemetry/frsky.h#L206

// What we want:
//		Base:		Status: Armed/Disarmed
//					Mode: Stabilized/Acro/Baro/GPS
//					System Status
//					Mavlink Version
//					Heading								ok (FC Heading)
//					Angle Roll
//					Angle Pitch
//					Angle Yaw
//					Speed Roll
//					Speed Pitch
//					Speed Yaw
//		Vario:		Alt									ok
//					Climbrate							ok
//		GPS:		Date
//					Type of Fix
//					Number of Sats
//					HDOP
//					Speed
//					Alt
//					Course / Movement Direction
//					Lat / Lon
//		Current:	Current all
//					Voltage all
//		Misc:		MotorOuts 1-4

#define FRSKY_SERIAL    FrSkySportSingleWireSerial::SERIAL_1
