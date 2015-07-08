#include "MavLink.h"

void mavlink_init() {
  MAVLINK_SERIAL.begin(MAVLINK_BAUD);
}

#define EXPIRY_MILLIS_MAVLINK_MSG_ID_HEARTBEAT   1200
#define EXPIRY_MILLIS_MAVLINK_MSG_ID_SYS_STATUS  1200
#define EXPIRY_MILLIS_MAVLINK_MSG_ID_GPS_RAW_INT 1200
#define EXPIRY_MILLIS_MAVLINK_MSG_ID_VFR_HUD     1200
#define EXPIRY_MILLIS_MAVLINK_MSG_ID_RAW_IMU     1200
#define EXPIRY_MILLIS_MAVLINK_MSG_ID_ATTITUDE    1200

#define STATUS_TEXT_MAX 128
char status_text_buffer[STATUS_TEXT_MAX];

void mavlink_average_push(int16_t data, int16_t* p_buffer, int16_t* p_start, int16_t* p_length, int16_t max_length) {
  p_buffer[(*p_start + *p_length) % max_length] = data;
  if(*p_length >= max_length) {
    *p_start = (*p_start + 1) % max_length;
  } else {
    (*p_length)++;
  }
}

int16_t mavlink_get_average(int16_t* p_buffer, int16_t start, int16_t length, int16_t use_samples, int16_t max_length) {
  int16_t i;
  int32_t sum = 0;
  
  if(length < use_samples) {
    use_samples = length;
  }
  uint16_t start_index = length - use_samples;
  for(i=start_index; i<length; i++) {
      sum += p_buffer[(start + i) % max_length];
  }
  int16_t round_val;
  if(sum >= 0) {
    round_val = use_samples / 2;
  } else {
    round_val = (0 - use_samples) / 2;
  }
  return (int16_t)((sum + round_val)/ use_samples);                     // round
}

uint16_t mavlink_calculate_imu_peak(int16_t current_acc, int16_t* p_previous_acc, int16_t* p_lowest_acc, int16_t* p_highest_acc, int16_t average) {
  if(current_acc < average && *p_previous_acc > average) {                      // going under average
    *p_lowest_acc = current_acc;
  } else if(current_acc > average && *p_previous_acc < average) {               // going over average
    *p_highest_acc = current_acc;
  }
  if (current_acc > average && current_acc > *p_highest_acc) {
    *p_highest_acc = current_acc;
  } else if(current_acc < average && current_acc < *p_lowest_acc) {
    *p_lowest_acc = current_acc;
  }
  *p_previous_acc = current_acc;
  return (*p_highest_acc - *p_lowest_acc) / 2;
}

int mavlink_online() {
  if(get_timestamp_age(TIMESTAMP_MAVLINK_MSG_ID_HEARTBEAT) < EXPIRY_MILLIS_MAVLINK_MSG_ID_HEARTBEAT) {
    return 1;
  } else {
    return 0;
  }
}

int mavlink_sys_status_data_valid() {
  if(get_timestamp_age(TIMESTAMP_MAVLINK_MSG_ID_SYS_STATUS) < EXPIRY_MILLIS_MAVLINK_MSG_ID_SYS_STATUS) {
    return 1;
  } else {
    return 0;
  }
}

int mavlink_gps_data_valid() {
  if(get_timestamp_age(TIMESTAMP_MAVLINK_MSG_ID_GPS_RAW_INT) < EXPIRY_MILLIS_MAVLINK_MSG_ID_GPS_RAW_INT) {
    return 1;
  } else {
    return 0;
  }
}

int mavlink_vfr_data_valid() {
  if(get_timestamp_age(TIMESTAMP_MAVLINK_MSG_ID_VFR_HUD) < EXPIRY_MILLIS_MAVLINK_MSG_ID_VFR_HUD) {
    return 1;
  } else {
    return 0;
  }
}

int mavlink_imu_data_valid() {
  if(get_timestamp_age(TIMESTAMP_MAVLINK_MSG_ID_RAW_IMU) < EXPIRY_MILLIS_MAVLINK_MSG_ID_RAW_IMU) {
    return 1;
  } else {
    return 0;
  }
}

int mavlink_attitude_data_valid() {
  if(get_timestamp_age(TIMESTAMP_MAVLINK_MSG_ID_ATTITUDE) < EXPIRY_MILLIS_MAVLINK_MSG_ID_ATTITUDE) {
    return 1;
  } else {
    return 0;
  }
}

void process_mavlink_packets() { 
  mavlink_message_t msg;
  mavlink_status_t status;

  while(MAVLINK_SERIAL.available()) { 
    uint8_t c = MAVLINK_SERIAL.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
/*			DEBUG_SERIAL.print("msg.msgid: ");
			DEBUG_SERIAL.print(msg.msgid);
			DEBUG_SERIAL.println();*/
				
      switch(msg.msgid) {
				
        case MAVLINK_MSG_ID_HEARTBEAT: 
			debug_print(LOG_MAV_HEARTBEAT, "MAVLINK_MSG_ID_HEARTBEAT: base_mode: %d, custom_mode: %d", mavlink_msg_heartbeat_get_base_mode(&msg), mavlink_msg_heartbeat_get_custom_mode(&msg));            
			add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_HEARTBEAT);
			/* MAV_MODE_FLAG / base_mode Bit Meanings, from Mavlink common/common.h
			  85         MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1,	// 0b00000001 Reserved for future use.
			  86         MAV_MODE_FLAG_TEST_ENABLED=2,			// 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
			  87         MAV_MODE_FLAG_AUTO_ENABLED=4,			// 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
			  88         MAV_MODE_FLAG_GUIDED_ENABLED=8,		// 0b00001000 guided mode enabled, system flies MISSIONs / mission items.
			  89         MAV_MODE_FLAG_STABILIZE_ENABLED=16,	// 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
			  90         MAV_MODE_FLAG_HIL_ENABLED=32,			// 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
			  91         MAV_MODE_FLAG_MANUAL_INPUT_ENABLED=64, // 0b01000000 remote control input is enabled.
			  92         MAV_MODE_FLAG_SAFETY_ARMED=128,		// 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
			  93         MAV_MODE_FLAG_ENUM_END=129,			//  | 
			*/	  
			mav.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
			mav.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
		break;
  
        case MAVLINK_MSG_ID_VFR_HUD:
//					DEBUG_SERIAL.print("Entered VFR "); DEBUG_SERIAL.println();
//					DEBUG_SERIAL.print("Heading: "); DEBUG_SERIAL.print(mavlink_msg_vfr_hud_get_heading(&msg)); DEBUG_SERIAL.println();
//          debug_print(LOG_MAV_HUD, "MAVLINK_MSG_ID_VFR_HUD: groundspeed: %d, heading: %d, throttle: %d, alt: %d, climbrate: %d", mavlink_msg_vfr_hud_get_groundspeed(&msg), mavlink_msg_vfr_hud_get_heading(&msg), mavlink_msg_vfr_hud_get_throttle(&msg), mavlink_msg_vfr_hud_get_alt(&msg), mavlink_msg_vfr_hud_get_climb(&msg));            
			mav.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);      // 100 = 1m/s
			mav.heading = mavlink_msg_vfr_hud_get_heading(&msg);              // 100 = 100 deg
			mav.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);            //  100 = 100%
			mav.bar_altitude = mavlink_msg_vfr_hud_get_alt(&msg) * 100;       //  m
			mav.ap_climb_rate=mavlink_msg_vfr_hud_get_climb(&msg) * 100;      //  m/s
			debug_print(LOG_MAV_HUD, "MAVLINK_MSG_ID_VFR_HUD: groundspeed: %d, heading: %d, throttle: %d, alt: %d, climbrate: %d", mav.groundspeed, mav.heading, mav.throttle, mav.bar_altitude, mav.ap_climb_rate);            
			add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_VFR_HUD);
          break; 
  
        case MAVLINK_MSG_ID_STATUSTEXT:
          add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_STATUSTEXT);
          mavlink_msg_statustext_decode(&msg, &mav.statustext);
          sprintf(status_text_buffer, "%d%s", mav.statustext.severity & 0x7, mav.statustext.text);
          frsky_send_text_message(status_text_buffer);          
          debug_print(LOG_MAV_TEXT, "MAVLINK_MSG_ID_STATUSTEXT: severity: %ld text: %s", mav.statustext.severity, mav.statustext.text);
          break;
  
        case MAVLINK_MSG_ID_SYS_STATUS : 
          mav.battery_voltage = max(mavlink_msg_sys_status_get_voltage_battery(&msg), 0);     // 1 = 1mV              // avoid negative values
          mav.battery_current = max(mavlink_msg_sys_status_get_current_battery(&msg), 0);     // 1 = 10mA
          mavlink_average_push(mav.battery_voltage, mav.battery_voltage_buffer, &(mav.battery_voltage_buffer_start), &(mav.battery_voltage_buffer_length), MAV_HISTORY_BUFFER_SIZE);
          mavlink_average_push(mav.battery_current, mav.battery_current_buffer, &(mav.battery_current_buffer_start), &(mav.battery_current_buffer_length), MAV_HISTORY_BUFFER_SIZE);
          mav.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg); //battery capacity reported in %
          debug_print(LOG_MAV_STATUS, "MAVLINK_MSG_ID_SYS_STATUS: voltage_battery: %d, current_battery: %d", mavlink_msg_sys_status_get_voltage_battery(&msg), mavlink_msg_sys_status_get_current_battery(&msg));            
          add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_SYS_STATUS);
          break;
  
        case MAVLINK_MSG_ID_GPS_RAW_INT:
          mav.gps_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                              // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          if(mav.gps_fixtype == 3) {
            mav.gps_satellites_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);      
            mav.gps_hdop = mavlink_msg_gps_raw_int_get_eph(&msg);                                    // hdop * 100
			mav.gps_vdop = mavlink_msg_gps_raw_int_get_epv(&msg);
            mav.gps_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
            mav.gps_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
            mav.gps_altitude = mavlink_msg_gps_raw_int_get_alt(&msg);                                // 1m =1000
            mav.gps_speed = mavlink_msg_gps_raw_int_get_vel(&msg);                                   // 100 = 1m/s
          } else {
            mav.gps_satellites_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);      
            mav.gps_hdop = 9999;
            mav.gps_latitude = 0L;
            mav.gps_longitude = 0L;
            mav.gps_altitude = 0L;                       
            mav.gps_speed = 0L;                     
          }
          debug_print(LOG_MAV_GPS, "MAVLINK_MSG_ID_GPS_RAW_INT: fixtype: %d, visiblesats: %d, gpsspeed: %f, hdop: %d, vdop: %d, alt: %d", mav.gps_fixtype, mav.gps_satellites_visible, mav.gps_speed/100.0, mav.gps_hdop, mav.gps_vdop, mav.gps_altitude);            
          add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_GPS_RAW_INT);
          break;
  
        case MAVLINK_MSG_ID_RAW_IMU:
          mav.imu_xacc = mavlink_msg_raw_imu_get_xacc(&msg);
          mav.imu_yacc = mavlink_msg_raw_imu_get_yacc(&msg);
          mav.imu_zacc = mavlink_msg_raw_imu_get_zacc(&msg);
		  mav.imu_xgyro = mavlink_msg_raw_imu_get_xgyro(&msg);
		  mav.imu_ygyro = mavlink_msg_raw_imu_get_ygyro(&msg);
		  mav.imu_zgyro = mavlink_msg_raw_imu_get_zgyro(&msg);
		  mav.imu_xmag = mavlink_msg_raw_imu_get_xmag(&msg);
		  mav.imu_ymag = mavlink_msg_raw_imu_get_ymag(&msg);
		  mav.imu_zmag = mavlink_msg_raw_imu_get_zmag(&msg);
		  
		  /* disabling averaging for now.
          uint16_t xacc_average, yacc_average, zacc_average;
          xacc_average = mavlink_get_average(mav.imu_xacc_buffer, mav.imu_xacc_buffer_start, mav.imu_xacc_buffer_length, 10, MAV_HISTORY_BUFFER_SIZE);
          yacc_average = mavlink_get_average(mav.imu_yacc_buffer, mav.imu_yacc_buffer_start, mav.imu_yacc_buffer_length, 10, MAV_HISTORY_BUFFER_SIZE);
          zacc_average = mavlink_get_average(mav.imu_zacc_buffer, mav.imu_zacc_buffer_start, mav.imu_zacc_buffer_length, 10, MAV_HISTORY_BUFFER_SIZE);
          mav.imu_xacc_peak = mavlink_calculate_imu_peak(mav.imu_xacc, &mav.imu_xacc_previous, &mav.imu_xacc_lowest, &mav.imu_xacc_highest, xacc_average);
          mav.imu_yacc_peak = mavlink_calculate_imu_peak(mav.imu_yacc, &mav.imu_yacc_previous, &mav.imu_yacc_lowest, &mav.imu_yacc_highest, yacc_average);
          mav.imu_zacc_peak = mavlink_calculate_imu_peak(mav.imu_zacc, &mav.imu_zacc_previous, &mav.imu_zacc_lowest, &mav.imu_zacc_highest, zacc_average);
          
          mavlink_average_push(mav.imu_xacc, mav.imu_xacc_buffer, &(mav.imu_xacc_buffer_start), &(mav.imu_xacc_buffer_length), MAV_HISTORY_BUFFER_SIZE);
          mavlink_average_push(mav.imu_yacc, mav.imu_yacc_buffer, &(mav.imu_yacc_buffer_start), &(mav.imu_yacc_buffer_length), MAV_HISTORY_BUFFER_SIZE);
          mavlink_average_push(mav.imu_zacc, mav.imu_zacc_buffer, &(mav.imu_zacc_buffer_start), &(mav.imu_zacc_buffer_length), MAV_HISTORY_BUFFER_SIZE);

          mavlink_average_push(mav.imu_xacc_peak, mav.imu_xacc_peak_buffer, &(mav.imu_xacc_peak_buffer_start), &(mav.imu_xacc_peak_buffer_length), MAV_HISTORY_BUFFER_SIZE);
          mavlink_average_push(mav.imu_yacc_peak, mav.imu_yacc_peak_buffer, &(mav.imu_yacc_peak_buffer_start), &(mav.imu_yacc_peak_buffer_length), MAV_HISTORY_BUFFER_SIZE);
          mavlink_average_push(mav.imu_zacc_peak, mav.imu_zacc_peak_buffer, &(mav.imu_zacc_peak_buffer_start), &(mav.imu_zacc_peak_buffer_length), MAV_HISTORY_BUFFER_SIZE);
		  */
		  
          debug_print(LOG_MAV_IMU, "MAVLINK_MSG_ID_RAW_IMU: xacc: %d, yacc: %d, zacc: %d", mav.imu_xacc, mav.imu_yacc, mav.imu_zacc);
		  debug_print(LOG_MAV_IMU, "MAVLINK_MSG_ID_RAW_IMU: xgyro: %d, ygyro: %d, zgyro: %d", mav.imu_xgyro, mav.imu_ygyro, mav.imu_zgyro);
		  debug_print(LOG_MAV_IMU, "MAVLINK_MSG_ID_RAW_IMU: xmag: %d, ymag: %d, zmag: %d", mav.imu_xmag, mav.imu_ymag, mav.imu_zmag);
          add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_RAW_IMU);
          break;
  
        case MAVLINK_MSG_ID_ATTITUDE:     //30
         mav.roll_angle = mavlink_msg_attitude_get_roll(&msg)*180/3.1416;  //value comes in rads, convert to deg
          if(abs(mav.roll_angle) <= 90) {
           mav.pitch_angle = mavlink_msg_attitude_get_pitch(&msg)*180/3.1416; //value comes in rads, convert to deg
          } 
          else {
           mav.pitch_angle = 180-mavlink_msg_attitude_get_pitch(&msg)*180/3.1416;   // upside down.  value comes in rads, convert to deg
          }
          mav.yaw_angle = (mavlink_msg_attitude_get_yaw(&msg)+3.1416)*162.9747; //value comes in rads, add pi and scale to 0 to 1024
          debug_print(LOG_MAV_ATTITUDE, "MAVLINK_MSG_ID_ATTITUDE: roll_angle: %f (%d deg), pitch_angle: %f (%d deg), yaw_angle: %f", mavlink_msg_attitude_get_roll(&msg),mav.roll_angle, mavlink_msg_attitude_get_pitch(&msg), mav.pitch_angle, mavlink_msg_attitude_get_yaw(&msg));            
          add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_ATTITUDE);
          break;
  
        case MAVLINK_MSG_ID_MISSION_CURRENT:                                  // 42
          mav.mission_current_seq = mavlink_msg_mission_current_get_seq(&msg);
          debug_print(LOG_MAV_OTHER, "MAVLINK MSG_ID: %d", msg.msgid);            
          add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_MISSION_CURRENT);
          break;

        case MAVLINK_MSG_ID_SCALED_PRESSURE:                                  // 29
          mav.temperature = mavlink_msg_scaled_pressure_get_temperature(&msg);
          debug_print(LOG_MAV_OTHER, "MAVLINK MSG_ID: %d", msg.msgid);            
          add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_SCALED_PRESSURE);
          break;

        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:                            // 62
          mav.wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
          debug_print(LOG_MAV_OTHER, "MAVLINK MSG_ID: %d", msg.msgid);            
          add_timestamp(TIMESTAMP_MAVLINK_MSG_ID_CONTROLLER_OUTPUT);
          break;
          
        case MAVLINK_MSG_ID_SYSTEM_TIME:                                      // 2
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:                              // 33
        case MAVLINK_MSG_ID_SENSOR_OFFSETS:                                   // 150
        case MAVLINK_MSG_ID_MEMINFO:                                          // 152
        case MAVLINK_MSG_ID_AHRS:                                             // 163
        case MAVLINK_MSG_ID_HWSTATUS:                                         // 165
          debug_print(LOG_MAV_OTHER, "MAVLINK MSG_ID: %d", msg.msgid);            
          break;
          
        default:
          debug_print(LOG_MAV_UNKNOWN, "Unhandled MAVLINK message %d", msg.msgid);           
          break;
					
      }
    }
  }
}



