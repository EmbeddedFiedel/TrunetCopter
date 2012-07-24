#ifndef MAIN_H_
#define MAIN_H_

#define SEA_LEVEL_PRESSURE 1016.00

#define DEBUG
//#define DEBUG_OUTPUT_QUARTENION_BINARY
#define SERIAL_DEBUG SD3

#define SERIAL_GPS SD2

#define I2C_BUS I2CD1
#define MAX_I2C_BUF 14

#define I2C_THREADS_PRIO        (NORMALPRIO + 3)
#define GPS_THREAD_PRIO			(NORMALPRIO + 2)
#define TIMEKEEPER_THREAD_PRIO  (NORMALPRIO + 1)

#define GYRO_CAL_FLAG        (1UL << 0)
#define ACCEL_CAL_FLAG       (1UL << 1)
#define MAG_CAL_FLAG         (1UL << 2)
#define EEPROM_FAILED_FLAG   (1UL << 3)
#define POSTAGE_FAILED_FLAG  (1UL << 4)
#define I2C_RESTARTED_FLAG   (1UL << 5)

#define setGlobalFlag(flag)   {chSysLock(); GlobalFlags |= (flag); chSysUnlock();}
#define clearGlobalFlag(flag) {chSysLock(); GlobalFlags &= (~(flag)); chSysUnlock();}

#if (CH_FREQUENCY) >= 1000
#define TIME_BOOT_MS ((chTimeNow()) / ((CH_FREQUENCY) / 1000))
#endif

#define EEPROM_SETTINGS_START    8192
#define EEPROM_SETTINGS_SIZE     4096
#define EEPROM_SETTINGS_FINISH   (EEPROM_SETTINGS_START + EEPROM_SETTINGS_SIZE)

// Multiwii Serial Protocol 0
#define MSP_VERSION                              0

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         1 altitude
#define MSP_BAT                  110   //out message         vbat, powermetersum
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113   //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114   //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_WP_SET               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

#endif /* MAIN_H_ */
