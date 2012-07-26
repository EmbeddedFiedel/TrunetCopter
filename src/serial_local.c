#include "serial_local.h"

#include <string.h>

static volatile uint8_t serialHeadRX[1],serialTailRX[1];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][1];
static volatile uint8_t headTX,tailTX;
static uint8_t bufTX[TX_BUFFER_SIZE];
static uint8_t inBuf[INBUF_SIZE];

//extern EventSource imu_event;
//extern baro_data_t baro_data;
//extern imu_data_t imu_data;
extern gps_data_t gps_data;
extern float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame

extern float sampleFreq;
extern Mailbox mb[3];
extern msg_t mbBuf[3][MAILBOX_MSG_SIZE];

extern volatile unsigned short RC_INPUT_CHANNELS[4];

static WORKING_AREA(SerialThreadWA, 512);
static msg_t SerialThread(void *arg){
        (void)arg;
        chRegSetThreadName("Serial");

#ifdef DEBUG_OUTPUT_QUARTENION_BINARY
	uint16_t cnt_debug = 0;
#endif

        while (TRUE) {
        	baro_data_t baro_data;
			chMBFetch(&mb[MAILBOX_BARO], mbBuf[MAILBOX_BARO], TIME_IMMEDIATE);
			memcpy(&baro_data, &mbBuf[MAILBOX_BARO], sizeof(baro_data_t));
			chMBReset(&mb[MAILBOX_BARO]);

			imu_data_t imu_data;
			chMBFetch(&mb[MAILBOX_IMU], mbBuf[MAILBOX_IMU], TIME_IMMEDIATE);
			memcpy(&imu_data, &mbBuf[MAILBOX_IMU], sizeof(imu_data_t));
			chMBReset(&mb[MAILBOX_IMU]);
#ifndef DEBUG_OUTPUT_QUARTENION_BINARY
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "frequency: %f\r\n", sampleFreq);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "----------------------------------\r\n");
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "RX (Channel 1, 2, 3, 4): %d, %d, %d, %d\r\n", RC_INPUT_CHANNELS[0], RC_INPUT_CHANNELS[1], RC_INPUT_CHANNELS[2], RC_INPUT_CHANNELS[3]);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "----------------------------------\r\n");
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "Temperature: %f\r\n", baro_data.ftempms);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "Pressure: %f\r\n", baro_data.fbaroms);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "Altitude: %f\r\n", baro_data.faltims);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "----------------------------------\r\n");
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "Accelerometer(x, y, z): %f, ", imu_data.acc_x);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "%f, ", imu_data.acc_y);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "%f\r\n", imu_data.acc_z);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "Gyroscope(x, y, z): %f, ", imu_data.gyro_x);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "%f, ", imu_data.gyro_y);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "%f\r\n", imu_data.gyro_z);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "Magnetometer(x, y, z): %f, ", imu_data.mag_x);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "%f, ", imu_data.mag_y);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "%f\r\n", imu_data.mag_z);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "----------------------------------\r\n");
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "q0, q1, q2, q3: %f, ", q0);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "%f, ", q1);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "%f, ", q2);
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "%f\r\n", q3);
		if (gps_data.valid == 1) {
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "----------------------------------\r\n");
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "satelites: %d\r\n", gps_data.satellites);
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "latitude: %d\r\n", gps_data.latitude);
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "longitude: %d\r\n", gps_data.longitude);
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "altitude: %f\r\n", gps_data.altitude);
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "speed(m/s): %f\r\n", gps_data.speed);
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "heading(degrees): %f\r\n", gps_data.heading);
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "UTC date: %d\r\n", gps_data.utc_date);
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "UTC time: %d\r\n", gps_data.utc_time);
			chprintf((BaseChannel *)&SERIAL_OUTPUT, "milliseconds from epoch: %d\r\n", gps_data.time);
		}
		chprintf((BaseChannel *)&SERIAL_OUTPUT, "==================================\r\n");
		chThdSleepMilliseconds(750);
#else
		if (cnt_debug == 4) {
			uint8_t i;

			uint8_t * b1 = (uint8_t *) &q0;
			for(i=0; i<4; i++) {
				uint8_t b1q1 = (b1[i] >> 4) & 0x0f;
				uint8_t b2q1 = (b1[i] & 0x0f);

				uint8_t c1q1 = (b1q1 < 10) ? ('0' + b1q1) : 'A' + b1q1 - 10;
				uint8_t c2q1 = (b2q1 < 10) ? ('0' + b2q1) : 'A' + b2q1 - 10;

				sdWrite(&SERIAL_OUTPUT, &c1q1, 1);
				sdWrite(&SERIAL_OUTPUT, &c2q1, 1);
			}
			chprintf((BaseChannel *)&SERIAL_OUTPUT, ",");
			uint8_t * b2 = (uint8_t *) &q1;
			for(i=0; i<4; i++) {
				uint8_t b1q2 = (b2[i] >> 4) & 0x0f;
				uint8_t b2q2 = (b2[i] & 0x0f);

				uint8_t c1q2 = (b1q2 < 10) ? ('0' + b1q2) : 'A' + b1q2 - 10;
				uint8_t c2q2 = (b2q2 < 10) ? ('0' + b2q2) : 'A' + b2q2 - 10;

				sdWrite(&SERIAL_OUTPUT, &c1q2, 1);
				sdWrite(&SERIAL_OUTPUT, &c2q2, 1);
			}
			chprintf((BaseChannel *)&SERIAL_OUTPUT, ",");
			uint8_t * b3 = (uint8_t *) &q2;
			for(i=0; i<4; i++) {
				uint8_t b1q3 = (b3[i] >> 4) & 0x0f;
				uint8_t b2q3 = (b3[i] & 0x0f);

				uint8_t c1q3 = (b1q3 < 10) ? ('0' + b1q3) : 'A' + b1q3 - 10;
				uint8_t c2q3 = (b2q3 < 10) ? ('0' + b2q3) : 'A' + b2q3 - 10;

				sdWrite(&SERIAL_OUTPUT, &c1q3, 1);
				sdWrite(&SERIAL_OUTPUT, &c2q3, 1);
			}
			chprintf((BaseChannel *)&SERIAL_OUTPUT, ",");
			uint8_t * b4 = (uint8_t *) &q3;
			for(i=0; i<4; i++) {
				uint8_t b1q4 = (b4[i] >> 4) & 0x0f;
				uint8_t b2q4 = (b4[i] & 0x0f);

				uint8_t c1q4 = (b1q4 < 10) ? ('0' + b1q4) : 'A' + b1q4 - 10;
				uint8_t c2q4 = (b2q4 < 10) ? ('0' + b2q4) : 'A' + b2q4 - 10;

				sdWrite(&SERIAL_OUTPUT, &c1q4, 1);
				sdWrite(&SERIAL_OUTPUT, &c2q4, 1);
			}
			chprintf((BaseChannel *)&SERIAL_OUTPUT, ",");
			uint8_t * b5 = (uint8_t *) &sampleFreq;
			for(i=0; i<4; i++) {
				uint8_t b1 = (b5[i] >> 4) & 0x0f;
				uint8_t b2 = (b5[i] & 0x0f);

				uint8_t c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
				uint8_t c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

				sdWrite(&SERIAL_OUTPUT, &c1, 1);
				sdWrite(&SERIAL_OUTPUT, &c2, 1);
			}
			chprintf((BaseChannel *)&SERIAL_OUTPUT, ",\r\n");

			cnt_debug = 0;
		}
		//chEvtBroadcastFlags(&imu_event, EVENT_MASK(5));
		cnt_debug++;
#endif
	}

	return 0;
}

void serial_start(void) {
	const SerialConfig outputPortConfig = { 
            115200, 
            0, 
            USART_CR2_STOP1_BITS | USART_CR2_LINEN, 
            0 
        }; 
        sdStart(&SERIAL_OUTPUT, &outputPortConfig); 
        chprintf((BaseChannel *)&SERIAL_OUTPUT, "\r\nTrunetcopter\r\n");

	chThdCreateStatic(SerialThreadWA,
                sizeof(SerialThreadWA),
                NORMALPRIO,
                SerialThread,
                NULL);
}

