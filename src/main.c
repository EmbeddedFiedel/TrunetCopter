/*
Copyright (C) 2012 Wagner Sartori Junior <wsartori@gmail.com>
http://www.wsartori.com

This file is part of TrunetCopter project.

TrunetCopter program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* ChibiOS includes */
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "chmboxes.h"

/* ARM includes */
#include "math.h"

/*
 ******************************************************************************
 * TrunetCopter INCLUDES
 ******************************************************************************
 */
#include "main.h"
#include "i2c_local.h"
#include "serial_local.h"

#include "eeprom/eeprom.h"

#include "sensors/baro_ms5611.h"
#include "sensors/imu_mpu6050.h"
#include "sensors/magn_hmc5883.h"
#include "sensors/gps_mtk.h"

#include "algebra.h"

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
uint32_t GlobalFlags = 0;

EepromFileStream EepromFile;

baro_data_t baro_data;
imu_data_t imu_data;
gps_data_t gps_data;
float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame

EventSource imu_event;
Mailbox mb[3];
msg_t mbBuf[3][MAILBOX_MSG_SIZE];

extern float sampleFreq;

#define RC_IN_RANGE(x) (((x)>900 && (x)<2300))
volatile unsigned short RC_INPUT_CHANNELS[4], RC_INPUT_LAST_TCNT;
char PPM_FRAME_GOOD = 1;

/*
 *  _____       _                             _
 * |_   _|     | |                           | |
 *   | |  _ __ | |_ ___ _ __ _ __ _   _ _ __ | |_ ___
 *   | | | '_ \| __/ _ \ '__| '__| | | | '_ \| __/ __|
 *  _| |_| | | | ||  __/ |  | |  | |_| | |_) | |_\__ \
 * |_____|_| |_|\__\___|_|  |_|   \__,_| .__/ \__|___/
 *                                     | |
 *                                     |_|
 */
void rx_channel1_interrupt(EXTDriver *extp, expchannel_t channel) {
		(void)extp;
		(void)channel;

		chSysLockFromIsr();
		if (palReadPad(GPIOC, 6) == PAL_LOW) {
			unsigned short tmp = TIM4->CNT - RC_INPUT_LAST_TCNT;
			if (RC_IN_RANGE(tmp)) RC_INPUT_CHANNELS[0] = tmp;
		}
		RC_INPUT_LAST_TCNT = TIM4->CNT;
		chSysUnlockFromIsr();
}
void rx_channel2_interrupt(EXTDriver *extp, expchannel_t channel) {
		(void)extp;
		(void)channel;

		chSysLockFromIsr();
		if (palReadPad(GPIOC, 7) == PAL_LOW) {
			unsigned short tmp = TIM4->CNT - RC_INPUT_LAST_TCNT;
			if (RC_IN_RANGE(tmp)) RC_INPUT_CHANNELS[1] = tmp;
		}
		RC_INPUT_LAST_TCNT = TIM4->CNT;
		chSysUnlockFromIsr();
}
void rx_channel3_interrupt(EXTDriver *extp, expchannel_t channel) {
		(void)extp;
		(void)channel;

		chSysLockFromIsr();
		if (palReadPad(GPIOC, 8) == PAL_LOW) {
			unsigned short tmp = TIM4->CNT - RC_INPUT_LAST_TCNT;
			if (RC_IN_RANGE(tmp)) RC_INPUT_CHANNELS[2] = tmp;
		}
		RC_INPUT_LAST_TCNT = TIM4->CNT;
		chSysUnlockFromIsr();
}
void rx_channel4_interrupt(EXTDriver *extp, expchannel_t channel) {
		(void)extp;
		(void)channel;

		chSysLockFromIsr();
		if (palReadPad(GPIOC, 9) == PAL_LOW) {
			unsigned short tmp = TIM4->CNT - RC_INPUT_LAST_TCNT;
			if (RC_IN_RANGE(tmp)) RC_INPUT_CHANNELS[3] = tmp;
		}
		RC_INPUT_LAST_TCNT = TIM4->CNT;
		chSysUnlockFromIsr();
}

static const EXTConfig extcfg = {
	{
	    {EXT_CH_MODE_DISABLED, NULL},
   	 	{EXT_CH_MODE_DISABLED, NULL},
   	 	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART, hmc5883_interrupt_handler},
		{EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, mpu6050_interrupt_handler},
		{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, rx_channel1_interrupt},
		{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, rx_channel2_interrupt},
		{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, rx_channel3_interrupt},
		{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, rx_channel4_interrupt},
    	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},
    	{EXT_CH_MODE_DISABLED, NULL},//{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, rx_channel1_interrupt},
    	{EXT_CH_MODE_DISABLED, NULL},//{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, rx_channel2_interrupt},
    	{EXT_CH_MODE_DISABLED, NULL},//{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, rx_channel3_interrupt},
    	{EXT_CH_MODE_DISABLED, NULL}//{EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, rx_channel4_interrupt}
	},
	EXT_MODE_EXTI(0, /* 0 */
	              0, /* 1 */
	              0, /* 2 */
	              0, /* 3 */
	              EXT_MODE_GPIOA, /* 4 */
	              EXT_MODE_GPIOB, /* 5 */
	              EXT_MODE_GPIOC, /* 6 */
	              EXT_MODE_GPIOC, /* 7 */
	              EXT_MODE_GPIOC, /* 8 */
	              EXT_MODE_GPIOC, /* 9 */
	              0, /* 10 */
	              0, /* 11 */
	              0,//EXT_MODE_GPIOB, /* 12 */
	              0,//EXT_MODE_GPIOB, /* 13 */
	              0,//EXT_MODE_GPIOB, /* 14 */
	              0)//EXT_MODE_GPIOB) /* 15 */
};

/*
 *  _______          ____  __
 * |  __ \ \        / /  \/  |
 * | |__) \ \  /\  / /| \  / |
 * |  ___/ \ \/  \/ / | |\/| |
 * | |      \  /\  /  | |  | |
 * |_|       \/  \/   |_|  |_|
 */
static PWMConfig pwmcfg2 = {1000000, // 1 uS clock.
							20000,   // Period 250Hz.
							NULL,
                            { {PWM_OUTPUT_ACTIVE_HIGH, NULL},   //CH1 - GPIOA 0
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL},   //CH2 - GPIOA 1
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL},   //CH3 - GPIOB 10
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL} }, //CH4 - GPIOB 11
                            0
                          };

/*
 * Red LED blinker thread, times are in milliseconds.
 * GPIOB,1 is the LED on the Maple Mini
 */
static WORKING_AREA(waThreadLed, 32);
static msg_t ThreadLed(void *arg) {
	(void)arg;
	chRegSetThreadName("LED");
	
	while (TRUE) {
		palClearPad(GPIOB, 3);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOB, 3);
		chThdSleepMilliseconds(500);
		palClearPad(GPIOB, 4);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOB, 4);
		chThdSleepMilliseconds(500);
		palClearPad(GPIOB, 9);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOB, 9);
		chThdSleepMilliseconds(500);
	}
	
	return 0;
}

static WORKING_AREA(waThreadMotors, 64);
static msg_t ThreadMotors(void *arg) {
	(void)arg;
	chRegSetThreadName("Motors");

	while (TRUE) {
		pwmEnableChannel(&PWMD2, 0, RC_INPUT_CHANNELS[2]); // start up PWMs so ESCs can initialize
		pwmEnableChannel(&PWMD2, 1, RC_INPUT_CHANNELS[2]);
		pwmEnableChannel(&PWMD2, 2, RC_INPUT_CHANNELS[2]);
		pwmEnableChannel(&PWMD2, 3, RC_INPUT_CHANNELS[2]);
		chThdSleepMilliseconds(10);
	}

	return 0;
}

/*
 *  __  __       _
 * |  \/  |     (_)
 * | \  / | __ _ _ _ __
 * | |\/| |/ _` | | '_ \
 * | |  | | (_| | | | | |
 * |_|  |_|\__,_|_|_| |_|
 *
 */
int main(void) {
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	palSetPadMode(GPIOB, 3, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOB, 4, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOB, 9, PAL_MODE_OUTPUT_PUSHPULL);

	serial_start();

	/*
	 * Enable Timer 4
	 */
	TIM4->CR1 = 0x00000000;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->SMCR = 0; // slave mode disabled
	TIM4->PSC = 84; // 84 mhz maximum apb1 bus speed
	TIM4->ARR = 0xffff;
	TIM4->SR = 0;
	TIM4->DIER = 0;
	TIM4->CR1 = 0x00000001;

	/*
	 * Enable PWM
	 */
	pwmInit();
	pwmObjectInit(&PWMD2);
	pwmStart(&PWMD2, &pwmcfg2);
	pwmEnableChannel(&PWMD2, 0, 1000);
	pwmEnableChannel(&PWMD2, 1, 1000);
	pwmEnableChannel(&PWMD2, 2, 1000);
	pwmEnableChannel(&PWMD2, 3, 1000);

	chMBInit(&mb[MAILBOX_BARO], mbBuf[MAILBOX_BARO], MAILBOX_BARO_SIZE);
	chMBInit(&mb[MAILBOX_IMU], mbBuf[MAILBOX_IMU], MAILBOX_IMU_SIZE);
	chMBInit(&mb[MAILBOX_GPS], mbBuf[MAILBOX_GPS], MAILBOX_GPS_SIZE);

	chEvtInit(&imu_event);
	chEvtBroadcastFlags(&imu_event, EVENT_MASK(4));
	
	EepromOpen(&EepromFile);
	
	chThdSleepMilliseconds(100);
	I2CInitLocal();
	chThdSleepMilliseconds(100);

	baro_ms5611_start();
	imu_mpu6050_start();
	magn_hmc5883_start();
	gps_mtk_start();
	
	algebra_start();

	extStart(&EXTD1, &extcfg);

	/*
	 * Creates the threads.
	 */
	chThdCreateStatic(waThreadLed, sizeof(waThreadLed), NORMALPRIO, ThreadLed, NULL);
	chThdCreateStatic(waThreadMotors, sizeof(waThreadMotors), NORMALPRIO, ThreadMotors, NULL);

	return 0;
}
