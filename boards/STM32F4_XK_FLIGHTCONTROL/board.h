/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STM32F4-XKopter Flight Control board
 */

/*
 * Board identifier.
 */
#define BOARD_STM32F4_XK_FLIGHTCONTROL
#define BOARD_NAME              "STM32F4-XK_FLIGHTCONTROL"

/*
 * Board frequencies.
 * NOTE: The LSE crystal is not fitted by default on the board.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            8000000

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD               300

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F4XX

/*
 * IO pins assignments.
 */
#define GPIOA_USART4_TX			0	/* receiver */
#define GPIOA_USART4_RX			1	/* receiver */
#define GPIOA_SERVO1			2	/* gimbal servo 1 */
#define GPIOA_SERVO2			3	/* gimbal servo 2 */
#define GPIOA_SPI1_NSS          4	/* MS5611 SEL */
#define GPIOA_SPI1_SCK          5	/* MS5611 SPI */
#define GPIOA_SPI1_MISO         6	/* MS5611 SPI */
#define GPIOA_SPI1_MOSI         7	/* MS5611 SPI */
#define GPIOA_HMCDRDY			8	/* HMC5883L data ready */
#define GPIOA_USB_VBUS			9	/* not in use */
#define GPIOA_OTG_FS_ID         10	/* USB OTG ID */
#define GPIOA_OTG_FS_DM         11	/* USB OTG DM */
#define GPIOA_OTG_FS_DP         12	/* USB OTG DP */
#define GPIOA_JTMS              13	/* JTAG */
#define GPIOA_JTCK              14	/* JTAG */
#define GPIOA_JTDI              15	/* JTAG */

#define GPIOB_PWM8				0	/* ESC_Motor 8 */
#define GPIOB_PWM7				1	/* ESC_Motor 7 */
#define GPIOB_BOOT1				2	/* Bootsel */
#define GPIOB_JTDO              3	/* JTAG */
#define GPIOB_NJTRST            4	/* JTAG */
#define GPIOB_NC5	            5	/* NC */
#define GPIOB_USART1_TX			6	/* OSD	*/
#define GPIOB_USART1_RX			7	/* OSD	*/
#define GPIOB_I2C1_SCL			8	/* Mikrokopter ESC (alt. CAN Bus)*/	
#define GPIOB_I2C1_SDA			9	/* Mikrokopter ESC (alt. CAN Bus)*/	
#define GPIOB_I2C2_SCL          10	/* direct HMC5883L access */
#define GPIOB_I2C2_SDA          11	/* direct HMC5883L access */
#define GPIOB_SPI2_NSS          12	/* MPU6000 SEL */
#define GPIOB_SPI2_SCK          13	/* MPU6000 SPI */
#define GPIOB_SPI2_MISO         14	/* MPU6000 SPI */
#define GPIOB_SPI2_MOSI         15	/* MPU6000 SPI */

#define GPIOC_NC0				0
#define GPIOC_PEXT2				1	/* Extension IO 2 */
#define GPIOC_PEXT3				2	/* Extension IO 3 */
#define GPIOC_PEXT4				3	/* Extension IO 4 */
#define GPIOC_CURRENT_SENSE		4
#define GPIOC_VIN_SENSE			5
#define GPIOC_PWM4				6	/* ESC_Motor 4 */
#define GPIOC_PWM3				7	/* ESC_Motor 3 */
#define GPIOC_PWM2				8	/* ESC_Motor 2 */
#define GPIOC_PWM1				9	/* ESC_Motor 1 */
#define GPIOC_SPI3_SCK          10	/* Ext. SPI */
#define GPIOC_SPI3_MISO         11	/* Ext. SPI */
#define GPIOC_SPI3_MOSI         12	/* Ext. SPI */
#define GPIOC_NC13				13
#define GPIOC_NC14				14
#define GPIOC_PEXT1				15	/* Extension IO 1 */


#define GPIOD_CAN_EN			0	/* CAN Bus enable */
#define GPIOD_BEEPER			1	/* Beeper */
#define GPIOD_SONAR				2	/* Sonar: serial, PWM or analog */
#define GPIOD_READY_LED			3
#define GPIOD_DEBUG_LED			4
#define GPIOD_USART2_WI232_RX	5	/* Telemetry */
#define GPIOD_USART2_WI232_TX	6	/* Telemetry */
#define GPIOD_GPS_LED			7
#define GPIOD_USART3_GPS_RX		8	/* GPS	*/
#define GPIOD_USART3_GPS_TX		9	/* GPS	*/
#define GPIOD_GPS_TIMEPULSE		10	/* GPS	*/
#define GPIOD_MPU6000_INT		11	/* MPU6000 IRQ */
#define GPIOD_RC_IN5			12	/* receiver input 5 */
#define GPIOD_RC_IN6			13	/* receiver input 6 */
#define GPIOD_RC_IN7			14	/* receiver input 7 */
#define GPIOD_RC_IN8			15	/* receiver input 8 */

#define GPIOE_SD_CS				0
#define GPIOE_EXT_SPI_SEL		1
#define GPIOE_AT45_SEL			2
#define GPIOE_WI232_CMD			3
#define GPIOE_BMA_INT2			4
#define GPIOE_PWM5				5
#define GPIOE_PWM6				6
#define GPIOE_NC7				7
#define GPIOE_BMA_INT1			8
#define GPIOE_RC_IN1			9	/* receiver input 1 */
#define GPIOE_BMA_SEL			10
#define GPIOE_RC_IN2			11	/* receiver input 2 */
#define GPIOE_MPU6000_FSYNC		12
#define GPIOE_RC_IN3			13	/* receiver input 3 */
#define GPIOE_RC_IN4			14	/* receiver input 4 */
#define GPIOE_TRIGGER			15

#define GPIOH_OSC_IN            0
#define GPIOH_OSC_OUT           1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUDR_FLOATING(n)        (0U << ((n) * 2))
#define PIN_PUDR_PULLUP(n)          (1U << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)        (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * Port A setup.
 */

#define VAL_GPIOA_MODER    (PIN_MODE_ALTERNATE(GPIOA_USART4_TX) |           \
							PIN_MODE_ALTERNATE(GPIOA_USART4_RX) |           \
							PIN_MODE_ALTERNATE(GPIOA_SERVO1) |              \
							PIN_MODE_ALTERNATE(GPIOA_SERVO2) |              \
							PIN_MODE_ALTERNATE(GPIOA_SPI1_NSS) |            \
							PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK) |            \
							PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO) |           \
							PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI) |           \
							PIN_MODE_INPUT(GPIOA_HMCDRDY) |                 \
							PIN_MODE_INPUT(GPIOA_USB_VBUS) |                \
							PIN_MODE_ALTERNATE(GPIOA_OTG_FS_ID) |           \
							PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |           \
							PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |           \
							PIN_MODE_ALTERNATE(GPIOA_JTMS) |                \
							PIN_MODE_ALTERNATE(GPIOA_JTCK) |                \
							PIN_MODE_ALTERNATE(GPIOA_JTDI))


#define VAL_GPIOA_OTYPER    0x00000000
#define VAL_GPIOA_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOA_PUPDR     (PIN_PUDR_FLOATING(GPIOA_USART4_TX) |          \
							PIN_PUDR_FLOATING(GPIOA_USART4_RX) |           \
							PIN_PUDR_PULLDOWN(GPIOA_SERVO1) |              \
							PIN_PUDR_PULLDOWN(GPIOA_SERVO2) |              \
							PIN_PUDR_FLOATING(GPIOA_SPI1_NSS) |            \
							PIN_PUDR_FLOATING(GPIOA_SPI1_SCK) |            \
							PIN_PUDR_FLOATING(GPIOA_SPI1_MISO) |           \
							PIN_PUDR_FLOATING(GPIOA_SPI1_MOSI) |           \
							PIN_PUDR_PULLUP(GPIOA_HMCDRDY) |               \
							PIN_PUDR_FLOATING(GPIOA_USB_VBUS) |            \
							PIN_PUDR_FLOATING(GPIOA_OTG_FS_ID) |           \
							PIN_PUDR_FLOATING(GPIOA_OTG_FS_DM) |           \
							PIN_PUDR_FLOATING(GPIOA_OTG_FS_DP) |           \
							PIN_PUDR_FLOATING(GPIOA_JTMS) |                \
							PIN_PUDR_FLOATING(GPIOA_JTCK) |                \
							PIN_PUDR_FLOATING(GPIOA_JTDI))

#define VAL_GPIOA_ODR       0xFFFFFFFF
#define VAL_GPIOA_AFRL      (PIN_AFIO_AF(GPIOA_USART4_TX,8 ) |             \
                             PIN_AFIO_AF(GPIOA_USART4_RX,8 ) |             \
                             PIN_AFIO_AF(GPIOA_SERVO1,2 ) |                \
                             PIN_AFIO_AF(GPIOA_SERVO2,2 ) |                \
                             PIN_AFIO_AF(GPIOA_SPI1_NSS,5 ) |              \
                             PIN_AFIO_AF(GPIOA_SPI1_SCK,5 ) |              \
                             PIN_AFIO_AF(GPIOA_SPI1_MISO,5 ) |             \
                             PIN_AFIO_AF(GPIOA_SPI1_MOSI,5 ))
#define VAL_GPIOA_AFRH      (PIN_AFIO_AF(GPIOA_OTG_FS_ID, 10) |            \
                             PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |		       \
                             PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) |            \
                             PIN_AFIO_AF(GPIOA_JTMS,0) |                   \
                             PIN_AFIO_AF(GPIOA_JTCK,0) |                   \
                             PIN_AFIO_AF(GPIOA_JTDI,0))

/*
 * Port B setup.
 */

#define VAL_GPIOB_MODER    (PIN_MODE_ALTERNATE(GPIOB_PWM8) |               \
							PIN_MODE_ALTERNATE(GPIOB_PWM7) |               \
							PIN_MODE_INPUT(GPIOB_BOOT1) |                  \
							PIN_MODE_ALTERNATE(GPIOB_JTDO) |               \
							PIN_MODE_ALTERNATE(GPIOB_NJTRST) |             \
							PIN_MODE_INPUT(GPIOB_NC5) |                    \
							PIN_MODE_ALTERNATE(GPIOB_USART1_TX) |          \
							PIN_MODE_ALTERNATE(GPIOB_USART1_RX) |          \
							PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) |           \
							PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) |           \
							PIN_MODE_ALTERNATE(GPIOB_I2C2_SCL) |           \
							PIN_MODE_ALTERNATE(GPIOB_I2C2_SDA) |           \
							PIN_MODE_ALTERNATE(GPIOB_SPI2_NSS) |           \
							PIN_MODE_ALTERNATE(GPIOB_SPI2_SCK) |           \
							PIN_MODE_ALTERNATE(GPIOB_SPI2_MISO) |          \
							PIN_MODE_ALTERNATE(GPIOB_SPI2_MOSI))



#define VAL_GPIOB_OTYPER    (PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |         \
							PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) |          \
							PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |          \
                            PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SDA))
#define VAL_GPIOB_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOB_PUPDR     (PIN_PUDR_PULLDOWN(GPIOB_PWM8) |               \
                            PIN_PUDR_PULLDOWN(GPIOB_PWM7) |                \
                            PIN_PUDR_FLOATING(GPIOB_BOOT1) |               \
                            PIN_PUDR_FLOATING(GPIOB_JTDO) |                \
                            PIN_PUDR_FLOATING(GPIOB_NJTRST) |              \
                            PIN_PUDR_PULLUP(GPIOB_NC5) |                   \
                            PIN_PUDR_FLOATING(GPIOB_USART1_TX) |           \
                            PIN_PUDR_FLOATING(GPIOB_USART1_RX) |           \
                            PIN_PUDR_FLOATING(GPIOB_I2C1_SCL) |            \
                            PIN_PUDR_FLOATING(GPIOB_I2C1_SDA) |            \
                            PIN_PUDR_FLOATING(GPIOB_I2C2_SCL) |            \
                            PIN_PUDR_FLOATING(GPIOB_I2C2_SDA) |            \
							PIN_PUDR_PULLUP(GPIOB_SPI2_NSS) |              \
							PIN_PUDR_PULLUP(GPIOB_SPI2_SCK) |              \
							PIN_PUDR_PULLUP(GPIOB_SPI2_MISO) |             \
                            PIN_PUDR_PULLUP(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_ODR       0xFFFFFFFF
#define VAL_GPIOB_AFRL      (PIN_AFIO_AF(GPIOB_PWM8,2) |                   \
                             PIN_AFIO_AF(GPIOB_PWM7,2) |                   \
                             PIN_AFIO_AF(GPIOB_JTDO,0) |                   \
                             PIN_AFIO_AF(GPIOB_NJTRST,0) |                 \
                             PIN_AFIO_AF(GPIOB_USART1_TX,7) |              \
                             PIN_AFIO_AF(GPIOB_USART1_RX,7))
#define VAL_GPIOB_AFRH      (PIN_AFIO_AF(GPIOB_I2C1_SCL,4) |               \
                             PIN_AFIO_AF(GPIOB_I2C1_SDA,4) |               \
                             PIN_AFIO_AF(GPIOB_I2C2_SCL,4) |               \
                             PIN_AFIO_AF(GPIOB_I2C2_SDA,4) |               \
                             PIN_AFIO_AF(GPIOB_SPI2_NSS,5) |               \
                             PIN_AFIO_AF(GPIOB_SPI2_SCK,5) |               \
                             PIN_AFIO_AF(GPIOB_SPI2_MISO,5) |              \
                             PIN_AFIO_AF(GPIOB_SPI2_MOSI,5))

/*
 * Port C setup.
 */
#define VAL_GPIOC_MODER    (PIN_MODE_INPUT(GPIOC_NC0) |					   \
							PIN_MODE_INPUT(GPIOC_PEXT2) |                  \
							PIN_MODE_INPUT(GPIOC_PEXT3) |                  \
							PIN_MODE_INPUT(GPIOC_PEXT4) |                  \
							PIN_MODE_ANALOG(GPIOC_CURRENT_SENSE) |         \
							PIN_MODE_ANALOG(GPIOC_VIN_SENSE) |             \
							PIN_MODE_ALTERNATE(GPIOC_PWM4) |               \
							PIN_MODE_ALTERNATE(GPIOC_PWM3) |               \
							PIN_MODE_ALTERNATE(GPIOC_PWM2) |               \
							PIN_MODE_ALTERNATE(GPIOC_PWM1) |               \
							PIN_MODE_ALTERNATE(GPIOC_SPI3_SCK) |           \
							PIN_MODE_ALTERNATE(GPIOC_SPI3_MISO) |          \
							PIN_MODE_ALTERNATE(GPIOC_SPI3_MOSI) |          \
							PIN_MODE_INPUT(GPIOC_NC13) |                   \
							PIN_MODE_INPUT(GPIOC_NC14) |                   \
							PIN_MODE_INPUT(GPIOC_PEXT1))

#define VAL_GPIOC_OTYPER    0x00000000
#define VAL_GPIOC_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOC_PUPDR     (PIN_PUDR_PULLUP(GPIOC_NC0) |                  \
                             PIN_PUDR_FLOATING(GPIOC_PEXT2) |              \
                             PIN_PUDR_FLOATING(GPIOC_PEXT3) |              \
                             PIN_PUDR_FLOATING(GPIOC_PEXT4) |              \
                             PIN_PUDR_FLOATING(GPIOC_CURRENT_SENSE) |      \
                             PIN_PUDR_FLOATING(GPIOC_VIN_SENSE) |          \
                             PIN_PUDR_PULLDOWN(GPIOC_PWM4) |               \
                             PIN_PUDR_PULLDOWN(GPIOC_PWM3) |               \
                             PIN_PUDR_PULLDOWN(GPIOC_PWM2) |               \
                             PIN_PUDR_PULLDOWN(GPIOC_PWM1) |               \
                             PIN_PUDR_PULLUP(GPIOC_SPI3_SCK) |             \
                             PIN_PUDR_PULLUP(GPIOC_SPI3_MISO) |            \
                             PIN_PUDR_PULLUP(GPIOC_SPI3_MOSI) |            \
                             PIN_PUDR_PULLUP(GPIOC_NC13) |                 \
                             PIN_PUDR_PULLUP(GPIOC_NC14) |                 \
                             PIN_PUDR_FLOATING(GPIOC_PEXT1))
#define VAL_GPIOC_ODR       0xFFFFFFFF
#define VAL_GPIOC_AFRL      (PIN_AFIO_AF(GPIOC_PWM4,3) |                   \
							PIN_AFIO_AF(GPIOC_PWM3,3))
#define VAL_GPIOC_AFRH      (PIN_AFIO_AF(GPIOC_PWM2,3) |                   \
							PIN_AFIO_AF(GPIOC_PWM1,3) |                    \
							PIN_AFIO_AF(GPIOC_SPI3_SCK,6) |                \
							PIN_AFIO_AF(GPIOC_SPI3_MISO,6) |               \
                            PIN_AFIO_AF(GPIOC_SPI3_MOSI,6))

/*
 * Port D setup.

 */
#define VAL_GPIOD_MODER     (PIN_MODE_OUTPUT(GPIOD_CAN_EN) |               \
                             PIN_MODE_OUTPUT(GPIOD_BEEPER) |               \
                             PIN_MODE_ALTERNATE(GPIOD_SONAR) |             \
                             PIN_MODE_OUTPUT(GPIOD_READY_LED) |            \
                             PIN_MODE_OUTPUT(GPIOD_DEBUG_LED) |            \
                             PIN_MODE_ALTERNATE(GPIOD_USART2_WI232_RX) |   \
                             PIN_MODE_ALTERNATE(GPIOD_USART2_WI232_TX) |   \
                             PIN_MODE_OUTPUT(GPIOD_GPS_LED) |              \
                             PIN_MODE_INPUT(GPIOD_USART3_GPS_RX) |         \
                             PIN_MODE_INPUT(GPIOD_USART3_GPS_TX) |         \
                             PIN_MODE_INPUT(GPIOD_GPS_TIMEPULSE) |         \
                             PIN_MODE_INPUT(GPIOD_MPU6000_INT) |           \
                             PIN_MODE_INPUT(GPIOD_RC_IN5) |                \
                             PIN_MODE_INPUT(GPIOD_RC_IN6) |                \
                             PIN_MODE_INPUT(GPIOD_RC_IN7) |                \
                             PIN_MODE_INPUT(GPIOD_RC_IN8))


#define VAL_GPIOD_OTYPER    0x00000000
#define VAL_GPIOD_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOD_PUPDR     (PIN_PUDR_FLOATING(GPIOD_CAN_EN) |             \
                             PIN_PUDR_FLOATING(GPIOD_BEEPER) |             \
                             PIN_PUDR_FLOATING(GPIOD_SONAR) |              \
                             PIN_PUDR_PULLDOWN(GPIOD_READY_LED) |          \
                             PIN_PUDR_PULLDOWN(GPIOD_DEBUG_LED) |          \
                             PIN_PUDR_FLOATING(GPIOD_USART2_WI232_RX) |    \
                             PIN_PUDR_FLOATING(GPIOD_USART2_WI232_TX) |    \
                             PIN_PUDR_PULLDOWN(GPIOD_GPS_LED) |            \
                             PIN_PUDR_FLOATING(GPIOD_USART3_GPS_RX) |      \
                             PIN_PUDR_FLOATING(GPIOD_USART3_GPS_TX) |      \
                             PIN_PUDR_FLOATING(GPIOD_GPS_TIMEPULSE) |      \
                             PIN_PUDR_PULLUP(GPIOD_MPU6000_INT) |          \
                             PIN_PUDR_PULLDOWN(GPIOD_RC_IN5) |             \
                             PIN_PUDR_PULLDOWN(GPIOD_RC_IN6) |             \
                             PIN_PUDR_PULLDOWN(GPIOD_RC_IN7) |             \
                             PIN_PUDR_PULLDOWN(GPIOD_RC_IN8))
#define VAL_GPIOD_ODR       0x00000FCF
#define VAL_GPIOD_AFRL      (PIN_AFIO_AF(GPIOD_SONAR,8) |                  \
							PIN_AFIO_AF(GPIOD_USART2_WI232_RX,7)) |        \
							PIN_AFIO_AF(GPIOD_USART2_WI232_TX,7))
#define VAL_GPIOD_AFRH      (PIN_AFIO_AF(GPIOD_USART3_GPS_RX,7) |          \
							PIN_AFIO_AF(GPIOD_USART3_GPS_TX,7))

/*
 * Port E setup.
 */
#define VAL_GPIOE_MODER     (PIN_MODE_OUTPUT(GPIOE_SD_CS) |                \
                             PIN_MODE_OUTPUT(GPIOE_EXT_SPI_SEL) |          \
                             PIN_MODE_OUTPUT(GPIOE_AT45_SEL) |             \
                             PIN_MODE_OUTPUT(GPIOE_WI232_CMD) |            \
                             PIN_MODE_INPUT(GPIOE_BMA_INT2) |              \
                             PIN_MODE_ALTERNATE(GPIOE_PWM5) |              \
                             PIN_MODE_ALTERNATE(GPIOE_PWM6) |              \
                             PIN_MODE_INPUT(GPIOE_NC7) |                   \
                             PIN_MODE_INPUT(GPIOE_BMA_INT1) |              \
                             PIN_MODE_INPUT(GPIOE_RC_IN1) |                \
                             PIN_MODE_OUTPUT(GPIOE_BMA_SEL) |              \
                             PIN_MODE_INPUT(GPIOE_RC_IN2) |                \
                             PIN_MODE_INPUT(GPIOE_MPU6000_FSYNC) |         \
                             PIN_MODE_INPUT(GPIOE_RC_IN3) |                \
                             PIN_MODE_INPUT(GPIOE_RC_IN4) |                \
                             PIN_MODE_OUTPUT(GPIOE_TRIGGER))



#define VAL_GPIOE_OTYPER    0x00000000
#define VAL_GPIOE_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOE_PUPDR     (PIN_PUDR_PULLUP(GPIOE_SD_CS) |                \
                             PIN_PUDR_PULLUP(GPIOE_EXT_SPI_SEL) |          \
                             PIN_PUDR_PULLUP(GPIOE_AT45_SEL) |             \
                             PIN_PUDR_PULLUP(GPIOE_WI232_CMD) |            \
                             PIN_PUDR_PULLUP(GPIOE_BMA_INT2) |             \
                             PIN_PUDR_PULLDOWN(GPIOE_PWM5) |               \
                             PIN_PUDR_PULLDOWN(GPIOE_PWM6) |               \
                             PIN_PUDR_PULLDOWN(GPIOE_NC7) |                \
                             PIN_PUDR_PULLUP(GPIOE_BMA_INT1) |             \
                             PIN_PUDR_PULLDOWN(GPIOE_RC_IN1) |             \
                             PIN_PUDR_PULLUP(GPIOE_BMA_SEL) |              \
                             PIN_PUDR_PULLDOWN(GPIOE_RC_IN2) |             \
                             PIN_PUDR_PULLDOWN(GPIOE_MPU6000_FSYNC) |      \
                             PIN_PUDR_PULLDOWN(GPIOE_RC_IN3) |             \
                             PIN_PUDR_PULLDOWN(GPIOE_RC_IN4) |             \
                             PIN_PUDR_FLOATING(GPIOE_TRIGGER))
#define VAL_GPIOE_ODR       0xFFFFFFFF
#define VAL_GPIOE_AFRL      (PIN_AFIO_AF(GPIOE_PWM5,3) |                  \
							PIN_AFIO_AF(GPIOE_PWM6,3))
#define VAL_GPIOE_AFRH      0x00000000

/*
 * Port F setup.
 * All input with pull-up.
 */
#define VAL_GPIOF_MODER     0x00000000
#define VAL_GPIOF_OTYPER    0x00000000
#define VAL_GPIOF_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOF_PUPDR     (PIN_PUDR_PULLUP(0) |                           \
                             PIN_PUDR_PULLUP(1) |                           \
                             PIN_PUDR_PULLUP(2) |                           \
                             PIN_PUDR_PULLUP(3) |                           \
                             PIN_PUDR_PULLUP(4) |                           \
                             PIN_PUDR_PULLUP(5) |                           \
                             PIN_PUDR_PULLUP(6) |                           \
                             PIN_PUDR_PULLUP(7) |                           \
                             PIN_PUDR_PULLUP(8) |                           \
                             PIN_PUDR_PULLUP(9) |                           \
                             PIN_PUDR_PULLUP(10) |                          \
                             PIN_PUDR_PULLUP(11) |                          \
                             PIN_PUDR_PULLUP(12) |                          \
                             PIN_PUDR_PULLUP(13) |                          \
                             PIN_PUDR_PULLUP(14) |                          \
                             PIN_PUDR_PULLUP(15))
#define VAL_GPIOF_ODR       0xFFFFFFFF
#define VAL_GPIOF_AFRL      0x00000000
#define VAL_GPIOF_AFRH      0x00000000

/*
 * Port G setup.
 * All input with pull-up.
 */
#define VAL_GPIOG_MODER     0x00000000
#define VAL_GPIOG_OTYPER    0x00000000
#define VAL_GPIOG_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOG_PUPDR     (PIN_PUDR_PULLUP(0) |                           \
                             PIN_PUDR_PULLUP(1) |                           \
                             PIN_PUDR_PULLUP(2) |                           \
                             PIN_PUDR_PULLUP(3) |                           \
                             PIN_PUDR_PULLUP(4) |                           \
                             PIN_PUDR_PULLUP(5) |                           \
                             PIN_PUDR_PULLUP(6) |                           \
                             PIN_PUDR_PULLUP(7) |                           \
                             PIN_PUDR_PULLUP(8) |                           \
                             PIN_PUDR_PULLUP(9) |                           \
                             PIN_PUDR_PULLUP(10) |                          \
                             PIN_PUDR_PULLUP(11) |                          \
                             PIN_PUDR_PULLUP(12) |                          \
                             PIN_PUDR_PULLUP(13) |                          \
                             PIN_PUDR_PULLUP(14) |                          \
                             PIN_PUDR_PULLUP(15))
#define VAL_GPIOG_ODR       0xFFFFFFFF
#define VAL_GPIOG_AFRL      0x00000000
#define VAL_GPIOG_AFRH      0x00000000

/*
 * Port H setup.
 * All input with pull-up except:
 * PH0  - GPIOH_OSC_IN          (input floating).
 * PH1  - GPIOH_OSC_OUT         (input floating).
 */
#define VAL_GPIOH_MODER     (PIN_MODE_INPUT(GPIOH_OSC_IN) |                 \
                             PIN_MODE_INPUT(GPIOH_OSC_OUT) |                \
                             PIN_MODE_INPUT(2) |                            \
                             PIN_MODE_INPUT(3) |                            \
                             PIN_MODE_INPUT(4) |                            \
                             PIN_MODE_INPUT(5) |                            \
                             PIN_MODE_INPUT(6) |                            \
                             PIN_MODE_INPUT(7) |                            \
                             PIN_MODE_INPUT(8) |                            \
                             PIN_MODE_INPUT(9) |                            \
                             PIN_MODE_INPUT(10) |                           \
                             PIN_MODE_INPUT(11) |                           \
                             PIN_MODE_INPUT(12) |                           \
                             PIN_MODE_INPUT(13) |                           \
                             PIN_MODE_INPUT(14) |                           \
                             PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER    0x00000000
#define VAL_GPIOH_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOH_PUPDR     (PIN_PUDR_FLOATING(GPIOH_OSC_IN) |              \
                             PIN_PUDR_FLOATING(GPIOH_OSC_OUT) |             \
                             PIN_PUDR_PULLUP(2) |                           \
                             PIN_PUDR_PULLUP(3) |                           \
                             PIN_PUDR_PULLUP(4) |                           \
                             PIN_PUDR_PULLUP(5) |                           \
                             PIN_PUDR_PULLUP(6) |                           \
                             PIN_PUDR_PULLUP(7) |                           \
                             PIN_PUDR_PULLUP(8) |                           \
                             PIN_PUDR_PULLUP(9) |                           \
                             PIN_PUDR_PULLUP(10) |                          \
                             PIN_PUDR_PULLUP(11) |                          \
                             PIN_PUDR_PULLUP(12) |                          \
                             PIN_PUDR_PULLUP(13) |                          \
                             PIN_PUDR_PULLUP(14) |                          \
                             PIN_PUDR_PULLUP(15))
#define VAL_GPIOH_ODR       0xFFFFFFFF
#define VAL_GPIOH_AFRL      0x00000000
#define VAL_GPIOH_AFRH      0x00000000

/*
 * Port I setup.
 * All input with pull-up.
 */
#define VAL_GPIOI_MODER     0x00000000
#define VAL_GPIOI_OTYPER    0x00000000
#define VAL_GPIOI_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOI_PUPDR     (PIN_PUDR_PULLUP(0) |                           \
                             PIN_PUDR_PULLUP(1) |                           \
                             PIN_PUDR_PULLUP(2) |                           \
                             PIN_PUDR_PULLUP(3) |                           \
                             PIN_PUDR_PULLUP(4) |                           \
                             PIN_PUDR_PULLUP(5) |                           \
                             PIN_PUDR_PULLUP(6) |                           \
                             PIN_PUDR_PULLUP(7) |                           \
                             PIN_PUDR_PULLUP(8) |                           \
                             PIN_PUDR_PULLUP(9) |                           \
                             PIN_PUDR_PULLUP(10) |                          \
                             PIN_PUDR_PULLUP(11) |                          \
                             PIN_PUDR_PULLUP(12) |                          \
                             PIN_PUDR_PULLUP(13) |                          \
                             PIN_PUDR_PULLUP(14) |                          \
                             PIN_PUDR_PULLUP(15))
#define VAL_GPIOI_ODR       0xFFFFFFFF
#define VAL_GPIOI_AFRL      0x00000000
#define VAL_GPIOI_AFRH      0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
