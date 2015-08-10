/*
 *  Use this code free of charge, but leave this text box here,
 *  This code is distributed "as is" with no warranties.
 *  https://github.com/plantalog/ is main repository hub for Cadi project.
 *
 *	27.08.2013 changed the virtaddvartab usage to copying the whole row or variables 0x05C0-0x0679
 *
 *
 */


#include "stm32f10x.h"

// #include "main.h"
#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_i2c.h"
#include "I2CRoutines.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f10x_flash.h"
#include "eeprom.h"
//#include "ff9a/src/diskio.h"
//#include "ff9a/src/ff.h"
#include "driver_sonar.h"
#include "LiquidCrystal_I2C.h"
#include "onewire.h"
// #include "dht22.h"

#define PROTOBUZZZ3			// use Protobuzzz v3 communications
#define GROLLY_VER		2	// Grolly version (different piping)

#define CMD_CONF_AMOUNT	10	// number of Command Execution Confirmations to be sent
volatile static uint8_t packets_received = 0;
volatile static uint16_t rxglobalcntr = 0;
volatile static uint8_t packetid_ = 0;
volatile static uint16_t wrtn_addr = 0;
volatile static uint16_t wrtn_val = 0;

/*
 *  Power control for peripheral devices
 *
 */

#define PWR_LCD		0
#define PWR_DHT		1
#define PWR_EC		2

/* =============================================================================
 * =====================  DDDDDD    H       H   TTTTTTTT   =====================
 * =====================  D     D   H       H      TT	   =====================
 * =====================  D     D   HHHHHHHHH      TT	   =====================
 * =====================  D     D   HHHHHHHHH      TT	   =====================
 * =====================  D     D   H       H      TT	   =====================
 * =====================  DDDDDD    H       H      TT	   =====================
 * =============================================================================
 *  */

/*  ====== DHT driver for STM32 =======
 * FreeRTOS delays (vTaskDelay()) used in code.
 *
 */

#define	DHT_PWR_RESTART_INTERVAL		10		// DHT sensors power supply restart interval, seconds
#define DHT2_TRIG_PLUG			15		// PB15
#define DHT1_TRIG_PLUG			14		// PB14
#define DHT2_DATA_START_POINTER	0		// DHT22 = 4, DHT11 = ?; sets the first bit number in captured sequence of DHT response bits
#define DHT1_DATA_START_POINTER	0		// DHT22 = 4, DHT11 = ?; sets the first bit number in captured sequence of DHT response bits
#define DHT1_PORT				GPIOB
#define DHT1_PIN_SRC			GPIO_PinSource14
#define DHT1_EXTI_LINE			EXTI_Line14
#define DHT1_NVIC_CH			EXTI15_10_IRQn
#define DHT1_ELC_PORT_SRC		GPIO_PortSourceGPIOB

#define DHT2_PORT				GPIOB
#define DHT2_PIN_SRC			GPIO_PinSource15
#define DHT2_EXTI_LINE			EXTI_Line15
#define DHT2_NVIC_CH			EXTI15_10_IRQn
#define DHT2_ELC_PORT_SRC		GPIO_PortSourceGPIOB

typedef struct {
	uint16_t DHT_Temperature; // temperature
	uint16_t DHT_Humidity; // from 0 (0%) to 1000 (100%)
	uint8_t DHT_CRC;
} DHT_data;

volatile static uint16_t rhWindowTop, rhWindowBottom;
volatile static uint8_t rhUnderOver = 0;

volatile static uint16_t dht_rise;
volatile static uint16_t dht_period;
volatile static uint16_t DutyCycle;

uint8_t dht_byte_buf[5];

volatile static uint8_t dht_shifter = DHT2_DATA_START_POINTER; // could be removed in production version
TIM_ICInitTypeDef TIM_ICInitStructure;
volatile static DHT_data DHTValue;
volatile static uint8_t dht1_data[5];
volatile static uint8_t dht2_data[5];
volatile static uint8_t dht_bit_position = 0;
volatile static uint8_t dht_data_ready = 0;
uint8_t dht_byte_pointer = 0;
volatile static uint8_t dht_bit_pointer = 7;
volatile static uint8_t dht1_rh_str[4], dht1_t_str[4];
volatile static uint8_t dht2_rh_str[4], dht2_t_str[4];
volatile static uint8_t dht_last_read_id = 255;

void dht_get_data_x(uint8_t dht_id);
void dht_req_x(uint8_t dht_id);
void dht_1(uint8_t dht_id);
void dht_0(uint8_t dht_id);
void dht_conv_data(void);
void dht_init_x(uint8_t dht_id);
void dht_init_out_x(uint8_t dht_id);
uint8_t crc8(uint8_t *buf, uint8_t start, uint8_t length);

void dht_0(uint8_t dht_id) {
	switch (dht_id) {
	case 0:
		(GPIOB->BRR = (1 << DHT1_TRIG_PLUG));
		break;
	case 1:
		(GPIOB->BRR = (1 << DHT2_TRIG_PLUG));
		break;
	}
}

void dht_1(uint8_t dht_id) {
	switch (dht_id) {
	case 0:
		(GPIOB->BSRR = (1 << DHT1_TRIG_PLUG));
		break;
	case 1:
		(GPIOB->BSRR = (1 << DHT2_TRIG_PLUG));
		break;
	}
}

void dht_req_x(uint8_t dht_id) {
	dht_init_out_x(dht_id);
	dht_0(dht_id);
//	IWDG_ReloadCounter();
	Delay_us_(21000);
//	IWDG_ReloadCounter();
	dht_1(dht_id);

	dht_init_x(dht_id);
	dht_bit_pointer = 7;
	dht_bit_position = 0;
	dht_byte_pointer = 0;
	dht_data_ready = 0;
	dht_rise = 0;

}

void dht_get_data_x(uint8_t dht_id) { // function starts getting data from DHT22 sensor
	dht_1(1);
	vTaskDelay(25);
	uint8_t i;
	for (i = 0; i < 5; i++) { // flush buffer
		dht_byte_buf[i] = 0; // for dht data bytes
	}
	dht_rise = 0;
	NVIC_DisableIRQ(USART1_IRQn);
	dht_req_x(dht_id);

	vTaskDelay(200);
	NVIC_EnableIRQ(USART1_IRQn);
	dht_conv_data();
}

void dht_conv_data() { // convert DHT impulse lengths array into numbers and strings of T and rH
	vTaskDelay(10);
	uint8_t i = 0;
	if (dht_data_ready == 1) {
		vTaskDelay(10);
		DHTValue.DHT_Humidity = (((uint16_t) dht_byte_buf[0] << 8) & 0xFF00)
				+ dht_byte_buf[1];
		DHTValue.DHT_Temperature = (((uint16_t) dht_byte_buf[2] << 8) & 0xFF00)
				+ dht_byte_buf[3];
		DHTValue.DHT_CRC = dht_byte_buf[4];

		if (DHTValue.DHT_Humidity > rhWindowBottom
				&& DHTValue.DHT_Humidity < rhWindowTop) {
			rhUnderOver = 0;
		}
		if (DHTValue.DHT_Humidity < rhWindowBottom) {
			rhUnderOver = 1;
		}
		if (DHTValue.DHT_Humidity > rhWindowTop) {
			rhUnderOver = 2;
		}

		if (crc8(&dht_byte_buf, 0, 4) == dht_byte_buf[4]) {
			vTaskDelay(10);
			switch (dht_last_read_id) {
			case 0:
				for (i = 0; i < 4; i++) {
					asm volatile ("" : : : "memory");
					dht1_data[i] = dht_byte_buf[i];
				}
				vTaskDelay(10);
				dht1_t_str[0] = (DHTValue.DHT_Temperature % 1000) / 100 + 48;
				dht1_t_str[1] = (DHTValue.DHT_Temperature % 100) / 10 + 48;
				dht1_t_str[2] = 46;
				dht1_t_str[3] = (DHTValue.DHT_Temperature % 10) + 48;

				vTaskDelay(10);
				dht1_rh_str[0] = (DHTValue.DHT_Humidity % 1000) / 100 + 48;
				dht1_rh_str[1] = (DHTValue.DHT_Humidity % 100) / 10 + 48;
				dht1_rh_str[2] = 46;
				dht1_rh_str[3] = (DHTValue.DHT_Humidity % 10) + 48;
				break;
			case 1:
				for (i = 0; i < 4; i++) {
					asm volatile ("" : : : "memory");
					dht2_data[i] = dht_byte_buf[i];
				}
				vTaskDelay(10);
				dht2_t_str[0] = (DHTValue.DHT_Temperature % 1000) / 100 + 48;
				dht2_t_str[1] = (DHTValue.DHT_Temperature % 100) / 10 + 48;
				dht2_t_str[2] = 46;
				dht2_t_str[3] = (DHTValue.DHT_Temperature % 10) + 48;

				vTaskDelay(10);
				dht2_rh_str[0] = (DHTValue.DHT_Humidity % 1000) / 100 + 48;
				dht2_rh_str[1] = (DHTValue.DHT_Humidity % 100) / 10 + 48;
				dht2_rh_str[2] = 46;
				dht2_rh_str[3] = (DHTValue.DHT_Humidity % 10) + 48;
				break;

			}
			dht_data_ready = 0;
		}
	}
}

void dht_init_x(uint8_t dht_id) {

	GPIO_InitTypeDef gpio_cfg;
	GPIO_StructInit(&gpio_cfg);

	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	uint8_t pin_src = 0;
	uint32_t exti_line = 0;
	uint8_t nvic_ch = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;

	switch (dht_id) {
	case 0:
		gpio_cfg.GPIO_Pin = 1 << DHT1_TRIG_PLUG;
		GPIO_Init(GPIOB, &gpio_cfg);
		NVIC_InitStructure.NVIC_IRQChannel = DHT1_NVIC_CH;
		GPIO_EXTILineConfig(DHT1_ELC_PORT_SRC, DHT1_PIN_SRC);
		EXTI_InitStructure.EXTI_Line = DHT1_EXTI_LINE;
		break;
	case 1:
		gpio_cfg.GPIO_Pin = 1 << DHT2_TRIG_PLUG;
		GPIO_Init(GPIOB, &gpio_cfg);
		NVIC_InitStructure.NVIC_IRQChannel = DHT2_NVIC_CH;
		GPIO_EXTILineConfig(DHT1_ELC_PORT_SRC, DHT2_PIN_SRC);
		EXTI_InitStructure.EXTI_Line = DHT2_EXTI_LINE;
		break;
	}

	// TIM15 is taken for counting the pulse lengths
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	TIM15->PSC = 0; //set divider
	TIM15->CR1 = TIM_CR1_CEN;

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void dht_init_out_x(uint8_t dht_id) {
	GPIO_TypeDef* port;
	uint32_t res_p = 0;
	uint32_t set_p = 0;
	switch (dht_id) {
	case 0:
		port = GPIOB;
		res_p = GPIO_CRH_CNF14;
		set_p = GPIO_CRH_MODE14_0;
		break;
	case 1:
		port = GPIOB;
		res_p = GPIO_CRH_CNF15;
		set_p = GPIO_CRH_MODE15_0;
		break;
	case 2: // additional dht sensors' pins are defined here
		break;
	}
	port->CRH &= ~res_p;
	port->CRH |= set_p;
}

uint8_t crc8(uint8_t *buf, uint8_t start, uint8_t length) {
	//calculate checksum
	uint8_t check_sum = 0;
	uint8_t i = 0;
	for (i = 0; i < length; i++) {
		check_sum += buf[(i + start)];
	}
	return check_sum;
}

// enabling test mode adds some test functionality
#define	TEST_MODE

#define PSI_STAB_ENABLE
#define TANK_STAB_ENABLE

uint16_t rxcntr = 0;
uint16_t icval = 0;
uint16_t icval2 = 0;
uint16_t phval_v = 0;
uint8_t tmpexti = 0;

// SENSOR AND CONTROL DEVICES DRIVERS
/*
 * Peripherals used with Cadi should be properly initialized before use.
 * Typical init has:
 * - Pin config (except integrated peripherals like RTC or Flash)
 * - Driver variable's pool definitions
 * - #defines for names used
 * Some drivers need interfacing and/or data processing functions to
 * be implemented
 */

// DRIVER: WATER FLOW METER
#define USE_WFM
#ifdef USE_WFM		//	START WFM DEFINITIONS
#define WFM_AMOUNT	4	// number of meters to work with
volatile uint32_t water_counter[WFM_AMOUNT];
#define WFM_PINS_PB0_1	// WFM assigned to pins PB0 and PB1
#endif				//	EOF WFM DEFINITIONS

// DRIVER: Spherical valves with feedback
#define USE_VALVES
#ifdef USE_VALVES	// START VALVES DEFINITIONS
#define VALVE_DISABLE	GPIOA->BSRR
#define VALVE_ENABLE	GPIOA->BRR
#define VALVE_SENSOR_GPIO_SHIFT		4		// valve position sensor GPIO shift
#define VALVE_AMOUNT				15		// number of valves to process
#define VALVE_MOTOR_GPIO_SHIFT		11		// valve control motor GPIO out shift
#define VALVE_CTRL_PORT			GPIOA
#define VALVE_SENSOR_PORT			GPIOA
#define VALVE_SENSOR_PORT_SOURCE	GPIO_PortSourceGPIOA
#define	VALVE_FAILURE_TIMEOUT		600	// timeout for valve open/close function to avoid hanging if valve broken
#define DRAIN_VALVE_ID				1
// Valve variables
volatile static uint16_t valveFlags;
#endif	// EOF VALVES DEFINITIONS

// DRIVER: Bluetooth module HC-06 USART
#define USE_BLUETOOTH
#ifdef USE_BLUETOOTH		// START BLUETOOTH USART DEFINITIONS
#define BT_USART	USART1
#define USARTx_IRQHandler   USART1_IRQHandler

volatile static uint8_t resp_counter = 0;
volatile static uint8_t resp_id = 0;
volatile uint8_t wt_args[7];
volatile uint8_t RxBuffer[40];
volatile uint8_t TxBuffer[42];
volatile static uint8_t RxByte;
volatile static uint8_t comm_state = 48; // communication state
volatile static uint8_t NbrOfDataToTransfer = 16;
volatile static uint8_t txbuff_ne = 0;
volatile static uint8_t TxCounter = 0;
volatile static uint8_t RxCounter = 0;
volatile static uint8_t rx_packet_crc = 0; // RxBuffer and ZXn packet header CRC
#endif				// EOF BLUETOOTH USART DEFINITIONS

// DRIVER: Digital Temperature and Humidity sensor DHT22

#define CADI_MB
// #define USE_LCD					// the same define should be enabled in LiquidCrystal_I2C.c

// #define LCD_I2C_MJKDZ				// use mjkdz I2C expander for LCD
#define LCD_I2C_DFROBOT			// use DFRobot I2C expander for LCD
#ifdef CADI_MB
#define lcd_shift	11				// seems to be not used here anymore?
#define use_gpio	GPIO_Pin_13		// last data pin number
#define pin_d7		use_gpio		// define pins from last
#define pin_d6		use_gpio>>1
#define pin_d5		use_gpio>>2
#define pin_d4		use_gpio>>3		// to d4 of 4bit bus of 1602 LCD
#define d7_0		GPIOA->BSRRH |= (GPIO_Pin_15);
#define d7_1		GPIOA->BSRRL |= (GPIO_Pin_15);
#define d6_0		GPIOC->BSRRH |= (GPIO_Pin_10);
#define d6_1		GPIOC->BSRRL |= (GPIO_Pin_10);
#define d5_0		GPIOC->BSRRH |= (GPIO_Pin_11);
#define d5_1		GPIOC->BSRRL |= (GPIO_Pin_11);
#define d4_0		GPIOC->BSRRH |= (GPIO_Pin_12);
#define d4_1		GPIOC->BSRRL |= (GPIO_Pin_12);

#define e_1 	GPIOD->ODR |=  GPIO_Pin_2
#define e_0		GPIOD->ODR &=~ GPIO_Pin_2
#define rw_1	GPIOB->ODR |=  GPIO_Pin_3
#define rw_0	GPIOB->ODR &=~ GPIO_Pin_3
#define rs_1	GPIOB->ODR |=  GPIO_Pin_4
#define rs_0	GPIOB->ODR &=~ GPIO_Pin_4

#endif

#define lcd_init_port_data			RCC_APB2Periph_GPIOB
#define lcd_init_port_cmd			RCC_APB2Periph_GPIOC
#define pin_e 					GPIO_Pin_10
#define pin_rw					GPIO_Pin_11
#define pin_rs					GPIO_Pin_12
#define lcd_port_data			GPIOB
#define lcd_port_cmd			GPIOC

#define Function_set 				0b00100000//4-bit,2 - line mode, 5*8 dots
#define Display_on_off_control		0b00001100/// display on,cursor off,blink off
#define Display_clear				0b00000001
#define Entry_mode_set				0b00000100//

volatile static uint8_t LCDLine1[16], LCDLine2[16]; // lcd frame buffer
volatile static uint8_t lcd_pointerx = 0;
volatile static uint8_t lcd_pointery = 0;

// DRIVER: Analog buttons
#define USE_BUTTONS
#ifdef USE_BUTTONS			// START BUTTONS DEFINITIONS
#define BUTTON_OK				2
#define BUTTON_CNL				3
#define BUTTON_BCK				1
#define BUTTON_FWD				4
#define BUTTON_RANGE_SHRINKER	30
volatile static uint16_t button_ranges[8]; // 0,2,4,6 - lower, 1,3,5,7 - higher values for buttons
volatile static uint8_t buttonReverse = 0;
#endif						// EOF BUTTONS DEFINITIONS

#ifdef CADI_MB
#define JDR_BUTTONS	ADC1->JDR1		// continuous ADC channel for buttons
#define ADC_AVG_BUTTONS		0	// N=ADC_AVG_BUTTONS, for adcAverage[N]
#endif

#ifndef CADI_MB
#define ADC_AVG_BUTTONS		3
#define JDR_BUTTONS	ADC1->JDR4		// continuous ADC channel for buttons
#endif

volatile static uint8_t button = 0;

volatile uint8_t wpProgress = 0;

// DRIVER:  RTC
#define USE_RTC
#ifdef USE_RTC				// START RTC DEFINITIONS
#define YEAR12SECS		1325462400	// vse raschety vedjom ot 01.01.2012
typedef struct {
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
} RTC_Time;

typedef struct {
	uint16_t hour;
	uint16_t min;
	uint16_t sec;
	uint16_t day;
	uint16_t month;
	uint16_t year;
	uint16_t doy; //day of year

} RTC_DateTime;

volatile RTC_Time toAdjust;
volatile static uint8_t auto_flags = 254; // enable all except the 0 bit - psi stab
volatile static uint8_t auto_failures = 0; // failure bits for corresponding auto programs' flags
volatile static uint32_t timerStateFlags, cTimerStateFlags;
#endif						// EOF RTC DEFINITIONS
volatile static uint8_t i2c_byte_rx = 0;
volatile static uint8_t i2c_byte_tx = 0;

volatile static uint32_t fup_time = 0; // first time underpressure met
volatile static uint16_t psi_upres_level = 0;
volatile static uint16_t psi_upres_timeout = 0;
volatile static uint16_t psi_a = 0;
volatile static uint16_t psi_b = 0;
volatile static uint16_t psi_mid = 0;
#define PSI_UNDERPRESSURE		0x0613			// 1555 minimum pressure meaning there is water in pump, when it is running
#define PSI_UP_TIMEOUT			0x0614			// 1556 seconds to stop PSI pump if underpressure met
// DRIVER: LOAD TRIGGERING
#define USE_LOADS
#ifdef USE_LOADS			// START LOADS DEFINITIONS
#define PLUG_DISABLE	GPIOC->BSRR
#define PLUG_ENABLE		GPIOC->BRR
#define	PLUG_INVERT		0		// enable reverse plugStateSet
#define PLUG_AMOUNT		4
volatile static uint16_t plugStateFlags; // this works
volatile static uint8_t plugSettings[PLUG_AMOUNT] = { 0, 1, 2, 3 }; // PLUG_AMOUNT - number of plugs HARDCODE
// elementy massiva - nomera programm (ex "tajmerov"), sootvetstvujushih Plug'am.
// 0 element - pervyj plug, 1 element - plug no 2, etc
#endif						// EOF LOADS DEFINITIONS

/*
 *   =====  END OF DRIVER SECTION =====
 */

// CRITCAL VALUES defines
#define PSI_OVERPRESSURE		2700	// maximal pressure to shut down PSI pump
#define FWTANK_OVERLEVEL		15		// minimum distance to top for Fresh Water Tank
#define MIXTANK_OVERLEVEL		9		// same for Fertilizer Mixing Tank
#define MIXTANK_UNDERLEVEL		43		// maximum distance to top. if more, the mixing pump could fail
#define PSI_M_LOW		40		// minimum possible speed for high pressure pump motor
#define PSI_M_HIGH		50		// minimum speed for successful motor spin up
#define PSI_MAX_SPEED	100		// maximum possible speed for high pressure pump, %
volatile static uint8_t spf = 0; // psi pump motor spin-up flag
volatile static uint8_t psi_m_low = 0;
volatile static uint8_t psi_m_high = 0;
volatile static uint8_t psi_max_speed = 0;

// define eeprom cells for keeping user settings. memory map (1472-1727 (5C0-6BF))
#define EE_PLUG_SETTINGS			0x05FF	// each plug one value (1535)
/* Plain Timers have 3 fields (5 bytes in total)
 * - Timer ON (32bit): Unixtime for Timer ON event
 * - Timer OFF (32bit): Unixtime for Timer OFF
 *  - Flags (16bit):
 *  	- 0: daily flag. When 1 - timer is 24H, when 0 - Full Range
 */
#define EE_TIMER1_ON				0x05DA	// for 4 timers 36 (hex=24) values (range: 7DA-7FF)

/*
 * Cyclic Timer has similar struct:
 * 	- Duration (32bit): Number of seconds the timer is ON
 * 	- Interval (32bit): Number of seconds between ON rising edges
 */

#define EE_CTIMER_DURATION			0x05C0	// 2 values for duration of cTimer. (1472)
#define EE_CTIMER_INTERVAL			0x05C2	// and 2 for interval. For 5 timers 25 (hex=19) values (1474)
#define EE_TIMER_SIZE				5
#define EE_CTIMER_SIZE				5
#define PH4_ADDR					0x0600
#define PH7_ADDR					0x0601
#define PH_INTERVAL					0x0602	// pH measurement interval in milliseconds
#define PH_BUFF_SIZE				0x0603	// pH buffer size
#define PH_WINDOW_TOP				0x0604	// pH window top adc value
#define PH_WINDOW_BOTTOM			0x0605	// pH window bottom adc value
#define SD_LOG_INTERVAL				0x0609	// sd logging interval, seconds
#define BUTTON_RANGES_START_ADDR	0x060A	// button ranges (8 values in a row) [60A-611]
#define EC1413_ADDR			0x0606
#define EC0_ADDR			0x067E	// after FMPs
#define RH_WINDOW_TOP		0x0607		// pH window top adc value
#define RH_WINDOW_BOTTOM	0x0608	// pH window bottom adc value

#define PSI_PUMP_TIM	TIM1	// Psi Pump Timer for PWMing
// Grolly 1
//these defines configure abstract layer for accessing Cadi devices
// based on schematic picture used in Cadiweb panel
#define GROLLY
#ifdef GROLLY
/*#define FWTANK						0
 #define MIXTANK						1
 #define PSI_PUMP_ID					0		// load ID for high pressure watering pump
 #define FWTANK_SONAR				0
 #define MIXTANK_SONAR				1
 #define	WI_VALVE					0
 #define FWI_VALVE					4
 #define WLINE_61_VALVE				1
 #define WLINE_62_VALVE				3
 #define WLINE_63_VALVE				2
 #define MIXING_PUMP					0		// doser MOSFET on T8 */
#endif

// Grolly 2 (S/N: 001)
/*
#define FWTANK						0
#define MIXTANK						1
#define FWTANK_SONAR					0
#define MIXTANK_SONAR					1
#define	MTI_VALVE					1	// MixTank intake valve
#define FWI_VALVE					2	// Fresh water intake valve
#define BACK_VALVE					4	// Back valve for solution mixing
#define	WI_VALVE					0
#define WLINE_61_VALVE					9	// Watering lines valves
#define WLINE_62_VALVE					3
#define WLINE_63_VALVE					10
#define WLINE_64_VALVE					11
#define WLINE_65_VALVE					8
#define WLINE_66_VALVE					12
*/

 // GROLLY 2 (002 and 003)
#define FWTANK						0
#define MIXTANK						1
#define FWTANK_SONAR					0
#define MIXTANK_SONAR					1
#define	MTI_VALVE					11	// MixTank intake valve
#define FWI_VALVE					10	// Fresh water intake valve
#define BACK_VALVE					4	// Back valve for solution mixing
#define	WI_VALVE					0
#define WLINE_61_VALVE					6	// Watering lines valves
#define WLINE_62_VALVE					3
#define WLINE_63_VALVE					1
#define WLINE_64_VALVE					7
#define WLINE_65_VALVE					2
#define WLINE_66_VALVE					12




#define MAX_SONAR_READ	400				// drop wrong reads

/*  === Watering programs ===
 *  Each WP runs at certain time, calculated as LAST_RUN+INTERVAL
 *  The execution is successfull, only when auto_flag for WP is enabled.
 *  The WP execution loop could be enabled and disabled according the 24H
 *  timer rule (WP_RULE_APPLIED>0)
 *
 */

// WATERING PROGRAMS SETTINGS ADRESSES
#define WP_SIZE						15		// size of block of settings data of watering program
#define WP_AMOUNT					8		// 3x16=48(hex=30) values (range: 61D-64D)
#define WP_OFFSET					0x061D	// 1565
#define WP_TIMEOUT					0		// 1565 watering timeout, seconds
#define WP_RULES_APPLIED			1		// 1566 24H timer and Full range timer rules (ids: 0..15 for each) [1.1 (8bits)]
#define	WP_VOLUME					1		// 1566 amount of water to be intaken for solution preparation (sonar units, 1..255) [1.2 (8bits)]
#define WP_INTERVAL					2		// 1567 2x16bits for WP run interval (in seconds)
#define WP_START					4		// 1569 2x16bit variables for program start time
#define WP_END						6		// 1571 after this time no triggering for this WP
#define WP_LAST_RUN_SHIFT			8		// 1573 2x16bit last run of watering program
#define WP_FLAGS					10		// 1575 valves to open during watering
#define WP_FLAGS2					11		// 1576 minimum level before preparing new solution
#define WP_LOG						12		// 1577 counts waterings and keeps WP progress

// 9x7=63 (hex=39) 0x0644 - 0x067D

#define FMP_PROGRAMS_AMOUNT						9

// Fertilizer Mixing Program adresses	()
#define FMP_OFFSET								0x06A4	// 1700
#define FMP_SIZE								5
#define FMP_DOSING_PUMP_ID_SHIFT				1		// 1701 FMP enabled if dosing pump id > 0 (05.09.2014)
#define FMP_DOSING_TIME_SHIFT					2		// 1676 1702
#define FMP_2_WP_ASSOC_SHIFT					3		// 17030 H
#define FMP_TRIG_FREQUENCY_SHIFT				3		// 17031 L
#define	FMP_AFTERMIX_TIME_SHIFT					4		// 1704
#define DOSER_SPEEDS			0x0617		// 4 bytes for doser speeds in percent (1..100)

#define COMM_MONITOR_MODE		48
#define COMM_GET_SETTINGS		49
#define COMM_SET_SETTINGS		50
#define COMM_DIRECT_DRIVE		51

#define FWTANK_TOP			0x05D6		// 1494
#define FWTANK_BOTTOM		0x05D7		// 1495
#define MIXTANK_TOP				0x05D8		// 1496
#define MIXTANK_BOTTOM			0x05D9		// 1497
#define WFM_CAL_OFFSET			0x0642	// 1602

#define DAY						1
#define NIGHT					0

#define BSRRL					BSRR
#define BSRRH					BRR

#define LOG_SHIFT	1

// analog inputs
#define JDR_EC		ADC1->JDR3		// continuous ADC channel for EC
#define JDR_PH		ADC1->JDR2		// continuous ADC channel for pH
#define JDR_PSI		ADC1->JDR2		// pressure sensor
#define JDR_CO2		ADC1->JDR3		// CO2 sensor
#define JDR_BUFFER_SIZE 10

#define PSI_SENSOR_0PSI			0x0645
#define PSI_SENSOR_32PSI		0x0646
#define PSI_SENSOR_BTM			0x0615			// 1557
#define PSI_SENSOR_TOP			0x0616			// 1558
volatile static uint8_t psi_underOver = 0;
volatile static uint16_t psi_pump_top_level = 0;
volatile static uint16_t psi_pump_btm_level = 0;

#define AVG_ADC_EC		3			// adcAverage[AVG_ADC_EC]
#define AVG_ADC_PH		2			// adcAverage[2]
#define AVG_ADC_PSI		2			// adcAverage[2]
#define AVG_ADC_CO2		2

volatile static uint16_t tank_windows_top[2];
volatile static uint16_t tank_windows_bottom[2];

ErrorStatus HSEStartUpStatus;
FLASH_Status FlashStatus;

volatile static uint16_t adcAverage[4];

volatile static uint8_t wpStateFlags;
volatile static uint8_t dosingPumpStateFlags; // this variable seems to be overwritten by some part of this firmware, therefore dosingPumpStateFlags2 used instead
volatile static uint8_t dosingPumpStateFlags2;
volatile uint16_t wfCalArray[WFM_AMOUNT];

#define STATE_NOTHING					0
#define STATE_CONFIGURATION_PAYLOAD		1
#define STATE_CONFIGURATION_CRC			2
#define STATE_COMMAND_TYPE				3
#define STATE_COMMAND_PAYLOAD			4
#define STATE_COMMAND_CRC				5

#define RXM_NONE						0
#define RXM_CMD							4
#define RXM_SET							3

#define I2C_MASTER
#ifndef I2C_MASTER
#define I2C_SLAVE
#endif

#define OWN_I2C_ADDR	OwnAddress1
#define DEST_I2C_ADDR	OwnAddress2

#define PH_I2C_ADDR	0x9A
#define EC_I2C_ADDR	0x30

ErrorStatus HSEStartUpStatus;
/* Buffer of data to be received by I2C1 */
uint8_t Buffer_Rx1[7];
/* Buffer of data to be transmitted by I2C1 */
uint8_t Buffer_Tx1[7];

/* Buffer of data to be received by I2C2 */
uint8_t Buffer_Rx2[7];
/* Buffer of data to be transmitted by I2C2 */
uint8_t Buffer_Tx2[7];

extern __IO uint8_t Tx_Idx1, Rx_Idx1;
extern __IO uint8_t Tx_Idx2, Rx_Idx2;

volatile static uint8_t curi2crxval = 0;
volatile static uint8_t curi2ctxval = 0;

uint8_t pending_wt = 0; // pending watering task (manual control)

volatile static uint8_t prefixDetectionIdx = 0;
volatile static uint8_t rx_pntr = 0; // packet buffer pointer
volatile static uint8_t packet_length = 0;
volatile static uint8_t rxm_state = 0;
volatile static uint8_t packet_ready = 0; // packet readiness flag. reset after command execution
volatile static wtprog = 0;					// Watering task progress indicates if Watering task is running already

volatile static uint8_t runners = 0;

volatile static uint16_t ee_dump_pointer = 0; // pointer for eeprom dump transmission

void send_ee_dump(void);
void hygroStatSettings(void);
uint8_t readPercentVal(uint8_t value);
void phMonSettings(void);
void setTimer(uint8_t timerId);
//void copy_arr(uint8_t *source, uint8_t *destination, uint8_t amount, uint8_t pos);
void copy_arr(volatile uint8_t *source, volatile uint8_t *destination,
		uint8_t amount, uint8_t pos, uint8_t src_pos);
void Lcd_write_arr(volatile uint8_t *STRING, uint8_t chars);
void Lcd_write_digit(uint8_t numb);
void Delay_us(uint32_t delay);
void buttonCalibration(void);
void Init_pin_out(void);
void Init_pin_in(void);
void Init_lcd(void);
void Lcd_write_data(uint8_t byte);
void Lcd_write_cmd(uint8_t byte);
void Lcd_clear(void);
void Return_home(void);
void Lcd_goto(uc8 x, uc8 y);
void Lcd_write_str(volatile uint8_t *STRING);
char* adc2str(uint_fast16_t d, volatile char* out);
void int32str(uint32_t d, volatile char *out);
void AdcInit(void);
uint32_t RTC_GetCounter(void);
void RTC_SetCounter(uint32_t value);
unsigned char RtcInit(void);
uint8_t readButtons(void);
void focusMenuItem(uint8_t itemId);
uint8_t menuSelector(void);
uint32_t timeAdjust(uint32_t cnt, uint8_t includeDays);
void programRunner(uint8_t programId);
RTC_DateTime unix2DateTime(uint32_t unixtime);
uint32_t DateTime2unix(RTC_DateTime datetime);
void EE_WriteWord(uint16_t Address, uint32_t Data);
uint32_t EE_ReadWord(uint16_t Address);
//void Lcd_print(char *STRING);
void setCTimer(uint8_t timerId);
uint32_t CTimerAdjust(uint32_t time);
void plugStateSet(uint8_t plug, uint8_t state);
void getPh();
void psiStab(void);
void getEc(void);
//FRESULT string2log(char* str, uint8_t bytes);
//FRESULT sdLog2(void);
uint8_t adjust8bit(uint8_t val);
void loggerSettings(void);
uint8_t yesNoSelector(char str, uint8_t curval);
void loadSettings(void);
void set4lowBits(uint8_t dta);
void set4highBits(uint8_t dta);
void flush_lcd_buffer(void);
// void phStabSettings(void);
void Lcd_write_16int( uint16_t);
void saveButtonRanges(void);
void readButtonRanges(void);
void set16bit(uint16_t value);
void convPh2str(uint8_t ph, char* phstr);
uint8_t readPhVal(uint8_t value);
//void dht_conv_data(void);
//void dht_init(void);
//void dht_init_out(void);
void TIM3_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void dht_arr_displayer(void);
void setPwmDc(uint8_t duty_cycle);
void setDutyCycle(void);
void displayAdcValues(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void i2c_rc_test(void);
void i2c_send_byte_test(uint8_t val);
void water_program_setup(uint8_t progId);
void fertilization_setup(void);
void watering_setup(void);
void fertilizer_mixing_program_setup(uint8_t progId);
void run_watering_program(uint8_t progId);
void run_fertilizer_mixer(uint8_t progId);
void run_watering_program_g2(uint8_t progId);
void run_fertilizer_mixer_g2(uint8_t progId);
void startWp(void);
uint16_t adjust16bit_fast(uint16_t val, uint8_t speed);
void wt_mt_add_water(uint16_t amount, uint8_t source);
void wt_mt_reach_level(uint16_t new_level, uint8_t source);


// count crc starting from input and taking buffer from start_byte for length bytes
uint8_t crc_block(uint8_t input, volatile uint8_t *start_byte, uint8_t length);

static void lstasks(void *pvParameters);
static void uart_task(void *pvParameters);
static void prvSetupHardware(void);
static void displayClock(void *pvParameters);
static void timerStateTrigger(void *pvParameters);
static void plugStateTrigger(void *pvParameters);
static void sdLog(void *pvParameters);
static void phMonitor(void *pvParameters);
static void vTaskaw(void *pvParameters);
static void watering_program_trigger(void *pvParameters);
void bluetooth_init(void);
void display_usart_rx(void);
void display_usart_tx(void);

void open_valve(uint8_t valveId);
void close_valve(uint8_t valveId);
void run_valve_motor(uint8_t valveId);
void stop_valve_motor(uint8_t valveId);
void valve_test(void);
void valve_feedback_init(void);
void dosing_motor_control_init(void);
void valve_motor_control_init(void);
void valve_init(void); // startup valve config
void EXTI15_10_IRQHandler(void);
void water_level_input_init(void);
void fertilizer_mixing_program_setup(uint8_t progId);
uint16_t adjust16bit(uint16_t val);
void enable_dosing_pump(uint8_t pumpId, uint8_t state);
void valveMotorStateSet(uint8_t valveId, uint8_t state);
void USART1_IRQHandler(void);
void StartDMAChannel4(uint8_t LengthBufer);
unsigned char GetStateDMAChannel4(void);
void TIM1_TRG_COM_TIM17_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void TIM4_IRQHandler(void);
void tankLevelStabSetup(void);
void tankLevelStab(void);
void Lcd_write_16b(uint16_t tmp);
void Lcd_write_8b(uint8_t tmp);
void run_uart_cmd(void);
void get_water(uint8_t valve, uint8_t counter_id, uint16_t amount);
void get_water_cl(uint8_t valve, uint8_t counter_id, uint16_t amount);
void send_packet();
void setTimerSelector(void);
uint8_t idSelector(uint8_t min, uint8_t max, uint8_t curid);
void printOk(void);
void get_fertilizer(uint8_t fertId, uint8_t secs);
void get_settings_block(uint8_t block_number);
void rx_flush(void);
void adcAverager(void);
void get_status_block(uint8_t blockId);
void run_circulation_pump(uint16_t time);
void send_ee_addr(uint16_t addr, uint8_t type);
void rx_ee(uint16_t addr, uint8_t type);
void autoSafe(void);
void run_doser_for(uint8_t pump_id, uint8_t amount, uint8_t speed);
uint16_t adjustFlags(uint16_t flags, uint8_t from, uint8_t to);
void setDoserSpeed(uint8_t doser, uint8_t speed);
void send_resp(uint8_t cmd_uid);
void send_resp_x(uint8_t cmd_uid, uint8_t x);	// send multiple responses
void send_ee_block(uint16_t addr);
void eeprom_test(void);
void push_tx(void);
void psi_motor_init(void);
void psi_pwm_test(void);
void getco2(void);
void co2_sens_supply(void);
uint8_t skip_button_cal(void);
void psiOff(void);
void I2C_init2(void);
void Delay_us_(uint32_t delay);
void get_settings_dump(void);
void get_settings_dump_(uint16_t amount, uint16_t startaddr);
void settings2tx_buff(uint16_t addr, uint8_t amount);
void mix_solution(uint32_t seconds);
void psi_stab_secs(uint32_t seconds);
void close_valves(void);
void psiOn(void);
void stop_dosers(void);
void dht_power_control_init(void); // power control for DHT sensors pin init
//void dht_power(uint8_t state);			// power on/off DHT sensors
void power_ctrl(uint8_t device, uint8_t state);
void lcd_restart(void);
void run_watering_pump(uint32_t seconds);
void run_demo(void);
void wt_get_water(uint8_t amount);

void wt_mt_drain2level(uint16_t new_level, uint8_t drain_valve);

int strlen(const char *);
char *strrev(char *);
char *itoa(int, char *, int);

void wt_mix_in(uint16_t duration, uint16_t vol, uint8_t doserId, uint8_t spd);

// ====== Software I2C ========
uint8_t I2C2SW_Read_Byte(uint8_t ACK);
void I2C2SW_Read_Block(uint8_t address, uint8_t length);
uint8_t I2C2SW_Write_Byte(uint8_t data);
void I2C2SW_Write_Block(uint8_t address, uint8_t length);
void I2C2SW_Start(void);
void I2C2SW_Stop(void);
// \\ ====== Software I2C ========

// auto_flags
/*
 * 0 - PSI pump stab		(1)
 * 1 - tank level stab		(2)
 * 2 - autoSafe				(4)
 * 3 - psi underpressure	(8)
 *
 */
volatile static uint8_t enableClock = 1; // run display clock, when nothing to do

#define CO2_TOP		0x0619			// if ADC reading lower than this value, CO2 valve closes
#define CO2_BTM		0x061A			// if ADC reading is higher than this value, CO2 valve opens
#define CO2_400PPM	0x061B			//
#define CO2_VALVE	2				// co2 valve id
volatile static uint16_t curco2 = 0;
volatile static uint16_t co2_400ppm = 0;
volatile static uint16_t co2_top = 0; // higher value, lower CO2 level
volatile static uint16_t co2_btm = 0;
volatile static uint32_t next_co2_run = 0;
volatile static uint16_t co2_timeout = 120;
volatile static uint32_t co2_sens_start = 0; // when co2 sensor enabled
volatile static uint8_t co2_sensor_ready = 0; //

#define CO2_SENSOR_12V					1		// co2 sensor driven by 12V valve id
volatile static uint8_t co2temp = 0;
void getco2(void) {
	/* 	uint16_t diff = 0;
	 if (next_co2_run<RTC_GetCounter() && ((auto_failures&16)>>4)==0 && co2_sensor_ready==1){	// if it's time to make a measurement
	 open_valve(CO2_SENSOR_12V);		// enable CO2 sensor 12V supply
	 vTaskDelay(50);		// delay to
	 co2temp |= (1<<1);
	 curco2 = adcAverage[AVG_ADC_CO2];	// read current CO2 ADC value

	 if (curco2>co2_btm && ((valveFlags&(1<<CO2_VALVE))>>CO2_VALVE)==0) {		// if current CO2 PPM level below needed and CO2 valve still closed
	 co2temp |= (1<<2);
	 open_valve(CO2_VALVE);	// open valve
	 co2_sens_start = RTC_GetCounter();	// remember the CO2 stab start time
	 }
	 vTaskDelay(10);
	 diff = (uint16_t)(RTC_GetCounter() - co2_sens_start);
	 if (curco2<co2_top) {					// if CO2 level reached desired
	 close_valve(CO2_VALVE);				// close CO2 valve
	 close_valve(CO2_SENSOR_12V);		// disable sensor 12V supply
	 next_co2_run = RTC_GetCounter()+10;	// +10 seconds delay until next measure
	 co2temp |= (1<<3);
	 }
	 if (diff>co2_timeout) {				// if CO2 control timeout exceeded
	 auto_failures |= (1<<4);		// set CO2 failure flag (bit 5)
	 close_valve(CO2_VALVE);				// close CO2 valve
	 close_valve(CO2_SENSOR_12V);		// disable sensor 12V supply
	 co2temp |= (1<<4);
	 }
	 }
	 */
}

int strlen(const char *str) {
	const char *s;

	s = str;
	while (*s)
		s++;
	return s - str;
}

char *strrev(char *str) {
	char *p1, *p2;

	if (!str || !*str)
		return str;

	for (p1 = str, p2 = str + strlen(str) - 1; p2 > p1; ++p1, --p2) {
		*p1 ^= *p2;
		*p2 ^= *p1;
		*p1 ^= *p2;
	}

	return str;
}

char *itoa(int n, char *s, int b) {
	static char digits[] = "0123456789abcdefghijklmnopqrstuvwxyz";
	int i = 0, sign;

	if ((sign = n) < 0)
		n = -n;

	do {
		s[i++] = digits[n % b];
	} while ((n /= b) > 0);

	if (sign < 0)
		s[i++] = '-';
	s[i] = '\0';

	return strrev(s);
}

#define CO2_WARMUP	60			// co2 sensor warmup
uint32_t co2_warmup_end = 0; // co sensor warmup finish time

/* void co2_sens_supply(void){
 if (RTC_GetCounter()>co2_warmup_end) {
 co2_sensor_ready=1;
 }
 else {
 open_valve(CO2_SENSOR_12V);		// force 12v supply to co2 sensor
 co2_sensor_ready=0;
 }
 } */

/* void co2_setup(void){
 uint16_t tmpval=0;
 Lcd_write_str("Set CO2 top lvl");
 Lcd_goto(1,0);
 Lcd_write_str("Curr = ");
 Lcd_write_16b(adcAverage[AVG_ADC_CO2]);
 vTaskDelay(3000);
 EE_ReadVariable(CO2_TOP, &co2_top);
 co2_top = adjust16bit_fast(co2_top, 1);
 EE_WriteVariable(CO2_TOP, co2_top);
 printOk();
 vTaskDelay(200);
 Lcd_write_str("Set CO2 btm lvl");
 Lcd_goto(1,0);
 Lcd_write_str("Curr = ");
 Lcd_write_16b(adcAverage[AVG_ADC_CO2]);
 vTaskDelay(3000);
 EE_ReadVariable(CO2_BTM, &co2_btm);
 tmpval = adjust16bit_fast(co2_btm, 1);
 EE_WriteVariable(CO2_BTM, co2_btm);
 printOk();
 vTaskDelay(200);
 Lcd_write_str("Set CO2=400ppm");
 Lcd_goto(1,0);
 Lcd_write_str("Curr = ");
 Lcd_write_16b(adcAverage[AVG_ADC_CO2]);
 vTaskDelay(3000);
 EE_ReadVariable(CO2_400PPM, &co2_400ppm);
 tmpval = adjust16bit_fast(co2_400ppm, 1);
 EE_WriteVariable(CO2_400PPM, co2_400ppm);
 printOk();
 vTaskDelay(200);
 loadSettings();
 } */

void autoSafe(void) {
	if (((auto_flags & 4) >> 2) == 1) {
		// disable PSI pump in case of over pressure. PSI_OVERPRESSURE sets max
		if (adcAverage[AVG_ADC_PSI] > PSI_OVERPRESSURE) {
			psiOff();
			wpProgress = 61;
		}

		// if while running PSI pump underpressure was constant during timeout, disable PSI pump
		vTaskDelay(1);
		uint32_t now = RTC_GetCounter();

		if (((auto_flags & 8) >> 3) == 1
				&& adcAverage[AVG_ADC_PSI] < psi_upres_level
				&& (auto_flags & 1) == 1) { // underpressure auto
			if (now > fup_time) {
				wpProgress = 62;
				psiOff();
				auto_failures |= 8; // set PSI program failure flag
			}
		} else {
			fup_time = RTC_GetCounter() + psi_upres_timeout; // fail under pressure flag set time
			// wpProgress = 63;
		}

		vTaskDelay(1);
		if (sonar_read[FWTANK_SONAR] < tank_windows_top[FWTANK]
				&& sonar_read[FWTANK_SONAR] > 0) {
			close_valve(WI_VALVE);
		}
		vTaskDelay(1);
		// if too much water
		if (sonar_read[MIXTANK_SONAR] < tank_windows_top[MIXTANK]
				&& sonar_read[MIXTANK_SONAR] > 0) {
			close_valve(FWI_VALVE);
		}
		vTaskDelay(1);
		if (sonar_read[MIXTANK_SONAR] > tank_windows_bottom[MIXTANK]) {
			// 	enable_dosing_pump(MIXING_PUMP,0);	// mixer pump off
		}
	}
	vTaskDelay(1);
}

// open multiple valves
void open_valves(uint16_t valves) {
	uint8_t i = 0;
	uint8_t curvalvestate = 0;
	for (i = 0; i < 16; i++) {
		curvalvestate = (uint8_t)((valves >> i) & 1);
		if (curvalvestate == 1) {
			open_valve(i);
		}
	}

}

void close_valves(void) { // close all valves
	uint8_t i = 0;
	for (i = 0; i < 15; i++) {
		close_valve(i);
	}
}

void psi_stab_secs(uint32_t seconds) {
	uint32_t overTime = 0;
	uint32_t now = 0;
	now = RTC_GetCounter();
	overTime = RTC_GetCounter() + seconds;
	psiOn();
	vTaskDelay(30);
	while ((((auto_failures & 8) >> 3) == 0) && (now < overTime)) {
		Lcd_goto(0, 0);
		Lcd_write_str("Left: ");
		Lcd_write_digit(wpProgress);
		vTaskDelay(500);
		psiStab();
		wpProgress = (uint8_t)(overTime - now);
		now = RTC_GetCounter();
		vTaskDelay(50);
		IWDG_ReloadCounter();
	}
	auto_flags &= ~(1); // disable PSI stab function flag
	psiOff();
}

void mix_solution(uint32_t seconds) { // Run pump with back valve opened during 'seconds' seconds
	close_valves();
	open_valve(BACK_VALVE); // open back valve to return solution back to MixTank
	vTaskDelay(500);
	open_valve(MTI_VALVE); // open MixTank intake line valve
	vTaskDelay(500);
	psi_stab_secs(seconds);
}

void Lcd_write_8b(uint8_t tmp) {
	if (lcd_pointery == 0) {
		LCDLine1[lcd_pointerx++] = (tmp / 100) + 48;
	} else {
		LCDLine2[lcd_pointerx++] = (tmp / 100) + 48;
	}
	Lcd_write_digit(tmp % 100);

}

void calibrateFlowMeter(void) {
	uint8_t valve = 0, counter_id = 0, volume = 0;
	uint16_t ticks_in_10cl = 0;
	uint32_t cnt_val1 = 0, cnt_val2 = 0;
	Lcd_clear();
	Lcd_write_str("Choose one:");
	vTaskDelay(500);
	button = 0;
	while (button == 0) {
		button = readButtons();
		Lcd_goto(0, 0);
		Lcd_write_str("1:");
		Lcd_write_16b(water_counter[0]);
		Lcd_write_str("2:");
		Lcd_write_16b(water_counter[1]);
		Lcd_goto(1, 0);
		vTaskDelay(10);
		Lcd_write_str("3:");
		Lcd_write_16b(water_counter[2]);
		Lcd_write_str("4:");
		Lcd_write_16b(water_counter[3]);
	}
	Lcd_clear();
	counter_id = button - 1;
	button = 0;
	Lcd_write_str("Choose valve");
	valve = adjust8bit(valve);
	close_valve(valve);
	Lcd_clear();
	Lcd_write_str("How much x10CL?");
	volume = adjust8bit(volume);
	cnt_val1 = water_counter[counter_id];
	Lcd_clear();
	Lcd_write_str("OK to finish");
	open_valve(valve);
	button = 0;
	while (button != BUTTON_OK) {
		vTaskDelay(10);
		button = readButtons();
		Lcd_clear();
		Lcd_write_str("Current value:");
		Lcd_goto(1, 0);
		Lcd_write_16b(water_counter[counter_id]);
	}
	cnt_val2 = water_counter[counter_id];
	close_valve(valve);
	vTaskDelay(200);

	ticks_in_10cl = (uint16_t)(cnt_val2 - cnt_val1) / (uint16_t) volume; // how much ticks for 10 CL?
	wfCalArray[counter_id] = ticks_in_10cl; // ticks in 10cl
	Lcd_clear();
	Lcd_write_str("Ticks:");
	Lcd_write_16b(cnt_val2);
	Lcd_goto(1, 0);
	Lcd_write_str("10CL=");
	Lcd_write_16b(ticks_in_10cl);
	EE_WriteVariable(WFM_CAL_OFFSET, ticks_in_10cl);
	button = 0;
	while (button != BUTTON_OK) {
		vTaskDelay(10);
		button = readButtons();
	}
	Lcd_clear();
	vTaskDelay(200);
}

void get_water_cl(uint8_t valve, uint8_t counter_id, uint16_t amount) {
	uint32_t cnt_to_reach = 0;
	cnt_to_reach = ((uint32_t) amount * (uint32_t) wfCalArray[0]) / 10;
	open_valve(0); // HARDCODE
	water_counter[counter_id] = 0;
	while (water_counter[0] < cnt_to_reach) {
		vTaskDelay(10);
	}
	close_valve(0); // HARDCODE
}

void get_water_tick(uint8_t valve, uint8_t counter_id, uint32_t ticks) {
	open_valve(0); // HARDCODE
	water_counter[counter_id] = 0;
	while (water_counter[0] < ticks) {
		vTaskDelay(10);
	}
	close_valve(0); // HARDCODE
}

void get_water(uint8_t valve, uint8_t counter_id, uint16_t amount) { // amount in CL (10ml) max 65535x10=650L
	uint32_t cnt_to_reach = 0;
	uint16_t volume = 0;
	Lcd_clear();
	Lcd_write_str("Calibrate WFM?");
	while (button == 0) {
		vTaskDelay(10);
		button = readButtons();
	}
	if (button == BUTTON_OK) {
		calibrateFlowMeter();
	}
	uint8_t repeat = 1;
	while (repeat == 1) {
		Lcd_clear();
		Lcd_write_str("CL amount:");
		Lcd_write_16b(wfCalArray[0]);
		volume = adjust16bit_fast(volume, 5);
		cnt_to_reach = ((uint32_t) volume * (uint32_t) wfCalArray[0]) / 10;

		Lcd_clear();
		Lcd_write_str("Adding");
		Lcd_write_16b((uint16_t) cnt_to_reach);
		Lcd_goto(1, 0);
		Lcd_write_str("to ");
		Lcd_write_16b((uint16_t) water_counter[water_counter[counter_id]]);
		button = 0;
		open_valve(0); // HARDCODE
		water_counter[counter_id] = 0;
		while (water_counter[0] < cnt_to_reach) {
			vTaskDelay(10);
			Lcd_clear();
			Lcd_write_str("Cur:");
			Lcd_write_16b((uint16_t) water_counter[counter_id]);
			Lcd_goto(1, 0);
			Lcd_write_str("of:");
			Lcd_write_16b(cnt_to_reach);
		}
		close_valve(0); // HARDCODE
		Lcd_clear();
		Lcd_write_str("Repeat?");
		while (button == 0) {
			vTaskDelay(10);
			button = readButtons();
		}
		if (button == BUTTON_OK) {
			repeat = 1;
			water_counter[0] = 0;
		} else {
			repeat = 0;
		}
	}
}

void Lcd_write_16b(uint16_t tmp) {
	Lcd_write_digit((uint16_t) tmp / 10000);
	Lcd_write_digit((uint16_t)(tmp % 10000) / 100);
	Lcd_write_digit((uint16_t) tmp % 100);
}

uint8_t crc_block(uint8_t input, volatile uint8_t *start_byte, uint8_t length) {
	uint8_t i = 0;
	for (i = 0; i < length; i++) {
		input ^= start_byte[i];
	}
	return input;
}

void USART1_IRQHandler(void) // Protobuzzz v3
{
	if (USART_GetITStatus(BT_USART, USART_IT_RXNE) != RESET) {
		RxByte = USART_ReceiveData(BT_USART); // get byte from Data Register
#ifndef		PRODUCTION_BUILD
		RxBuffer[RxCounter++] = RxByte; // write it into RxBuffer
#endif
		rxcntr++;
		if (rxcntr > 60000) {
			rxcntr = 0;
		}
		USART1->SR &= ~USART_FLAG_RXNE; // clear Rx Not Empty flag
	}

	// sending packet in case of Tx Not Empty flag is set and buffer not sent completely yet
	if (txbuff_ne == 1 && TxCounter < NbrOfDataToTransfer) {
		USART1->DR = TxBuffer[TxCounter++]; // write TxBuff byte into Data Register for sending
	}
	BT_USART->SR &= ~USART_SR_TC; // clear Transfer Complete flag

	if (packet_ready == 0) {
		if (rxm_state == RXM_CMD) {
			if (rx_pntr == 0) { // if packet buffer pointer is 0, it points to payload size (payload, including this size byte)
				packet_length = RxByte; // get packet length
			}
			RxBuffer[rx_pntr++] = RxByte; // receive byte into cmd buffer and increase command packet buffer pointer
			if (rx_pntr == (packet_length - 3)) { // if packet buffer pointer reached packet length (rely on RXM_NONE set later)
				// crc count
				rx_packet_crc = crc_block(48, &RxBuffer[0],
						(packet_length - 4)); // 48 is XOR of "ZX2", -1 for skipping cmd id resp
				// crc check
				rxm_state = RXM_NONE; // RX machine state set to NONE, completes RX cycle
				rx_pntr = 0; // packet buffer pointer to 0
				prefixDetectionIdx = 0;
				if (rx_packet_crc == 0) { //
					packetid_ = RxBuffer[(packet_length - 4)];
					packets_received++;
					packet_ready = 1; // packet received correctly and is ready to process
				} else {
					rx_flush(); // discard broken packet
				}
			}
		}

		if (RxByte == 90) {
			// ready
			prefixDetectionIdx = 1;
		} else if (RxByte == 88 && prefixDetectionIdx == 1) {
			// steady
			prefixDetectionIdx = 2;
		} else if (prefixDetectionIdx == 2) {
			if (RxByte == 48) {
				// escaping zero
			} else if (RxByte == 49) {
				// settings
				rxm_state = RXM_SET;
				rx_pntr = 0;
			} else if (RxByte == 50) {
				// command
				rxm_state = RXM_CMD;
				rx_pntr = 0;
				rx_flush();
			}
			prefixDetectionIdx = 0;
		}
	}
}

/* void USART1_IRQHandler(void)		// Protobuzz v2
 {

 if (USART_GetITStatus(BT_USART, USART_IT_RXNE) != RESET) {
 rxglobalcntr++;
 RxByte = USART_ReceiveData(BT_USART);	// get byte from Data Register
 #ifndef		PRODUCTION_BUILD
 RxBuffer[RxCounter++] = RxByte;			// write it into RxBuffer
 #endif
 rxcntr++;
 if (rxcntr>60000) {
 rxcntr = 0;
 }
 USART1->SR &= ~USART_FLAG_RXNE;			// clear Rx Not Empty flag
 }



 // sending packet in case of Tx Not Empty flag is set and buffer not sent completely yet
 if (txbuff_ne==1 && TxCounter<NbrOfDataToTransfer) {
 USART1->DR = TxBuffer[TxCounter++];	// write TxBuff byte into Data Register for sending
 }
 BT_USART->SR &= ~USART_SR_TC;		// clear Transfer Complete flag

 if (packet_ready==0) {
 if (rxm_state==RXM_CMD) {
 if (rx_pntr==0) {	// if packet buffer pointer is 0, it points to payload size (payload, including this size byte)
 packet_length=RxByte;	// get packet length
 }
 RxBuffer[rx_pntr++]=RxByte;	// receive byte into cmd buffer and increase command packet buffer pointer
 if (rx_pntr==(packet_length-3)) {	// if packet buffer pointer reached packet length (rely on RXM_NONE set later)
 // crc count
 rx_packet_crc = crc_block(48, &RxBuffer[0],(packet_length-4));	// 48 is XOR of "ZX2", -1 for skipping cmd id resp
 // crc check
 rxm_state=RXM_NONE; // RX machine state set to NONE, completes RX cycle
 rx_pntr=0;			// packet buffer pointer to 0
 prefixDetectionIdx = 0;
 if (rx_packet_crc==0) {	//
 packet_ready=1;	// packet received correctly and is ready to process
 }
 else {
 rx_flush();	// discard broken packet
 }
 }
 }

 if (RxByte==90) {
 // ready
 prefixDetectionIdx = 1;
 }
 else if (RxByte==88 && prefixDetectionIdx==1) {
 // steady
 prefixDetectionIdx = 2;
 }
 else if (prefixDetectionIdx==2) {
 if (RxByte==48) {
 // escaping zero
 }
 else if (RxByte==49){
 // settings
 rxm_state=RXM_SET;
 }
 else if (RxByte==50) {
 // command
 rxm_state=RXM_CMD;
 rx_flush();
 }
 prefixDetectionIdx = 0;
 }
 }
 } */

void save_settings(void) { // wrapper for rx_ee, simplifies settings packet receiving
	if (RxBuffer[0] == 5) { // 16 bit (5: size(1b), addr(2b), data(2b))
		rx_ee((RxBuffer[2] & (RxBuffer[3] << 8)), 1);
	}
	if (RxBuffer[0] == 7) { // 32 bit (7: size(1b), addr(2b), data(4b))
		rx_ee((RxBuffer[2] & (RxBuffer[3] << 8)), 2);
	}
}

x = DMA1_FLAG_TC6;

uint8_t ds_buf[9];

void dsread(void) {
	uint32_t i = 0;

	OW_Init();
	OW_Send(OW_SEND_RESET, "\xcc\x44", 2, NULL, NULL, OW_NO_READ);

	// 0b10000010 0x82

	// Ã�Â½Ã�Â°Ã�Â·Ã�Â½Ã�Â°Ã‘â€¡Ã�Â°Ã�ÂµÃ�Â¼ Ã‘â€žÃ‘Æ’Ã�Â½Ã�ÂºÃ‘â€ Ã�Â¸Ã‘Å½ Ã�Â´Ã�Â²Ã‘Æ’Ã‘â€¦Ã‘â€šÃ�Â°Ã�ÂºÃ‘â€šÃ�Â½Ã�Â¾Ã�Â³Ã�Â¾ Ã�Â²Ã‘â€¹Ã‘â€¦Ã�Â¾Ã�Â´Ã�Â° - Ã�Â¿Ã�Â¾Ã�Â´Ã�Â°Ã�ÂµÃ�Â¼ "Ã�Â¿Ã�Â¸Ã‘â€šÃ�Â°Ã�Â½Ã�Â¸Ã�Âµ" Ã�Â½Ã�Â° Ã‘Ë†Ã�Â¸Ã�Â½Ã‘Æ’

//		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;     // Enable clock on GPIOC.
	//  GPIOA->CRL      &= ~GPIO_CRL_CNF2;		// ... to PC3
//	    GPIOA->CRL      |= (GPIO_CRL_CNF2_0 | GPIO_CRL_CNF2_1);
//	    GPIOA->CRL  |= (GPIO_CRL_MODE2_1 | GPIO_CRL_MODE2_0);

	OW_out_set_as_Power_pin();
	GPIOA->BSRR |= (1 << 2);

	// Ã�Â²Ã‘â€¹Ã�Â´Ã�ÂµÃ‘â‚¬Ã�Â¶Ã�Â¸Ã�Â²Ã�Â°Ã�ÂµÃ�Â¼ Ã�Â²Ã‘â‚¬Ã�ÂµÃ�Â¼Ã‘ï¿½ Ã�Â¸Ã�Â·Ã�Â¼Ã�ÂµÃ‘â‚¬Ã�ÂµÃ�Â½Ã�Â¸Ã‘ï¿½ (Ã�Â½Ã�Â°Ã�Â¿Ã‘â‚¬Ã�Â¸Ã�Â¼Ã�ÂµÃ‘â‚¬ 750 Ã�Â¼Ã‘ï¿½ Ã�Â´Ã�Â»Ã‘ï¿½ 12-Ã�Â±Ã�Â¸Ã‘â€šÃ�Â½Ã�Â¾Ã�Â³Ã�Â¾ Ã�Â¸Ã�Â·Ã�Â¼Ã�ÂµÃ‘â‚¬Ã�ÂµÃ�Â½Ã�Â¸Ã‘ï¿½)
	for (i = 0; i < 2000000; i++) {

	}

//	    OW_Reset(); // reset

	// Ã�Â²Ã�Â¾Ã‘ï¿½Ã‘ï¿½Ã‘â€šÃ�Â°Ã�Â½Ã�Â°Ã�Â²Ã�Â»Ã�Â¸Ã�Â²Ã�Â°Ã�ÂµÃ�Â¼ Ã‘â€žÃ‘Æ’Ã�Â½Ã�ÂºÃ‘â€ Ã�Â¸Ã‘Å½ Ã�Â¿Ã�ÂµÃ‘â‚¬Ã�ÂµÃ�Â´Ã�Â°Ã‘â€šÃ‘â€¡Ã�Â¸Ã�ÂºÃ�Â° UART
	OW_out_set_as_TX_pin();

	OW_Send(OW_SEND_RESET, "\xcc\xbe\xff\xff", 4, ds_buf, 2, 2);

	OW_Reset();

}

uint16_t ph1_adc_val = 0;
uint16_t ec1_adc_val = 0;

static void lstasks(void *pvParameters) {
	vTaskDelay(4000);
//	dht_init_out();
	uint8_t i = 0;
	uint32_t pwr_restart = 0; // next power restart for DHT sensors
	OW_Init();
	power_ctrl(PWR_DHT, 1); // enable power for DHT sensors
	uint8_t i2c_ping = 0;
//	OW_Init();
	pwr_restart = RTC_GetCounter() + 60;
	while (1) {

		if (pwr_restart < RTC_GetCounter()) {
			power_ctrl(PWR_DHT, 0);
			vTaskDelay(200);
			power_ctrl(PWR_DHT, 1);
			pwr_restart = RTC_GetCounter() + DHT_PWR_RESTART_INTERVAL;
		}

		// read pH
//		phval_v = Buffer_Rx2[1] + (((uint16_t)Buffer_Rx2[0]<<8) & 0xFF00);	// !!!
//		i2c_ping = I2C_Master_BufferWrite(I2C2,Buffer_Tx2,1,Polling, PH_I2C_ADDR);
		/*		if (i2c_ping>0) {
		 I2C_Master_BufferRead(I2C2,Buffer_Rx2,2,DMA, PH_I2C_ADDR);
		 ph1_adc_val = (((uint16_t)Buffer_Rx2[0]<<8)&0xFF00) + (((uint16_t)Buffer_Rx2[1])&0xFF);
		 }
		 else {
		 // restart I2C2 sensors bus
		 I2C_DeInit(I2C2);
		 I2C_LowLevel_Init(I2C2);
		 } */

		vTaskDelay(100);

		// read EC
		/*		i2c_ping = 0;
		 // i2c_ping = I2C_Master_BufferWrite(I2C2,Buffer_Tx2,1,Polling, EC_I2C_ADDR);
		 i2c_ping = I2C_Master_BufferWrite(I2C2,Buffer_Tx2,1,Polling, EC_I2C_ADDR);
		 if (i2c_ping>0) {
		 I2C_Master_BufferRead(I2C2,Buffer_Rx2,4,Polling, EC_I2C_ADDR);
		 vTaskDelay(150);
		 ec1_adc_val = (((uint16_t)Buffer_Rx2[2]<<8)&0xFF00) + (((uint16_t)Buffer_Rx2[1])&0xFF);
		 }
		 else {
		 // restart I2C2 sensors bus
		 I2C_DeInit(I2C2);
		 I2C_LowLevel_Init(I2C2);
		 } */

		vTaskDelay(100);
		psiStab();
		dht_get_data_x(0);
		sonar_ping();
		autoSafe();
//		dsread();

		/*
		 PLUG_DISABLE = (1<<13);
		 vTaskDelay(50);
		 PLUG_ENABLE = (1<<13);
		 vTaskDelay(50);
		 dht2_get_data(); */
		/*	for (i=0;i<50;i++) {
		 PLUG_DISABLE = (1<<13);
		 vTaskDelay(i);
		 PLUG_ENABLE = (1<<13);
		 vTaskDelay(i);
		 }
		 for (i=50;i>0;i--) {
		 PLUG_DISABLE = (1<<13);
		 vTaskDelay(i);
		 PLUG_ENABLE = (1<<13);
		 vTaskDelay(i);
		 } */
		// get_status_block(1);
		//vTaskDelay(5000);
		dht_get_data_x(1);
		vTaskDelay(500);
	}
}

uint8_t last_cmd = 0;

static void uart_task(void *pvParameters) {
	uint8_t i = 0;
	while (1) {
		IWDG_ReloadCounter();
		runners &= (1 << 3);
		vTaskDelay(30);
		if (packet_ready == 1) {
			run_uart_cmd(); // when command packet received, run the command
			if (RxBuffer[1] < 50 || RxBuffer[1] > 99) { // filter out tasks not needed the confirmation
				last_cmd = RxBuffer[1];
				for (i = 0; i < CMD_CONF_AMOUNT; i++) {
					send_resp(RxBuffer[RxBuffer[0] - 4]); // send cmd id recently run, to confirm execution
					vTaskDelay(25);
					IWDG_ReloadCounter();
				}
				rx_flush();
				rx_pntr = 0;
			}

			packet_ready = 0;
		}
		vTaskDelay(30);
	}
}

void rx_flush(void) {
	uint8_t i = 0;
	for (i = 0; i < 42; i++) { /// HARDCODE
		RxBuffer[i] = 0;
	}
}

void tx_flush(void) {
	uint8_t i = 0;
	NbrOfDataToTransfer = 0;
	for (i = 0; i < 42; i++) { // HARDCODE
		TxBuffer[i] = 0;
		vTaskDelay(1);
	}
}

void push_tx(void) {
	txbuff_ne = 0; // stop Tx transfers
	NbrOfDataToTransfer = TxBuffer[3] + 1; // packet size with code 13
	TxBuffer[TxBuffer[3]] = 13;
	TxCounter = 0; // reset tx buffer pointer
	txbuff_ne = 1;
	BT_USART->DR = TxBuffer[TxCounter++]; //vozmozhno, tak budet luchshe (bez otpravki lishnego nulja)
	while (TxCounter < NbrOfDataToTransfer) {
		vTaskDelay(5);
		IWDG_ReloadCounter();
	}
	NbrOfDataToTransfer = 0;
	tx_flush(); // flush Tx buffer
	txbuff_ne = 0; // packet sent, reset tx not empty flag
}

void send_resp_x(uint8_t cmd_uid, uint8_t x){
	uint8_t i = 0;
	for (i=0;i<x;i++) {
		send_resp(cmd_uid);
		vTaskDelay(50);
		IWDG_ReloadCounter();
	}
}

void send_resp(uint8_t cmd_uid) { // Protobuzzz v3
	tx_flush();
	TxBuffer[0] = 90; // "Z"
	TxBuffer[1] = 88; // "X"
	TxBuffer[2] = 55; // "7" command execution success response packet
	TxBuffer[3] = 7; // packet size
	TxBuffer[4] = cmd_uid;
	TxBuffer[(TxBuffer[3] - 2)] = crc_block(0, &TxBuffer[0], (TxBuffer[3] - 1)); // with crc
	TxBuffer[(TxBuffer[3] - 1)] = 0; // packet_id
	vTaskDelay(1);
	push_tx();
	resp_counter++;
	resp_id = cmd_uid;
}

void send_resp_bak(uint8_t cmd_uid) { // Protobuzz v2
	tx_flush();
	TxBuffer[0] = 90; // "Z"
	TxBuffer[1] = 88; // "X"
	TxBuffer[2] = 55; // "7" command execution success response packet
	TxBuffer[3] = 6; // packet size
	TxBuffer[4] = cmd_uid;
	TxBuffer[(TxBuffer[3] - 1)] = crc_block(0, &TxBuffer[0], (TxBuffer[3 - 1])); // with crc
	vTaskDelay(1);
	push_tx();
}

#define EE_DUMPSIZE	200		// eeprom dump size
#define EE_DUMPSTART	0x05C0	// eeprom dump start address
void water_lines(uint8_t source, uint16_t target, uint16_t amount) {
	close_valves();
	open_valve(source);
	open_valves(target);
}

// sends dump of eeprom in standart ZX1 packts of 39bytes size (32bytes payload)
void send_ee_dump(void) {
	uint16_t eevarcntr = 0;
	uint16_t addr = 0;
	while (eevarcntr < EE_DUMPSIZE) {
		addr = EE_DUMPSTART + eevarcntr;
		send_ee_block(addr);
		eevarcntr += 16; // 16 variables - dump block size accepted by send_ee_block(addr);
		vTaskDelay(2);
	}
}

void run_uart_cmd(void) {
	uint16_t addr = 0;
	uint16_t val16 = 0;
	uint16_t val16_2 = 0;
	uint8_t val8 = 0;
	uint32_t val32 = 0;
	switch (RxBuffer[1]) {
	// below 50 there are tasks that send responsewhen successfully executed
	case 0:
		get_water_cl(RxBuffer[2], RxBuffer[3], RxBuffer[4] * 256 + RxBuffer[5]);
		break;
	case 1:
		plugStateSet(RxBuffer[2], RxBuffer[3]);
		break;
	case 2:
		comm_state = COMM_DIRECT_DRIVE;
		psiOff();
		break;
	case 3:
		comm_state = COMM_MONITOR_MODE;
		break;
	case 4:
		open_valve(RxBuffer[2]);
		break;
	case 5:
		close_valve(RxBuffer[2]);
		break;
	case 6:
		get_settings_block(RxBuffer[2]);
		break;
	case 8:
		if (comm_state == COMM_DIRECT_DRIVE) {
			auto_flags = RxBuffer[2]; // set auto-processing flags
		}
		break;
	case 9:
		// force triple command execution confirmation send before command execution
		// because we want to avoid double doses of fertilizers
		send_resp_x(RxBuffer[RxBuffer[0]],15);
		vTaskDelay(200);
		run_doser_for(RxBuffer[2], RxBuffer[3], RxBuffer[4]);
		break;
	case 10:
//			valve_failed = RxBuffer[2];
		fup_time = RTC_GetCounter() + 60;
		auto_failures = 0;
		wpProgress = 0;
		runners = 0;
		break;
	case 11: // stop all processes and force manual control
		auto_flags = 0;
		comm_state = COMM_DIRECT_DRIVE;
		auto_failures &= 1;
		plugStateSet(0, 0);
		plugStateSet(1, 0);
		plugStateSet(2, 0);
		plugStateSet(3, 0);
		psiOff();
		close_valves();
		break;
	case 12:
		RTC_SetCounter(
				RxBuffer[2] * 16777216 + RxBuffer[3] * 65536 + RxBuffer[4] * 256
						+ RxBuffer[5]);
		break;
	case 13:
		send_resp_x(RxBuffer[RxBuffer[0]],5);
		vTaskDelay(10000); // wait a lot for watchdog to trigger reset
		break;
	case 14:
		// force triple command execution confirmation send before command execution
		// because we want to avoid double doses of fertilizers
		send_resp_x(RxBuffer[RxBuffer[0]],15);
		vTaskDelay(5);
		get_fertilizer(RxBuffer[2], RxBuffer[3]);
		break;
	case 15: // receive 16 bit value to store into EEPROM
		addr = ((uint16_t) RxBuffer[2] + (uint16_t) RxBuffer[3] * 256);
		rx_ee(addr, 1);
		break;
	case 16: // receive 32byte block to store into EEPROM
		addr = ((uint16_t) RxBuffer[2] + (uint16_t) RxBuffer[3] * 256);
		rx_ee(addr, 16);
		break;
	case 17: // receive 8 bit value to store into EEPROM
		addr = ((uint16_t) RxBuffer[2] + (uint16_t) RxBuffer[3] * 256);
		rx_ee(addr, 10); // higher
		break;
	case 18: // receive 8 lower bits to store into EEPROM
		addr = ((uint16_t) RxBuffer[2] + (uint16_t) RxBuffer[3] * 256);
		rx_ee(addr, 11); // lower
		break;
	case 19:
		loadSettings();
		break;
	case 20:
//			valve_busy=0;
		break;
	case 21:
		break;

	case 22:
		psi_m_low = RxBuffer[2];
		break;
	case 23:
		psi_m_high = RxBuffer[2];
		break;
	case 24:
		psi_max_speed = RxBuffer[2];
		break;

		// CCD-to-Cadi Settings
	case 31: // 8 bit EEPROM value write
		addr = ((((uint16_t) RxBuffer[2]) << 8) & 0xFF00)
				+ (((uint16_t) RxBuffer[3]) & 0x00FF);
		EE_ReadVariable(addr, &val16);
		if (RxBuffer[4] == 0) { // PARITY: higher byte received
			val16 &= (uint16_t) 0x00FF;
			val16 += ((((uint16_t) RxBuffer[5]) << 8) & 0xFF00);
			EE_WriteVariable(addr, val16);
		} else if (RxBuffer[4] == 1) {
			val16 &= (uint16_t) 0xFF00;
			val16 += ((((uint16_t) RxBuffer[5])) & ((uint16_t) 0x00FF));
			EE_WriteVariable(addr, val16);
		} else {
			// wrong parity bit received with packet
		}
		break;
	case 32: // 16 bit EEPROM value write
		addr = ((((uint16_t) RxBuffer[2]) << 8) & 0xFF00)
				+ (((uint16_t) RxBuffer[3]) & 0x00FF);
		val16 = ((((uint16_t) RxBuffer[4]) << 8) & 0xFF00)
				+ (((uint16_t) RxBuffer[5]) & 0x00FF);
		EE_WriteVariable(addr, val16);
		wrtn_val = val16;
		wrtn_addr = addr;
		break;
	case 33: // 32 bit EEPROM value write
		addr = ((((uint16_t) RxBuffer[2]) << 8) & 0xFF00)
				+ (((uint16_t) RxBuffer[3]) & 0x00FF);
	/*	val32 = ((((uint32_t) RxBuffer[4]) << 24) & 0xFF000000)
				+ ((((uint32_t) RxBuffer[5]) << 16) & 0xFF0000)
				+ ((((uint32_t) RxBuffer[6]) << 8) & 0xFF00)
				+ (((uint32_t) RxBuffer[7]) & 0xFF); */
		val32 = RxBuffer[4] * 16777216 + RxBuffer[5] * 65536 + RxBuffer[6] * 256
								+ RxBuffer[7];
		EE_WriteWord(addr, val32);
		break;
	case 34:
		run_demo();
		break;
	case 35:	// simply run the pump
		run_watering_pump(((uint32_t)RxBuffer[2])&0xFF);
		break;
	case 36:	// run pump, watering lines desired, during time desired
		val16 = (((uint16_t) RxBuffer[4]) + (((uint16_t) RxBuffer[3]) * 256));
		val16_2 = (((uint16_t) RxBuffer[6]) + (((uint16_t) RxBuffer[5]) * 256));
		water_lines(RxBuffer[2], val16, val16_2);
		break;
	case 37:	// close all valvs
		close_valves();
		break;
	case 38:	// mix solution
		val32 = (((((uint32_t) RxBuffer[6]) )<<0)&0xFF) +
				(((((uint32_t) RxBuffer[5]) )<<8)&0xFF00) +
				(((((uint32_t) RxBuffer[4]) )<<16)&0xFF0000) +
				(((((uint32_t) RxBuffer[3]) )<<24)&0xFF000000);
		mix_solution(val32);
		break;
	case 51:
		get_status_block(RxBuffer[2]);
		break;
		// send full status sequence
	case 52:
		get_status_block(1);
		get_status_block(2);
		break;
	case 56: // receive 32 bit value to store into EEPROM
		rx_ee((RxBuffer[2] & (RxBuffer[3] << 8)), 2);
		break;
	case 57: // send 16 bit EEPROM value
		send_ee_addr((RxBuffer[2] + RxBuffer[3] * 256), 1);
		break;
	case 58: // send 32 bit EEPROM value
		addr = ((uint16_t) RxBuffer[2] + (uint16_t) RxBuffer[3] * 256);
		send_ee_addr(addr, 2);
		break;
	case 59: // send block of EEPROM values
		addr = ((uint16_t) RxBuffer[2] + (uint16_t) RxBuffer[3] * 256);
		send_ee_block(addr);
		break;
	case 60: // send full EEPROM dump (use 61 instead)
		send_ee_dump();
		break;
	case 61: // Request packet 90,88,50,7,61,10,0  (xor CRC = 10)
		get_settings_dump(); // Protobuzzz v3 and newer
		// the ones not needed to send additional confirmation response
		break;
	case 62: // Request packet 90,88,50,11,62,sizeL,sizeH,addrL,addrH,crc,0
		get_settings_dump_(SETTINGS_PACKET_SIZE, SETTINGS_START_ADDR);
		// the ones not needed to send additional confirmation response
		break;
	case 63: // Request packet 90,88,50,11,63,sizeL,sizeH,addrL,addrH,crc,0  (most advanced)
		addr = ((uint16_t) RxBuffer[2] + (uint16_t) RxBuffer[3] * 256);
		get_settings_dump_(SETTINGS_PACKET_SIZE,
				((uint16_t) RxBuffer[4] + (uint16_t) RxBuffer[5] * 256)); // Protobuzzz v3 and newer
		// the ones not needed to send additional confirmation response
		break;
	}
	vTaskDelay(50);
	if (RxBuffer[1] > 99) { // translate Watering Task into background process
		copy_arr(&RxBuffer, &wt_args, 7, 0, 2); // cache arguments for watering task
		pending_wt = RxBuffer[1];
	}
}

void run_watering_pump(uint32_t seconds) {
	uint32_t time2reach = 0;
	time2reach = RTC_GetCounter() + seconds;
	while (RTC_GetCounter() < time2reach) {
		TIM1->CCR1 = 0;
		vTaskDelay(100);
		IWDG_ReloadCounter();
	}
	TIM1->CCR1 = 1000;
}

void run_watering_pump_task(uint16_t secs){

}

void mix_tank(uint16_t secs){
	close_valves();
	open_valve(MTI_VALVE);
	open_valve(BACK_VALVE);
	run_watering_pump_task(secs);

}

void settings2tx_buff(uint16_t addr, uint8_t amount) {
	uint16_t tmpaddr = 0;
	uint32_t val = 0;
	uint8_t i = 0;
	tx_flush();
	for (i = 0; i < (amount / 2); i++) {
		tmpaddr = 0;
		tmpaddr = addr + (((uint16_t)(i * 2)) & 0x00FF);
		val = 0;
		val = EE_ReadWord(tmpaddr);
		TxBuffer[(i * 4)] = (uint8_t)((val >> 24) & 0x00FF);
		TxBuffer[((i * 4) + 1)] = (uint8_t)((val >> 16) & 0x00FF);
		TxBuffer[((i * 4) + 2)] = (uint8_t)((val >> 8) & 0x00FF);
		TxBuffer[((i * 4) + 3)] = (uint8_t)(val & 0x00FF);
	}
}

void get_settings_dump(void) {
	uint16_t i = 0;
	uint8_t i2 = 0;
	uint32_t tmpval = 0;
	uint16_t addr = 0;
	uint8_t tmpxor = 0;

	/// preliminary 0,1,2,3
	TxBuffer[0] = 90;
	TxBuffer[1] = 88;
	TxBuffer[2] = 49;
	TxBuffer[3] = (uint8_t)(
			((((SETTINGS_PACKET_SIZE / 16) * 32) + 10) >> 8) & 0x00FF); // HSByte first
	TxBuffer[4] = (uint8_t)((((SETTINGS_PACKET_SIZE / 16) * 32) + 10) & 0x00FF); // LSByte
	txbuff_ne = 0; // stop Tx transfers
	NbrOfDataToTransfer = 5; //going to send 32 bits of settings
	TxCounter = 0; // reset tx buffer pointer
	txbuff_ne = 1;
	BT_USART->DR = TxBuffer[TxCounter++]; // volshebnyj pendal
	while (TxCounter < NbrOfDataToTransfer) {
		vTaskDelay(5);
		IWDG_ReloadCounter();
	}
	vTaskDelay(15);
	tmpxor = crc_block(0, &TxBuffer[0], 5);
	tx_flush(); // flush Tx buffer
	txbuff_ne = 0; // packet sent, reset tx not empty flag

	vTaskDelay(100);

	for (i = 0; i < ((SETTINGS_PACKET_SIZE / 16) - 1); i++) { // -1 HARDCODE
//	for (i=0;i<32;i++) {
		settings2tx_buff(
				(SETTINGS_START_ADDR + ((((uint16_t) i) & 0x00FF) * 16)), 16); // 16 variables = 32 bytes
		for (i2 = 0; i2 < 32; i2++) {
			tmpxor ^= TxBuffer[i2];
		}
		// tmpxor ^= crc_block(0,&TxBuffer[0],32);

		txbuff_ne = 0; // stop Tx transfers
		NbrOfDataToTransfer = 32; //going to send 32 bits of settings
		TxCounter = 0; // reset tx buffer pointer
		txbuff_ne = 1;
		BT_USART->DR = TxBuffer[TxCounter++]; // volshebnyj pendal
		while (TxCounter < NbrOfDataToTransfer) {
			vTaskDelay(1);
			IWDG_ReloadCounter();
		}
		tx_flush(); // flush Tx buffer
		txbuff_ne = 0; // packet sent, reset tx not empty flag
	}

	/// postliminary 0,1,2,3
	for (i2 = 1; i2 < 5; i2++) {
		TxBuffer[i2] = i2;
	}

	TxBuffer[0] = tmpxor;
	txbuff_ne = 0; // stop Tx transfers
	NbrOfDataToTransfer = 4; //going to send 32 bits of settings
	TxCounter = 0; // reset tx buffer pointer
	txbuff_ne = 1;
	BT_USART->DR = 0; // volshebnyj pendal
	while (TxCounter < NbrOfDataToTransfer) {
		vTaskDelay(1);
		IWDG_ReloadCounter();
	}
	vTaskDelay(15);
	tx_flush(); // flush Tx buffer
	txbuff_ne = 0; // packet sent, reset tx not empty flag

}

// get block of EEPROM from 'startaddr' offset, with size of 'amount'
void get_settings_dump_(uint16_t amount, uint16_t startaddr) {
	uint16_t i = 0;
	uint8_t i2 = 0;
	uint32_t tmpval = 0;
	uint16_t addr = 0;
	uint8_t tmpxor = 0;

	/// preliminary ZX1, size and address offset
	TxBuffer[0] = 90; // Z
	TxBuffer[1] = 88; // X
	TxBuffer[2] = 49; // 1
	TxBuffer[3] = (uint8_t)(((((amount / 16) * 32) + 12) >> 8) & 0x00FF); // HSB
	TxBuffer[4] = (uint8_t)((((amount / 16) * 32) + 12) & 0x00FF); // packet size LSB
	TxBuffer[5] = (uint8_t)((startaddr >> 8) & 0x00FF); // address offset HSB
	TxBuffer[6] = (uint8_t)((startaddr) & 0x00FF);
	txbuff_ne = 0; // stop Tx transfers
	NbrOfDataToTransfer = 7; //going to send 32 bits of settings
	TxCounter = 0; // reset tx buffer pointer
	txbuff_ne = 1;
	BT_USART->DR = TxBuffer[TxCounter++]; // volshebnyj pendal
	while (TxCounter < NbrOfDataToTransfer) {
		vTaskDelay(5);
		IWDG_ReloadCounter();
	}
	vTaskDelay(15);
	tmpxor = crc_block(0, &TxBuffer[0], 7);
	tx_flush(); // flush Tx buffer
	txbuff_ne = 0; // packet sent, reset tx not empty flag

	vTaskDelay(100);

	for (i = 0; i < (amount / 16); i++) { //  HARDCODE
//	for (i=0;i<32;i++) {
		settings2tx_buff((startaddr + ((((uint16_t) i) & 0x00FF) * 16)), 16); // 16 variables = 32 bytes
		for (i2 = 0; i2 < 32; i2++) {
			tmpxor ^= TxBuffer[i2];
		}
		// tmpxor ^= crc_block(0,&TxBuffer[0],32);

		txbuff_ne = 0; // stop Tx transfers
		NbrOfDataToTransfer = 32; //going to send 32 bits of settings
		TxCounter = 0; // reset tx buffer pointer
		txbuff_ne = 1;
		BT_USART->DR = TxBuffer[TxCounter++]; // volshebnyj pendal
		while (TxCounter < NbrOfDataToTransfer) {
			vTaskDelay(1);
			IWDG_ReloadCounter();
		}
		tx_flush(); // flush Tx buffer
		txbuff_ne = 0; // packet sent, reset tx not empty flag
	}

	/// postliminary 0,1,2,3
	for (i2 = 1; i2 < 5; i2++) {
		TxBuffer[i2] = i2;
	}

	TxBuffer[0] = tmpxor;
	txbuff_ne = 0; // stop Tx transfers
	NbrOfDataToTransfer = 4; //going to send 32 bits of settings
	TxCounter = 0; // reset tx buffer pointer
	txbuff_ne = 1;
	BT_USART->DR = TxBuffer[TxCounter++]; // volshebnyj pendal
	while (TxCounter < NbrOfDataToTransfer) {
		vTaskDelay(1);
		IWDG_ReloadCounter();
	}
	vTaskDelay(15);
	tx_flush(); // flush Tx buffer
	txbuff_ne = 0; // packet sent, reset tx not empty flag

}

void wp_next_run(uint8_t wp_id, uint32_t time) {
	// setting the last run
}

// adds 'amount' seconds of fertilizer using pump with ID	 = pump_id
void run_doser_for(uint8_t pump_id, uint8_t amount, uint8_t speed) {
	uint32_t finish = 0, now = 0;
	now = RTC_GetCounter();
	finish = now + amount;
	enable_dosing_pump(pump_id, 1);
	vTaskDelay(2);
	enable_dosing_pump(pump_id, speed);
	// close_valves();
	// open_valve(MTI_VALVE);
	// open_valve(BACK_VALVE);
	// psiOn();
	while (now < finish) {
		// psiStab();
		// psiOn();
		now = RTC_GetCounter();
		IWDG_ReloadCounter();
		vTaskDelay(10);
//		send_resp();
		get_status_block(1); // send status blocks
		vTaskDelay(10);
	}
//	psiOff();
	enable_dosing_pump(pump_id, 0);
}

// 3 type EEPROM user-defined settings receiver: 8, 16 and 32 bit values accepted
void rx_ee(uint16_t addr, uint8_t type) {
	uint32_t val32 = 0;
	static uint16_t val16 = 0;
	uint8_t i = 0;
	if (type == 1) { // 2 bytes
		val16 = (RxBuffer[4] + (RxBuffer[5] * 256));
		EE_WriteVariable(addr, val16);
	}
	if (type == 2) { // 4 bytes
		val32 = (RxBuffer[4] + (RxBuffer[5] * 256) + (RxBuffer[6] * 65536)
				+ (RxBuffer[7] * 16777216));
		EE_WriteWord(addr, val32);
	}
	if (type == 16) { // 32 bytes block
		for (i = 0; i < 32; i++) {
			val16 = 0;
			val16 = ((uint16_t) RxBuffer[(4 + i * 2)]
					+ (uint16_t) RxBuffer[(5 + i * 2) * 256]);
			EE_WriteVariable(addr++, val16);
			vTaskDelay(1);
		}
	}
	if (type == 10) { // 1 higher byte
		uint16_t tmpval = 0;
		EE_ReadVariable(addr, &tmpval);
		tmpval &= (uint16_t) 0xFF;
		tmpval &= (((uint16_t) RxBuffer[4]) << 8);
		EE_WriteVariable(addr, tmpval);
	}
	if (type == 11) { // 1 lower byte
		uint16_t tmpval = 0;
		EE_ReadVariable(addr, &tmpval);
		tmpval &= (uint16_t) 0xFF00;
		tmpval &= (uint16_t) RxBuffer[4];
		EE_WriteVariable(addr, tmpval);
	}
}

void send_ee_addr(uint16_t addr, uint8_t type) { // sends EEPROM cell contents via BT-USART
	uint16_t val16;
	uint32_t val32;
	// types: 0 - 8 bit, 1 - 16bit, 2 - 32bit
	TxBuffer[0] = 90; // "Z"	-	ZX1 means settings
	TxBuffer[1] = 88; // "X"
	TxBuffer[2] = 49; // "1"
	if (type == 1) {
		EE_ReadVariable(addr, &val16);
		TxBuffer[3] = 9; // packet size. 9 byte for 16bit and 11b for 32bit values
		TxBuffer[4] = RxBuffer[2];
		TxBuffer[5] = RxBuffer[3];
		TxBuffer[6] = (uint8_t)(val16 & 0xFF);
		TxBuffer[7] = (uint8_t)((val16 >> 8) & 0xFF);
	}
	if (type == 2) {
		val32 = EE_ReadWord(addr);
		TxBuffer[3] = 11; // ZX1+size+4bytepayload+crc
		TxBuffer[4] = RxBuffer[2];
		TxBuffer[5] = RxBuffer[3];
		TxBuffer[6] = (uint8_t)(val32 & 0xFF);
		TxBuffer[7] = (uint8_t)((val32 >> 8) & 0xFF);
		TxBuffer[8] = (uint8_t)((val32 >> 16) & 0xFF);
		TxBuffer[9] = (uint8_t)((val32 >> 24) & 0xFF);
	}
	TxBuffer[(TxBuffer[3] - 1)] = crc_block(0, &TxBuffer[0], (TxBuffer[3] - 1));
	NbrOfDataToTransfer = TxBuffer[3];
	TxCounter = 0;
	txbuff_ne = 1;
	while (TxCounter < NbrOfDataToTransfer) {
		vTaskDelay(1);
	}
	txbuff_ne = 0;
}

void send_ee_block(uint16_t addr) {
	uint32_t val32;
	uint8_t i = 0, pointer = 6;
	// types: 0 - 8 bit, 1 - 16bit, 2 - 32bit
	TxBuffer[0] = 90; // "Z"	-	ZX1 means settings
	TxBuffer[1] = 88; // "X"
	TxBuffer[2] = 49; // "1"
	TxBuffer[3] = 39; // packet size is 39bytes for settings block of 32b
	TxBuffer[4] = RxBuffer[2]; // start address lower byte
	TxBuffer[5] = RxBuffer[3]; // higher byte
	for (i = 0; i < 8; i++) {
		val32 = 0;
		val32 = EE_ReadWord(addr + i * 2);
		// at this point data converted into bytes - higher first.
		// Note this when convert back on receiving (rx_ee() function)
		TxBuffer[pointer++] = (uint8_t)((val32 >> 24) & 0xFF);
		TxBuffer[pointer++] = (uint8_t)((val32 >> 16) & 0xFF);
		TxBuffer[pointer++] = (uint8_t)((val32 >> 8) & 0xFF);
		TxBuffer[pointer++] = (uint8_t)((val32)) & 0xFF;
	}

	TxBuffer[pointer] = crc_block(0, &TxBuffer[0], (TxBuffer[3] - 1));
	NbrOfDataToTransfer = TxBuffer[3];
	TxCounter = 0;
	txbuff_ne = 1;
	while (TxCounter < NbrOfDataToTransfer) {
		vTaskDelay(1);
	}
	txbuff_ne = 0;
	tx_flush();

}

void get_settings_block(uint8_t block_number) {
	/* block_number is a number from 0 to N, where N is a
	 * Settings memory array length / 8
	 * 8 - is a default block size for this function
	 */
	uint8_t i = 0;
	uint16_t addr = 0, tmpbuf = 0;
	TxBuffer[0] = 90; // Z
	TxBuffer[1] = 88; // X
	TxBuffer[2] = 49; // sending settings
	TxBuffer[3] = 42; // packet size (ZX0+packet_size+payload). Payload is 8bytes. Here, packet size means only payload with size byte, not like in STATUS packets, where size means full packet size
	TxBuffer[4] = block_number;
	for (i = 0; i < 18; i++) { // 18 blocks x 2bytes = 36bytes
		// here goes EEPROM reading loop, that puts payload data into TX buff, equipped with appropriate packet header
		addr = SETTINGS_START_ADDR + block_number * 18 + i * 2; // address for 18 variable (16bit each) block
		EE_ReadVariable(addr, &tmpbuf); // read 16 bit variable
		TxBuffer[5 + i * 2] = (uint8_t)(tmpbuf * 0xFF00) >> 8; // place it into..
		TxBuffer[5 + i * 2 + 1] = (uint8_t)(tmpbuf * 0xFF); // ..two parts
		vTaskDelay(1);
	}
	txbuff_ne = 1; // signal to transmit packet
}

/* void onPacketType(uint8_t b) {
 // depending on packet type we transit to next state
 switch (b) {
 case 1: // for configuration packet type...
 // uint8_t payload[400];
 payloadIdx = 0;
 rxm_state = STATE_CONFIGURATION_PAYLOAD;
 break;
 case 2: // for command packet type...
 rxm_state = STATE_COMMAND_TYPE; // ...expect command type to be read next
 break;
 }
 crc = 0;
 }

 void onPayloadByte(uint8_t b, uint8_t nextState) {
 // on each payload byte - fill payload buffer and calculate crc
 payload[payloadIdx++] = b;
 if (payloadIdx == countof(payload)) {
 rxm_state = nextState;
 }
 crc(b);
 } */

void get_status_block(uint8_t blockId) { // sends block with Cadi STATUS data
	if (txbuff_ne == 0) {
		TxBuffer[0] = 90; // Z
		TxBuffer[1] = 88; // X
		TxBuffer[2] = 51; // 3 (sending STATUS)
		TxBuffer[3] = 41; // packet size (ZX0+packet_size+payload+crc). Payload is 16bytes
		if (blockId == 1) { // state block 1
			TxBuffer[4] = comm_state;
			TxBuffer[5] = ((uint8_t)(timerStateFlags & 0xFF));
			TxBuffer[6] = ((uint8_t)(cTimerStateFlags & 0xFF));
			TxBuffer[7] = 0; // valve_flags  for 8 or less valves
			TxBuffer[8] = ((uint8_t) plugStateFlags & 0xFF);
			TxBuffer[9] = wpStateFlags; // watering program run flags
			TxBuffer[10] = dht1_data[0];
			TxBuffer[11] = dht1_data[1];
			TxBuffer[12] = dht1_data[2];
			TxBuffer[13] = dht1_data[3];
			TxBuffer[14] = (uint8_t)(RTC->CNTH & (0xFF)); // RTC unixtime
			TxBuffer[15] = (uint8_t)((RTC->CNTH >> 8) & (0xFF));
			TxBuffer[16] = (uint8_t)(RTC->CNTL & (0xFF));
			TxBuffer[17] = (uint8_t)(((RTC->CNTL) >> 8) & (0xFF));
			TxBuffer[18] = (uint8_t)(sonar_read[FWTANK_SONAR] & (0xFF)); // First sonar lower byte
			TxBuffer[19] = (uint8_t)((sonar_read[FWTANK_SONAR] >> 8) & (0xFF)); // first sonar higher byte
			TxBuffer[20] = (uint8_t)(sonar_read[MIXTANK_SONAR] & (0xFF)); // second sonar
			TxBuffer[21] = (uint8_t)((sonar_read[MIXTANK_SONAR] >> 8) & (0xFF));
			TxBuffer[22] = (uint8_t)(ph1_adc_val & (0xFF)); // ADC1 average reading
			TxBuffer[23] = (uint8_t)(((ph1_adc_val) >> 8) & (0xFF));
			TxBuffer[24] = (uint8_t)(adcAverage[1] & (0xFF)); // ADC2 average reading
			TxBuffer[25] = (uint8_t)(((adcAverage[1]) >> 8) & (0xFF));
			TxBuffer[26] = (uint8_t)(adcAverage[2] & (0xFF)); // ADC3 average reading
			TxBuffer[27] = (uint8_t)(((adcAverage[2]) >> 8) & (0xFF));
			TxBuffer[28] = (uint8_t)(adcAverage[3] & (0xFF)); // ADC4 average reading
			TxBuffer[29] = (uint8_t)(((adcAverage[3]) >> 8) & (0xFF));
			TxBuffer[30] = (uint8_t) runners; // first wfm counter
			TxBuffer[31] = (uint8_t)((valveFlags >> 8) & (0xFF));
			TxBuffer[32] = (uint8_t)(valveFlags & 0xFF);
			TxBuffer[33] = (uint8_t)(0xFF & ((PSI_PUMP_TIM->CCR1) / 10));
			TxBuffer[34] = dosingPumpStateFlags2; // dosing pump flags (dosingPumpStateFlags overwritten by some part of FW)
			TxBuffer[35] = auto_flags; // first sonar higher byte
			TxBuffer[36] = (uint8_t) wpProgress; // second sonar
			TxBuffer[37] = (uint8_t) auto_failures;
			TxBuffer[38] = blockId;
		}
		if (blockId == 2) { // state block 2
			TxBuffer[4] = (uint8_t)(psi_pump_top_level & (0xFF));
			TxBuffer[5] = (uint8_t)((psi_pump_top_level >> 8) & (0xFF));
			TxBuffer[6] = (uint8_t)(psi_pump_btm_level & (0xFF));
			TxBuffer[7] = (uint8_t)(((psi_pump_btm_level) >> 8) & (0xFF));
			TxBuffer[8] = (uint8_t) 0;
			TxBuffer[9] = (uint8_t)(((adcAverage[0]) >> 8) & (0xFF));
			TxBuffer[10] = (uint8_t)(((adcAverage[1]) >> 0) & (0xFF));
			TxBuffer[11] = (uint8_t)(((adcAverage[1]) >> 8) & (0xFF));
			TxBuffer[12] = (uint8_t)(((adcAverage[2]) >> 0) & (0xFF));
			TxBuffer[13] = (uint8_t)(((adcAverage[2]) >> 8) & (0xFF));
			TxBuffer[38] = blockId;
		}
		if (blockId == 3) { // state block 3
			TxBuffer[4] = (uint8_t)(GPIOA->IDR & (0xFF));
			TxBuffer[5] = (uint8_t)(((GPIOA->IDR) >> 8) & (0xFF));
			TxBuffer[6] = (uint8_t)(((GPIOA->IDR) >> 16) & (0xFF));
			TxBuffer[7] = (uint8_t)(((GPIOA->IDR) >> 24) & (0xFF));
			TxBuffer[8] = (uint8_t)(GPIOB->IDR & (0xFF));
			TxBuffer[9] = (uint8_t)(((GPIOB->IDR) >> 8) & (0xFF));
			TxBuffer[10] = (uint8_t)(((GPIOB->IDR) >> 16) & (0xFF));
			TxBuffer[11] = (uint8_t)(((GPIOB->IDR) >> 24) & (0xFF));
			TxBuffer[12] = dht2_data[2];
			TxBuffer[13] = dht2_data[3];
			TxBuffer[38] = blockId;
		}

		if (blockId == 4) { // state block 4
//				TxBuffer[4] = currentEc;
//				TxBuffer[5] = currentPh;
//				TxBuffer[6] = phUnderOver+ecUnderOver*4;
//				TxBuffer[7] = (uint8_t)(phWindowTop&(0xFF));
//				TxBuffer[8] = (uint8_t)((phWindowBottom>>8)&(0xFF));
			TxBuffer[9] = 00; // EMPTY
			TxBuffer[10] = dosingPumpStateFlags2;
			TxBuffer[11] = (uint8_t)(wfCalArray[0] & (0xFF));
			TxBuffer[12] = (uint8_t)((wfCalArray[0] >> 8) & (0xFF));
			TxBuffer[13] = 77; // dumb hardcode
			TxBuffer[38] = blockId;
		}

		if (blockId == 5) { // state block 5
			TxBuffer[4] = (uint8_t)(sonar_read[FWTANK_SONAR] & (0xFF)); // First sonar lower byte
			TxBuffer[5] = (uint8_t)((sonar_read[FWTANK_SONAR] >> 8) & (0xFF)); // first sonar higher byte
			TxBuffer[6] = (uint8_t)(sonar_read[MIXTANK_SONAR] & (0xFF)); // second sonar
			TxBuffer[7] = (uint8_t)((sonar_read[MIXTANK_SONAR] >> 8) & (0xFF));
			TxBuffer[8] = (uint8_t)(water_counter[0] & (0xFF)); // first wfm counter
			TxBuffer[9] = (uint8_t)((water_counter[0] >> 8) & (0xFF));
			TxBuffer[10] = (uint8_t)((water_counter[0] >> 16) & (0xFF));
			TxBuffer[11] = (uint8_t)((water_counter[0] >> 24) & (0xFF));
//				TxBuffer[12] = valve_failed;		//
			TxBuffer[13] = 77; // dumb hardcode
			TxBuffer[38] = blockId;
		}

		if (blockId == 6) { // block 6
			TxBuffer[4] = (uint8_t)(sonar_read[FWTANK_SONAR] & (0xFF)); // First sonar lower byte
			TxBuffer[5] = (uint8_t)((sonar_read[FWTANK_SONAR] >> 8) & (0xFF)); // first sonar higher byte
			TxBuffer[6] = (uint8_t)(sonar_read[MIXTANK_SONAR] & (0xFF)); // second sonar
			TxBuffer[5] = (uint8_t)((sonar_read[MIXTANK_SONAR] >> 8) & (0xFF));
			TxBuffer[8] = (uint8_t)(water_counter[1] & (0xFF)); // 2nd water flow meter counter
			TxBuffer[9] = (uint8_t)((water_counter[1] >> 8) & (0xFF));
			TxBuffer[10] = (uint8_t)((water_counter[1] >> 16) & (0xFF));
			TxBuffer[11] = (uint8_t)((water_counter[1] >> 24) & (0xFF));
			TxBuffer[12] = (uint8_t)(wfCalArray[1] & (0xFF));
			TxBuffer[13] = (uint8_t)((wfCalArray[1] >> 8) & (0xFF));
			TxBuffer[22] = blockId;
		}
		TxBuffer[(TxBuffer[3] - 2)] = crc_block(0, &TxBuffer[0],
				(TxBuffer[3] - 2)); // with crc

		TxBuffer[(TxBuffer[3] - 1)] = 0; // packet id

		push_tx();
	}
}

void DMA1_Channel4_IRQHandler(void) {
	if (DMA1->ISR & DMA_ISR_TCIF4) {
	}
}

void DMA1_Channel5_IRQHandler(void) {

	if (DMA1->ISR & DMA_ISR_TCIF5) {
	}
}

void bluetooth_init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Tx on PA9 as alternate function push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Rx on PA10 as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;

	// USART 1 init
	USART_DeInit(USART1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ClearFlag(USART1,
			USART_FLAG_CTS | USART_FLAG_LBD | USART_FLAG_TC | USART_FLAG_RXNE);
	USART1->CR1 |= USART_CR1_RXNEIE;
	USART1->CR1 |= USART_CR1_TCIE;
	USART_Cmd(USART1, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
}

unsigned char GetStateDMAChannel4(void) {
	if (DMA1->ISR & DMA_ISR_TCIF4)
		return 1; //transmission finished
	return 0; //transmission in prograss
}
//********************************************************************************
//Function: start exchange in  direction "memory-DMA-USART1"                           //
//Argument: amount of data                                        //
//********************************************************************************
void StartDMAChannel4(uint8_t LengthBufer) {
	DMA1_Channel4->CCR = ~DMA_CCR4_EN; //disable DMA channel
	DMA1_Channel4->CNDTR = LengthBufer; //load data amount to transmit
	DMA1->IFCR |= DMA_IFCR_CTCIF4; //reset end of transmit flag
	DMA1_Channel4->CCR |= DMA_CCR4_EN; //enable DMA channel
}

void valve_feedback_init(void) { // init PA6-8 as input for 3V valve feedback

}

void water_level_input_init(void) {
#ifdef WFM_PINS_PB0_1			// Configuring PINs for Water Flow Meters
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable GPIO clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// Configure PB0-2 pins as input pull-down
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

// interrupt config for water level sensor inputs

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0E;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif
}

void dosing_motor_control_init(void) { // init PC6-PC9 as PWM output for dosing pump control
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8
			| GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alt Function - Push Pull
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); // Map TIM3_CH3 to GPIOC.Pin8, TIM3_CH4 to GPIOC.Pin9

	// Let PWM frequency equal 100Hz.
	// Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
	// Solving for prescaler gives 240.
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1; // 0..999
	TIM_TimeBaseInitStruct.TIM_Prescaler = 240 - 1; // Div 240
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 0%. Value can range from zero to 1000.
	TIM_OCInitStruct.TIM_Pulse = 0; // 0 .. 1000 (0=Always Off, 1000=Always On)

	TIM_OC1Init(TIM3, &TIM_OCInitStruct); // Channel 3 Blue LED
	TIM_OC2Init(TIM3, &TIM_OCInitStruct); // Channel 3 Blue LED
	TIM_OC3Init(TIM3, &TIM_OCInitStruct); // Channel 3 Blue LED
	TIM_OC4Init(TIM3, &TIM_OCInitStruct); // Channel 4 Green LED

	TIM_Cmd(TIM3, ENABLE);

	stop_dosers();
}

void stop_dosers(void) { // stop all dosing pumps
	uint8_t i = 0;
	for (i = 0; i < 4; i++) {
		enable_dosing_pump(i, 0);
	}
}

// initializes the PWM pin for PSI pump motor variable speed drive
void psi_motor_init_tim2(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alt Function - Push Pull
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); // Map TIM3_CH3 to GPIOC.Pin8, TIM3_CH4 to GPIOC.Pin9

	// Let PWM frequency equal 100Hz.
	// Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
	// Solving for prescaler gives 240.
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1; // 0..999
	TIM_TimeBaseInitStruct.TIM_Prescaler = 240 - 1; // Div 240
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 0%. Value can range from zero to 1000.
	TIM_OCInitStruct.TIM_Pulse = 0; // 0 .. 1000 (0=Always Off, 1000=Always On)

	TIM_OC3Init(TIM2, &TIM_OCInitStruct); // Channel 3 Blue LED
	TIM_OC4Init(TIM2, &TIM_OCInitStruct); // Channel 4 Green LED

	TIM_Cmd(TIM2, ENABLE);
	psiOff();
}

// tim1 PWM init for PSI pump
void psi_motor_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alt Function - Push Pull
	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_PinRemapConfig(GPIO_Remap_TIM1, ENABLE);        // Map TIM3_CH3 to GPIOC.Pin8, TIM3_CH4 to GPIOC.Pin9

	// Let PWM frequency equal 100Hz.
	// Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
	// Solving for prescaler gives 240.
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1; // 0..999
	TIM_TimeBaseInitStruct.TIM_Prescaler = 24 - 1; // Div 240
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(PSI_PUMP_TIM, &TIM_TimeBaseInitStruct);

	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	// Initial duty cycle equals 0%. Value can range from zero to 1000.
	TIM_OCInitStruct.TIM_Pulse = 500; // 0 .. 1000 (0=Always Off, 1000=Always On)

	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;

	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	/*  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	 TIM_OCInitStruct.TIM_Pulse = 1000;
	 TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	 TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset;*/
	TIM_OC1Init(PSI_PUMP_TIM, &TIM_OCInitStruct); // Channel 1

	TIM_Cmd(PSI_PUMP_TIM, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	psiOff();
}

void psi_pwm_test(void) {
	uint8_t button = 0;
	uint8_t curpwm = 0;
	while (button != BUTTON_FWD) {
		button = readButtons();
		vTaskDelay(5);
		if (button == BUTTON_BCK && curpwm > 0) {
			curpwm--;
		}
		if (button == BUTTON_OK && curpwm < 255) {
			curpwm++;
		}
		PSI_PUMP_TIM->CCR1 = 1000 - curpwm * 10;
//		PSI_PUMP_TIM->CCR2 = 1000-curpwm*10;
		Lcd_clear();
		Lcd_write_str("PSI PWM= ");
		Lcd_write_8b(curpwm);
	}
	Lcd_clear();
}

void tankLevelStabSetup(void) {
	Lcd_clear();
	uint16_t curlevel = 0;
	uint8_t tmp = 0;
	Lcd_clear();
	vTaskDelay(10);
	Lcd_clear();
	Lcd_write_str("Set tank top lvl");
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		curlevel = sonar_read[FWTANK_SONAR];
		vTaskDelay(5);
		Lcd_goto(1, 0);
		Lcd_write_digit(curlevel / 100);
		Lcd_write_digit(curlevel);
	}
	vTaskDelay(10);
	EE_WriteVariable(FWTANK_TOP, curlevel);
	printOk();
	vTaskDelay(2000);
	Lcd_write_str("Set btm lvl");
	button = 0;
	Lcd_clear();
	loadSettings();
}

void tankLevelStab(void) {
#ifdef TANK_STAB_ENABLE
	vTaskDelay(1);
//	uint16_t tmp=0;
	uint8_t supply_valve = 0;
//	EE_ReadVariable(WATER_TANK_SUPPLY_VALVE,&tmp);
	vTaskDelay(1);
//	supply_valve = (uint8_t)(tmp&0x00FF);
	if (((auto_flags & 2) >> 1) == 1) { // TANK STAB flag 1
		if (sonar_read[FWTANK_SONAR] > (tank_windows_top[FWTANK] + 2)) {
			open_valve(WI_VALVE);
		}
		vTaskDelay(1);
		if (sonar_read[FWTANK_SONAR] < tank_windows_top[FWTANK]) {
			close_valve(WI_VALVE);
		}
	}
#endif
}


void open_valve(uint8_t valveId){
	if (valveId > 0 && valveId < 5) {		// [PA4-PA7]
		GPIOA->BRR |= (1 << (valveId + 3));
	}

	else if (valveId > 4 && valveId < 8) {		// [PC10-PC12]
		GPIOC->BRR |= (1 << (valveId + 5));
	}

	else if (valveId > 7 && valveId < 12) {		// [PA11-PA14]
		GPIOA->BRR |= (1 << (valveId + 3));
	}

	else if (valveId == 12){
		GPIOA->BRR |= (1 << 8);			// PA8
	}

	valveFlags |= (1 << valveId); // set flag
}

void close_valve(uint8_t valveId){
	if (valveId > 0 && valveId < 5) {		// [PA4-PA7]
		GPIOA->BSRR |= (1 << (valveId + 3));
	}

	else if (valveId > 4 && valveId < 8) {		// [PC10-PC12]
		GPIOC->BSRR |= (1 << (valveId + 5));
	}

	else if (valveId > 7 && valveId < 12) {		// [PA11-PA14]
		GPIOA->BSRR |= (1 << (valveId + 3));
	}

	else if (valveId == 12){
		GPIOA->BSRR |= (1 << 8);
	}

	valveFlags &= ~(1 << valveId); // reset flag

}

void open_valve_old(uint8_t valveId) {
	// valve 0
	if (valveId == 0) {
		VALVE_CTRL_PORT->BRR |= (1 << 8); // valveId=0 is FWI valve
	}
	// valves [1..4]]
	else if (valveId > 0 && valveId < 5) { // PA11-PA14 driven Cadi board MOSFET drivers
		VALVE_CTRL_PORT->BRR |= (1 << (valveId + 10)); // 10 - shift for PA11 for valve 1
	}
	// valves [5..8]
	else if (valveId > 4 && valveId < 9) { // PA4-PA7 driven extender 4-ch. MOSFET driver
		VALVE_CTRL_PORT->BRR |= (1 << ((valveId - 1))); // (-1) - shift for PA11 for valve 1

	}
	// valves [9..11] on PC10-PC12
	else if (valveId > 8 && valveId < 12) { // PC10-12 driven outputs for valve 9 to 11. Relay driven
		GPIOC->BSRR |= (1 << ((valveId + 1))); // +1 - shift for PC10 from valveId=9
	}
	valveFlags |= (1 << valveId); // set flag
}

void close_valve_old(uint8_t valveId) {
	if (valveId == 0) {
		VALVE_CTRL_PORT->BSRR |= (1 << 8); // valveId=0 is FWI valve
	} else if (valveId > 0 && valveId < 5) { // PA11-PA14 driven Cadi board MOSFET drivers
		VALVE_CTRL_PORT->BSRR |= (1 << valveId + 10); // 10 - shift for PA11 as valve 1
	} else if (valveId > 4 && valveId < 9) { // PA4-PA7 driven extender 4-ch. MOSFET driver
		VALVE_CTRL_PORT->BSRR |= (1 << ((valveId - 1))); // 10 - shift for PA11 for valve 1

	} else if (valveId > 8 && valveId < 12) { // 9110	 driven valves, with 0 for OFF, and 1 for ON states
		GPIOC->BRR |= (1 << ((valveId + 1)));
	}
	valveFlags &= ~(1 << valveId); // reset flag
}

void EXTI0_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line0);
#ifdef WFM_PINS_PB0_1	// water flow meter (WFM) driver actions
	water_counter[0]++;
#endif
}

void EXTI1_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line1);
#ifdef WFM_PINS_PB0_1
	water_counter[1]++;
#endif

}

void EXTI2_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line2);

}

void EXTI15_10_IRQHandler(void) {

	// Clear the  EXTI line 10-12 pending bit
	EXTI_ClearITPendingBit(EXTI_Line10);
	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_ClearITPendingBit(EXTI_Line12);

	if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line15);
		if ((GPIOB->IDR & (1 << 15)) == 0 && dht_rise > 0) {
			// GPIOB->BSRR = (1<<15);	// debug
			dht_period = TIM15->CNT;
			TIM15->CNT = 0;
			DutyCycle = (dht_rise * 100) / dht_period;
			if (dht_bit_position > DHT1_DATA_START_POINTER
					&& dht_data_ready == 0) {

				if (DutyCycle > 60 && DutyCycle < 75) { // magic constants 56/
					dht_byte_buf[dht_byte_pointer] &= ~(1 << (dht_bit_pointer)); // reset bit in dht_data[i]
				} else {
					dht_byte_buf[dht_byte_pointer] |= (1 << (dht_bit_pointer)); // set bit
					// GPIOB->BRR = (1<<15);	// debug
				}

				if (dht_bit_pointer == 0) {
					dht_bit_pointer = 7;
					dht_byte_pointer++;
					if (dht_byte_pointer == 5) {
						dht_data_ready = 1;
						dht_byte_pointer = 0;
						dht_bit_pointer = 7;
						dht_last_read_id = 1;
					}
				} else {
					dht_bit_pointer--;
				}
			}
			dht_bit_position++;
			dht_rise = 0;
		} else {
			dht_rise = TIM15->CNT;
		}

	}

	if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line14);

		if ((GPIOB->IDR & (1 << 14)) == 0 && dht_rise > 0) {
			// GPIOB->BSRR = (1<<15);	// debug
			dht_period = TIM15->CNT;
			TIM15->CNT = 0;
			DutyCycle = (dht_rise * 100) / dht_period;
			if (dht_bit_position > DHT1_DATA_START_POINTER
					&& dht_data_ready == 0) {

				if (DutyCycle > 60 && DutyCycle < 75) { // magic constants 56/
					dht_byte_buf[dht_byte_pointer] &= ~(1 << (dht_bit_pointer)); // reset bit in dht_data[i]
				} else {
					dht_byte_buf[dht_byte_pointer] |= (1 << (dht_bit_pointer)); // set bit
					// GPIOB->BRR = (1<<15);	// debug
				}

				if (dht_bit_pointer == 0) {
					dht_bit_pointer = 7;
					dht_byte_pointer++;
					if (dht_byte_pointer == 5) {
						dht_data_ready = 1;
						dht_byte_pointer = 0;
						dht_bit_pointer = 7;
						dht_last_read_id = 0;
					}
				} else {
					dht_bit_pointer--;
				}
			}
			dht_bit_position++;
			dht_rise = 0;
		} else {
			dht_rise = TIM15->CNT;
		}

	}
}

/* void EXTI9_5_IRQHandler(void)
 {
 // Clear the  EXTI line 10-12 pending bit
 EXTI_ClearITPendingBit(EXTI_Line5);
 EXTI_ClearITPendingBit(EXTI_Line6);
 EXTI_ClearITPendingBit(EXTI_Line7);
 } */

void eeprom_test(void) {
	uint16_t addr = 1500;
	uint16_t val = 0;
	button = 0;
	while (button != BUTTON_FWD) {
		button = readButtons();
		val = 0;
		EE_ReadVariable(addr, &val);

		vTaskDelay(5);
		Lcd_clear();
		Lcd_write_str("Addr: ");
		Lcd_write_16b(addr);
		Lcd_goto(1, 0);
		Lcd_write_str("Val: ");
		Lcd_write_16b(val);
		if (button == BUTTON_OK) {
			if (addr < 2000) {
				addr++;
			} else {
				addr = 1400;
			}
		}
		if (button == BUTTON_BCK) {
			if (addr > 1400) {
				addr--;
			} else {
				addr = 2000;
			}
		}
		vTaskDelay(25);
	}
}

void valve_test2(void) {
	vTaskDelay(5);
	uint8_t curvalve = 0;
	Lcd_clear();
	button = 0;
	while (button != BUTTON_OK) {
		vTaskDelay(20);
		button = readButtons();
#ifdef USE_VALVES
		Lcd_goto(0, 0);
		Lcd_write_str("curvalve:");
		Lcd_write_digit(curvalve);

		if (button == BUTTON_FWD) {
			//open_valve(curvalve);
			open_valve(curvalve);
		}
		if (button == BUTTON_CNL) {
			//close_valve(curvalve);
			close_valve(curvalve);
		}
		if (button == BUTTON_BCK) {
			if (curvalve < 10) {
				curvalve++;
			} else {
				curvalve = 0;
			}
		}
		vTaskDelay(1);
		Lcd_goto(1, 0);
		Lcd_write_str("VF");
		Lcd_write_16b(valveFlags);

		vTaskDelay(1);

		Lcd_write_str("States:");

		if (VALVE_SENSOR_PORT->IDR & (1 << 4)) {
			Lcd_write_str("1");
		} else {
			Lcd_write_str("0");
		}
		if (VALVE_SENSOR_PORT->IDR & (1 << 5)) {
			Lcd_write_str("1");
		} else {
			Lcd_write_str("0");

			if (VALVE_SENSOR_PORT->IDR & (1 << 6)) {
				Lcd_write_str("1");
			} else {
				Lcd_write_str("0");
			}

			if (VALVE_SENSOR_PORT->IDR & (1 << 8)) {
				Lcd_write_str("1");
			} else {
				Lcd_write_str("0");
			}
		}

#endif
	}
	Lcd_clear();
}

void valve_test3(void) {
	uint8_t curvalve = 2;
	Lcd_clear();

	while (button != BUTTON_OK) {
		vTaskDelay(20);
		button = readButtons();
#ifdef USE_VALVES
		Lcd_goto(0, 0);
		Lcd_write_str("curvalve:");
		Lcd_write_digit(curvalve);
		if (button == BUTTON_FWD) {
			//open_valve(curvalve);
			VALVE_CTRL_PORT->BSRR |= (1 << (11 + curvalve));
		}
		if (button == BUTTON_CNL) {
			//close_valve(curvalve);
			VALVE_CTRL_PORT->BRR |= (1 << (11 + curvalve));
		}
		if (button == BUTTON_BCK) {
			if (curvalve < 4) {
				curvalve++;
			} else {
				curvalve = 2;
			}
		}
		vTaskDelay(1);
		Lcd_goto(1, 0);
		Lcd_write_str("VF");
		Lcd_write_16b(valveFlags);

		vTaskDelay(1);

		Lcd_write_str("States:");

		if (GPIOA->ODR & (1 << 13)) {
			Lcd_write_str("1");
		} else {
			Lcd_write_str("0");
		}
		if (GPIOA->ODR & (1 << 14)) {
			Lcd_write_str("1");
		} else {
			Lcd_write_str("0");
		}

#endif
	}
	Lcd_clear();
}

void valve_test(void) {
	uint16_t tmp = 0;
	Lcd_clear();
	button = 0;
	while (button != BUTTON_OK) {
		vTaskDelay(20);
		button = readButtons();
#ifdef USE_VALVES
		Lcd_goto(0, 0);

		tmp = sonar_read[FWTANK_SONAR];
		Lcd_write_digit(tmp);
		tmp = sonar_read[MIXTANK_SONAR];
		Lcd_write_digit(tmp);
		Lcd_write_str("cm");

		tmp = TIM17->CNT;
		Lcd_write_16b(tmp);

		if (button == BUTTON_FWD) {
			open_valve(1);
		}
		if (button == BUTTON_BCK) {
			close_valve(1);
		}
		vTaskDelay(1);
		Lcd_goto(1, 0);
		Lcd_write_16b(valveFlags);
		Lcd_write_str("VS");
		uint16_t val = 0;
		val = VALVE_SENSOR_PORT->IDR >> 5;
		Lcd_write_16b(val);
		Lcd_write_str(" ");

		vTaskDelay(1);
		if (VALVE_SENSOR_PORT->IDR >> 5 & 1) {
			Lcd_write_str("1");
		} else {
			Lcd_write_str("0");
		}

		if (VALVE_SENSOR_PORT->IDR >> 6 & 1) {
			Lcd_write_str("1");
		} else {
			Lcd_write_str("0");
		}

		if (VALVE_SENSOR_PORT->IDR >> 7 & 1) {
			Lcd_write_str("1");
		} else {
			Lcd_write_str("0");
		}
#endif

		Lcd_write_str("W");

	}
	psi_pwm_test();
	eeprom_test();
	Lcd_clear();
	valve_test2();
	valve_test3();
}

uint16_t adjustFlags(uint16_t flags, uint8_t from, uint8_t to) {
	uint8_t i = 0;
	uint8_t cursor = 0;
	uint8_t var = 0;
	var = (flags >> from);
	Lcd_goto(0, 0);
	Lcd_write_str("_");
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		Lcd_goto(1, 0);
		for (i = 0; i < (to - from); i++) {
			if ((var >> i) & 1 == 0) {
				Lcd_write_str("0");
			} else {
				Lcd_write_str("1");
			}
		}
		vTaskDelay(1);
		for (i = 0; i < (to - from); i++) {
			Lcd_goto(0, 0);
			Lcd_write_str(" ");
		}
		Lcd_goto(0, cursor);
		Lcd_write_str("_");
		vTaskDelay(5);
		if (button == BUTTON_FWD && cursor < (to - from)) {
			cursor++;
		}
		if (button == BUTTON_BCK && cursor > 0) {
			cursor--;
		}
		if (button == BUTTON_CNL) {
			// invert current cursor positioned bit
			var ^= (1 << cursor);
		}
	}
	// put temporary var back into flags
	for (i = 0; i < (to - from); i++) {
		if ((var >> i) & 1 == 1) {
			flags |= (1 < i + from); // set bit
		} else {
			flags &= ~(1 << (i + from)); // reset bit
		}
	}
	printOk();
	return flags;
}

void water_program_setup(uint8_t progId) {
	uint16_t addr = 0, volume = 0, flags = 0;
	uint32_t time = 0;

	// setting water volume
	Lcd_clear();
	Lcd_write_str("Volume"); // in sonar distance units
	vTaskDelay(200);
	addr = WP_OFFSET + progId * WP_SIZE + WP_VOLUME;
	EE_ReadVariable(addr, &volume);
	volume = (uint16_t) adjust8bit(volume);
	EE_WriteVariable(addr, volume);

	// setting watering lines valves to open
	Lcd_clear();
	Lcd_goto(0, 5);
	Lcd_write_str("ON VALVES"); // in sonar distance units
	vTaskDelay(200);
	addr = WP_OFFSET + progId * WP_SIZE + WP_FLAGS;
	EE_ReadVariable(addr, &flags);
	flags = adjustFlags(flags, 0, 3);
	uint8_t flag = 0;
	while (button == 0) {
		button = readButtons();
		if (button == BUTTON_FWD) {
			flag = 1 - flag;
		}
		vTaskDelay(25);
	}
	if (flag == 1) {
		flags |= 1; // set bit
	} else {
		flags &= ~1; // reset bit
	}
	EE_WriteVariable(addr, flags);
	vTaskDelay(5);
	printOk();

	// adjust start time
	uint32_t ts = 0; // timestamp unixtime
	addr = WP_OFFSET + progId * WP_SIZE + WP_START;
	ts = EE_ReadWord(addr);
	ts = timeAdjust(ts, 1);
	EE_WriteVariable(addr, ts);

	// adjust end time
	addr = WP_OFFSET + progId * WP_SIZE + WP_END;
	ts = EE_ReadWord(addr);
	ts = timeAdjust(ts, 1);
	EE_WriteVariable(addr, ts);

	// adjust interval
	addr = WP_OFFSET + progId * WP_SIZE + WP_INTERVAL;
	ts = EE_ReadWord(addr);
	ts = CTimerAdjust(ts);
	EE_WriteWord(addr, ts);

}

void fertilization_setup(void) {
	uint8_t progId = 0;
	Lcd_clear();
	Lcd_write_str("Program nr");
	vTaskDelay(200);
	progId = adjust8bit(progId);
	fertilizer_mixing_program_setup(progId);
}

void watering_setup(void) {
	uint8_t progId = 0;
	Lcd_clear();
	Lcd_write_str("Program nr");
	vTaskDelay(200);
	progId = adjust8bit(progId);
	water_program_setup(progId);
}

// set fertilizer mixing programs
void fertilizer_mixing_program_setup(uint8_t progId) {
	uint16_t addr = 0, tempvalue = 0;

	// choose dosing pump to use
	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_DOSING_PUMP_ID_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Fertilizer 2 use");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set dosing time, seconds
	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_DOSING_TIME_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Dosing duration");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set circulation pump mixing time
//	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_CIRCULATION_MIXING_TIME_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Aftermix time"); // setting the time for running circulation pump to mix the fertilizer into solution
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set watering program to associate this fertilizer mix program with
	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_2_WP_ASSOC_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Link to WP");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set frequency (N), showing how often this fertilizer mixing program activates (every N watering program run)
	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_TRIG_FREQUENCY_SHIFT;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("Trig. freq. N");
	tempvalue = adjust16bit(tempvalue);
	EE_WriteVariable(addr, tempvalue);

	// set ENABLED/DISABLED status for this program
//	addr = FMP_OFFSET+progId*FMP_SIZE+FMP_ENABLE;
	EE_ReadVariable(addr, &tempvalue);
	Lcd_clear();
	Lcd_write_str("ENABLE FMP?");
	tempvalue = adjust16bit(tempvalue);
	vTaskDelay(10);
	EE_WriteVariable(addr, tempvalue);
	Lcd_clear();
	vTaskDelay(10);
	loadSettings();
}

void startWp(void) {
	uint8_t progId = 0, button = 0;
	uint32_t addr, tmp32, interval;
	Lcd_clear();
	Lcd_write_str("Program nr");
	vTaskDelay(200);
	progId = adjust8bit(progId);
	Lcd_clear();
	Lcd_write_str("Start WP#");
	Lcd_write_digit(progId);
	Lcd_write_str("?");
	button = 0;
	while (button != BUTTON_OK && button != BUTTON_CNL) {
//		dht_get_data();
		vTaskDelay(30);
		button = readButtons();
	}
	vTaskDelay(200);
	if (button == BUTTON_OK) {
		addr = WP_OFFSET + progId * WP_SIZE + WP_INTERVAL;
		interval = EE_ReadWord(addr);
		// set initial time point of watering program
		addr = WP_OFFSET + progId * WP_SIZE + WP_START;
		tmp32 = RTC_GetCounter();
		EE_WriteWord(addr, tmp32);

		addr = WP_OFFSET + progId * WP_SIZE + WP_LAST_RUN_SHIFT;
		EE_WriteWord(addr, (RTC_GetCounter() - interval + 5));
		run_watering_program_g2(progId);
	}
	Lcd_clear();
	vTaskDelay(200);
}

/* Grolly 1
 * This Watering Program has following strategy:
 * - the mixing tank is always empty before watering program starts
 * - the watering solution volume is fixed for each watering
 * - water intaken by opening FWI valve (for Grolly ID 4)
 * - MIXTANK_SONAR is used for measuring distance to water edge,
 *  from the top of the tank where sonar installed
 *
 */

/*

 void run_watering_program(uint8_t progId){
 enableClock=0;
 wpProgress = 2;
 wpStateFlags|=(1<<progId);	// set active flag for this program
 uint32_t wpStartTs = 0;
 wpStartTs = RTC_GetCounter();
 // FRESH WATER INTAKE
 wpProgress = 3;
 volatile uint16_t n=0;
 uint8_t i=0;
 uint32_t curN=0;
 uint32_t interval=0;
 uint32_t startime=0;
 uint32_t now=0;
 uint32_t overTime=4000000000;
 uint16_t fwl=0;
 uint16_t swl = 0;
 uint16_t curprcnt = 0;
 uint16_t vlm = 0;
 uint16_t addr=0;
 uint16_t fmpLink=0;
 uint16_t flags=0;
 uint16_t curLvl = 0;
 uint8_t got_fw = 0;
 uint8_t ninetenth=0;
 now = RTC_GetCounter();
 startime = now;
 wpProgress = 4;
 vTaskDelay(100);
 //
 //  Filling water strategy A
 *   The water is filled up in amount of 'volume',
 *    when there is less than 100% of watering program volume currently
 *    in the MIXTANK. This behavior expects already prepared solution in
 *    the MIXTANK, suitable for this (any, that could run) watering
 *    program
 *    swl - start water level
 *    fwl - final water level
 *
 *    !!! REMEMBER !!!
 *    More sonar read value, less water in the tank!
 *
 swl = sonar_read[MIXTANK_SONAR];			// remember start water level
 addr = WP_OFFSET+progId*WP_SIZE+WP_VOLUME;	// read the volume needed to add in MIXTANK
 EE_ReadVariable(addr, &vlm);
 vlm &= (uint16_t)0xFF;
 vTaskDelay(100);
 // 	volume &= (uint16_t)0x00FF;	// lower byte - WP volume, higher - WP rules
 fwl = tank_windows_bottom[MIXTANK] - sonar_read[MIXTANK_SONAR];	// how much water we have already
 Lcd_clear();
 // 14, 52, 66, 14
 Lcd_write_str("NW");		// 25
 Lcd_write_digit(fwl/100);
 Lcd_write_digit(fwl);
 Lcd_write_str("CL");		// 41
 Lcd_write_16b(sonar_read[MIXTANK_SONAR]);
 Lcd_goto(1,0);
 Lcd_write_str("BT");		// 66
 Lcd_write_16b(tank_windows_bottom[MIXTANK]);
 Lcd_write_str("VL");		// 10
 Lcd_write_16b(vlm);
 vTaskDelay(8000);

 volatile static uint8_t left = 0;							// indicates how much water let to fill
 if (fwl < (vlm/3)) {							// if current water level < than 1/3 of WP volume
 fwl = swl - vlm;						// get Final Water Level: now+WPvol
 Lcd_clear();
 Lcd_write_str("FW");
 Lcd_write_16b(fwl);
 Lcd_goto(1,0);
 Lcd_write_str("SW");
 Lcd_write_16b(swl);
 Lcd_write_str("VL");
 Lcd_write_16b(vlm);
 vTaskDelay(10000);
 open_valve(FWI_VALVE);	// FWI valve
 Lcd_clear();
 while (overTime > now) { // 5 seconds sonar should report >100% fill to close FWI valve
 left = (uint8_t)((sonar_read[MIXTANK_SONAR] - fwl)&(0xFF));
 if ( sonar_read[MIXTANK_SONAR] >= fwl){	// more sonar read - less water
 overTime = RTC_GetCounter() + 5;
 open_valve(FWI_VALVE);
 }
 Lcd_goto(0,0);
 Lcd_write_str("Left=");
 Lcd_write_digit(left/100);
 Lcd_write_digit(left);
 vTaskDelay(10);
 Lcd_goto(1,0);
 Lcd_write_str("Lvl=");
 Lcd_write_16b(sonar_read[MIXTANK_SONAR]);
 vTaskDelay(200);
 now = RTC_GetCounter();
 wpProgress = 5;
 }
 close_valve(FWI_VALVE);
 vTaskDelay(1000);
 wpProgress = 6;

 // mix fertilizers
 got_fw = (uint8_t)swl - (uint8_t)sonar_read[MIXTANK_SONAR];
 ninetenth = (uint8_t)((vlm*9)/10);

 for (i=0; i<FMP_PROGRAMS_AMOUNT; i++) {
 vTaskDelay(10);
 wpProgress = i+87;
 addr=0;
 // count current N value
 addr = WP_OFFSET+progId*WP_SIZE+WP_INTERVAL;
 interval = EE_ReadWord(addr);
 addr = WP_OFFSET+progId*WP_SIZE+WP_START;
 startime = EE_ReadWord(addr);
 vTaskDelay(100);
 addr = FMP_OFFSET+i*FMP_SIZE+FMP_TRIG_FREQUENCY_SHIFT;
 EE_ReadVariable(addr, &n);
 uint16_t enabled=0;
 uint8_t rest = 0;
 vTaskDelay(100);
 if (n>0) {
 curN = (RTC_GetCounter()-startime)/interval;
 vTaskDelay(100);
 wpProgress = 200;
 vTaskDelay(200);
 rest = curN%(n%256);	// lower byte of 16bit of FMP_TRIG_FREQUENCY_SHIFT
 enabled=1;
 }
 else {
 enabled=0;
 }
 vTaskDelay(50);
 fmpLink=n/256;	// higher byte of FMP_TRIG_FREQUENCY = WP link
 addr = 0;
 // addr = FMP_OFFSET+i*FMP_SIZE+FMP_DOSING_PUMP_ID_SHIFT;
 // EE_ReadVariable(addr, &enabled);	//
 if (fmpLink==progId && enabled>0) {
 wpProgress = 99;
 vTaskDelay(400);
 run_fertilizer_mixer_g2(i);
 wpProgress = 100+i;
 vTaskDelay(2000);
 }
 vTaskDelay(100);
 wpProgress = 8;
 }
 }
 vTaskDelay(50);
 wpProgress = 9;
 // open corresponding watering line valve(s)
 addr = WP_OFFSET+progId*WP_SIZE+WP_FLAGS;
 EE_ReadVariable(addr, &flags);
 for (i=1; i<=VALVE_AMOUNT; i++) {
 if (((flags>>i)&1)==1) {
 open_valve(i);
 }
 else {
 close_valve(i);
 }
 vTaskDelay(100);
 }
 wpProgress = 10;
 uint8_t srcomm = 0;
 uint16_t tmpval=0;
 srcomm = comm_state;	// backup curent commstate. to recover after watering
 comm_state = COMM_MONITOR_MODE;
 vTaskDelay(50);
 addr = WP_OFFSET+progId*WP_SIZE+WP_TIMEOUT;
 vlm=0;
 EE_ReadVariable(addr, &vlm);
 overTime = RTC_GetCounter();
 overTime += 0x0000FFFF&((uint32_t)vlm);	// volume used to retrieve timeout
 auto_flags |= (1<<2);// set overpressure autosafe flag
 auto_failures  &= ~1;	// reset PSI main failure flag
 auto_failures  &= ~(1<<3);	// reset PSI underpressure flag
 now = 0;
 // run watering
 auto_flags|=9;	// enable watering through psi stab function with underpressure trig
 vTaskDelay(30);
 while (((auto_failures&8)>>3)==0 && now<overTime) {
 Lcd_goto(0,0);
 Lcd_write_str("Left: ");
 Lcd_write_digit(wpProgress);
 vTaskDelay(500);
 psiStab();
 wpProgress = (uint8_t)(overTime-now);
 now=RTC_GetCounter();
 vTaskDelay(50);
 }
 auto_flags&=~(1);	// disable PSI stab function flag
 psiOff();
 //	if ((auto_flags&1)==1 && ((auto_failures&8)>>3)==0 && (auto_failures&1)==0 && comm_state==COMM_MONITOR_MODE) {			// PSI STAB flag is number 0

 wpProgress = 12;
 vTaskDelay(20000);	// release pressure in watering line
 comm_state = srcomm;	// recover comm_state
 auto_failures  &= ~(1<<3);	// reset PSI failure flag
 wpProgress = 13;
 vTaskDelay(1000);
 valve_init();	// close all valves

 addr = WP_OFFSET+progId*WP_SIZE+WP_LAST_RUN_SHIFT;
 EE_WriteWord(addr, wpStartTs);	// write last run time

 wpStateFlags &= ~(1<<progId); // sbrosit' flag
 enableClock=1;
 vTaskDelay(200);
 }

 */


void wt_add_ferts_for_wp(uint8_t progId){
	uint8_t i=0;
	uint16_t addr = 0;
	uint32_t interval = 0;
	uint32_t startime = 0;
	uint16_t enabled = 0;
	uint8_t rest = 0;
	uint32_t curN = 0;
	uint16_t n = 0;
	uint16_t fmpLink = 0;

	 for (i = 0; i < FMP_PROGRAMS_AMOUNT; i++) {
	 			vTaskDelay(10);
	 			wpProgress = i + 87;
	 			addr = 0;
	 			// count current N value
	 			addr = WP_OFFSET + progId * WP_SIZE + WP_INTERVAL;
	 			interval = EE_ReadWord(addr);
	 			addr = WP_OFFSET + progId * WP_SIZE + WP_START;
	 			startime = EE_ReadWord(addr);
	 			vTaskDelay(100);
	 			addr = FMP_OFFSET + i * FMP_SIZE + FMP_TRIG_FREQUENCY_SHIFT;
	 			EE_ReadVariable(addr, &n);
	 			vTaskDelay(100);
	 			enabled = 0;
	 			rest = 0;
	 			if (n > 0) {
	 				curN = (RTC_GetCounter() - startime) / interval;
	 				vTaskDelay(100);
	 				wpProgress = 200;
	 				vTaskDelay(200);
	 				rest = curN % (n % 256); // lower byte of 16bit of FMP_TRIG_FREQUENCY_SHIFT
	 				enabled = 1;
	 			} else {
	 				enabled = 0;
	 			}
	 			vTaskDelay(50);
	 			fmpLink = n / 256; // higher byte of FMP_TRIG_FREQUENCY = WP link
	 			addr = 0;
	 			// addr = FMP_OFFSET+i*FMP_SIZE+FMP_DOSING_PUMP_ID_SHIFT;
	 			// EE_ReadVariable(addr, &enabled);	//
	 			if (fmpLink == progId && enabled > 0) {
	 				wpProgress = 99;
	 				vTaskDelay(400);
	 				run_fertilizer_mixer_g2(i);
	 				wpProgress = 100 + i;
	 				vTaskDelay(2000);
	 			}
	 			vTaskDelay(100);
	 			wpProgress = 8;
	 		}
}

#define WT_MIN_LVL_DIV	3		// divider for watering amount, to detect minimum level needed

void run_watering_program_g2_new(uint8_t progId){
	uint16_t addr = 0;
	open_valve(MTI_VALVE);	// test for click sound
	vTaskDelay(100);
	close_valve(MTI_VALVE);


	uint16_t minimum_level = 0;
	uint16_t amount = 0;
	// First, check if we have enough water in the MixTank

	// Get the minimum water level, below which Grolly mixes up the new solution
	minimum_level = tank_windows_bottom[MIXTANK] - (amount / WT_MIN_LVL_DIV);
	if (sonar_read[MIXTANK_SONAR] > minimum_level) {	// make a new fert. solution for watering
		wt_mt_add_water(amount,0);
	}

	wt_add_ferts_for_wp(progId);

	// start watering then

	uint32_t wpStartTs = 0;
	wpStartTs = RTC_GetCounter();
	wt_watering(10, 9);	// run watering for 10 seconds on valve id 9
	addr = WP_OFFSET + progId * WP_SIZE + WP_LAST_RUN_SHIFT;
	EE_WriteWord(addr, wpStartTs); // write last run time
}


/*void open_valves(uint16_t valveFlags){
	uint8_t i = 0;
	uint8_t flag = 0;
	for (i=0; i<16; i++) {
		flag = 0;
		flag = (valveFlags >> i) & 1;
		if (flag == 1) {
			open_valve(i);
		}
		else {
			close_valve(i);
		}

	}

} */


void run_watering_program_g2(uint8_t progId) {




	enableClock = 0;
	wpProgress = 2;
	wpStateFlags |= (1 << progId); // set active flag for this program
	uint32_t wpStartTs = 0;
	wpStartTs = RTC_GetCounter();
	// FRESH WATER INTAKE
	wpProgress = 3;
	volatile uint16_t n = 0;
	uint8_t i = 0;
	uint32_t curN = 0;
	uint32_t interval = 0;
	uint32_t startime = 0;
	uint32_t now = 0;
	uint32_t overTime = 4000000000;
	uint16_t fwl = 0;
	uint16_t swl = 0;
	uint16_t curprcnt = 0;
	uint16_t vlm = 0;
	uint16_t addr = 0;
	uint16_t fmpLink = 0;
	uint16_t flags = 0;
	uint16_t curLvl = 0;
	uint8_t got_fw = 0;
	uint8_t ninetenth = 0;
	now = RTC_GetCounter();
	startime = now;
	wpProgress = 4;
	vTaskDelay(100);
	/*

	 *  Filling water strategy A
	 *   The water is filled up in amount of 'volume',
	 *    when there is less than 100% of watering program volume currently
	 *    in the MIXTANK. This behavior expects already prepared solution in
	 *    the MIXTANK, suitable for this (any, that could run) watering
	 *    program
	 *    swl - start water level
	 *    fwl - final water level
	 *
	 *    !!! REMEMBER !!!
	 *    More sonar read value, less water in the tank!

	 */
	swl = sonar_read[MIXTANK_SONAR]; // remember start water level
	addr = WP_OFFSET + progId * WP_SIZE + WP_VOLUME; // read the volume needed to add in MIXTANK
	EE_ReadVariable(addr, &vlm);
	vlm &= (uint16_t) 0xFF;
	vTaskDelay(100);
// 	volume &= (uint16_t)0x00FF;	// lower byte - WP volume, higher - WP rules
	fwl = tank_windows_bottom[MIXTANK] - sonar_read[MIXTANK_SONAR]; // how much water we have already
	Lcd_clear();
	// 14, 52, 66, 14
	Lcd_write_str("NW"); // 25
	Lcd_write_digit(fwl / 100);
	Lcd_write_digit(fwl);
	Lcd_write_str("CL"); // 41
	Lcd_write_16b(sonar_read[MIXTANK_SONAR]);
	Lcd_goto(1, 0);
	Lcd_write_str("BT"); // 66
	Lcd_write_16b(tank_windows_bottom[MIXTANK]);
	Lcd_write_str("VL"); // 10
	Lcd_write_16b(vlm);
	vTaskDelay(8000);
	close_valves(); // close all valves before doing anything
	volatile static uint8_t left = 0; // indicates how much water let to fill
	if (fwl < (vlm / 3)) { // if current water level < than 1/3 of WP volume
		fwl = swl - vlm; // get Final Water Level: now+WPvol
		wt_mt_reach_level(fwl,0);	// reach final water level
		vTaskDelay(200);
		IWDG_ReloadCounter();
		wpProgress = 6;

		// mix fertilizers
		got_fw = (uint8_t) swl - (uint8_t) sonar_read[MIXTANK_SONAR];
		ninetenth = (uint8_t)((vlm * 9) / 10);

		for (i = 0; i < FMP_PROGRAMS_AMOUNT; i++) {
			vTaskDelay(10);
			wpProgress = i + 87;
			addr = 0;
			// count current N value
			addr = WP_OFFSET + progId * WP_SIZE + WP_INTERVAL;
			interval = EE_ReadWord(addr);
			addr = WP_OFFSET + progId * WP_SIZE + WP_START;
			startime = EE_ReadWord(addr);
			vTaskDelay(100);
			addr = FMP_OFFSET + i * FMP_SIZE + FMP_TRIG_FREQUENCY_SHIFT;
			EE_ReadVariable(addr, &n);
			uint16_t enabled = 0;
			uint8_t rest = 0;
			vTaskDelay(100);
			if (n > 0) {
				curN = (RTC_GetCounter() - startime) / interval;
				vTaskDelay(100);
				wpProgress = 200;
				vTaskDelay(200);
				rest = curN % (n % 256); // lower byte of 16bit of FMP_TRIG_FREQUENCY_SHIFT
				enabled = 1;
			} else {
				enabled = 0;
			}
			vTaskDelay(50);
			fmpLink = n / 256; // higher byte of FMP_TRIG_FREQUENCY = WP link
			addr = 0;
			// addr = FMP_OFFSET+i*FMP_SIZE+FMP_DOSING_PUMP_ID_SHIFT;
			// EE_ReadVariable(addr, &enabled);	//
			if (fmpLink == progId && enabled > 0) {
				wpProgress = 99;
				vTaskDelay(400);
				run_fertilizer_mixer_g2(i);
				wpProgress = 100 + i;
				vTaskDelay(2000);
			}
			vTaskDelay(100);
			wpProgress = 8;
		}
	}
	vTaskDelay(50);
	wpProgress = 9;
	// open corresponding watering line valve(s)
	addr = WP_OFFSET + progId * WP_SIZE + WP_FLAGS;
	EE_ReadVariable(addr, &flags);
	close_valves(); // close all valves
	open_valve(MTI_VALVE); // MixTank intake line valve open
	open_valves(flags);

/*	for (i = 1; i <= VALVE_AMOUNT; i++) { // open output lines
		if (((flags >> i) & 1) == 1) {
			open_valve(i);
		} else {
			close_valve(i);
		}
		vTaskDelay(100);
	} */
	wpProgress = 10; // test
	uint8_t srcomm = 0;
	uint16_t tmpval = 0;
	srcomm = comm_state; // backup curent commstate. to recover after watering
	comm_state = COMM_MONITOR_MODE;
	vTaskDelay(50);
	addr = WP_OFFSET + progId * WP_SIZE + WP_TIMEOUT;
	vlm = 0;
	EE_ReadVariable(addr, &vlm);

	wt_watering_x(vlm,flags);

/*	overTime = RTC_GetCounter();
	overTime += 0x0000FFFF & ((uint32_t) vlm); // volume used to retrieve timeout
	auto_flags |= (1 << 2); // set overpressure autosafe flag  !!!!! (psiOn())
	auto_failures &= ~1; // reset PSI main failure flag	!!!!! (psiOn())
	auto_failures &= ~(1 << 3); // reset PSI underpressure flag	!!!!! (psiOn())



	open_valve(MTI_VALVE);	// water intake line open

	now = 0;
	// run watering
	auto_flags |= 9; // enable watering through psi stab function with underpressure trig !!!!! (psiOn())
	vTaskDelay(30);
	psiOn();
	vTaskDelay(30);
	while (((auto_failures & 8) >> 3) == 0 && now < overTime) {
		IWDG_ReloadCounter();
		vTaskDelay(500);
		psiOn();
		wpProgress = (uint8_t)(overTime - now);
		now = RTC_GetCounter();
		vTaskDelay(50);
	}
	psiOff();

	*/

	addr = WP_OFFSET + progId * WP_SIZE + WP_LAST_RUN_SHIFT;
	EE_WriteWord(addr, wpStartTs); // write last run time

	wpProgress = 12;
	vTaskDelay(20000); // release pressure in watering line
	comm_state = srcomm; // recover comm_state
	auto_failures &= ~(1 << 3); // reset PSI failure flag
	wpProgress = 13;
	vTaskDelay(1000);
	close_valves(); // close all valves



	wpStateFlags &= ~(1 << progId); // sbrosit' flag
	enableClock = 1;
	vTaskDelay(200);
}

void run_fertilizer_mixer_g2(uint8_t progId) {

	close_valves();

	vTaskDelay(50);

	uint16_t dosingTime, dosingPumpId, circulationMixingTime, addr;
	uint32_t dosingEndTime = 0;

	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_DOSING_TIME_SHIFT;
	EE_ReadVariable(addr, &dosingTime);
	vTaskDelay(50);
	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_DOSING_PUMP_ID_SHIFT;
	EE_ReadVariable(addr, &dosingPumpId);
	wpProgress = 95;
	vTaskDelay(40);
	IWDG_ReloadCounter();
	addr = 0;
	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_AFTERMIX_TIME_SHIFT;
	wpProgress = 94;
	vTaskDelay(40);
	EE_ReadVariable(addr, &circulationMixingTime);
	wpProgress = 93;
	vTaskDelay(40);
	IWDG_ReloadCounter();
//	dosingEndTime = RTC_GetCounter() + (uint32_t) dosingTime;

	// void wt_mix_in(uint16_t duration, uint16_t vol, uint8_t doserid, uint8_t spd)
	wt_mix_in(circulationMixingTime,dosingTime,dosingPumpId,1);

	vTaskDelay(400);

}

void run_fertilizer_mixer(uint8_t progId) {

	// start mixing pump
//	enable_dosing_pump(MIXING_PUMP,1);	// Grolly has Mixing pump connected to T8 MOSFET of Cadi MB 1402
	close_valve(FWI_VALVE);

	vTaskDelay(50);

	uint16_t dosingTime, dosingPumpId, circulationMixingTime, addr;
	uint32_t dosingEndTime = 0;

	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_DOSING_TIME_SHIFT;
	EE_ReadVariable(addr, &dosingTime);
	vTaskDelay(50);
	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_DOSING_PUMP_ID_SHIFT;
	EE_ReadVariable(addr, &dosingPumpId);
	wpProgress = 95;
	vTaskDelay(400);
	addr = 0;
	addr = FMP_OFFSET + progId * FMP_SIZE + FMP_AFTERMIX_TIME_SHIFT;
	wpProgress = 94;
	vTaskDelay(400);
	EE_ReadVariable(addr, &circulationMixingTime);
	wpProgress = 93;
	vTaskDelay(400);
	dosingEndTime = RTC_GetCounter() + (uint32_t) dosingTime;
	wpProgress = 92;
	vTaskDelay(400);
	enable_dosing_pump(dosingPumpId, 101); // OPASNO!!! dosingPumpId - 16-bit, while function requires 8-bit
	wpProgress = 91;
	vTaskDelay(400);
	while (RTC_GetCounter() < dosingEndTime) {
		wpProgress = 91;
		vTaskDelay(100);
	}
	wpProgress = 90;
	vTaskDelay(400);
	enable_dosing_pump(dosingPumpId, 0);
	wpProgress = 89;
	vTaskDelay(400);

	run_circulation_pump(circulationMixingTime);
	vTaskDelay(50);
}

void run_circulation_pump(uint16_t time) {
	uint32_t endTime = 0;
	endTime = RTC_GetCounter() + time;
//	enable_dosing_pump(MIXING_PUMP,1);	// Grolly has Mixing pump connected to T8 MOSFET of Cadi MB 1402
	close_valve(FWI_VALVE);
	while (RTC_GetCounter() < endTime) {
		vTaskDelay(10);
//		enable_dosing_pump(MIXING_PUMP,1);
	}
//	enable_dosing_pump(MIXING_PUMP,0);
}

void enable_dosing_pump2(uint8_t pumpId, uint8_t state) {
	if (state == 1) {
		dosingPumpStateFlags |= (1 << pumpId);
	} else {
		dosingPumpStateFlags &= ~(1 << pumpId);
	}

	state = 1 - state; // 0% pwm means running motor

	if (pumpId == 0) {
		TIM3->CCR1 = state * 1000;
	}
	if (pumpId == 1) {
		TIM3->CCR2 = state * 1000;
	}
	if (pumpId == 2) {
		TIM3->CCR3 = state * 1000;
	}
	if (pumpId == 3) {
		TIM3->CCR4 = state * 1000;
	}
}

void enable_dosing_pump(uint8_t pumpId, uint8_t state) {
	uint16_t prcnt = 0;
	if (state > 0) {
		dosingPumpStateFlags2 |= (1 << pumpId);
	} else {
		dosingPumpStateFlags2 &= ~(1 << pumpId);
	}

	if (state == 1) {
		state = 0;
	} else if (state == 101) { // run doser respecting eeprom programmed speeds
		EE_ReadVariable(DOSER_SPEEDS, &prcnt);
		vTaskDelay(2);
		state = (uint8_t)(prcnt & 0xFF);
	} else {
		state = 100 - state; // 0 - running motor, 100 - stopped
	}

	if (pumpId == 0) {
		TIM3->CCR1 = state * 10;
	}
	if (pumpId == 1) {
		TIM3->CCR2 = state * 10;
	}
	if (pumpId == 2) {
		TIM3->CCR3 = state * 10;
	}
	if (pumpId == 3) {
		TIM3->CCR4 = state * 10;
	}
}

void get_fertilizer(uint8_t fertId, uint8_t secs) { // secs - number of seconds to run the fertId pump
	uint32_t reach = 0;
	// to make counting more precise wait until the next second STARTS
	reach = RTC_GetCounter() + 1;
	while (RTC_GetCounter() < reach) {
		vTaskDelay(2);
	}
	reach = RTC_GetCounter() + secs;
	enable_dosing_pump(fertId, 1); // enable dosing pump
	while (RTC_GetCounter() < reach) { // now dose the fertilizer withi 'secs' seconds
		vTaskDelay(2); //
	}
	enable_dosing_pump(fertId, 0);
}

void valve_init(void) {
	close_valve(3);
	close_valve(4);
	close_valve(0);
	close_valve(1);
	close_valve(2);
}

void watering_program_trigger(void *pvParameters) {
	uint32_t curtime = 0;
	uint32_t lastRun = 0;
	uint32_t interval = 0;
	uint32_t diff = 0;
	uint32_t startTime = 0;
	uint32_t endTime = 0;
	uint16_t addr = 0;
	uint16_t temp = 0;
	uint8_t wpStateFlag = 0;
	uint8_t progId = 0;
	uint8_t enabled = 0;
	uint8_t rules = 0;
	uint16_t val16 = 0;
	// close all valves
	close_valves();
	vTaskDelay(20000);
	while (1) {

		// main loop
		for (progId = 0; progId < WP_AMOUNT; progId++) {
			vTaskDelay(10);
			addr = WP_OFFSET + progId * WP_SIZE + WP_START;
			startTime = EE_ReadWord(addr); // program start time point
			addr = WP_OFFSET + progId * WP_SIZE + WP_END;
			endTime = EE_ReadWord(addr); // program end time point
			vTaskDelay(50);
			addr = WP_OFFSET + progId * WP_SIZE + WP_RULES_APPLIED; // read timer rules
			EE_ReadVariable(addr, &temp);
			rules = (uint8_t)((temp >> 8) & 0xFF); // get timer rules applied to WP
			vTaskDelay(50);
			enabled = rules & ((uint8_t)(timerStateFlags & 0xFF));
			if (enabled > 0) { // if any of the timer rules OK, enable WP
				rules = 1;
			} else {
				rules = 0;
			}
			vTaskDelay(20);
			enabled = 0;
			addr = WP_OFFSET + progId * WP_SIZE + WP_FLAGS;
			temp = 0;
			EE_ReadVariable(addr, &temp); // watering program flags
			vTaskDelay(500);
			val16 = temp & 0xFFFF;

			addr = WP_OFFSET + progId * WP_SIZE + WP_LAST_RUN_SHIFT;
			lastRun = EE_ReadWord(addr);
			addr = WP_OFFSET + progId * WP_SIZE + WP_INTERVAL;
			interval = EE_ReadWord(addr);
			vTaskDelay(10);
			if (lastRun < RTC_GetCounter()) {
				diff = RTC_GetCounter() - lastRun;
			} else {
				diff = 0; // if lastRun is in future, prevent program running
			}
			curtime = RTC_GetCounter();
			if ((diff > interval) && (curtime > startTime) && (curtime < endTime)
					&& (val16 > 0) &&	(rules == 1) && wtprog == 0) {
				run_watering_program_g2(progId);
				// wpProgress++;
			}
			vTaskDelay(50);
			if (pending_wt > 100) { // pending watering task id is being mapped to watering program id
				wtprog = 1;
				run_watering_task(pending_wt);
				wtprog = 0;
			}
			vTaskDelay(50);
		}
	}

}

void effect11on(uint32_t delay) {
	vTaskSuspendAll();
	open_valve(WLINE_61_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_62_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_63_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_64_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_65_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_66_VALVE);
	IWDG_ReloadCounter();
	xTaskResumeAll();
}

void effect11off(uint32_t delay) {
	vTaskSuspendAll();
	close_valve(WLINE_61_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_62_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_63_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_64_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_65_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_66_VALVE);
	IWDG_ReloadCounter();
	xTaskResumeAll();
}

void effect12on(uint32_t delay) {
	vTaskSuspendAll();
	open_valve(WLINE_66_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_65_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_64_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_63_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_62_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_61_VALVE);
	IWDG_ReloadCounter();
	xTaskResumeAll();
}

void effect12off(uint32_t delay) {
	vTaskSuspendAll();
	close_valve(WLINE_66_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_65_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_64_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_63_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_62_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_61_VALVE);
	IWDG_ReloadCounter();
	xTaskResumeAll();
}

void effect21on(uint32_t delay) {
	vTaskSuspendAll();
	open_valve(WLINE_63_VALVE);
	open_valve(WLINE_64_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_62_VALVE);
	open_valve(WLINE_65_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_61_VALVE);
	open_valve(WLINE_66_VALVE);
	IWDG_ReloadCounter();
	xTaskResumeAll();
}

void effect21off(uint32_t delay) {
	vTaskSuspendAll();
	close_valve(WLINE_63_VALVE);
	close_valve(WLINE_64_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_62_VALVE);
	close_valve(WLINE_65_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_61_VALVE);
	close_valve(WLINE_66_VALVE);
	IWDG_ReloadCounter();
	xTaskResumeAll();
}

void effect_running_light(uint32_t delay) {
	vTaskSuspendAll();
	open_valve(WLINE_61_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_62_VALVE);
	close_valve(WLINE_61_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_63_VALVE);
	close_valve(WLINE_62_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_64_VALVE);
	close_valve(WLINE_63_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_65_VALVE);
	close_valve(WLINE_64_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_66_VALVE);
	close_valve(WLINE_65_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_66_VALVE);
	IWDG_ReloadCounter();
	xTaskResumeAll();
}

void effect_running_light_rev(uint32_t delay) {
	vTaskSuspendAll();
	IWDG_ReloadCounter();
	open_valve(WLINE_66_VALVE);
	Delay_us(delay);
	open_valve(WLINE_65_VALVE);
	close_valve(WLINE_66_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_64_VALVE);
	close_valve(WLINE_65_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_63_VALVE);
	close_valve(WLINE_64_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_62_VALVE);
	close_valve(WLINE_63_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	open_valve(WLINE_61_VALVE);
	close_valve(WLINE_62_VALVE);
	IWDG_ReloadCounter();
	Delay_us(delay);
	close_valve(WLINE_61_VALVE);
	IWDG_ReloadCounter();
	xTaskResumeAll();
}

void trig_valve(uint8_t valveId){
	uint8_t x = 0;
	open_valve(valveId);
	for (x=0;x<50;x++){
		IWDG_ReloadCounter();
		vTaskDelay(10);
	}
	close_valve(valveId);
}

void run_demo2(void) {
	uint8_t x = 0;
	for (x=0;x<15;x++) {
		trig_valve(x);
	}
	beep_overload(100);
	uint8_t i = 0;
	effect11on(1000);
	effect11off(1000);
	vTaskDelay(500);
	effect12on(1000);
	effect12off(1000);
	vTaskDelay(500);
	close_valves();
	for (i = 0; i < 5; i++) {
		effect_running_light(i * 500);
		effect_running_light_rev(i * 500);
	}
	vTaskDelay(100);
	effect21on(1000);
	vTaskDelay(100);
	effect21off(1000);
	vTaskDelay(500);

}

void run_demo(void) {
	uint8_t i = 0;
	uint8_t i2 = 0;
	// trigger valves
	for (i2 = 0; i2 < 5; i2++) {
		run_demo2();
		for (i = 0; i < 16; i++) {
			open_valve(i);
			vTaskDelay(200);
			close_valve(i);
			vTaskDelay(200);
		}
	}

	// switch on/off EC meter
	power_ctrl(PWR_EC, 1);
	vTaskDelay(200);
	power_ctrl(PWR_EC, 0);
	vTaskDelay(200);
	power_ctrl(PWR_EC, 1);
	vTaskDelay(200);
	power_ctrl(PWR_EC, 0);
	vTaskDelay(200);

	// run pumps at different speeds
	// spin-down
	for (i = 0; i < 60; i++) {
		TIM3->CCR1 = i * 10;
		vTaskDelay(10);
	}
	//  spin-up
	for (i = 60; i > 0; i--) {
		TIM3->CCR1 = i * 10;
		vTaskDelay(10);
	}

	TIM3->CCR1 = 1000;

	for (i = 0; i < 60; i++) {
		TIM3->CCR2 = i * 10;
		vTaskDelay(10);
	}
	TIM3->CCR2 = 1000;
	for (i = 0; i < 60; i++) {
		TIM3->CCR3 = i * 10;
		vTaskDelay(10);
	}
	TIM3->CCR3 = 1000;
	for (i = 0; i < 60; i++) {
		TIM3->CCR4 = i * 10;
		vTaskDelay(10);
	}
	TIM3->CCR4 = 1000;

	// PSI pump test
	PSI_PUMP_TIM->CCR1 = 0;
	vTaskDelay(1000);
	PSI_PUMP_TIM->CCR1 = 1000;

	vTaskDelay(1000);
}

// adds 'amount' cms of water from FWTank into MixTank
void wt_get_water(uint8_t amount) {
	/* Strategy:
	 * 	- close ALL valves
	 * 	- open FWI valve
	 * 	- open Back valve
	 * 	- store current water level in MixTank into L. Substract 'amount' from L to get level to reach
	 *	- run watering pump while CurWtrLvl>L
	 *	- wait until water being stabilized and add a bit more water to adjust the water level properly
	 */
	close_valves();
	open_valve(BACK_VALVE);
	open_valve(FWI_VALVE);
	uint8_t L = 0;
	L = sonar_read[MIXTANK_SONAR] - amount;
	psiOn();
	while (sonar_read[MIXTANK_SONAR] > L) {
		psiStab();
		vTaskDelay(40);
	}
	psiOff();
	close_valves();
}

void wt_mixing(uint16_t duration){
	uint32_t timeout = 0;
	uint32_t now = 0;
//	timeout = RTC_GetCounter() + 15;	// HARDCODE
	timeout = duration;
	timeout &= 0xFFFF;
	timeout += RTC_GetCounter();
	close_valves();		// close all valves before doing anything
	open_valve(MTI_VALVE);		// open MIXTANK valve for incoming water
	open_valve(BACK_VALVE);		// open desired valve_id for watering out
	now = RTC_GetCounter();
	while (now<timeout) {
		now = RTC_GetCounter();
		vTaskDelay(200);
		psiOn();
		IWDG_ReloadCounter();
	}
	psiOff();
	vTaskDelay(500);
	close_valves();
}

void wt_mix_in(uint16_t duration, uint16_t vol, uint8_t doserid, uint8_t spd){
	uint32_t timeout = 0;
	uint32_t finishDosing = 0;
	uint32_t now = 0;

	now = RTC_GetCounter();
//	timeout = RTC_GetCounter() + 15;	// HARDCODE
	finishDosing = vol;
	finishDosing &= 0xFFFF;
	finishDosing += now;
	timeout = duration;
	timeout &= 0xFFFF;
	timeout += now;
	close_valves();		// close all valves before doing anything
	open_valve(MTI_VALVE);		// open MIXTANK valve for incoming water
	open_valve(BACK_VALVE);		// open desired valve_id for watering out
	while (now<timeout) {
		if (now < finishDosing) {
			enable_dosing_pump(doserid, spd);
		}
		else {
			enable_dosing_pump(doserid, 0);
		}
		now = RTC_GetCounter();
		IWDG_ReloadCounter();
		vTaskDelay(200);
		psiOn();
		IWDG_ReloadCounter();
	}
	psiOff();
	enable_dosing_pump(doserid, 0);
	vTaskDelay(500);
	close_valves();
}

void wt_watering(uint16_t duration, uint8_t line_id){
	uint32_t timeout = 0;
	uint32_t now = 0;
	timeout = duration;
	timeout &= 0xFFFF;
	timeout += RTC_GetCounter();
	close_valves();		// close all valves before doing anything
	open_valve(MTI_VALVE);		// open MIXTANK valve for incoming water
	open_valve(line_id);		// open desired valve_id for watering out
	now = RTC_GetCounter();
	while (now<timeout) {
		now = RTC_GetCounter();
		vTaskDelay(200);
		psiOn();
	}
	psiOff();
	vTaskDelay(1000);
	close_valves();
}

void wt_watering_x(uint16_t duration, uint16_t valveFlags){
	uint32_t timeout = 0;
	uint32_t now = 0;
	timeout = duration;
	timeout &= 0xFFFF;
	timeout += RTC_GetCounter();
	close_valves();		// close all valves before doing anything
	open_valve(MTI_VALVE);		// open MIXTANK valve for incoming water
	open_valves(valveFlags);		// open desired valve_id for watering out
	now = RTC_GetCounter();
	while (now<timeout) {
		now = RTC_GetCounter();
		vTaskDelay(200);
		psiOn();
	}
	psiOff();
	vTaskDelay(1000);
	close_valves();
}





// get water into MixTank from the water 'source', reaching 'level' desired. Default source - :FWI
void wt_mt_reach_level(uint16_t new_level, uint8_t source){
	uint16_t curlevel = 0;
	curlevel = sonar_read[MIXTANK_SONAR];
	close_valves();
	open_valve(BACK_VALVE);
	while (sonar_read[MIXTANK_SONAR] > new_level) {
		open_valve(FWI_VALVE);
		psiOn();
		vTaskDelay(400);
		IWDG_ReloadCounter();
	}
	psiOff();
	close_valves();
}

// add 'amount' of water from source to MixTank
void wt_mt_add_water(uint16_t amount, uint8_t source){
	uint16_t curlevel = 0;
	uint16_t new_level = 0;
	close_valves();
	curlevel = sonar_read[MIXTANK_SONAR];
	new_level = curlevel - amount;	// cpunt new level to reach
	open_valve(BACK_VALVE);
	while (sonar_read[MIXTANK_SONAR] > new_level) {
		open_valve(FWI_VALVE);
		psiOn();
		vTaskDelay(400);
		IWDG_ReloadCounter();
	}
	psiOff();
	close_valve(FWI_VALVE);
	vTaskDelay(1000);
	close_valves();
}

void wt_mt_drain2level(uint16_t new_level, uint8_t drain_valve){
	uint16_t curlevel = 0;
	close_valves();
	curlevel = sonar_read[MIXTANK_SONAR];
	open_valve(MTI_VALVE);
	while (sonar_read[MIXTANK_SONAR] < new_level) {
		open_valve(drain_valve);
		psiOn();
		vTaskDelay(400);
		IWDG_ReloadCounter();
	}
	psiOff();
	vTaskDelay(1000);
	close_valves();
}

void run_watering_task(uint8_t pwt) {
	uint16_t val16 = 0;
	uint16_t val16_2 = 0;
	switch (pwt) {
		case 101:
			// psiStab_secs(wt_args[0]);
			break;
		case 102: //	watering
			val16 = (((((uint16_t)wt_args[0])<<8)&((uint16_t)0xFF00)))+((((uint16_t)wt_args[1])&((uint16_t)0xFF)));
			wt_watering(val16,wt_args[2]);
			break;
		case 103: //	mixing solution

			val16 = ((((uint16_t) wt_args[0]) << 8) & 0xFF00)
					+ (((uint16_t) wt_args[1]) & 0x00FF);
			//val16 = val16 + 15;
			wt_mixing(val16);
			break;
		case 104: //	mixing solution and adding fertilizer

			val16 = ((((uint16_t)wt_args[0]) << 8) & 0xFF00)
					+ (((uint16_t)wt_args[1]) & 0x00FF);
			val16_2 = ((((uint16_t)wt_args[2]) << 8) & 0xFF00)
					+ (((uint16_t)wt_args[3]) & 0x00FF);
			//val16 = val16 + 15;
			wt_mix_in(val16, val16_2, wt_args[4], wt_args[5]);
			break;
		case 105:
			val16 = ((((uint16_t)wt_args[0]) << 8) & 0xFF00)
					+ (((uint16_t)wt_args[1]) & 0x00FF);
			// wt_mt_reach_level(uint16_t new_level, uint8_t source)
			wt_mt_reach_level(val16,wt_args[2]);
			break;
		case 106:
			val16 = ((((uint16_t)wt_args[0]) << 8) & 0xFF00)
					+ (((uint16_t)wt_args[1]) & 0x00FF);

			// wt_mt_add_water(uint16_t amount, uint8_t source)
			wt_mt_add_water(val16,wt_args[2]);
			break;
		case 107:
			val16 = ((((uint16_t)wt_args[0]) << 8) & 0xFF00)
					+ (((uint16_t)wt_args[1]) & 0x00FF);
			// void wt_mt_drain2level(uint16_t new_level, uint8_t drain_valve)
			wt_mt_drain2level(val16,wt_args[2]);
			break;



	}
	pending_wt = 0;
}



void dht_init_exti(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; //enable SONAR_TIM clock
	TIM15->PSC = 1000; //set divider
	TIM15->CR1 = TIM_CR1_CEN;
	TIM15->CNT = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Configure PB0-2 pins as input pull-down
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// interrupt config for DHT

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15);
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0A;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/*
 void dht_init(void){	// 150404
 // RCC Config
 // TIM15 clock enable
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);

 // GPIO Config
 GPIO_InitTypeDef GPIO_InitStructure;

 // TIM15 channel 2 pin (PB15) configuration
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

 GPIO_Init(GPIOB, &GPIO_InitStructure);

 // NVIC Config

 GPIO_PinRemapConfig(GPIO_Remap_TIM15, ENABLE);
 NVIC_InitTypeDef NVIC_InitStructure;

 // Enable the TIM15 global Interrupt

 NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM15_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);

 TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
 //	   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
 TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
 TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
 TIM_ICInitStructure.TIM_ICFilter = 0x0;

 TIM_PWMIConfig(TIM15, &TIM_ICInitStructure);

 // Select the TIM15 Input Trigger: TI2FP2
 TIM_SelectInputTrigger(TIM15, TIM_TS_TI2FP2);

 // Select the slave Mode: Reset Mode
 TIM_SelectSlaveMode(TIM15, TIM_SlaveMode_Reset);

 // Enable the Master/Slave Mode
 TIM_SelectMasterSlaveMode(TIM15, TIM_MasterSlaveMode_Enable);

 // TIM enable counter
 TIM_Cmd(TIM15, ENABLE);

 // Enable the CC2 Interrupt Request
 TIM_ITConfig(TIM15, TIM_IT_CC2, ENABLE);

 }



 void dht_init_out(void){	// init GPIO pin as OUTput
 GPIOB->CRH      &= ~GPIO_CRH_CNF15;		// ... and PB15 for DHT triggering
 GPIOB->CRH   |= GPIO_CRH_MODE15_0;
 }

 void dht2_init_out(void){	// init GPIO pin as OUTput
 GPIOB->CRH      &= ~GPIO_CRH_CNF14;		// ... and PB15 for DHT triggering
 GPIOB->CRH   |= GPIO_CRH_MODE14_0;
 } */

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/*
 void TIM1_BRK_TIM15_IRQHandler(void)		// DHT moved from PA7 to PB15. 11.07.2013
 {
 //	if ((TIM_GetITStatus(TIM15, TIM_IT_CC2) != RESET) && (TIM_GetITStatus(TIM15, TIM_IT_CC2) != RESET))  {
 // Clear TIM3 Capture compare interrupt pending bit

 TIM_ClearITPendingBit(TIM15, TIM_IT_CC2);
 TIM_ClearITPendingBit(TIM15, TIM_IT_CC1);

 // Get the Input Capture value
 IC2Value = TIM15->CCR2;

 if (IC2Value != 0)
 {
 // Duty cycle computation
 DutyCycle = ((TIM15->CCR1) * 100) / IC2Value;
 icval++;
 }
 else
 {
 icval2++;
 DutyCycle = 0;
 Frequency = 0;
 }



 if (dht_bit_position>dht_shifter && dht_data_ready==0) {
 if (DutyCycle>25 && DutyCycle<35) {
 dht_byte_buf[dht_byte_pointer] &= ~(1<<(dht_bit_pointer));	// reset bit in dht_data[i]
 }
 else {
 dht_byte_buf[dht_byte_pointer] |= (1<<(dht_bit_pointer)); // set bit
 }
 if (dht_bit_pointer==0){
 dht_bit_pointer=7;
 dht_byte_pointer++;
 if (dht_byte_pointer==4) {
 dht_data_ready=1;
 dht_byte_pointer = 0;
 dht_bit_pointer = 7;
 }
 }
 else {
 dht_bit_pointer--;
 }
 }
 //	}


 }
 */

void TIM1_UP_TIM16_IRQHandler(void) // mixtank sonar interrupt
{
	/*  if (TIM_GetITStatus(TIM16, TIM_IT_CC1) != RESET) {
	 // Clear TIM16 Capture compare interrupt pending bit
	 TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);

	 // Get the Input Capture value
	 if ((TIM16->CNT) < MAX_SONAR_READ && (TIM16->CNT)>0) {
	 sonar_read[MIXTANK_SONAR] = TIM16->CNT;
	 }
	 TIM16->CNT = 0;
	 } */
}

void TIM1_TRG_COM_TIM17_IRQHandler(void) // fresh water tank sonar interrupt
{
	/*	if (TIM_GetITStatus(TIM17, TIM_IT_CC1) != RESET) {
	 // Clear TIM16 Capture compare interrupt pending bit
	 TIM_ClearITPendingBit(TIM17, TIM_IT_CC1);
	 // Get the Input Capture value
	 if (!(GPIOB->IDR & (1<<9)) && (TIM17->CNT < MAX_SONAR_READ) && ((TIM17->CNT) > 0)) {
	 sonar_read[FWTANK_SONAR]=SONAR_TIM->CNT;
	 }
	 TIM17->CNT = 0;
	 } */
}

// Irq handler for sonars
volatile static uint16_t sonar1_rise = 0;
volatile static uint16_t sonar2_rise = 0;

void EXTI9_5_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line8);
		if (SONAR1_FALL) {
			sonar_read[0] = SONAR_TIM->CNT - sonar1_rise;
		} else {
			sonar1_rise = SONAR_TIM->CNT;
		}
	}
	if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line9);
		if (SONAR2_FALL) {
			sonar_read[1] = SONAR_TIM->CNT - sonar2_rise;
		} else {
			sonar2_rise = SONAR_TIM->CNT;
		}
	}
}

void TIM4_IRQHandler(void) {
	/* Get the Input Capture value */
	// for channel 3
	if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET) {
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);

	}
	if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET) {
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);

	}
}

/* void dht_get_data(void){	// function starts getting data from DHT22 sensor
 vTaskDelay(25);
 uint8_t i;
 for (i=0;i<5;i++) {
 dht_byte_buf[i]=0;
 }
 dht_bit_position = 0;
 dht_data_ready=0;
 dht_init_out();
 DHT2_0;
 vTaskDelay(5);
 DHT2_1;
 dht_init();
 vTaskDelay(200);

 vTaskDelay(5);
 dht_conv_data();
 } */

/*
 void dht2_get_data(void){	// function starts getting data from DHT22 sensor
 vTaskDelay(25);
 uint8_t i;
 for (i=0;i<5;i++) {
 dht_byte_buf[i]=0;
 }
 dht_rise = 0;
 dht2_init_out();

 DHT1_0;
 IWDG_ReloadCounter();
 Delay_us_(21000);
 IWDG_ReloadCounter();
 DHT1_1;
 dht2_init();
 dht_bit_pointer = 7;
 dht_bit_position = 0;
 dht_byte_pointer = 0;
 dht_data_ready=0;
 dht_rise = 0;
 vTaskDelay(200);
 dht_conv_data();
 }
 */

void dht_arr_displayer(void) {
	button = 0;
	while (button != BUTTON_OK) {
		vTaskDelay(30);
#ifdef TEST_MODE
		button = readButtons();
		Lcd_goto(0, 0);
		Lcd_write_digit(dht_shifter);
		Lcd_write_str(": ");
		if (button == BUTTON_FWD) {
			dht_shifter++;
		}
		if (button == BUTTON_BCK) {
			dht_shifter--;
		}
#endif

		Lcd_goto(1, 0);
		Lcd_write_str("T:");
		copy_arr(&dht1_t_str, &LCDLine2, 4, 2, 0);
		Lcd_goto(1, 7);
		Lcd_write_str("rH:");
		copy_arr(&dht1_rh_str, &LCDLine2, 4, 10, 0);
	}
	Lcd_clear();
}

uint8_t readPercentVal(uint8_t value) {
	char prcntStr[5];
	value %= 101; // drop all except 0..100
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		Lcd_goto(1, 3);
		prcntStr[0] = (value % 1000) / 100;
		prcntStr[1] = (value % 100) / 10;
		prcntStr[2] = (value % 10);
		Lcd_write_str("<");
		copy_arr(prcntStr, LCDLine2, 3, 6, 0);
		Lcd_goto(1, 12);
		Lcd_write_str(">");
		vTaskDelay(10);
		if (button == BUTTON_FWD) {
			if (value < 100) {
				value++;
			} else {
				value = 0;
			}
		}
		if (button == BUTTON_BCK) {
			if (value < 1) {
				value = 0;
			} else {
				value--;
			}
		}
		vTaskDelay(10);
	}
	return value;
}

void hygroStatSettings(void) {
	uint16_t rh_value = 0;
	uint8_t rh_val = 0;
	Lcd_clear();
	Lcd_write_str("Set top rH lvl");
	vTaskDelay(10);
	EE_ReadVariable(RH_WINDOW_TOP, &rh_value);
	rh_val = readPercentVal(rh_value);
	vTaskDelay(10);
	EE_WriteVariable(RH_WINDOW_TOP, rh_val);
	Lcd_clear();
	Lcd_write_str("Set bottom lvl");
	vTaskDelay(10);
	EE_ReadVariable(RH_WINDOW_BOTTOM, &rh_value);
//	phval = adc2ph(phadcvalue);
	rh_val = readPercentVal(rh_value);
	vTaskDelay(10);
	EE_WriteVariable(RH_WINDOW_BOTTOM, rh_val);
	Lcd_clear();
}

void convPh2str(uint8_t ph, char* phstr) {
	phstr[3] = ph - (ph / 10) * 10 + 48;
	vTaskDelay(5);
	ph /= 10;
	phstr[2] = 46;
	phstr[1] = ph - (ph / 10) * 10 + 48;
	vTaskDelay(5);
	ph /= 10;
	phstr[0] = ph - (ph / 10) * 10 + 48;
	phstr[4] = 0; // konec stroki
}

uint8_t readPhVal(uint8_t value) {

	char phStr[5];
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		Lcd_goto(1, 3);
		convPh2str(value, &phStr);
		Lcd_write_str("<");
		copy_arr(phStr, LCDLine2, 4, 6, 0);
		Lcd_goto(1, 12);
		Lcd_write_str(">");
		vTaskDelay(10);
		if (button == BUTTON_FWD) {
			if (value < 254) {
				value++;
			} else {
				value = 0;
			}
		}
		if (button == BUTTON_BCK) {
			if (value < 1) {
				value = 0;
			} else {
				value--;
			}
		}
		vTaskDelay(10);
	}
	return value;
}

void Lcd_write_16int(uint16_t d) {
	uint8_t i = 0;
	char out[5];
	out[5] = '\0';
	out[4] = '0' + (d) % 10;
	out[3] = '0' + (d /= 10) % 10;
	out[2] = '0' + (d /= 10) % 10;
	out[1] = '0' + (d /= 10) % 10;
	out[0] = '0' + (d /= 10) % 10;
	copy_arr(&out, LCDLine2, 6, 0, 0);
	for (i = 0; i < 5; i++) {
		Lcd_write_str(out[i]);
	}
}

void Lcd_write_32int(uint32_t d) {
	char tmpstr[10];
	int32str(d, &tmpstr);
	copy_arr(&tmpstr, LCDLine2, 11, 0, 0);
}

/* void enablePlug5ms(uint8_t plug, uint16_t amount){	// 1 amount = 5ms
 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   //enable TIM2 clock
 TIM2->PSC     = 40000-1;               //set divider for 5 milliseconds
 TIM2->ARR = amount;
 TIM2->CR1     = TIM_CR1_OPM;          //one pulse mode
 TIM2->CR1 |= TIM_CR1_CEN;
 plugStateSet(plug, 1);
 while((TIM2->SR & TIM_SR_UIF)==0){
 vTaskDelay(10);
 }
 plugStateSet(plug, 0);
 }
 */

typedef struct {
	uint32_t quot;
	uint8_t rem;
} divmod10_t;

#define MENURECS	34

#ifndef TEST_MODE
// menu items
const char menuItemArray[MENURECS][18]=
{
	{	"MONITOR MODE"}, // 0
	{	"TIMERS"}, // 1
	{	"Timer 1"}, // 2
	{	"Timer 2"}, // 3
	{	"Timer 3"}, // 4
	{	"SET CLOCK"}, // 5
	{	"CYCLIC TIMERS"}, // 6
	{	"C Timer 1"}, // 7
	{	"C Timer 2"}, // 8
	{	"C Timer 3"}, // 9
	{	"PLUG SETTINGS"}, // 10
	{	"Plug 1"}, // 11
	{	"Plug 2"}, // 12
	{	"Plug 3"}, // 13
	{	"pH-monitor"}, // 14
	{	"Calibration"}, // 15
	{	"pH-stabilizer"}, // 16

	{	"EC-monitor"}, // 17
	{	"Calibration"}, // 18
	{	"EC-stabilizer"}, // 19

	{	"DAYLIGHT SENSOR"}, // 20
	{	"   "}, // 21
	{	"Keeping window"}, // 22
	{	"Get water"}, // 23
	{	"Add Light"}, // 24
	{	"Temp. & humidity"}, // 25
	{	"Hygrostat"}, // 26
	{	"Thermostat"}, // 27
	{	"Valve test"}, // 28
	{	"Watering progs"}, // 29
	{	"Fertilization"}, // 30
	{	"Button test"}, // 31
	{	"pH&EC test"}, // 32
	{	"Start watering"} // 33

};

// 0 - nr zapisi, 1 - link na tekst, 2 - <, 3 - >, 4 - OK, 5 - CNCL, 6 - tip zapisi (0 - folder, 1 - program)
const uint8_t fatArray[MENURECS][7]=
{
	{	0, 0, 33, 1, 1, 0, 1},
	{	1, 1, 0, 5, 2, 1, 1},
	{	2, 2, 4, 3, 2, 1, 1},
	{	3, 3, 2, 4, 3, 1, 1},
	{	4, 4, 3, 2, 4, 1, 1},
	{	5, 5, 1, 6, 5, 5, 1},
	{	6, 6, 5, 10, 7, 6, 0},
	{	7, 7, 9, 8, 6, 6, 1},
	{	8, 8, 7, 9, 7, 6, 1},
	{	9, 9, 8, 7, 8, 6, 1},
	{	10,10, 6, 14, 11, 10, 0},
	{	11,11, 13, 12, 9, 10, 1},
	{	12,12, 11, 13, 10, 10, 1},
	{	13,13, 12, 11, 11, 10, 1},
	{	14,14, 10, 17, 15, 14, 0},
	{	15,15, 16, 16, 15, 14, 1},
	{	16,16, 15, 15, 16, 14, 1},
	{	17,17, 14, 20, 18, 17, 0},
	{	18,18, 19, 19, 23, 17, 1}, // ec calibration
	{	19,19, 18, 18, 24, 17, 1}, // ec stab settings

	{	20,20, 17, 21, 14, 17, 1}, // daylight sensor
	{	21,21, 20, 25, 22, 18, 0}, // combi timers
	{	22,22, 24, 23, 15, 22, 1}, // Tank level Keeping window setup
	{	23,23, 22, 24, 16, 23, 1}, // t+2ct
	{	24,24, 23, 22, 17, 24, 1}, // add light
	{	25,25, 21, 26, 17, 25, 1}, // temp and humidity
	{	26,26, 25, 27, 18, 26, 1}, // hygrostat
	{	27,27, 26, 28, 19, 27, 1}, // thermostat
	{	28,28, 27, 29, 20, 28, 1}, // valve test
	{	29,29, 28, 30, 21, 29, 1}, // watering progs
	{	30,30, 29, 31, 22, 30, 1}, // fertilization
	{	31,31, 30, 32, 25, 31, 1}, // button test
	{	32,32, 31, 33, 26, 32, 1}, // pH and ec test
	{	33,33, 32, 0, 27, 33, 1} // test function

};

#endif

#ifdef TEST_MODE
// menu items

/*
 * *
 * |-MONITOR MODE 			/ 0
 * |-TESTS					/ 1
 * |	|-Valves			/ 2
 * |	|-Temp&Humidity		/ 3
 * |	|-EEPROM			/ 4
 * |	|-Plugs				/ 5
 * |	|-Dosers			/ 6
 * |	|-rest				/ 7
 * |-TIMERS					/ 8
 * |	|-24H/Full Range	/ 9
 * |	|-Cyclic			/ 10
 * |-STABILIZERS			/ 11
 * |	|-CO2				/ 12
 * |	|-Pressure			/ 13
 * |	|-Tank levels		/ 14
 * |	|-Temperature		/ 15
 * |	|-Humidity			/ 16
 * |	|-pH				/ 17
 * |-SETTINGS				/ 18
 * |	|-Dosers			/ 19
 * |	|-Plug settings		/ 20
 * |	|-Set clock			/ 21
 * |-WATERING				/ 22
 * |	|-Watering progs	/ 23
 * |	|-Fertilization		/ 24
 */

const char menuItemArray[MENURECS][18] = { { "MONITOR MODE" }, // 0
		{ "TESTS" }, // 1
		{ "Valves" }, // 2
		{ "Temp&Humidity" }, // 3
		{ "EEPROM" }, // 4
		{ "Plugs" }, // 5
		{ "Dosers" }, // 6
		{ "rest" }, // 7
		{ "TIMERS" }, // 8
		{ "24h / Full range" }, // 9
		{ "Cyclic" }, // 10
		{ "STABILIZERS" }, // 11
		{ "CO2" }, // 12
		{ "Pressure" }, // 13
		{ "Tank levels" }, // 14
		{ "Temperature" }, // 15
		{ "Humidity" }, // 16
		{ "pH" }, // 17
		{ "SETTINGS" }, // 18
		{ "Dosers" }, // 19
		{ "Plugs" }, // 20
		{ "Set clock" }, // 21
		{ "WATERING" }, // 22
		{ "Watering progs" }, // 23
		{ "Fertilization" } // 24

};

/*
 const char menuItemArray[MENURECS][18]=
 {
 {"MONITOR MODE"},		// 0
 {"TIMERS"},				// 1
 {"Tests"},	    		// 2
 {"Plug test"},			// 3
 {"PSI sensor cal."},			// 4
 {"SET CLOCK"},			// 5
 {"CYCLIC TIMERS"},			// 6
 {"C Timer 1"},			// 7
 {"C Timer 2"},			// 8
 {"C Timer 3"},			// 9
 {"PLUG SETTINGS"},			// 10
 {"Plug 1"},			// 11
 {"Plug 2"},			// 12
 {"Plug 3"},			// 13
 {"pH-monitor"},		// 14
 {"Calibration"},	// 15
 {"CO2-stabilizer"},	// 16

 {"EC-monitor"},		// 17
 {"Calibration"},	// 18
 {"EC-stabilizer"},	// 19

 {"DAYLIGHT SENSOR"},// 20
 {"Tank lvl keeper"},	// 21
 {"Keeper window"},	// 22
 {"Tank tests"},	// 23
 {"Add Light"},	// 24
 {"Temp. & humidity"},	// 25
 {"Hygrostat"},	// 26
 {"Thermostat"},	// 27
 {"Valve test"},	// 28
 {"Watering progs"},	// 29
 {"Fertilization"},	// 30
 {"Button test"},	// 31
 {"pH&EC test"},	// 32
 {"Start watering"}	// 33

 };
 */

// 0 - nr zapisi, 1 - link na tekst, 2 - <, 3 - >, 4 - OK, 5 - CNCL, 6 - tip zapisi (0 - folder, 1 - program)
const uint8_t fatArray[MENURECS][7] = { { 0, 0, 22, 1, 1, 0, 1 }, // MONITOR MODE
		{ 1, 1, 0, 8, 2, 1, 0 }, // TESTS
		{ 2, 2, 7, 3, 2, 1, 1 }, // Valves
		{ 3, 3, 2, 4, 3, 1, 1 }, // Temp and Humidity
		{ 4, 4, 3, 5, 4, 1, 1 }, // EEPROM
		{ 5, 5, 4, 6, 5, 1, 1 }, // Dosers
		{ 6, 6, 5, 7, 7, 1, 1 }, // Plugs
		{ 7, 7, 7, 2, 6, 1, 1 }, // Rest
		{ 8, 8, 1, 11, 9, 8, 0 }, // TIMERS
		{ 9, 9, 10, 10, 8, 8, 1 }, // 24H/Full range
		{ 10, 10, 9, 9, 9, 8, 1 }, // Cyclic
		{ 11, 11, 8, 18, 12, 11, 0 }, // STABILIZERS
		{ 12, 12, 17, 13, 10, 11, 1 }, // CO2
		{ 13, 13, 12, 14, 11, 11, 1 }, // Pressure
		{ 14, 14, 13, 15, 15, 11, 0 }, // Tank levels
		{ 15, 15, 14, 16, 15, 11, 1 }, // Temperature
		{ 16, 16, 15, 17, 16, 11, 1 }, // Humidity
		{ 17, 17, 16, 12, 18, 11, 0 }, // pH
		{ 18, 18, 11, 22, 19, 18, 0 }, // SETTINGS
		{ 19, 19, 21, 20, 24, 18, 1 }, // Dosers
		{ 20, 20, 19, 21, 14, 18, 1 }, // Plugs
		{ 21, 21, 20, 19, 14, 18, 1 }, // Plugs
		{ 22, 22, 18, 0, 23, 18, 0 }, // WATERING
		{ 23, 23, 24, 24, 28, 22, 1 }, // Watering progs
		{ 24, 24, 23, 23, 23, 22, 1 } // Fertilization
};

#endif

char* adc2str(uint_fast16_t d, volatile char* out) {
	char out2[17];
	uint8_t i, k, c;
	out2[16] = '\0';
	out2[15] = '0' + (d) % 10;
	out2[14] = '0' + (d /= 10) % 10;
	out2[13] = '0' + (d /= 10) % 10;
	out2[12] = '0' + (d /= 10) % 10;
	out2[11] = '0' + (d /= 10) % 10;
	out2[10] = '0' + (d /= 10) % 10;
	vTaskDelay(25);
	out2[9] = '0' + (d /= 10) % 10;
	out2[8] = '0' + (d /= 10) % 10;
	out2[7] = '0' + (d /= 10) % 10;
	out2[6] = '0' + (d /= 10) % 10;
	out2[5] = '0' + (d /= 10) % 10;
	out2[4] = '0' + (d /= 10) % 10;
	out2[3] = '0' + (d /= 10) % 10;
	out2[2] = '0' + (d /= 10) % 10;
	out2[1] = '0' + (d /= 10) % 10;
	out2[0] = '0' + (d / 10) % 10;
	k = 0;
	c = out2[k];
	c = 48;

	vTaskDelay(25);
	while (c == 48) {
		k++;
		c = out2[k];
	}
	vTaskDelay(25);
	for (i = k; i < 17; i++) {
		out[i - k] = out2[i];
	}

	return out;
}

void Delay_us_(uint32_t delay) { // adjusted for 8mhz
	volatile uint32_t del = 0;
	del = delay * 3;
	while (del--) {

	}
}

void Delay_us(uint32_t delay) {
	volatile uint32_t del = 0;
	del = delay * 250;
	while (del--) {

	}
}

void buttonCalibration(void) { // buttons calibration function
	volatile static uint16_t button_val[4];
	uint16_t diff = 0;
	Lcd_clear();
	Lcd_write_str("<");
	vTaskDelay(2000);
	adcAverager();
	vTaskDelay(40);
	button_val[0] = adcAverage[ADC_AVG_BUTTONS];
	Lcd_clear();
	Lcd_write_str("OK");
	vTaskDelay(2000);
	adcAverager();
	vTaskDelay(40);
	button_val[1] = adcAverage[ADC_AVG_BUTTONS];
	Lcd_clear();
	Lcd_write_str("CANCEL");
	vTaskDelay(2000);
	adcAverager();
	vTaskDelay(40);
	button_val[2] = adcAverage[ADC_AVG_BUTTONS];
	Lcd_clear();
	Lcd_write_str(">");
	vTaskDelay(2000);
	adcAverager();
	vTaskDelay(40);
	button_val[3] = adcAverage[ADC_AVG_BUTTONS];

	if ((button_val[3] >> 3) < (button_val[0] >> 3)) {
		buttonReverse = 1;
	} else if ((button_val[3] >> 3) > (button_val[0] >> 3)) {
		buttonReverse = 0;
	} else {
		buttonReverse = 2; //means loading button settings from EEPROM
	}
	if (buttonReverse == 0) {
		diff = ((button_val[1] - button_val[0]) / 2) - 5;
		button_ranges[0] = (button_val[0] - diff / 2);
		button_ranges[1] = button_val[0] + diff;
		button_ranges[2] = button_val[1] - diff;
		diff = ((button_val[2] - button_val[1]) / 2) - 5;
		button_ranges[3] = button_val[1] + diff;
		button_ranges[4] = button_val[2] - diff;
		diff = ((button_val[3] - button_val[2]) / 2) - 5;
		button_ranges[5] = button_val[2] + diff;
		button_ranges[6] = button_val[3] - diff;
		button_ranges[7] = (button_val[3] + diff / 2);
		saveButtonRanges();
	} else if (buttonReverse == 1) {
		diff = ((button_val[0] - button_val[1]) / 2) - 5;
		button_ranges[0] = button_val[0] - diff;
		button_ranges[1] = button_val[0] + diff;
		button_ranges[2] = button_val[1] - diff;
		diff = ((button_val[1] - button_val[2]) / 2) - 5;
		button_ranges[3] = button_val[1] + diff;
		button_ranges[4] = button_val[2] - diff;
		diff = ((button_val[2] - button_val[3]) / 2) - 5;
		button_ranges[5] = button_val[2] + diff;
		button_ranges[6] = button_val[3] - diff;
		button_ranges[7] = button_val[3] + diff;
		saveButtonRanges();
	} else {
		Lcd_write_str("EEPROM buttons");
		readButtonRanges();
	}

	if (button_ranges[0] > button_ranges[8]) {
		buttonReverse = 1;
	}
	Lcd_goto(0, 0);
	Lcd_write_str("Complete");
	Lcd_clear();
}

void buttonCalibration2(void) { // buttons calibration function / became ol 08.09.2014
	Lcd_clear();
	Lcd_goto(0, 0);
	uint16_t button_val[4], diff;
	Lcd_clear();
	Lcd_goto(0, 0);
	Lcd_write_arr("<", 1);
	Delay_us(30000);
	adcAverager();
	Delay_us(100);
	button_val[0] = adcAverage[ADC_AVG_BUTTONS];
	Lcd_goto(0, 0);
	Lcd_write_arr("OK", 2);
	Delay_us(30000);
	adcAverager();
	Delay_us(100);
	button_val[1] = adcAverage[ADC_AVG_BUTTONS];
	Lcd_goto(0, 0);
	Lcd_write_arr("CANCEL", 6);
	Delay_us(30000);
	adcAverager();
	Delay_us(100);
	button_val[2] = adcAverage[ADC_AVG_BUTTONS];
	Lcd_clear();
	Lcd_write_arr(">", 1);
	Delay_us(30000);
	adcAverager();
	Delay_us(100);
	button_val[3] = adcAverage[ADC_AVG_BUTTONS];

	if ((button_val[3] >> 3) < (button_val[0] >> 3)) {
		buttonReverse = 1;
	} else if ((button_val[3] >> 3) > (button_val[0] >> 3)) {
		buttonReverse = 0;
	} else {
		buttonReverse = 2; //means loading button settings from EEPROM
	}
	if (buttonReverse == 0) {
		diff = ((button_val[1] - button_val[0]) / 2) - 5;
		button_ranges[0] = (button_val[0] - diff / 2);
		button_ranges[1] = button_val[0] + diff;
		button_ranges[2] = button_val[1] - diff;
		diff = ((button_val[2] - button_val[1]) / 2) - 5;
		button_ranges[3] = button_val[1] + diff;
		button_ranges[4] = button_val[2] - diff;
		diff = ((button_val[3] - button_val[2]) / 2) - 5;
		button_ranges[5] = button_val[2] + diff;
		button_ranges[6] = button_val[3] - diff;
		button_ranges[7] = (button_val[3] + diff / 2);
		saveButtonRanges();
	} else if (buttonReverse == 1) {
		diff = ((button_val[0] - button_val[1]) / 2) - 5;
		button_ranges[0] = button_val[0] - diff;
		button_ranges[1] = button_val[0] + diff;
		button_ranges[2] = button_val[1] - diff;
		diff = ((button_val[1] - button_val[2]) / 2) - 5;
		button_ranges[3] = button_val[1] + diff;
		button_ranges[4] = button_val[2] - diff;
		diff = ((button_val[2] - button_val[3]) / 2) - 5;
		button_ranges[5] = button_val[2] + diff;
		button_ranges[6] = button_val[3] - diff;
		button_ranges[7] = button_val[3] + diff;
		saveButtonRanges();
	} else {
		Lcd_write_str("EEPROM buttons");
		readButtonRanges();
	}

	if (button_ranges[0] > button_ranges[8]) {
		buttonReverse = 1;
	}
	Lcd_goto(0, 0);
	Lcd_write_str("Complete");
}

// display ADC values on LCD
void displayAdcValues(void) {
#ifdef TEST_MODE
	Lcd_clear();
	vTaskDelay(500);
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		Lcd_goto(0, 0);
		Lcd_write_str("1:");
		Lcd_write_digit(adcAverage[0] / 100);
		Lcd_write_digit(adcAverage[0]);
		Lcd_write_str(" 2:");
		Lcd_write_digit(adcAverage[1] / 100);
		Lcd_write_digit(adcAverage[1]);
		Lcd_goto(1, 0);
		Lcd_write_str("3:");
		Lcd_write_digit(adcAverage[2] / 100);
		Lcd_write_digit(adcAverage[2]);
		Lcd_write_str(" 4:");
		Lcd_write_digit(adcAverage[3] / 100);
		Lcd_write_digit(adcAverage[3]);
		vTaskDelay(20);
	}
	Lcd_clear();
#endif
}

void displayAdcValues_bak(void) {
#ifdef TEST_MODE
	Lcd_clear();
	vTaskDelay(500);
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		Lcd_goto(0, 0);
		Lcd_write_str("1:");
		Lcd_write_digit(ADC1->JDR1 / 100);
		Lcd_write_digit(ADC1->JDR1);
		Lcd_write_str(" 2:");
		Lcd_write_digit(ADC1->JDR2 / 100);
		Lcd_write_digit(ADC1->JDR2);
		Lcd_goto(1, 0);
		Lcd_write_str("3:");
		Lcd_write_digit(ADC1->JDR3 / 100);
		Lcd_write_digit(ADC1->JDR3);
		Lcd_write_str(" 4:");
		Lcd_write_digit(ADC1->JDR4 / 100);
		Lcd_write_digit(ADC1->JDR4);
		vTaskDelay(20);
	}
	Lcd_clear();
#endif
}

void display_usart_rx2(void) { // another usart test fr displaying RxBuffer contents
	uint8_t i = 0;
	Lcd_clear();
	vTaskDelay(500);
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		Lcd_goto(0, 0);
		vTaskDelay(2);
		for (i = 0; i < 5; i++) {
			Lcd_write_8b(RxBuffer[i]);
		}
		vTaskDelay(1);
		Lcd_goto(1, 0);
		Lcd_write_8b(RxBuffer[5]);
		Lcd_write_8b(RxBuffer[6]);

		Lcd_write_str(" ");
		Lcd_write_digit(rxm_state);
		Lcd_write_digit(RxCounter);
		Lcd_write_digit(rx_pntr);
		Lcd_write_digit(packet_length);
		vTaskDelay(20);
	}
	Lcd_clear();
}

void display_usart_rx(void) {
	uint8_t i = 0;
	Lcd_clear();
	vTaskDelay(500);
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		Lcd_goto(0, 0);
		vTaskDelay(2);
		for (i = 0; i < 5; i++) {
			Lcd_write_8b(RxBuffer[i]);
		}
		vTaskDelay(1);
		Lcd_goto(1, 0);
		Lcd_write_8b(RxBuffer[5]);
		Lcd_write_8b(RxBuffer[6]);
		Lcd_write_str(" ");
		Lcd_write_digit(prefixDetectionIdx);
		Lcd_write_digit(rx_pntr);
//			copy_arr(&RxBuffer, &LCDLine2, 5,11,0);
		vTaskDelay(20);
	}
	Lcd_clear();
}

void display_usart_tx(void) {
	uint8_t i = 0;
	Lcd_clear();
	vTaskDelay(500);
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		Lcd_goto(0, 0);
		vTaskDelay(2);
		for (i = 0; i < 5; i++) {
			Lcd_write_8b(TxBuffer[i]);
		}
		vTaskDelay(1);
		Lcd_goto(1, 0);
		Lcd_write_8b(TxBuffer[5]);
		Lcd_write_8b(TxBuffer[6]);
		Lcd_write_str(" ");
		push_tx();
		Lcd_write_digit(TxCounter);
		copy_arr(&TxBuffer, &LCDLine2, 7, 9, 0);
		vTaskDelay(20);
	}
	Lcd_clear();
}

void saveButtonRanges(void) {
	uint8_t i = 0;
	for (i = 0; i < 8; i++) {
		EE_WriteVariable(BUTTON_RANGES_START_ADDR + i, button_ranges[i]);
	}
}

void readButtonRanges(void) {
	uint8_t i = 0;
	for (i = 0; i < 8; i++) {
		EE_ReadVariable(BUTTON_RANGES_START_ADDR + i, &button_ranges[i]);
	}
}

/* FRESULT string2log(char* str, uint8_t bytes){

 } */

void adcAverager(void) {
	uint8_t i = 0;
	uint8_t i2 = 0;

	uint16_t jdrBuff1[JDR_BUFFER_SIZE];
	uint16_t jdrBuff2[JDR_BUFFER_SIZE];
	uint16_t jdrBuff3[JDR_BUFFER_SIZE];
	uint16_t jdrBuff4[JDR_BUFFER_SIZE];
	uint32_t jdrBuff1Total = 0;
	uint32_t jdrBuff2Total = 0;
	uint32_t jdrBuff3Total = 0;
	uint32_t jdrBuff4Total = 0;

	for (i2 = 0; i2 < 9; i2++) {
		for (i = 0; i < 9; i++) {
			jdrBuff1[i] = jdrBuff1[i + 1];
			jdrBuff2[i] = jdrBuff2[i + 1];
			jdrBuff3[i] = jdrBuff3[i + 1];
			jdrBuff4[i] = jdrBuff4[i + 1];
		}
		jdrBuff1[JDR_BUFFER_SIZE - 1] = ADC1->JDR1;
		jdrBuff2[JDR_BUFFER_SIZE - 1] = ADC1->JDR2;
		jdrBuff3[JDR_BUFFER_SIZE - 1] = ADC1->JDR3;
		jdrBuff4[JDR_BUFFER_SIZE - 1] = ADC1->JDR4;
		jdrBuff1Total += jdrBuff1[JDR_BUFFER_SIZE - 1];
		jdrBuff2Total += jdrBuff2[JDR_BUFFER_SIZE - 1];
		jdrBuff3Total += jdrBuff3[JDR_BUFFER_SIZE - 1];
		jdrBuff4Total += jdrBuff4[JDR_BUFFER_SIZE - 1];
	}
	adcAverage[0] = jdrBuff1Total / 10;
	adcAverage[1] = jdrBuff2Total / 10;
	adcAverage[2] = jdrBuff3Total / 10;
	adcAverage[3] = jdrBuff4Total / 10;
}

void set16bit(uint16_t value) {
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		vTaskDelay(25);
		if (button == BUTTON_BCK) {
			if (value < 1) {
				value = 65535;
			} else {
				value--;
			}
		}
		if (button == BUTTON_FWD) {
			if (value > 65534) {
				value = 0;
			} else {
				value++;
			}
		}
		Lcd_goto(1, 0);
		Lcd_write_str("<");
		Lcd_write_digit(value);
		Lcd_write_str(">");
		vTaskDelay(25);
	}
	vTaskDelay(50);
	printOk();
}

void setPlug(uint8_t plugId) {
	uint8_t timerId = 0;
	uint16_t Address = 0;
	Lcd_clear();
	Lcd_goto(0, 0);
	Lcd_write_str("Plug ");
	Lcd_write_digit(++plugId);
	Lcd_write_str(" timer");
	plugId--;
	Address = EE_PLUG_SETTINGS + plugId;
	EE_ReadVariable(Address, &timerId);
	vTaskDelay(50);
	timerId = adjust8bit(timerId);
	vTaskDelay(50);
	Address = EE_PLUG_SETTINGS + plugId;
	EE_WriteVariable(Address, timerId);
	vTaskDelay(5);
	loadSettings();
	vTaskDelay(5);
}

void printOk(void) {
	vTaskDelay(1);
	Lcd_clear();
	Lcd_write_str("OK");
	vTaskDelay(500);
	Lcd_clear();
	vTaskDelay(1);
}

void psiOn(void) {
	auto_flags |= (1 << 2); // set overpressure autosafe flag
	auto_failures &= ~1; // reset PSI main failure flag
	auto_failures &= ~(1 << 3); // reset PSI underpressure flag
	auto_flags |= 9; // enable watering through psi stab function with underpressure trig
}

// force disable PSI pump
void psiOff(void) {
	PSI_PUMP_TIM->CCR1 = 1000; // 100% PWM for disabling the pump
	auto_flags &= ~(1); // disable PSI stab function flag
}

uint16_t psi_sw_speed = 1000; // 1000 - stop motor, 0 - full speed.

void psiStab(void) {
//	uint8_t stabFlag=0;
//	uint8_t af1 = 0;
//	uint8_t af2 = 0;
	uint16_t factor = 0;
	if ((auto_flags & 1) == 1&& ((auto_failures&8)>>3)==0 && (auto_failures&1)==0 && comm_state==COMM_MONITOR_MODE) { // PSI STAB flag is number 0
		if	(adcAverage[AVG_ADC_PSI]>psi_pump_top_level) {
				psi_sw_speed = 1000;
				psiOff();
			}
		// vTaskDelay(1);
		if (adcAverage[AVG_ADC_PSI]<psi_pump_btm_level) {
			psi_sw_speed = (100-psi_max_speed)*10;
			PSI_PUMP_TIM->CCR1 = (100-psi_max_speed)*10;
		}
	/*		if (adcAverage[AVG_ADC_PSI]>psi_pump_btm_level && adcAverage[AVG_ADC_PSI]<psi_pump_top_level) {
	 // Cadi VSD range
	 if (adcAverage[AVG_ADC_PSI] < psi_a) {
	 // factor here is a number from 0 to 10, showing the rate of speed change
	 factor = ((psi_pump_top_level + psi_pump_btm_level)/2 - adcAverage[AVG_ADC_PSI])/(((psi_pump_top_level - psi_pump_btm_level)/2)/10);
	 psi_sw_speed -= factor*10;
	 }
	 if (adcAverage[AVG_ADC_PSI] > psi_b) {
	 factor = ((adcAverage[AVG_ADC_PSI] - (psi_pump_top_level + psi_pump_btm_level)/2))/(((psi_pump_top_level - psi_pump_btm_level)/2)/10);
	 psi_sw_speed += factor*10;
	 }
	 if (psi_sw_speed<((100-psi_m_low)*10)) {	// if software speed is > than minimum, then apply changes
	 if (TIM2->CCR3>((100-psi_m_low)*10)) {
	 psi_spin_up();
	 }
	 else {
	 spf = 0;
	 }
	 TIM2->CCR3 = psi_sw_speed;	// apply speed change (decrease). 1000 - full stop
	 TIM2->CCR4 = psi_sw_speed;
	 }
	 else {
	 TIM2->CCR3 = 1000;		// otherwise stop motor hardware
	 TIM2->CCR4 = 1000;
	 }


	 }*/
	}
	else {
		psiOff();
	}
}

void psiStabVsd(void) {
//	uint8_t stabFlag=0;
//	uint8_t af1 = 0;
//	uint8_t af2 = 0;
	uint16_t factor = 0;
	if ((auto_flags & 1)
			== 1&& ((auto_failures&8)>>3)==0 && (auto_failures&1)==0 && comm_state==COMM_MONITOR_MODE) { // PSI STAB flag is number 0
if	(adcAverage[AVG_ADC_PSI]>psi_pump_top_level) {
		psi_sw_speed = 1000;
		psiOff();
	}
	vTaskDelay(1);
	if (adcAverage[AVG_ADC_PSI]<psi_pump_btm_level) {
		psi_sw_speed = (100-PSI_MAX_SPEED)*10;
		PSI_PUMP_TIM->CCR1 = (100-PSI_MAX_SPEED)*10;
	}
	if (adcAverage[AVG_ADC_PSI]>psi_pump_btm_level && adcAverage[AVG_ADC_PSI]<psi_pump_top_level) {
		// Cadi VSD range
		if (adcAverage[AVG_ADC_PSI] < psi_a) {
			// factor here is a number from 0 to 10, showing the rate of speed change
			factor = ((psi_pump_top_level + psi_pump_btm_level)/2 - adcAverage[AVG_ADC_PSI])/(((psi_pump_top_level - psi_pump_btm_level)/2)/10);
			psi_sw_speed -= factor*10;
		}
		if (adcAverage[AVG_ADC_PSI] > psi_b) {
			factor = ((adcAverage[AVG_ADC_PSI] - (psi_pump_top_level + psi_pump_btm_level)/2))/(((psi_pump_top_level - psi_pump_btm_level)/2)/10);
			psi_sw_speed += factor*10;
		}
		if (psi_sw_speed<((100-PSI_M_LOW)*10)) { // if software speed is > than minimum, then apply changes
			if (PSI_PUMP_TIM->CCR1>((100-PSI_M_LOW)*10)) {
				psi_spin_up();
			}
			else {
				spf = 0;
			}
			PSI_PUMP_TIM->CCR1 = psi_sw_speed; // apply speed change (decrease). 1000 - full stop
		}
		else {
			psiOff();
		}

	}
}
}

void timerStateTrigger(void *pvParameters) {
	uint8_t i = 0;
	uint8_t timerStateFlag = 0;
	uint32_t now = 0;
	uint32_t timer1 = 0;
	uint32_t timer2 = 0;
	uint16_t Address;
	vTaskDelay(20000);
	while (1) {
		now = RTC_GetCounter();
		for (i = 0; i < 4; i++) { // 4 tajmera
			Address = EE_TIMER1_ON + EE_TIMER_SIZE * i;
			timer1 = EE_ReadWord(Address); // Timer ON
			timer2 = EE_ReadWord(Address + 2); // Timer OFF

			// for everyday triggering
			timer1 = timer1 % 86400; // 86400s = 24h
			timer2 = timer2 % 86400;
			now = now % 86400;

			vTaskDelay(5);

			// logika obychnogo tajmera dlja timerOn<timerOff
			if (now < timer2 && now > timer1) {
				timerStateFlag = 1;
			} else {
				timerStateFlag = 0;
			}

			// obratnaja logika tajmera
			if (timer1 > timer2) {
				if (now < timer1 && now > timer2) {
					timerStateFlag = 0;
				} else {
					timerStateFlag = 1;
				}
			}

			if (timerStateFlag == 1) {
				timerStateFlags |= (1 << i); // set flag
			} else {
				timerStateFlags &= ~(1 << i); // sbrosit' flag
			}
		}
		vTaskDelay(5);
		for (i = 0; i < 3; i++) { // do 3 tajmerov
			Address = EE_CTIMER_DURATION + EE_CTIMER_SIZE * i;
			timer1 = EE_ReadWord(Address);
			timer2 = EE_ReadWord(Address + 2);
			vTaskDelay(5);
			now = RTC_GetCounter();

			// logika ciklicheskogo tajmera
			now %= timer2;
			if (now < timer1) {
				timerStateFlag = 1;
				cTimerStateFlags |= (1 << i);
			} else {
				timerStateFlag = 0;
				cTimerStateFlags &= ~(1 << i); // sbrosit' flag
			}
			vTaskDelay(5);
		}
	}
}

void plugStateTrigger(void *pvParameters) {
	uint8_t plugStateFlag = 0;
	uint8_t plugTimerId = 0;
	uint8_t plugType = 0;
	uint8_t i = 0;
	while (1) {
		if (runners > 250) {
			runners = 0;
		}
		runners++;
		if (comm_state != COMM_DIRECT_DRIVE) {
			for (i = 0; i < PLUG_AMOUNT; i++) { // PC0 to PC2
				plugType = 0;
				plugTimerId = plugSettings[i]; // get the ID of timer for this plug
				if (plugTimerId >= 0 && plugTimerId <= 31) {
					plugStateFlag = timerStateFlags & (1 << plugTimerId); // check if timer active now
					plugStateFlag >>= plugTimerId; // ostavit' toka flag
					plugStateSet(i, plugStateFlag);
				}
				if (plugTimerId > 63 && plugTimerId < 67) { // ph up
					plugType = 2;
					plugTimerId -= 32;
				}
				if (plugTimerId > 66 && plugTimerId < 69) { // ph up
					plugType = 3;
					plugTimerId -= 35;
				}

				if (plugTimerId > 69 && plugTimerId < 72) { // ph up
					plugType = 4;
				}
				if (plugTimerId == 80) { // 80 - PSI stab
					plugType = 5;
				}

				if (plugTimerId == 99) { //always on
					plugType = 255;
					plugStateSet(i, 1);
				}
				if (plugTimerId == 98) { // always off
					plugType = 254;
					plugStateSet(i, 0);
				}

				if (plugTimerId > 31 && plugTimerId < 64) {
					plugTimerId -= 32;
					plugStateFlag = cTimerStateFlags & (1 << plugTimerId); // check if timer active now
					plugStateFlag >>= plugTimerId; // ostavit' toka flag
				}

				if (plugType == 2) {
					if (plugStateFlag == 0
							&& ((plugStateFlags >> i) & 1) == 1) {
						plugStateSet(i, 0); // disable plug
					}
				} else if (plugType == 3) { // mister (or another humidity "upper")
					if (plugStateFlag == 1 && ((plugStateFlags >> i) & 1) == 0
							&& plugTimerId == 0 && rhUnderOver == 1) {
						plugStateSet(i, 1); // enable mister for underwindow
					}
					if (plugStateFlag == 1 && ((plugStateFlags >> i) & 1) == 0
							&& plugTimerId == 0 && rhUnderOver == 0) {
						plugStateSet(i, 1); // enable mister for in-window
					}
					if (plugStateFlag == 0
							&& ((plugStateFlags >> i) & 1) == 1) {
						plugStateSet(i, 0); // disable plug
					}
				} else if (plugType == 4) {
					// watering and circulation pumps for watering controller
				} else if (plugType == 5) {
					/*	if (auto_flags&1==1 && (auto_failures&1)==0) {			// PSI STAB flag is number 0
					 if (adcAverage[AVG_ADC_PSI]>psi_pump_top_level) {
					 plugStateSet(PSI_PUMP_ID, 0);
					 }
					 vTaskDelay(1);
					 if (adcAverage[AVG_ADC_PSI]<psi_pump_btm_level) {
					 plugStateSet(PSI_PUMP_ID, 1);
					 }
					 }	*/
					// psi pump pressure stabilizer
				}
				if (plugType == 254) {

				}
				if (plugType == 255) {

				} else {
				}
				vTaskDelay(1);
			}
		}
		vTaskDelay(5);
	}
}

void psiSetup(void) { // REMOVE !!!
/*	uint8_t curbutton=0, tmp;
 uint16_t curpsiadc=0, tmp2=0, tempvalue=0;
 comm_state=COMM_DIRECT_DRIVE;
 Lcd_clear();
 //	tmp2 = EE_PLUG_SETTINGS+PSI_PUMP_ID;
 EE_WriteVariable(tmp2, 80);		// HARDCODE!!! program (timer) id for booster pump
 plugSettings[tempvalue] = 80;
 Lcd_write_str("ADC psi reading");
 while (curbutton!=BUTTON_FWD){
 Lcd_goto(1,0);
 Lcd_write_str("TOP:");
 if (curbutton==BUTTON_BCK){
 //		plugStateSet(PSI_PUMP_ID, 1);
 }
 if (curbutton==BUTTON_OK){
 //			plugStateSet(PSI_PUMP_ID, 0);
 }
 if (curbutton==BUTTON_CNL){
 psi_pump_top_level = adcAverage[AVG_ADC_PSI];
 }
 vTaskDelay(25);
 Lcd_write_16b(adcAverage[AVG_ADC_PSI]);
 Lcd_write_str("/");
 Lcd_write_16b(psi_pump_top_level);
 curbutton=readButtons();
 }
 //	plugStateSet(PSI_PUMP_ID, 0);
 printOk();
 curbutton=0;
 while (curbutton!=BUTTON_FWD){
 Lcd_goto(1,0);
 Lcd_write_str("BTM:");
 if (curbutton==BUTTON_BCK){
 //			plugStateSet(PSI_PUMP_ID, 1);
 }
 if (curbutton==BUTTON_OK){
 //			plugStateSet(PSI_PUMP_ID, 0);
 }
 if (curbutton==BUTTON_CNL){
 psi_pump_btm_level = adcAverage[AVG_ADC_PSI];
 }
 vTaskDelay(10);
 Lcd_write_16b(adcAverage[AVG_ADC_PSI]);
 Lcd_write_str("/");
 Lcd_write_16b(psi_pump_btm_level);
 curbutton=readButtons();
 }
 //	plugStateSet(PSI_PUMP_ID, 0);
 comm_state=COMM_MONITOR_MODE;
 EE_WriteVariable(PSI_SENSOR_TOP, psi_pump_top_level);
 EE_WriteVariable(PSI_SENSOR_BTM, psi_pump_btm_level);
 printOk(); */
}

void I2C_init2(void) {
	// STMicroelectronics IORoutines example

	// NVIC_Configuration()
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_SetPriority(I2C1_EV_IRQn, 0x01);
	NVIC_EnableIRQ(I2C1_EV_IRQn);

	NVIC_SetPriority(I2C1_ER_IRQn, 0x02);
	NVIC_EnableIRQ(I2C1_ER_IRQn);

	I2C_LowLevel_Init(I2C1); // make an init on low level

	NVIC_SetPriority(I2C2_EV_IRQn, 0x01);
	NVIC_EnableIRQ(I2C2_EV_IRQn);

	NVIC_SetPriority(I2C2_ER_IRQn, 0x02);
	NVIC_EnableIRQ(I2C2_ER_IRQn);

	I2C_LowLevel_Init(I2C2);

	/*	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	 GPIO_InitTypeDef GPIO_InitStructure;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	 I2C_InitTypeDef I2C_InitStructure;
	 I2C_StructInit(&I2C_InitStructure);
	 I2C_InitStructure.I2C_ClockSpeed = 100000;
	 I2C_InitStructure.I2C_OwnAddress1 = 0x01;
	 I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	 I2C_Init(I2C1, &I2C_InitStructure);
	 I2C_Cmd(I2C1, ENABLE); */
}

void I2C2_init(void) {
	NVIC_SetPriority(I2C2_EV_IRQn, 0x01);
	NVIC_EnableIRQ(I2C2_EV_IRQn);

	NVIC_SetPriority(I2C2_ER_IRQn, 0x02);
	NVIC_EnableIRQ(I2C2_ER_IRQn);

	I2C_LowLevel_Init(I2C2);
}

void I2C_single_write(uint8_t HW_address, uint8_t addr, uint8_t data) {
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		;
	I2C_Send7bitAddress(I2C1, HW_address, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;
	I2C_SendData(I2C1, addr);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
	I2C_SendData(I2C1, data);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
	I2C_GenerateSTOP(I2C1, ENABLE);
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		;
}

uint8_t I2C_single_read(uint8_t HW_address, uint8_t addr) {
	uint8_t data;
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		;
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		;
	I2C_Send7bitAddress(I2C1, HW_address, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;
	I2C_SendData(I2C1, addr);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		;
	I2C_Send7bitAddress(I2C1, HW_address, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
		;
	data = I2C_ReceiveData(I2C1);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
		;
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		;
	return data;
}

/*void delay()
 {
 for(volatile uint32_t del = 0; del<250000; del++){

 }
 } */

// sends a value 0..255 via I2C
void i2c_send_byte_test(uint8_t val) {

}

void i2c_rc_test(void) {
	uint8_t curbyte = 0;
	uint8_t curbutton = 0;
	comm_state = COMM_DIRECT_DRIVE;
	while (curbutton != BUTTON_FWD) {
		Lcd_goto(0, 0);
		Lcd_write_str("Byte:");
		Lcd_write_digit(curbyte);
		if (curbutton == BUTTON_BCK) {
			curbyte++;
			if (curbyte == PLUG_AMOUNT) {
				curbyte = 0;
			}
		}
		if (curbutton == BUTTON_OK) {
			plugStateSet(1, 1);
			i2c_send_byte_test(curbyte);
		}
		if (curbutton == BUTTON_CNL) {
			plugStateSet(1, 0);
		}
		vTaskDelay(10);
		curbutton = readButtons();
	}
	comm_state = COMM_MONITOR_MODE;
}

void plugTest(void) {
	uint8_t curplug = 0;
	uint8_t curbutton = 0;
	comm_state = COMM_DIRECT_DRIVE;
	while (curbutton != BUTTON_FWD) {
		Lcd_goto(0, 0);
		Lcd_write_str("Plug:");
		Lcd_write_digit(curplug);
		if (curbutton == BUTTON_BCK) {
			curplug++;
			if (curplug == PLUG_AMOUNT) {
				curplug = 0;
			}
		}
		if (curbutton == BUTTON_OK) {
			plugStateSet(curplug, 1);
		}
		if (curbutton == BUTTON_CNL) {
			plugStateSet(curplug, 0);
		}
		vTaskDelay(10);
		curbutton = readButtons();
	}
	comm_state = COMM_MONITOR_MODE;
}

void plugStateSet(uint8_t plug, uint8_t state) {
#ifdef GROLLY
	if (plug == 0) {
		state = 1 - state; // invert. PSI pump is "0" driven
	}
#endif
	if (state == 1) {
		PLUG_DISABLE = (1 << plug);
		plugStateFlags |= (1 << plug);
	} else {
		PLUG_ENABLE = (1 << plug);
		plugStateFlags &= ~(1 << plug);
	}
}

void valveMotorStateSet(uint8_t valveId, uint8_t state) {
#ifdef USE_VALVES
#endif
}

void EE_WriteWord(uint16_t Address, uint32_t Data) {
	uint16_t tmp, tmp2;
	tmp2 = Data & 0xFFFF;
	tmp = Data >> 16;
	EE_WriteVariable(Address + 1, tmp2);
	EE_WriteVariable(Address, tmp);
}

void programRunner(uint8_t programId) {

	/*
	 * *
	 * |-MONITOR MODE 			/ 0
	 * |-TESTS					/ 1
	 * |	|-Valves			/ 2
	 * |	|-Temp&Humidity		/ 3
	 * |	|-EEPROM			/ 4
	 * |	|-Plugs				/ 5
	 * |	|-Dosers			/ 6
	 * |	|-rest				/ 7
	 * |-TIMERS					/ 8
	 * |	|-24H/Full Range	/ 9
	 * |	|-Cyclic			/ 10
	 * |-STABILIZERS			/ 11
	 * |	|-CO2				/ 12
	 * |	|-Pressure			/ 13
	 * |	|-Tank levels		/ 14
	 * |	|-Temperature		/ 15
	 * |	|-Humidity			/ 16
	 * |	|-pH				/ 17
	 * |-SETTINGS				/ 18
	 * |	|-Dosers			/ 19
	 * |	|-Plug settings		/ 20
	 * |	|-Set clock			/ 21
	 * |-WATERING				/ 22
	 * |	|-Watering progs	/ 23
	 * |	|-Fertilization		/ 24
	 */

	uint32_t tmp = 0;
	uint8_t tmp8 = 0;
	Lcd_clear();
	switch (programId) {
	case 1:
		break;
	case 2: // valve_test
		valve_test2();
		break;
	case 3: // temp. and humidity test
		dht_arr_displayer();
#ifdef	TEST_MODE
		displayAdcValues(); // test function to display ADC values
#endif
		display_usart_rx();
		display_usart_tx();
		break;
	case 4: // eeprom test
		eeprom_test();
		break;
	case 5: // plug test
		plugTest();
		break;
	case 6: // doser test
		break;
	case 7: // rest tests
		break;
	case 8: // 24h / full range timer setup
		Lcd_write_str("Timer to adjust:");
		tmp8 = idSelector(1, 4, 1);
		setTimer(--tmp8);
		break;
	case 9: // cyclic timer setup
		Lcd_write_str("CTimer 2 adjust:");
		tmp8 = idSelector(1, 4, 1);
		setCTimer(--tmp8);
		break;
	case 10: // co2 stabilizer
//		co2_setup();
		break;
	case 11: // pressure stabilizer setup
		break;
	case 13: // tank level stabilizer
		tankLevelStabSetup();
		break;
	case 14: // temp stab setup
		break;
	case 15: // humidity stab setup
		hygroStatSettings();
		break;
	case 16: // pH stab setup
//		phStabSettings();
		break;
	case 17: // doser settings
		break;
	case 18: // plug settings
		Lcd_write_str("Choose plug");
		tmp8 = idSelector(1, 4, 1);
		setPlug(--tmp8); // decrement needed because of actual start from 0
		break;
	case 19: // set clock
		tmp = RTC_GetCounter();
		uint32_t unixtime = timeAdjust(tmp, 1);
		RTC_SetCounter(unixtime);
		Lcd_clear();
		break;
	case 20:
		break;
	case 21:
		watering_setup();
		break;
	case 22:
		fertilization_setup();
		break;
	case 23:
		get_water(0, 0, 100);
		break;
	case 24:
		break;
	case 25:
		break;
	case 26:
		break;
	case 27:
		startWp();
		break;
	case 28:
		break;
	}
}

void loggerSettings(void) {
	uint32_t logSetting;
	vTaskDelay(10);
	Lcd_clear();
	EE_ReadVariable(SD_LOG_INTERVAL, &logSetting);
	vTaskDelay(100);
	Lcd_goto(0, 0);
	Lcd_write_str("Log frequency");
	vTaskDelay(3000);
	logSetting = adjust8bit(logSetting); // poluchit' novoe znachenie ot user'a
	EE_WriteVariable(SD_LOG_INTERVAL, logSetting);
	vTaskDelay(50);
	Lcd_clear();
	Lcd_goto(0, 2);
	Lcd_write_str("Complete!");
	vTaskDelay(500);
	loadSettings();
	Lcd_clear();
}

void phMonSettings(void) {
	uint16_t phSetting = 0;
	vTaskDelay(10);
	Lcd_clear();
	EE_ReadVariable(PH_INTERVAL, &phSetting);
	vTaskDelay(100);
	Lcd_goto(0, 0);
	Lcd_write_str("Probing interval");
	vTaskDelay(1000);
	phSetting = adjust8bit(phSetting); // poluchit' novoe znachenie ot user'a
	EE_WriteVariable(PH_INTERVAL, phSetting);
	vTaskDelay(50);

	// analogichno dlja buffer size
	EE_ReadVariable(PH_BUFF_SIZE, &phSetting);
	Lcd_clear();
	Lcd_goto(0, 2);
	Lcd_write_str("Buffer size");
	vTaskDelay(500);
	phSetting = adjust8bit(phSetting); // poluchit' novoe znachenie ot user'a
	EE_WriteVariable(PH_BUFF_SIZE, phSetting);
	Lcd_clear();
	Lcd_goto(0, 2);
	Lcd_write_str("Complete!");
	vTaskDelay(500);
	loadSettings();
	Lcd_clear();
}

uint8_t adjust8bit(uint8_t val) {
	if (val > 99) {
		val = 99;
	}
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		vTaskDelay(25);
		if (button == BUTTON_BCK) {
			if (val < 2) {
				val = 255;
			} else {
				val--;
			}
		}
		if (button == BUTTON_FWD) {
			if (val > 254) {
				val = 1;
			} else {
				val++;
			}
		}
		Lcd_goto(1, 0);
		Lcd_write_str("< ");
		Lcd_write_8b(val);
		Lcd_write_str(" >");
		vTaskDelay(25);
	}
	printOk();
	return val;
}

uint16_t adjust16bit(uint16_t val) {
	if (val > 65534) {
		val = 0;
	}
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		vTaskDelay(25);
		if (button == BUTTON_BCK) {
			if (val < 1) {
				val = 65535;
			} else {
				val--;
			}
		}
		if (button == BUTTON_FWD) {
			if (val > 65534) {

				val = 0;
			} else {
				val++;
			}
		}
		Lcd_goto(1, 4);
		Lcd_write_str("< ");
		Lcd_write_16b(val);
		Lcd_write_str(" >");
		vTaskDelay(25);
	}
	printOk();
	return val;
}

uint16_t adjust16bit_fast(uint16_t val, uint8_t speed) {
	char buffer[11];
	if (val > 65534) {
		val = 0;
	}
	vTaskDelay(200);
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		vTaskDelay(speed);
		if (button == BUTTON_BCK) {
			if (val < 1) {
				val = 65535;
			} else {
				val--;
			}
		}
		if (button == BUTTON_FWD) {
			if (val > 65534) {

				val = 0;
			} else {
				val++;
			}
		}
		Lcd_goto(1, 4);
		Lcd_write_str("< ");
		Lcd_write_16b(val);
		Lcd_write_str(" >");
		vTaskDelay(speed);
	}
	printOk();
	return val;
}

uint32_t CTimerAdjust(uint32_t time) {
	uint8_t hours, minutes, seconds, hours2, minutes2, seconds2, button;
	char timestr[6];
	seconds = time % 60;
	time /= 60;
	minutes = time % 60;
	time /= 60;
	hours = time % 99;
	Lcd_clear();
	vTaskDelay(20);
	while (button != BUTTON_FWD) {
		button = readButtons();
		if (button == BUTTON_BCK) {
			if (hours > 98) {
				hours = 0;
			} // chtoby pechatat' pobolee znakov, nado zamenit' Lcd_write_digit()
			else {
				hours++;
			}
		}
		if (button == BUTTON_OK) {
			if (minutes > 58) {
				minutes = 0;
			} else {
				minutes++;
			}
		}
		if (button == BUTTON_CNL) {
			if (seconds > 58) {
				seconds = 0;
			} else {
				seconds++;
			}
		}

		vTaskDelay(5);
		Lcd_goto(0, 4);
		Lcd_write_digit(hours);
		Lcd_write_str(":");
		Lcd_write_digit(minutes);
		Lcd_write_str(":");
		Lcd_write_digit(seconds);
		vTaskDelay(5);
		hours2 = hours; // daby izbezhat' konversii tipov pri umnozhenii na 3600 i podobnom
		minutes2 = minutes;
		seconds2 = seconds;
		time = hours2 * 3600;
		time += minutes2 * 60;
		time += seconds2;
		vTaskDelay(5);
		Lcd_goto(1, 3);
		int32str(time, &timestr);
		copy_arr(&timestr, LCDLine2, 10, 2, 0);
		vTaskDelay(10);
	}
	printOk();
	return (time);
}

void setCTimer(uint8_t timerId) {
	uint16_t Address;
	uint32_t CTimerData;
	vTaskDelay(10);
	Lcd_clear();
	Address = EE_CTIMER_DURATION + timerId * EE_CTIMER_SIZE;
	CTimerData = EE_ReadWord(Address);
	if (CTimerData > 356400) {
		CTimerData = 356400;
	}
	Lcd_goto(0, 2);
	Lcd_write_str("Set DURATION");
	vTaskDelay(3000);
	CTimerData = CTimerAdjust(CTimerData); // poluchit' novoe znachenie ot user'a
	Address = EE_CTIMER_DURATION + timerId * EE_CTIMER_SIZE; // adres postojanno "napominaetsja", potomu chto po-hodu ispolnenija koda on kuda-to "terjaetsja". bylo by neploho razobratsja kuda
	EE_WriteWord(Address, CTimerData);
	vTaskDelay(50);

	printOk();

	// analogichno dlja INTERVALa
	Address = EE_CTIMER_INTERVAL + timerId * EE_CTIMER_SIZE;
	CTimerData = EE_ReadWord(Address);
	Lcd_clear();
	Lcd_goto(0, 2);
	Lcd_write_str("Set INTERVAL");
	vTaskDelay(500);
	CTimerData = CTimerAdjust(CTimerData); // poluchit' novoe znachenie ot user'a
	Address = EE_CTIMER_INTERVAL + timerId * EE_CTIMER_SIZE;
	EE_WriteWord(Address, CTimerData);
	printOk();
}

uint8_t idSelector(uint8_t min, uint8_t max, uint8_t curid) {
	button = 0;
	while (button != BUTTON_OK) {
		vTaskDelay(25);
		Lcd_goto(1, 0);
		Lcd_write_digit(curid);
		if (button == BUTTON_FWD) {
			if (curid < max) {
				curid++;
			}
		}
		if (button == BUTTON_BCK) {
			if (curid > min) {
				curid--;
			}
		}
		button = readButtons();
		vTaskDelay(25);
	}
	printOk();
	return curid;
}

void setTimer(uint8_t timerId) {
	uint32_t Data, adjusteDate;
	uint16_t Address;
	Address = EE_TIMER1_ON + timerId * EE_TIMER_SIZE; // set ON for plain timer
	Lcd_clear();
	Lcd_goto(0, 2);
	Lcd_write_str("Set ON time");
	vTaskDelay(1500);
	Lcd_clear();
	vTaskDelay(50);
	Data = EE_ReadWord(Address);
	vTaskDelay(50);
	adjusteDate = timeAdjust(Data, 1);
	EE_WriteWord(Address, adjusteDate);
	Lcd_goto(0, 2);
	Lcd_write_str("Set OFF time");
	vTaskDelay(50);
	vTaskSuspendAll();
	Address = EE_TIMER1_ON + timerId * EE_TIMER_SIZE + 2; // set OFF for plain timer
	Data = EE_ReadWord(Address);
	xTaskResumeAll();
	vTaskDelay(1500);
	Lcd_clear();
	adjusteDate = timeAdjust(Data, 1);
	EE_WriteWord(Address, adjusteDate);
	vTaskSuspendAll();
	Address = EE_TIMER1_ON + timerId * EE_TIMER_SIZE + 4; // set daily flag
	Data = EE_ReadWord(Address);
	xTaskResumeAll();
	vTaskDelay(50);
}

uint8_t yesNoSelector(char str, uint8_t curval) {
	Lcd_clear();
	Lcd_goto(0, 0);
	Lcd_write_str(str);
	vTaskDelay(50);
	vTaskDelay(50);
	button = 0;
	while (button != BUTTON_OK) {
		if (button == BUTTON_FWD || button == BUTTON_BCK) {
		}
		vTaskDelay(50);
	}
	return (curval);
}

uint32_t timeAdjust(uint32_t cnt, uint8_t includeDays) {
	Lcd_clear();
	uint32_t unixtime2;
	RTC_DateTime curtime, curtime2;
	char unixtimestr[11];
	curtime = unix2DateTime(cnt);

	while (button != BUTTON_FWD) {
		button = readButtons();
		if (button == BUTTON_BCK) {
			if (curtime.hour > 22) {
				curtime.hour = 0;
			} else {
				curtime.hour++;
			}
		}
		if (button == BUTTON_OK) {
			if (curtime.min > 58) {
				curtime.min = 0;
			} else {
				curtime.min++;
			}
		}
		if (button == BUTTON_CNL) {
			if (curtime.sec > 58) {
				curtime.sec = 0;
			} else {
				curtime.sec++;
			}
		}
		vTaskDelay(5);
		Lcd_goto(0, 0);
		Lcd_write_digit(curtime.hour);
		Lcd_write_str(":");
		Lcd_write_digit(curtime.min);
		Lcd_write_str(":");
		Lcd_write_digit(curtime.sec);
#ifdef	TEST_MODE
		unixtime2 = DateTime2unix(curtime);
		curtime2 = unix2DateTime(unixtime2);
		int32str(unixtime2, &unixtimestr);
		Lcd_goto(0, 10);
		Lcd_write_digit(curtime2.hour);
		Lcd_write_digit(curtime2.min);
		Lcd_write_digit(curtime2.sec);
		Lcd_goto(1, 2);
		vTaskDelay(5);
		copy_arr(&unixtimestr, LCDLine2, 10, 2, 0);
#endif
		vTaskDelay(20);
	}
	button = 0;
	printOk();
	if (includeDays == 1) {
		while (button != BUTTON_FWD) {
			button = readButtons();
			if (button == BUTTON_CNL) {
				if (curtime.year > 50) {
					curtime.year = 12;
				} else {
					curtime.year++;
				}
			}
			if (button == BUTTON_OK) {
				if (curtime.month > 11) {
					curtime.month = 1;
				} else {
					curtime.month++;
				}
			}
			if (button == BUTTON_BCK) {
				if (curtime.day > 30) {
					curtime.day = 1;
				} else {
					curtime.day++;
				}
			}

			vTaskDelay(5);
#ifdef	TEST_MODE
			// preobrazovanija tuda-sjuda dlja togo chtoby ubeditsja, chto funkcii rabotajut verno
			unixtime2 = DateTime2unix(curtime);
			curtime2 = unix2DateTime(unixtime2);
			int32str(unixtime2, &unixtimestr);
#endif
			vTaskDelay(5);
			Lcd_goto(0, 0);
			Lcd_write_digit(curtime.day);
			Lcd_write_str("-");
			Lcd_write_digit(curtime.month);
			Lcd_write_str("-");
			Lcd_write_digit(curtime.year);
			vTaskDelay(5);
#ifdef	TEST_MODE
			Lcd_goto(0, 10);
			Lcd_write_digit(curtime2.day);
			Lcd_write_digit(curtime2.month);
			Lcd_write_digit(curtime2.year);
			vTaskDelay(5);
			copy_arr(&unixtimestr, LCDLine2, 10, 2, 0);
#endif
			vTaskDelay(50);
		}
	} else {
		unixtime2 = 0;
		unixtime2 += curtime.hour * 3600;
		unixtime2 += curtime.min * 60;
		unixtime2 += curtime.sec;
	}
	printOk();
	return (unixtime2);
}

void Lcd_write_arr2(uc8 *STRING, uint8_t chars) {
	char c;
	uint8_t i = 0;
	for (i = 0; i < chars; i++) {
		c = STRING[i];
//		vTaskDelay(5);
		Lcd_write_data(c);
	}
}

void Lcd_write_arr(volatile uint8_t *STRING, uint8_t chars) {
	uint8_t c = 0;
	uint8_t i = 0;
	for (i = 0; i < chars; i++) {
		c = STRING[i];
		// Lcd_write_data(c);
		LCDI2C_send(c, Rs);
	}
}

RTC_DateTime unix2DateTime(uint32_t unixtime) {
	RTC_DateTime datetime;
	uint32_t tmp;
	uint8_t leaps, month, day;
	uint16_t daysFromLeap, dayOfYearLeaped;
	tmp = unixtime - YEAR12SECS; // vse raschety vedutsja ot 01.01.2012 00:00:00

	datetime.sec = unixtime % 60;
	unixtime /= 60;
	datetime.min = unixtime % 60;
	unixtime /= 60;
	datetime.hour = unixtime % 24;
	tmp /= 86400;
	daysFromLeap = tmp % 1461; // 86400*1461=126230400 secs in leap cycle
	leaps = tmp / 1461;
	daysFromLeap++;

	if (daysFromLeap < 59) // 1..60
			{
		datetime.year = leaps * 4 + 12;
		dayOfYearLeaped = daysFromLeap + 1; // for shifting days because of 29.03

	}
	if (daysFromLeap >= 59 && daysFromLeap <= 366) // 61..366
			{
		datetime.year = leaps * 4 + (daysFromLeap - 1) / 365 + 12;
		dayOfYearLeaped = daysFromLeap;
	}
	if (daysFromLeap >= 366) { // 367..1461
		datetime.year = leaps * 4 + (daysFromLeap - 1) / 365 + 12;
		dayOfYearLeaped = ((daysFromLeap - 366) % 365) + 1;
	}

	if (dayOfYearLeaped >= 1 && dayOfYearLeaped <= 31) {
		month = 1; // january
		day = dayOfYearLeaped;
	}
	if (dayOfYearLeaped >= 32 && dayOfYearLeaped <= 59) {
		month = 2; // february
		day = dayOfYearLeaped - 31;
	}
	if (dayOfYearLeaped >= 60 && dayOfYearLeaped <= 90) {
		month = 3; // march
		day = dayOfYearLeaped - 59;
	}
	if (dayOfYearLeaped >= 91 && dayOfYearLeaped <= 120) {
		month = 4; // april
		day = dayOfYearLeaped - 90;
	}
	if (dayOfYearLeaped >= 121 && dayOfYearLeaped <= 151) {
		month = 5; // may
		day = dayOfYearLeaped - 120;
	}
	if (dayOfYearLeaped >= 152 && dayOfYearLeaped <= 181) {
		month = 6; // june
		day = dayOfYearLeaped - 151;
	}
	if (dayOfYearLeaped >= 182 && dayOfYearLeaped <= 212) {
		month = 7; // july
		day = dayOfYearLeaped - 181;
	}
	if (dayOfYearLeaped >= 213 && dayOfYearLeaped <= 243) {
		month = 8; // august
		day = dayOfYearLeaped - 212;
	}
	if (dayOfYearLeaped >= 244 && dayOfYearLeaped <= 273) {
		month = 9; // september
		day = dayOfYearLeaped - 243;
	}
	if (dayOfYearLeaped >= 274 && dayOfYearLeaped <= 304) {
		month = 10; // october
		day = dayOfYearLeaped - 273;
	}
	if (dayOfYearLeaped >= 305 && dayOfYearLeaped <= 334) {
		month = 11; // november
		day = dayOfYearLeaped - 304;
	}
	if (dayOfYearLeaped >= 335 && dayOfYearLeaped <= 365) {
		month = 12; // december
		day = dayOfYearLeaped - 334;
	}
	datetime.day = day;
	datetime.month = month;
	if (daysFromLeap == 59) {
		datetime.day = 29;
		datetime.month = 2;
	}
	return (datetime);
}

uint32_t DateTime2unix(RTC_DateTime datetime) {
	uint32_t tmp;
	uint16_t days, dayFromYear;

	switch (datetime.month) {
	case 0:
		days = 0;
	case 1:
		days = 0;
		break;
	case 2:
		days = 31;
		break;
	case 3:
		days = 59;
		break;
	case 4:
		days = 90;
		break;
	case 5:
		days = 120;
		break;
	case 6:
		days = 151;
		break;
	case 7:
		days = 181;
		break;
	case 8:
		days = 212;
		break;
	case 9:
		days = 243;
		break;
	case 10:
		days = 273;
		break;
	case 11:
		days = 304;
		break;
	case 12:
		days = 334;
		break;
	}

	dayFromYear = days + datetime.day;

	if (dayFromYear < 60 && (datetime.year % 4) == 0) {
		dayFromYear--;
	}
	if (datetime.month == 2 && datetime.day >= 29 && (datetime.year % 4) == 0) {
		dayFromYear--;
	}
	if (dayFromYear < 1) {
		dayFromYear = 1;
	}
	tmp = ((datetime.year - 12) * 365 * 86400);
	tmp += (datetime.hour * 3600);
	tmp += (datetime.min * 60);
	tmp += (datetime.sec + YEAR12SECS);
	tmp += ((dayFromYear - 2) * 86400);
	tmp += (((datetime.year - 8) / 4) * 86400);
	return tmp;
}

// function returns the programId selected in menu to run the program
uint8_t menuSelector(void) {
	uint8_t curItem = 0; // default item to display entering the menu
	uint8_t programId = 0;
	uint8_t textId = fatArray[curItem][1];
	uint8_t button = 0;
	while (programId == 0) {
		textId = fatArray[curItem][1];
		Lcd_goto(0, 0);
		Lcd_write_str(menuItemArray[textId]);
		button = readButtons();
		if (button > 0) {
			Lcd_clear();
			vTaskDelay(100);
		}
		vTaskDelay(10);
		if (button == BUTTON_OK) {
			if (fatArray[curItem][6] == 1) // run program
					{
				programId = fatArray[curItem][4];
			} else {
				curItem = fatArray[curItem][4];
			} // enter folder
			Lcd_clear();
		} else if (button == BUTTON_CNL) {
			curItem = fatArray[curItem][5];
			Lcd_clear();
		} else if (button == BUTTON_BCK) {
			curItem = fatArray[curItem][2];
			Lcd_clear();
		} else if (button == BUTTON_FWD) {
			curItem = fatArray[curItem][3];
			Lcd_clear();
		}
	}
	vTaskDelay(1);
	return (programId);
}

// mycontroller.ru code

uint32_t RTC_GetCounter(void) {
	return (uint32_t)((RTC->CNTH << 16) | RTC->CNTL);
}

void RTC_SetCounter(uint32_t value) {
	RTC->CRL |= RTC_CRL_CNF;
	RTC->CNTH = value >> 16;
	RTC->CNTL = value;
	RTC->CRL &= ~RTC_CRL_CNF;
}

unsigned char RtcInit(void) {

	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;

	PWR->CR |= PWR_CR_DBP;

	if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN) {
		RCC->BDCR |= RCC_BDCR_BDRST;
		RCC->BDCR &= ~RCC_BDCR_BDRST;
		RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE;
		RTC->CRL |= RTC_CRL_CNF;
		RTC->PRLL = 0x7FFF; // divider to 32786 hZ
		RTC->CRL &= ~RTC_CRL_CNF;
		RCC->BDCR |= RCC_BDCR_LSEON;
		while ((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON) {
		}

		RTC->CRL &= (uint16_t) ~RTC_CRL_RSF;
		while ((RTC->CRL & RTC_CRL_RSF) != RTC_CRL_RSF) {
		}
		return 1;
	}
	return 0;
}

uint32_t TimeToRtc(RTC_Time *time) {
	uint32_t result;
	result = (uint32_t) time->hour * 3600;
	result += (uint32_t) time->min * 60;
	result += time->sec;
	return result;
}

void RtcToTime(uint32_t cnt, RTC_Time *time) {
	time->sec = cnt % 60;
	cnt /= 60;
	time->min = cnt % 60;
	cnt /= 60;
	time->hour = cnt % 24;
}

// EOF mycontroller.ru RTC functions

// function slides the buffer window for pH ADC values (and EC)
void phMonitor(void *pvParameters) {
	while (1) {
		vTaskDelay(1);
		adcAverager();
	}
}

void copy_arr(volatile uint8_t *source, volatile uint8_t *destination,
		uint8_t amount, uint8_t pos, uint8_t src_pos) {
	uint8_t i = 0;
	for (i = 0; i < amount; i++) {
		destination[(i + pos)] = source[(i + src_pos)];
	}
}

// copy_arr(&RxBuffer, &wt_args, 7, 0, 2);

void psi_spin_up(void) {
	PSI_PUMP_TIM->CCR1 = 1000
			- (PSI_M_HIGH & (uint16_t)(0xFF)) * (uint16_t) 200;
	vTaskDelay(1);
	spf = 1;
}

uint8_t skip_button_cal(void) {
	uint32_t skipend = 0;
	Lcd_clear();
	Lcd_write_str("Skip button cal?");
	vTaskDelay(2000);
	button = readButtons();
	skipend = RTC_GetCounter() + 3;
	while (skipend > RTC_GetCounter() && button != BUTTON_OK) {
		button = readButtons();
		vTaskDelay(50);
	}
	Lcd_clear();
	if (button == BUTTON_OK) {
		Lcd_write_str("Skipping");
		vTaskDelay(500);
		Lcd_clear();
		return 1;
	} else {
		vTaskDelay(500);
		Lcd_clear();
		return 0;
	}

}

void displayClock(void *pvParameters) {
	RTC_DateTime DateTime;
	uint32_t tmp = 0;
	uint8_t tempn = 0;
	tmp = RTC_GetCounter();
	fup_time = RTC_GetCounter() + 60;
	if (tmp < 1388534400) {
		RTC_SetCounter(1400000000);
	}
	Lcd_clear();

//		displayAdcValues();

	Lcd_write_str("Hello Buddy :)");
	vTaskDelay(200);
	Lcd_goto(1, 0);
	Lcd_write_str("Let's GROW!");
	vTaskDelay(200);
	Lcd_clear();
	if (skip_button_cal() == 0) {
		Lcd_clear();
		Lcd_write_str("First, calibrate");
		Lcd_goto(1, 0);
		Lcd_write_str("the buttons...");
		vTaskDelay(2000);
		buttonCalibration();
	}
	while (1) {

		button = readButtons();
		vTaskDelay(3);
		vTaskDelay(5);
		if (enableClock == 1) {
			runners = 24;
			vTaskDelay(10);
			tmp = RTC_GetCounter();
			DateTime = unix2DateTime(tmp);
			LCDLine1[0] = (DateTime.day / 10) + 48;
			LCDLine1[1] = (DateTime.day % 10) + 48;
			LCDLine1[2] = 45;
			LCDLine1[3] = (DateTime.month / 10) + 48;
			LCDLine1[4] = (DateTime.month % 10) + 48;
			LCDLine1[5] = 45;
			LCDLine1[6] = (DateTime.year / 10) + 48;
			LCDLine1[7] = (DateTime.year % 10) + 48;
			LCDLine2[6] = 32;
			LCDLine2[7] = 32;
			Lcd_goto(0, 8);
			//Lcd_write_8b(dht_byte_buf[0]);
			//Lcd_write_8b(dht_byte_buf[1]);
			Lcd_goto(1, 0);
			Lcd_write_digit(DateTime.hour);
			Lcd_write_digit(DateTime.min);
			Lcd_write_digit(DateTime.sec);
			vTaskDelay(19);
			button = readButtons();
			vTaskDelay(3);
			Lcd_write_str(" ");
			Lcd_goto(1, 7);
			vTaskDelay(10);

#ifdef I2C_MASTER
//			    I2C_Master_BufferRead(I2C1,Buffer_Rx1,1,Polling, DEST_I2C_ADDR);
#endif
			curi2crxval = 10 + Buffer_Rx1[0];
//				Lcd_write_digit(Buffer_Tx1[0]&0xFF);
//				Lcd_write_digit(curi2crxval&0xFF);
//				Lcd_write_8b(sonar_read[0]);
//				Lcd_write_16b((sonar_read[1]-50)/10);
			/*Lcd_goto(1,8);
			 Lcd_write_8b(ds_buf[0]);
			 Lcd_write_8b(ds_buf[1]); */

			Lcd_goto(0, 9);
			//Lcd_write_16b(wrtn_val);
			Lcd_write_8b(resp_id);
			Lcd_write_8b(resp_counter);

			Lcd_goto(1, 9);
			//Lcd_write_16b(wrtn_val);
			Lcd_write_16b(sonar_read[1]);

			//Lcd_write_8b(packets_received);
			//Lcd_write_8b(rx_packet_crc);
			Lcd_goto(1, 7);
			Lcd_write_16b(wrtn_addr);
			//Lcd_write_8b(rxglobalcntr);
			//Lcd_write_8b(rx_packet_crc);
			//Lcd_write_8b(packetid_);
			Lcd_write_8b(packets_received);

			//copy_arr(&dht1_t_str, &LCDLine2, 4, 8, 0);
			/* copy_arr(&dht1_rh_str, &LCDLine1, 4, 12, 0);
			 copy_arr(&dht2_t_str, &LCDLine2, 4, 7, 0);
			 copy_arr(&dht2_rh_str, &LCDLine2, 4, 12, 0); */
			vTaskDelay(10);
			if (button == BUTTON_FWD) {
				//tempn++;
				vTaskDelay(10);
				Buffer_Tx1[0] = curi2ctxval++;
#ifdef I2C_MASTER
				// I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Interrupt, 0x40);
#endif
			}
			if (button == BUTTON_BCK) {
				//tempn++;
				vTaskDelay(10);
				Buffer_Tx1[0] = curi2ctxval--;
#ifdef I2C_MASTER
				// I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Interrupt, 0x40);
#endif
			}
			if (button == BUTTON_OK) {
				vTaskDelay(100);
				Lcd_clear();
				vTaskDelay(500);
				uint8_t progId = menuSelector();
				programRunner(progId);
			}
			vTaskDelay(10);
		}
		vTaskDelay(2);
	}
	while (1) {
	}
}

uint8_t readButtons(void) {
	uint16_t curval = 0;
	uint8_t i2 = 0;
	uint8_t i = 0;
	uint8_t a = 0;
	for (i2 = 0; i2 < 3; i2++) {
		adcAverager();
		curval = adcAverage[ADC_AVG_BUTTONS];
		vTaskDelay(1);
		for (i = 0; i < 4; i++) {
			if (curval > button_ranges[i * 2] + BUTTON_RANGE_SHRINKER
					&& curval < button_ranges[i * 2 + 1] - BUTTON_RANGE_SHRINKER) {
				a += (i + 1);
			}
		}
	}
	if (a % 3 == 0) {
		return (a / 3);
	} else {
		return 0;
	}
}

uint8_t readButtons2(void) {
	uint16_t curval = 0;
	uint8_t i;
	adcAverager();
	curval = adcAverage[ADC_AVG_BUTTONS];
	for (i = 0; i < 4; i++) {
		if (curval > button_ranges[i * 2] + BUTTON_RANGE_SHRINKER
				&& curval < button_ranges[i * 2 + 1] - BUTTON_RANGE_SHRINKER) {
			return (i + 1);
		}
	}
	return 0;
}

void AdcInit(void) {
	// WARNING NTBU (needs to be updated)! shifting pins down to 0 from 1st, saves some space on discovery board
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOC->CRL &= ~GPIO_CRL_CNF0; // LOAD triggering from PC0...
	GPIOC->CRL &= ~GPIO_CRL_CNF1;
	GPIOC->CRL &= ~GPIO_CRL_CNF2;
	GPIOC->CRL &= ~GPIO_CRL_CNF3;

	GPIOA->CRL &= ~GPIO_CRL_MODE0;
	GPIOA->CRL &= ~GPIO_CRL_CNF0;

	GPIOA->CRL &= ~GPIO_CRL_MODE1;
	GPIOA->CRL &= ~GPIO_CRL_CNF1;

	GPIOA->CRL &= ~GPIO_CRL_MODE2;
	GPIOA->CRL &= ~GPIO_CRL_CNF2;

	GPIOA->CRL &= ~GPIO_CRL_MODE3;
	GPIOA->CRL &= ~GPIO_CRL_CNF3;

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->CFGR &= ~RCC_CFGR_ADCPRE;
	ADC1->CR1 = 0;
	ADC1->CR2 |= ADC_CR2_CAL;
	while (!(ADC1->CR2 & ADC_CR2_CAL)) {
	};
	ADC1->CR2 = ADC_CR2_JEXTSEL;
	ADC1->CR2 |= ADC_CR2_JEXTTRIG;
	ADC1->CR2 |= ADC_CR2_CONT;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR1 |= ADC_CR1_JAUTO;
	ADC1->JSQR = (uint32_t)(4 - 1) << 20;
	ADC1->JSQR |= (uint32_t) 1 << (5 * 0);
	ADC1->JSQR |= (uint32_t) 2 << (5 * 1);
	ADC1->JSQR |= (uint32_t) 3 << (5 * 2);
	ADC1->JSQR |= (uint32_t) 4 << (5 * 3);
	ADC1->SMPR1 |= 0x00FFFFFF;
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC1->CR2 |= ADC_CR2_JSWSTART;
}

void Lcd_write_digit2(uint8_t numb) {
	if (numb < 10) {
		Lcd_write_data(48);
		Lcd_write_data(48 + numb);
	} else {
		Lcd_write_data((numb / 10) + 48);
		Lcd_write_data((numb - (numb / 10) * 10) + 48);
	}
}

void Lcd_write_digit(uint8_t numb) {
	if (lcd_pointery == 0) {
		LCDLine1[lcd_pointerx] = ((numb % 100) / 10) + 48;
		lcd_pointerx++;
		LCDLine1[lcd_pointerx] = (numb % 10) + 48;
		lcd_pointerx++;
	} else {
		LCDLine2[lcd_pointerx] = ((numb % 100) / 10) + 48;
		lcd_pointerx++;
		LCDLine2[lcd_pointerx] = (numb % 10) + 48;
		lcd_pointerx++;
	}
}

void prvSetupHardware() {
	// LOAD triggering control pins init
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Enable clock on GPIOC.
	GPIOC->CRL &= ~GPIO_CRL_CNF0; // LOAD triggering from PC0...
	GPIOC->CRL &= ~GPIO_CRL_CNF1;
	GPIOC->CRL &= ~GPIO_CRL_CNF2;
	GPIOC->CRL &= ~GPIO_CRL_CNF3; // ... to PC3

	GPIOC->CRL |= GPIO_CRL_MODE0_0;
	GPIOC->CRL |= GPIO_CRL_MODE1_0;
	GPIOC->CRL |= GPIO_CRL_MODE2_0;
	GPIOC->CRL |= GPIO_CRL_MODE3_0;
}

uint8_t tmpdata[1];

void vTaskLCDdraw(void *pvParameters) { // draws lcd
	tmpdata[0] = 0;
	uint32_t lcd_restart = 0;
	lcd_restart = RTC_GetCounter() + 20;
	for (;;) {
		vTaskSuspendAll();
		// hardware restart display if timeout reached
		if (RTC_GetCounter() > lcd_restart) {
			lcd_restart = RTC_GetCounter() + 60;
			// lcd_restart();
		}
		// vTaskSuspendAll();
//		Lcd_write_cmd(0x80);	// lcd_goto(0,0)
		LCDI2C_send(0x80, 0);
		Lcd_write_arr(&LCDLine1, 16);
//		Lcd_write_cmd(0x80+0x40);	// lcd_goto(1,0)
		LCDI2C_send((0x80 + 0x40), 0);
		Lcd_write_arr(&LCDLine2, 16);

		xTaskResumeAll();
		vTaskDelay(17);
	}
}

void lcd_restart(void) {
	power_ctrl(PWR_LCD, 0);
	I2C_DeInit(I2C1);
	Delay(25);
	power_ctrl(PWR_LCD, 1);
	Delay(25);
	I2C_LowLevel_Init(I2C1);
	// === LCD init ===

#ifdef LCD_I2C_DFROBOT
	LCDI2C_init(0x4E, 16, 2);
#endif

}

void int32str(uint32_t d, volatile char *out) {
	out[10] = '\0';
	out[9] = '0' + (d) % 10;
	out[8] = '0' + (d /= 10) % 10;
	out[7] = '0' + (d /= 10) % 10;
	out[6] = '0' + (d /= 10) % 10;
	out[5] = '0' + (d /= 10) % 10;
	out[4] = '0' + (d /= 10) % 10;
	out[3] = '0' + (d /= 10) % 10;
	out[2] = '0' + (d /= 10) % 10;
	out[1] = '0' + (d /= 10) % 10;
	out[0] = '0' + (d / 10) % 10;
}

uint32_t EE_ReadWord(uint16_t Address) {
	uint16_t tmp = 0, tmp2 = 0;
	uint32_t Data = 0;
	EE_ReadVariable(Address, &tmp);
	EE_ReadVariable(Address + 1, &tmp2);
	Data = ((uint32_t) tmp << 16) + (uint32_t) tmp2;
	return Data;
}

void flush_lcd_buffer(void) {
	uint8_t i = 0;
	for (i = 0; i < 16; i++) {
		LCDLine1[i] = 32;
		LCDLine2[i] = 32;
	}
}

void setPwmDc(uint8_t duty_cycle) { // duty_cycle in %
	TIM3->CCR3 = duty_cycle * 10;
	TIM3->CCR4 = 1000 - duty_cycle * 10;
}

void setDoserSpeed(uint8_t doser, uint8_t speed) {
	uint16_t spd = 0;
	uint32_t speeds1 = 0, speeds2 = 0;
	speeds1 = EE_ReadWord(DOSER_SPEEDS);
	if (doser == 0) {
		TIM3->CCR1 = (100 - speed) * 10;
		speeds2 = (speeds1 & 0xFFFFFF00) | speed;
	}
	if (doser == 1) {
		TIM3->CCR2 = (100 - speed) * 10;
		speeds2 = (speeds1 & 0xFFFF00FF) | (speed << 8);
	}
	if (doser == 2) {
		TIM3->CCR3 = (100 - speed) * 10;
		speeds2 = (speeds1 & 0xFF00FFFF) | (speed << 16);
	}
	if (doser == 3) {
		TIM3->CCR4 = (100 - speed) * 10;
		speeds2 = (speeds1 & 0xFFFFFF) | (speed << 24);
	}
	EE_WriteWord(DOSER_SPEEDS, speeds2);
}

void setDutyCycle(void) {
	uint8_t duty_cycle;
	duty_cycle = TIM3->CCR3 / 10;
	Lcd_clear();
	button = 0;
	while (button != BUTTON_OK) {
		button = readButtons();
		Lcd_goto(0, 0);
		Lcd_write_str("Crnt duty cycle");
		Lcd_goto(1, 0);
		Lcd_write_digit(duty_cycle);
		if (button == BUTTON_FWD) {
			if (duty_cycle == 99) {
			} else {
				duty_cycle++;
				setPwmDc(duty_cycle);
			}
		}
		if (button == BUTTON_BCK) {
			if (duty_cycle == 0) {
			} else {
				duty_cycle--;
				setPwmDc(duty_cycle);
			}
		}
		vTaskDelay(20);
	}
	Lcd_clear();
}

void watchdog_init(void) {
	// WATCHDOG check ###################
	__IO
	uint32_t LsiFreq = 40000;
	/* Check if the system has resumed from IWDG reset */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
		/* Clear reset flags */
		RCC_ClearFlag();
	}

	/* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
	 dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* Set counter reload value to obtain 250ms IWDG TimeOut.
	 Counter Reload Value = 250ms/IWDG counter clock period
	 = 250ms / (LSI/32)
	 = 0.25s / (LsiFreq/32)
	 = LsiFreq/(32 * 4)
	 = LsiFreq/128
	 */
	IWDG_SetReload(LsiFreq / 2);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}

// DHT power control used to restart DHT sensors
void dht_power_control_init(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef init_pin;

	init_pin.GPIO_Pin = GPIO_Pin_12;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &init_pin);
}

uint8_t tmpdata[1];

void power_ctrl(uint8_t device, uint8_t state) {
	switch (device) {
	case PWR_LCD:
		if (state == 1) {
			GPIOB->BSRR = (1 << 5);
		} else {
			GPIOB->BRR = (1 << 5);
		}
		break;
	case PWR_DHT:
		if (state == 1) {
			GPIOB->BSRR = (1 << 12);
		} else {
			GPIOB->BRR = (1 << 12);
		}
		break;
	case PWR_EC:
		if (state == 1) {
			GPIOB->BSRR = (1 << 4);
		} else {
			GPIOB->BRR = (1 << 4);
		}
		break;
	}

}

void beep_overload(uint16_t amount) {
	TIM1->CCR1 = 600;
	Delay_us((uint32_t) amount);
	TIM1->CCR1 = 1000;
}

void relay_test(void) {
	TIM3->CCR1 = 1000;
	TIM3->CCR2 = 1000;
	TIM3->CCR3 = 1000;
	TIM3->CCR4 = 1000;
	Delay_us(30000);
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	Delay_us(30000);
}

uint8_t main(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef init_pin;

	init_pin.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_14
			| GPIO_Pin_13;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &init_pin);

	GPIOC->BRR |= (1 << 10);
	GPIOC->BRR |= (1 << 11);
	GPIOC->BRR |= (1 << 12);

	dosing_motor_control_init();
	uint8_t curpinstate = 0;
	/*	while (1) {
	 if (curpinstate == 0) {
	 curpinstate = 1;
	 GPIOC->BRR |= (1<<12);
	 }
	 else {
	 curpinstate = 0;
	 GPIOC->BSRR |= (1<<12);
	 }
	 Delay_us(100);
	 }*/

	psi_motor_init();

	beep_overload(100);

	// valves[5..8] on PA4-PA7 init, and valves [9..11] on PC10-PC12
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // PA13 and PA14 use as GPIO
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	init_pin.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8
			| GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13
			| GPIO_Pin_14 | GPIO_Pin_15;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &init_pin);

	close_valves();

	// DHT init
	dht_init_out_x(0);
	dht_init_out_x(1);
	dht_power_control_init();

	// init LCD and EC power control pin
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	init_pin.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &init_pin);

	// communications init
	bluetooth_init();

	power_ctrl(PWR_EC, 1);

	SystemInit();

	RtcInit();

	auto_failures = 0;
	uint32_t i;

	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();
	/* EEPROM Init */
	EE_Init();
	AdcInit();
//	OW_Reset();

	power_ctrl(PWR_LCD, 0);

//	Lcd_clear();
//	dht_init();		// start TIM15
	Delay(1000);
	power_ctrl(PWR_LCD, 1);
	Delay(1000);

	loadSettings();
	flush_lcd_buffer(); // fills the LCD frame buffer with spaces
//	Lcd_write_str("LCD1 ON!");

//	I2C_init2();	// master-slave test send	(pH sensor)

#ifdef I2C_SLAVE
//	I2C_Slave_BufferReadWrite(I2C1, Interrupt);
#endif

	tmpdata[0] = 0;

//	I2C_Master_BufferWrite(I2C2, tmpdata,1,Interrupt,  0x9A);

	// === LCD init ===

	i = 0;

#ifdef USE_LCD
	/*	LCDI2C_init(0x4E,16,2);
	 LCDI2C_backlight(); // finish with backlight
	 LCDI2C_clear();
	 LCDI2C_write_String("Hello Buddy!");
	 Delay_us(2000);
	 LCDI2C_clear();
	 LCDI2C_write_String("Let's grow!");
	 Delay_us(2000); */
#endif

	psi_m_low = PSI_M_LOW;
	psi_m_high = PSI_M_HIGH;
	psi_max_speed = PSI_MAX_SPEED;

	sonar_init(); // init sonar ECHOs on PB8 and PB9 and TRIG on PC13

	xTaskCreate(lstasks, (signed char*)"LST", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(watering_program_trigger, (signed char*)"WP", 180, NULL,
			tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(uart_task, (signed char*)"UART", 100, NULL,
			tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(displayClock, (signed char*)"CLK", 140, NULL,
			tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(timerStateTrigger, (signed char*)"TIMERS",
			configMINIMAL_STACK_SIZE+10, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(plugStateTrigger, (signed char*)"PLUGS",
			configMINIMAL_STACK_SIZE+35, NULL, tskIDLE_PRIORITY + 1, NULL);
	/*    xTaskCreate(vTaskLCDdraw,(signed char*)"LCDDRW",configMINIMAL_STACK_SIZE,
	 NULL, tskIDLE_PRIORITY + 1, NULL); */

	Delay_us(100);
	beep_overload(1000);

	watchdog_init();	// start watchdog timer

	/* Start the scheduler. */

	vTaskStartScheduler();

	while (1)
		;
}

void loadSettings(void) { // function loads the predefined data
	uint16_t i, Address, Data;
	char tmpstr[11], putstring[50];

	for (i = 0; i < PLUG_AMOUNT; i++) {
		Address = EE_PLUG_SETTINGS + i;
		EE_ReadVariable(Address, &Data);
		plugSettings[i] = Data;
	}

	EE_ReadVariable(FWTANK_TOP, &tank_windows_top[FWTANK]);
	EE_ReadVariable(FWTANK_BOTTOM, &tank_windows_bottom[FWTANK]);
	EE_ReadVariable(MIXTANK_TOP, &tank_windows_top[MIXTANK]);
	EE_ReadVariable(MIXTANK_BOTTOM, &tank_windows_bottom[MIXTANK]);

	EE_ReadVariable(PSI_UNDERPRESSURE, &psi_upres_level);
	EE_ReadVariable(PSI_UP_TIMEOUT, &psi_upres_timeout);

	EE_ReadVariable(PSI_SENSOR_TOP, &psi_pump_top_level);
	EE_ReadVariable(PSI_SENSOR_BTM, &psi_pump_btm_level);

	psi_mid = psi_pump_top_level
			+ (psi_pump_top_level - psi_pump_btm_level) / 2; // middle of PSI stab window
	psi_a = psi_mid - ((psi_pump_top_level - psi_pump_btm_level) / 10); //
	psi_b = psi_mid + ((psi_pump_top_level - psi_pump_btm_level) / 10);

	for (i = 0; i < WFM_AMOUNT; i++) {
		EE_ReadVariable(WFM_CAL_OFFSET + i, &wfCalArray[i]);
	}

	EE_ReadVariable(CO2_TOP, &co2_top);
	EE_ReadVariable(CO2_BTM, &co2_btm);
	EE_ReadVariable(CO2_400PPM, &co2_400ppm);

	readButtonRanges();
}

void Lcd_write_str(volatile uint8_t *STRING) {
	uint8_t c = 0;
	while (c = *STRING++) {
		if (lcd_pointery == 0) {
			LCDLine1[lcd_pointerx] = c;
			lcd_pointerx++;
		} else {
			LCDLine2[lcd_pointerx] = c;
			lcd_pointerx++;
		}
	}
}

void Lcd_goto(uc8 x, uc8 y) {
	uint8_t str;
	str = y + 0x80;
	if (x == 1) {
		str += 0x40;
	}
//	Lcd_write_cmd(str);
	LCDI2C_send(str, 0);
	lcd_pointerx = y;
	lcd_pointery = x;
}

// Cadi mainboard LCD Init (4bit direct mode)
void Init_pin_out() {
	// Cadi MB pins for LCD are following:
	// data: PA15, PC10, PC11, PC12
	// cmd: PD2, PB3, PB4

	/*
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
	 GPIO_InitTypeDef init_pin;
	 init_pin.GPIO_Pin  = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	 init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	 init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOC, &init_pin);
	 init_pin.GPIO_Pin  = GPIO_Pin_15;
	 init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	 init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &init_pin);
	 init_pin.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4;
	 init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	 init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &init_pin);
	 init_pin.GPIO_Pin  = GPIO_Pin_2;
	 init_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	 init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOD, &init_pin); */
}

void Init_pin_in() {
	RCC_APB2PeriphClockCmd(lcd_init_port_data, ENABLE);
	GPIO_InitTypeDef init_pin;
	init_pin.GPIO_Pin = pin_d7 | pin_d6 | pin_d5 | pin_d4;
	init_pin.GPIO_Mode = GPIO_Mode_IPD;
	init_pin.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(lcd_port_data, &init_pin);
}

void Lcd_write_cmd(uint8_t cmd) {
	Delay_us(6); // stable 240
	rs_0;
	rw_0;
	e_1;
	Delay_us(1);
	set4highBits(cmd);
	e_0;
	Delay_us(1);
	e_1;
	Delay_us(1);
	set4lowBits(cmd);
	e_0;
	Delay_us(6);
}

void Lcd_write_data(uint8_t data) {
	Delay_us(6);
	e_1;
	rs_1;
	rw_0;
	Delay_us(1);
	set4highBits(data);
	Delay_us(1);
	e_0;
	Delay_us(1);
	e_1;
	Delay_us(1);
	set4lowBits(data);
	Delay_us(1);
	e_0;
	Delay_us(1);
	e_0;
	rs_0;
	rw_0;
	Delay_us(6);
}

void set4highBits(uint8_t dta) { // setting higher 4 bits of word on corresponding GPIO pins
/*	if (dta&16) {
 d4_1;
 }
 else {
 d4_0;
 }
 if (dta&32) {
 d5_1;
 }
 else {
 d5_0;
 }
 if (dta&64) {
 d6_1;
 }
 else {
 d6_0;
 }
 if (dta&128) {
 d7_1;
 }
 else {
 d7_0;
 } */
}

void set4lowBits(uint8_t dta) {
	/*	if (dta&1) {
	 d4_1;
	 }
	 else {
	 d4_0;
	 }
	 if (dta&2) {
	 d5_1;
	 }
	 else {
	 d5_0;
	 }
	 if (dta&4) {
	 d6_1;
	 }
	 else {
	 d6_0;
	 }
	 if (dta&8) {
	 d7_1;
	 }
	 else {
	 d7_0;
	 } */
}

void Init_lcd() {
	Init_pin_out();
	e_1;
	rs_0;
	rw_0;
	Delay_us(100); // assume 10ms
	set4lowBits(0b0010); // set 4 bit bus
	e_0;
	Delay_us(10); // assume 10ms

	Lcd_write_cmd(0b00101000); // again, 4bit bus and the rest 4bits of whole command will get the destination now
	Delay_us(10);
	Lcd_write_cmd(Display_clear);
	Lcd_write_cmd(0b00000110); // function set
	Lcd_write_cmd(0b00001100); // display on cursor off
	Lcd_write_cmd(Display_clear); // function set
	Lcd_write_str("12345678");
	Delay_us(10);
}

void Lcd_clear(void) {
	// LCDI2C_clear();
	Delay_us(500);
	// next line responsible for direct LCD 4bit mode
	// Lcd_write_cmd(Display_clear);
	flush_lcd_buffer();
	Lcd_goto(0, 0);
}

void Return_home() {
	Lcd_write_cmd(0b0000001);
}

