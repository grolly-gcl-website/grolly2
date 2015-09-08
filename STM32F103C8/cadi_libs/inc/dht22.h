/*  ====== DHT driver for STM32 =======
 * FreeRTOS delays (vTaskDelay()) used in code.
 *
 */
/*

#define DHT2_TRIG_PLUG			15		// PB15
#define DHT1_TRIG_PLUG			14		// PB14
#define DHT2_DATA_START_POINTER	0		// DHT22 = 4, DHT11 = ?; sets the first bit number in captured sequence of DHT response bits
#define DHT1_DATA_START_POINTER	0		// DHT22 = 4, DHT11 = ?; sets the first bit number in captured sequence of DHT response bits
#define DHT2_0					(GPIOB->BRR = (1<<DHT2_TRIG_PLUG))
#define DHT2_1					(GPIOB->BSRR = (1<<DHT2_TRIG_PLUG))
#define DHT1_0					(GPIOB->BRR = (1<<DHT1_TRIG_PLUG))
#define DHT1_1					(GPIOB->BSRR = (1<<DHT1_TRIG_PLUG))

#define DHT1_PORT				GPIOB
#define DHT1_PIN_SRC			GPIO_PinSource14
#define DHT1_EXTI_LINE			EXTI_Line14
#define DHT1_NVIC_CH			EXTI15_10_IRQn
#define DHT1_ELC_PORT_SRC		GPIO_PortSourceGPIOB

typedef struct
{
  uint16_t 			DHT_Temperature;        // temperature
  uint16_t 			DHT_Humidity;			// from 0 (0%) to 1000 (100%)
  uint8_t 			DHT_CRC;
}DHT_data;



volatile static uint16_t dht_rise;
volatile static uint16_t dht_period;
volatile static uint16_t DutyCycle;


volatile static uint8_t dht_byte_buf[5];

volatile static uint8_t dht_shifter=DHT2_DATA_START_POINTER;		// could be removed in production version
TIM_ICInitTypeDef  TIM_ICInitStructure;
volatile static DHT_data DHTValue;
volatile static uint8_t		dht1_data[5];
volatile static uint8_t		dht2_data[5];
volatile static uint8_t dht_bit_position = 0;
volatile static uint8_t	dht_data_ready = 0;
volatile static uint8_t  dht_byte_pointer;
volatile static uint8_t  dht_bit_pointer = 7;
volatile static uint8_t 	dht_rh_str[4], dht_t_str[4];
volatile static uint16_t rhWindowTop, rhWindowBottom;
volatile static uint8_t rhUnderOver = 0;
volatile static uint8_t dht_last_read_id = 255;



void dht_get_data_x(uint8_t dht_id);
void dht_req_x(uint8_t dht_id);
void dht_1(uint8_t dht_id);
void dht_0(uint8_t dht_id);
void dht_conv_data(void);
void dht_init_x(uint8_t dht_id);
void dht_init_out_x(uint8_t dht_id);
uint8_t crc8(uint8_t *buf, uint8_t start, uint8_t length);
 */
