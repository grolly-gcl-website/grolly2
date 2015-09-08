// DRIVER: Sonar
#define USE_SONAR
#ifdef USE_SONAR
// #define SONAR_PINS_PA6_PC4
// #define SONAR_PINS_PB8_9
#define SONAR_F103_PINS_PB8_9
#ifdef SONAR_PINS_PA6_PC4
#define SONAR_AMOUNT	2
#define	SONAR1_TIM				TIM17
#define SONAR_ECHO_PORT			GPIOA
#endif
#ifdef SONAR_PINS_PB8_9
#define SONAR_AMOUNT			2
#define	SONAR1_TIM				TIM17
#define	SONAR2_TIM				TIM16
#define SONAR_ECHO_PORT			GPIOB
#define SONAR_TRIG_PORT			GPIOA
#define SONAR_TRIG_PIN			12
#endif

#ifdef SONAR_F103_PINS_PB8_9
#define	SONAR_TIM				TIM4	// SINGLE TIMER FOR ALL SONARS
#define SONAR_AMOUNT			2
#define	SONAR1_TIM				TIM4
#define	SONAR2_TIM				TIM4
#define SONAR_ECHO_PORT			GPIOB
#define SONAR_TRIG_PORT			GPIOD
#define SONAR_TRIG_PIN			2
#endif


#define SONAR1_FALL		!((GPIOB->IDR)>>8 & 1)
#define SONAR2_FALL		!((GPIOB->IDR)>>9 & 1)

#define SONAR_MAX_READING	3000

volatile static uint16_t	sonar_read[SONAR_AMOUNT];	// 2 sonars on pins PA6 and PA7 with trig on PC4
volatile static uint16_t	sonar_mm[SONAR_AMOUNT];
#endif


void sonar_init(void);
void sonar_ping(void);


