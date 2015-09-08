#include "stm32f10x.h"
#include "driver_sonar.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#define PLUG_DISABLE	GPIOC->BSRR
#define PLUG_ENABLE		GPIOC->BRR

void sonar_init(void){
#ifdef	SONAR_PINS_PA6_PC4
	// sonar trigger on PC4

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SONAR_TIM, 17 remapping pins */
	GPIO_PinRemapConfig(GPIO_Remap_TIM17, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_TIM16, ENABLE);



	  /*

	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_SONAR_TIM_IRQn;	// & TIM1_TRG_COM_SONAR_TIM_IRQn
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  TIM_ICInitTypeDef TIM_ICInitStructure;
	   TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	   TIM_ICInitStructure.TIM_ICFilter = 0x0;

	   TIM_ICInit(SONAR_TIM, &TIM_ICInitStructure); */

	   /* TIM enable counter */
//	   TIM_Cmd(SONAR_TIM, ENABLE);

	   /* Enable the CC2 Interrupt Request */
//	   TIM_ITConfig(SONAR_TIM, TIM_IT_CC1, ENABLE);

	  NVIC_InitTypeDef NVIC_InitStructure;

	   /* Enable the TIM15 global Interrupt */

	  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM17_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);


	  // SONAR_TIM setup
	  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    //enable SONAR_TIM clock
	  SONAR_TIM->PSC	= 80-1;                //set divider to get ms
//	  SONAR_TIM->ARR     = 500;                   //2hz = 500ms
	  SONAR_TIM->DIER    = TIM_DIER_CC1IE;        	 //enable timer interrupt
	  //enable reload and interrupt
	  SONAR_TIM->CCMR1	&= ~ TIM_CCMR1_CC1S_1; // Select the active input: TIMx_CCR1 must be linked to the TI1 input, so write the CC1S  bits to 01 in the TIMx_CCMR1 register.
	  SONAR_TIM->CCMR1	|=	TIM_CCMR1_CC1S_0;
	  SONAR_TIM->CCER &= ~ TIM_CCER_CC1P;	// Select the edge of the active transition on the TI1 channel by writing CC1P bit to 0 in the TIMx_CCER register (rising edge in this case).
	  SONAR_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_0; // Program the input prescaler. In our example, we wish the capture to be performed at each valid transition, so the prescaler is disabled (write IC1PS bits to â€˜00â€™ in the TIMx_CCMR1 register).
	  SONAR_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_1;
	  SONAR_TIM->CCER  |= TIM_CCER_CC1E; // Enable capture from the counter into the capture register by setting the CC1E bit in the TIMx_CCER register.
	  SONAR_TIM->CR1  = TIM_CR1_UDIS| TIM_CR1_CEN;

#endif

#ifdef SONAR_PINS_PB8_9
	  // sonar trigger..
      GPIOC->CRL      &= ~GPIO_CRL_CNF4;		// ... to PC3
      GPIOC->CRL   |= GPIO_CRL_MODE4_0;
 //     GPIOA->CRH      &= ~GPIO_CRH_CNF12;		// ... to PA12
 //     GPIOA->CRH   |= GPIO_CRH_MODE12_0;


		RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17 | RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(SONAR_ECHO_PORT, &GPIO_InitStructure);


		  NVIC_InitTypeDef NVIC_InitStructure;

		   // Enable the TIM17 global Interrupt

		  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
		  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		  NVIC_Init(&NVIC_InitStructure);

		  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM17_IRQn;
		  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		  NVIC_Init(&NVIC_InitStructure);

		  // SONAR_TIM setup
		  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    //enable SONAR_TIM clock
		  SONAR_TIM->PSC	= 800-1;                //set divider
	//	  SONAR_TIM->ARR     = 500;                   //2hz = 500ms
		  //enable reload and interrupt
		  SONAR_TIM->CCMR1	&= ~ TIM_CCMR1_CC1S_1; // Select the active input: TIMx_CCR1 must be linked to the TI1 input, so write the CC1S  bits to 01 in the TIMx_CCMR1 register.
		  SONAR_TIM->CCMR1	|=	TIM_CCMR1_CC1S_0;
		  SONAR_TIM->CCER |= TIM_CCER_CC1P;	// Select the edge of the active transition on the TI1 channel by writing CC1P bit to 0 in the TIMx_CCER register (rising edge in this case).
		  SONAR_TIM->CCER |= TIM_CCER_CC1NP;
		  SONAR_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_0; // Program the input prescaler. In our example, we wish the capture to be performed at each valid transition, so the prescaler is disabled (write IC1PS bits to â€˜00â€™ in the TIMx_CCMR1 register).
		  SONAR_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_1;
		  SONAR_TIM->CCER  |= TIM_CCER_CC1E; // Enable capture from the counter into the capture register by setting the CC1E bit in the TIMx_CCER register.
		  SONAR_TIM->DIER    = TIM_DIER_CC1IE;        	 //enable timer interrupt
		  SONAR_TIM->CR1  =  TIM_CR1_CEN;

		  // SONAR_TIM setup
		  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;    //enable SONAR_TIM clock
		  SONAR_TIM->PSC	= 800-1;                //set divider
	//	  SONAR_TIM->ARR     = 500;                   //2hz = 500ms
		  //enable reload and interrupt
		  SONAR_TIM->CCMR1	&= ~ TIM_CCMR1_CC1S_1; // Select the active input: TIMx_CCR1 must be linked to the TI1 input, so write the CC1S  bits to 01 in the TIMx_CCMR1 register.
		  SONAR_TIM->CCMR1	|=	TIM_CCMR1_CC1S_0;
		  SONAR_TIM->CCER |= TIM_CCER_CC1P;	// Select the edge of the active transition on the TI1 channel by writing CC1P bit to 0 in the TIMx_CCER register (rising edge in this case).
		  SONAR_TIM->CCER |= TIM_CCER_CC1NP;
		  SONAR_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_0; // Program the input prescaler. In our example, we wish the capture to be performed at each valid transition, so the prescaler is disabled (write IC1PS bits to â€˜00â€™ in the TIMx_CCMR1 register).
		  SONAR_TIM->CCMR1 &= ~ TIM_CCMR1_IC1PSC_1;
		  SONAR_TIM->CCER  |= TIM_CCER_CC1E; // Enable capture from the counter into the capture register by setting the CC1E bit in the TIMx_CCER register.
		  SONAR_TIM->DIER    = TIM_DIER_CC1IE;        	 //enable timer interrupt
		  SONAR_TIM->CR1  =  TIM_CR1_CEN;

#endif

#ifdef SONAR_F103_PINS_PB8_9
     RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE);

     // sonar TRIG signal is taken from
     GPIOD->CRL      &= ~GPIO_CRL_CNF2;		// ... p PD2
     GPIOD->CRL   |= GPIO_CRL_MODE2_0;

     GPIO_InitTypeDef gpio_cfg;
     GPIO_StructInit(&gpio_cfg);
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
     gpio_cfg.GPIO_Mode = GPIO_Mode_IPD;
     gpio_cfg.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
     GPIO_Init(GPIOB, &gpio_cfg);

     RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;    //enable SONAR_TIM clock
     SONAR_TIM->PSC	= 100-1;                //set divider
     SONAR_TIM->CR1  =  TIM_CR1_CEN;

     GPIO_InitTypeDef GPIO_InitStructure;
     EXTI_InitTypeDef EXTI_InitStructure;
     NVIC_InitTypeDef NVIC_InitStructure;

     // interrupt config for water level sensor inputs

     GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
     EXTI_InitStructure.EXTI_Line = EXTI_Line8;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
     EXTI_Init(&EXTI_InitStructure);

     GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);
     EXTI_InitStructure.EXTI_Line = EXTI_Line9;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
     EXTI_Init(&EXTI_InitStructure);

     NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
#endif
}



void sonar_ping(void){
	SONAR_TRIG_PORT->BRR = (1<<SONAR_TRIG_PIN);
	vTaskDelay(2);
	// TRIG pin  logic level "1"
	SONAR_TRIG_PORT->BSRR = (1<<SONAR_TRIG_PIN);
	SONAR_TIM->CNT=0;
	vTaskDelay(2);
	// TRIG pin  logic level "0"
	SONAR_TRIG_PORT->BRR = (1<<SONAR_TRIG_PIN);


}
