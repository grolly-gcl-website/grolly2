#include "stm32f10x.h"

void Delay_us_(uint32_t delay) { // adjusted for 8mhz
	volatile uint32_t del = 0;
	del = delay * 3;
	while (del--) {

	}
}
