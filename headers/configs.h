#ifndef SISEM_FINAL_CONFIGS_H_
#define SISEM_FINAL_CONFIGS_H_

#include "../soter.h"

extern int Flag_USART;

void RCC_Config_HSI_PLL_Max();
void GPIO_I2C();
void USART2_config();
void Setup_PA3();
void Setup_PA2();
void USART2_interrupt();
void keypad_setup( void );

#endif /* SISEM_FINAL_CONFIGS_H_ */
