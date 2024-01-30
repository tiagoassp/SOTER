/*
 * soter.h
 *
 *  Created on: Jan 7, 2024
 *      Author: Tiago Sá Pereira
 */

#ifndef SOTER_H_
#define SOTER_H_

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lcd.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define verde 0x07F0
#define rosa 0xf0ff
#define amarelo 0x0fff
#define branco 0xffff
#define vermelho 0xf800
#define azulc 0xff0f
#define preto 0x0000
#define azul 0x0018
#define cinzento 0x29C9

#define HIGHSCORE_SIZE 5

#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainMENU_DELAY			( ( TickType_t ) 300 / portTICK_RATE_MS )
#define mainLCD_DELAY			( ( TickType_t ) 200 / portTICK_RATE_MS )
#define mainLED_DELAY			( ( TickType_t ) 500 / portTICK_RATE_MS )
#define mainPROCESS_DELAY		( ( TickType_t ) 100  / portTICK_RATE_MS )
#define mainSTEP_DELAY			( ( TickType_t ) 10  / portTICK_RATE_MS )
#define mainI2C_DELAY			( ( TickType_t ) 100  / portTICK_RATE_MS )

#define START_MSG 0x64
#define MINUS_MSG 0x00
#define POS_MSG	  0x01
#define MAX_MENU_COUNTER 2
#define MIN_MENU_COUNTER 0

#define ARRAY_SIZE_PROCESSING 17
#define DIFFERENCE_THRESHOLD 170

typedef struct {
	uint8_t apples;
	uint8_t maxApplesCaught;
	uint16_t score;
	uint16_t highscore[HIGHSCORE_SIZE];
	uint8_t isDead;
} LiveData;

typedef struct {
	int x;
	int y;
} drawRectangle;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} SensorData;

#define ERROR_SEMAPHORE 	"Semaphore Error"
#define ERROR_QUEUE 		"Queue Error"

#include "headers/configs.h"

extern void RCC_Config_HSI_PLL_Max	( void );
extern void GPIO_I2C				( void );
extern void USART2_config			( void );
extern void Setup_PA3				( void );
extern void Setup_PA2				( void );
extern void USART2_interrupt		( void );
extern void keypad_setup			( void );

#include "headers/menus.h"

extern void 	update_results		( const char RxData, LiveData *data );
extern void 	clean_rectangle 	( const drawRectangle ptr );
extern uint16_t keypad_read 		( void );
extern uint16_t keypad_read_ticks 	( uint16_t ticks );

#include "headers/read_i2c.h"

extern SensorData I2C_Com( void );

#include "headers/soter_usart.h"

extern void 	USARTInit 			( const uint32_t baudRate, const uint16_t buffer_depth
															, USART_TypeDef *USARTx );
extern char 	USARTGetChar		( void );
extern uint8_t  USARTGetInt			( void );
extern void 	USARTClose			( void );
extern void 	prvUSARTPutString	( const char *message, const uint8_t string_lenght );
extern void 	prvUSARTPutChar		( const char message );
extern void 	prvUSARTPutByte		( const uint16_t byte );
extern void 	prvUSARTPutArray	( const float *array);

#endif /* SOTER_H_ */
