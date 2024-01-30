/*
 * soter_usart.h
 *
 *  Created on: Jan 12, 2024
 *      Author: tiago
 */

#include "../soter.h"

#ifndef SOTER_USART_H_
#define SOTER_USART_H_

void USARTInit ( const uint32_t baudRate, const uint16_t buffer_depth, USART_TypeDef *USARTx );
char USARTGetChar( void );
uint8_t USARTGetInt( void );
void prvUSARTPutString( const char *message, const uint8_t string_lenght );
void prvUSARTPutArray(const float *array);
void prvUSARTPutChar( const char message );
void USARTClose		( void );
void prvUSARTPutByte( const uint16_t byte );

#endif /* SOTER_HEADERS_SOTER_USART_H_ */
