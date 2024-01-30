#include "../headers/soter_usart.h"

extern QueueHandle_t xQueueTalk;
extern QueueHandle_t xQueueListen;
extern SemaphoreHandle_t xSemaphoreUSART2;

void USART2_IRQHandler( void )
{
	uint8_t RxData;
	uint8_t txData;
	static BaseType_t pxHigherPriorityTaskWoken;

	if ( USART_GetFlagStatus( USART2, USART_FLAG_RXNE ) != RESET ) {
		RxData = USART_ReceiveData( USART2 );
		xQueueSendToBackFromISR( xQueueListen, &RxData, &pxHigherPriorityTaskWoken );

	}

	if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
	        // Verificar se há dados na fila de transmissão
		if ( xQueueReceiveFromISR( xQueueTalk, &txData, &pxHigherPriorityTaskWoken ) == pdTRUE ) {
			// Enviar dados
			xSemaphoreTakeFromISR( xSemaphoreUSART2, &pxHigherPriorityTaskWoken );
			USART_SendData(USART2, txData);
			xSemaphoreGiveFromISR( xSemaphoreUSART2, &pxHigherPriorityTaskWoken );
		}
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	}

	// Limpar flags
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	USART_ClearITPendingBit(USART2, USART_IT_TXE);

	if ( pxHigherPriorityTaskWoken == pdTRUE ){
		taskYIELD();
	}
}

void USARTInit ( const uint32_t baudRate, const uint16_t buffer_depth, USART_TypeDef *USARTx ) {
	if( xQueueTalk == 0 )
	{
		xQueueTalk = xQueueCreate( buffer_depth, sizeof( uint8_t ) );
	}
	if( xQueueListen == 0 )
	{
		xQueueListen = xQueueCreate( buffer_depth, sizeof( uint8_t ) );
	}
	USART_InitTypeDef USART_InitStructure;
	/* USART Periph clock enable */
	USART_DeInit( USARTx );

	if ( USARTx == USART2 )
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	}

	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	/* Configure the USART2 */
	USART_Init(USARTx, &USART_InitStructure);
	/* Enable the USART2 */
	USART_Cmd(USARTx, ENABLE);
}

void prvUSARTPutString( const char *message, const uint8_t string_lenght ) {

	for ( size_t i = 0; i < string_lenght; i++ ) {

		char currentChar = message[i];

		// Enable TXE interrupt to start sending data
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

		// Send the current character to the transmission queue
		xQueueSendToBack(xQueueTalk, &currentChar, portMAX_DELAY);

	}
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
}

void prvUSARTPutChar( const char message ) {

	// Enable TXE interrupt to start sending data
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

	// Send the current character to the transmission queue
	xQueueSendToBack(xQueueTalk, &message, portMAX_DELAY);

}

void prvUSARTPutByte( const uint16_t byte ) {
    // Enable TXE interrupt to start sending data
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

    // Send the byte to the transmission queue
    xQueueSendToBack(xQueueTalk, &byte, portMAX_DELAY);

}


void prvUSARTPutArray(const float *array) {
    // Enable TXE interrupt to start sending data

    // Iterate through the array and send each byte to the transmission queue
    for (int i = 0; i < ARRAY_SIZE_PROCESSING; i++) {
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        xQueueSendToBack(xQueueTalk, (uint8_t) &array[i], portMAX_DELAY);
    }
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
}

char USARTGetChar( void ) {

	char letter;
	xQueueReceive( xQueueListen, &letter, portMAX_DELAY );
	return letter;

}

uint8_t USARTGetInt( void ) {

	uint8_t number;
	xQueueReceive( xQueueListen, &number, portMAX_DELAY );
	return number;

}

void USARTClose( void ) {
	USART_Cmd(USART2, DISABLE);
}
