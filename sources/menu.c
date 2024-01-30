#include "../headers/menus.h"

extern QueueHandle_t xQueueLiveData;
extern QueueHandle_t xQueueButton;
extern SemaphoreHandle_t xSemaphoreCounter;
extern SemaphoreHandle_t xSemaphoreLCD;

/*************************************************************************/
void EXTI1_IRQHandler(void)
{
	static BaseType_t pxHigherPriorityTaskWoken;
	const uint16_t button = GPIO_Pin_1;
	static TickType_t old = 0;
	TickType_t new = xTaskGetTickCountFromISR();
	TickType_t difference = 0;
	difference = new - old / portTICK_PERIOD_MS ;
	old = new;
	if ( difference > 100 ) {

		xQueueSendToBackFromISR( xQueueButton, &button, &pxHigherPriorityTaskWoken );

		if ( pxHigherPriorityTaskWoken == pdTRUE ){
			taskYIELD(); /* forces the context change */
		}
	}

	EXTI_ClearITPendingBit(EXTI_Line1);
}

void EXTI15_10_IRQHandler(void)
{
	static BaseType_t pxHigherPriorityTaskWoken;
	static TickType_t old = 0;
	TickType_t new = xTaskGetTickCountFromISR();
	uint16_t button;
	TickType_t difference = new - old / portTICK_PERIOD_MS ;
	old = new;

	if( difference > 100 )
	{
		if ( EXTI_GetITStatus( EXTI_Line10 ) )
		{
			button = GPIO_Pin_10;
			xQueueSendToBackFromISR( xQueueButton, &button, &pxHigherPriorityTaskWoken );
			EXTI_ClearITPendingBit(EXTI_Line10);
		}
		else if ( EXTI_GetITStatus( EXTI_Line13 ) )
		{
			button = GPIO_Pin_13;
			xQueueSendToBackFromISR( xQueueButton, &button, &pxHigherPriorityTaskWoken );
			EXTI_ClearITPendingBit(EXTI_Line13);
		}
		else if ( EXTI_GetITStatus( EXTI_Line12 ) )
		{
			button = GPIO_Pin_12;
			xQueueSendToBackFromISR( xQueueButton, &button, &pxHigherPriorityTaskWoken );
			EXTI_ClearITPendingBit(EXTI_Line12);
		}
		else
		{

			button = GPIO_Pin_11;
			xQueueSendToBackFromISR( xQueueButton, &button, &pxHigherPriorityTaskWoken );
			EXTI_ClearITPendingBit(EXTI_Line11);
		}
		if ( pxHigherPriorityTaskWoken == pdTRUE ){
			taskYIELD(); /* forces the context change */
		}
	}
	else
	{
		EXTI_ClearITPendingBit(EXTI_Line10);
		EXTI_ClearITPendingBit(EXTI_Line13);
		EXTI_ClearITPendingBit(EXTI_Line12);
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

uint16_t keypad_read( void ) {
	uint16_t pin;
	xQueueReceive( xQueueButton, &pin, portMAX_DELAY );
	return pin;
}

uint16_t keypad_read_ticks ( uint16_t ticks ) {
	uint16_t pin;
	if ( xQueueReceive( xQueueButton, &pin, (TickType_t) ticks ) == pdFALSE ) {
		return 0;
	}
	return pin;
}

void update_results( const char RxData, LiveData *data ) {
	SensorData sensor;
	uint8_t valueToSend;
	switch( RxData ) {
		case 'x':
			sensor = I2C_Com();
			decimalToHex( sensor.x/10, &valueToSend );
			prvUSARTPutByte( valueToSend );
			break;
		case 'a':
			data->apples++;
			xQueueSendToBack( xQueueLiveData, data, portMAX_DELAY );
			break;
		case 'y':
//			sensor = I2C_Com();
//			decimalToHex(sensor.y/10, &valueToSend);
//			coms_sendData2(valueToSend);
			break;
		case 'l':
			data->score = data->apples * 10;
			died(data);
			break;
		default:
			break;
	}
}
