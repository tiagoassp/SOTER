#include "soter.h"

/*********************************** Tasks *******************************************/

static void pvrBlinkLED 			( void *pvParameters );

static void prvLCD 					( void *pvParameters );

static void prvstepCounter			( void *pvParameters );

static void prvI2CReadTask			( void *pvParameters );

static void prvProcessData 			( void *pvParameters );

static void prvI2CProxy 			( void *pvParameters );

static void prvreadUSART 			( void *pvParameters );

/********************************** Functions *****************************************/

static uint16_t countPeaks			( const uint16_t *processedArray, float media );

static float 	fillVector			( const SensorData *sensorArray, uint16_t *processedArray );

static float 	calculateMean		( const uint16_t *array );

static float 	StandardDeviation	( const uint16_t *array );

static void 	prvCreateTasks 		( void );

static void	 	callConfigs			( void );

static void 	printSteps			( uint16_t stepCounter );

static int	 	InitializeQueue 	( void );

static void 	mainMenu 			( void );

static void 	secondMenu 			( void );

static void 	printTitle 			( void );

/*******************************************************************************************/

TaskHandle_t HandleLED;
TaskHandle_t HandleLCD;
TaskHandle_t HandleStepCounter;
TaskHandle_t HandleI2C;
TaskHandle_t HandleProcessData;
TaskHandle_t HandleProxy;
TaskHandle_t HandleRX;

QueueHandle_t xQueueButton;
QueueHandle_t xQueueTalk;
QueueHandle_t xQueueListen;
QueueHandle_t xQueueLiveData;
QueueHandle_t xQueueStepCounter;
QueueHandle_t xQueueProcessData;
QueueHandle_t xQueueProxy;

SemaphoreHandle_t xSemaphoreUSART2;
SemaphoreHandle_t xSemaphoreLCD;
SemaphoreHandle_t xSemaphoreCounter;
SemaphoreHandle_t xSemaphoreKm;

/*******************************************************************************************/

int main () {

	if ( InitializeQueue() ==  1 ) return 0;

	xSemaphoreLCD = xSemaphoreCreateMutex();
	xSemaphoreUSART2 = xSemaphoreCreateMutex();
	xSemaphoreCounter = xSemaphoreCreateBinary();
	xSemaphoreKm = xSemaphoreCreateBinary();

	if ( !xSemaphoreUSART2 || !xSemaphoreLCD || !xSemaphoreCounter || !xSemaphoreKm )
	{
		return 0;
	}

	prvCreateTasks();

	callConfigs();

	vTaskStartScheduler();

	return 0;
}

static void callConfigs( void ) {

	RCC_Config_HSI_PLL_Max();
	GPIO_I2C();
	keypad_setup();
	USARTInit( 115200, 5, USART2 );
	USART2_interrupt();
	lcd_init();

}

static void prvCreateTasks ( void ) {

	xTaskCreate( pvrBlinkLED, "LED Blink", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleLED );
	xTaskCreate( prvLCD, "LCD", configMINIMAL_STACK_SIZE + 50, NULL, mainFLASH_TASK_PRIORITY, &HandleLCD );
	xTaskCreate( prvstepCounter, "Steps", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleStepCounter );
	xTaskCreate( prvI2CReadTask, "I2C", configMINIMAL_STACK_SIZE + ARRAY_SIZE_PROCESSING + 50, NULL, mainFLASH_TASK_PRIORITY, &HandleI2C );
	xTaskCreate( prvProcessData, "Data Analysis", configMINIMAL_STACK_SIZE + 2 * ARRAY_SIZE_PROCESSING, NULL, mainFLASH_TASK_PRIORITY, &HandleProcessData );
	xTaskCreate( prvI2CProxy, "Proxy", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleProxy );
	xTaskCreate( prvreadUSART, "Read RX", configMINIMAL_STACK_SIZE, NULL, mainFLASH_TASK_PRIORITY, &HandleRX);
}

/**************************************************************
 * Task: pvrBlinkLED
 *
 * @brief: Blink PB0 at 2Hz
 *
 * @Param: None
 *
 * @Output: LED ON/OFF
 **************************************************************/

void pvrBlinkLED ( void *pvParameters ) {
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil(&xLastExecutionTime, mainLED_DELAY);
		GPIO_WriteBit( GPIOB, GPIO_Pin_0, 1 - GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_0 ) );
	}

}

/**************************************************************
 * Task: prvLCD
 *
 * @brief: Print Useful Info To LCD
 *
 * @Param: None
 *
 * @Output: LCD
 **************************************************************/

static void prvLCD( void *pvParameters ) {

	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
	printTitle();

	static int currentMenu = 0;
	static uint16_t pin = 0;
    while (1) {
    	pin = keypad_read_ticks( 10 );
    	vTaskDelayUntil( &xLastExecutionTime , (TickType_t) 200 / portTICK_RATE_MS );

    	switch ( pin ) {

    		case GPIO_Pin_11:
			case GPIO_Pin_12:
				if (currentMenu == 1) break;
				xSemaphoreTake( xSemaphoreLCD, portMAX_DELAY );
				lcd_clean();
				xSemaphoreGive( xSemaphoreLCD );
				currentMenu = 1;
				break;
			case GPIO_Pin_10:
			case GPIO_Pin_13:
				if (currentMenu == 0) break;
				xSemaphoreTake( xSemaphoreLCD, portMAX_DELAY );
				lcd_clean();
				xSemaphoreGive( xSemaphoreLCD );
				currentMenu = 0;
				break;
			default:
				break;
		}

    	xSemaphoreGive( xSemaphoreCounter );

    	if ( currentMenu == 0 ) mainMenu();
    	else secondMenu();
    }
}

static void prvstepCounter( void *pvParameters ) {
	uint16_t stepCounter = 0;
	static uint16_t steps = 0;
	char buffer [10];

	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil( &xLastExecutionTime , (TickType_t) 300 / portTICK_RATE_MS );

		if ( xQueueReceive( xQueueStepCounter, &stepCounter , (TickType_t) 20 ) == pdTRUE ) {
			if ( stepCounter == 0 ) steps = 0;
			else steps += stepCounter;
		}

		sprintf (buffer, "Steps: %u \r\n", steps );
		prvUSARTPutString(buffer, strlen(buffer));

		if ( xSemaphoreTake( xSemaphoreCounter, ( TickType_t ) 20 ) == pdTRUE )
		{
			printSteps( steps );
		}

	}
}

static void prvI2CProxy ( void *pvParameters ) {

	static SensorData arrayI2C[ARRAY_SIZE_PROCESSING] = {};
	static int arrayIndex = 0;
	SensorData sensor;
	BaseType_t status;
	const char Sending_Error [] = "Error Sending Queue Process";
	const char Receiving_Error [] = "Error Receiving Queue Process";

	while (1){

		status = xQueueReceive( xQueueProxy, &sensor, portMAX_DELAY );

		if ( status == pdFALSE ) {
			prvUSARTPutString ( Receiving_Error, strlen ( Receiving_Error ) );
			continue;
		}

		arrayI2C[arrayIndex] = sensor;

		arrayIndex++;

		if ( arrayIndex == ARRAY_SIZE_PROCESSING )
		{
			arrayIndex = 0;
			status = xQueueSendToBack( xQueueProcessData, &arrayI2C, portMAX_DELAY );
			if ( status == pdFALSE ) {
				prvUSARTPutString( Sending_Error, strlen( Sending_Error ) );
			}
		}
	}
}

static void prvI2CReadTask( void *pvParameters ) {
	SensorData sensor;
	BaseType_t status;
	const char Sending_Error [] = "Error Sending Message I2CRead";

	while (1) {

		sensor = I2C_Com();

		status = xQueueSendToBack( xQueueProxy, &sensor, portMAX_DELAY );

		if ( status == pdFALSE ) {
			prvUSARTPutString( Sending_Error, strlen( Sending_Error ) );
		}

		vTaskDelay( ( TickType_t ) 40 / portTICK_RATE_MS );

	}
}

static void prvProcessData ( void *pvParameters ) {
	SensorData arrayI2C [ ARRAY_SIZE_PROCESSING ];
	uint16_t arrayProcess [ ARRAY_SIZE_PROCESSING ];
	uint16_t steps = 0;
	BaseType_t status;
	const char Error_Receive [] = "Error Receiving Queue ProcessData";
	const char Sending_Error [] = "Error Sending Queue Process";
	TickType_t xLastExecutionTime;
	xLastExecutionTime = xTaskGetTickCount();
	while (1) {
		status = xQueueReceive( xQueueProcessData, &arrayI2C, portMAX_DELAY );

		if ( status == pdFALSE ) {
			prvUSARTPutString( Error_Receive, strlen( Error_Receive ) );
			continue;
		}

		vTaskDelayUntil( &xLastExecutionTime, (TickType_t) 100 / portTICK_RATE_MS );

		GPIO_WriteBit( GPIOB, GPIO_Pin_1, 1-GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_1 ) );

		float media = fillVector( arrayI2C, arrayProcess );
		steps = countPeaks( arrayProcess, media );

		if ( steps != 0 ) {
			status = xQueueSendToBack( xQueueStepCounter, &steps, portMAX_DELAY );
			if ( status == pdFALSE ) prvUSARTPutString( Sending_Error, strlen( Sending_Error ) );
		}
	}
}

static void prvreadUSART ( void *pvParameters ) {

	uint8_t number;
	BaseType_t status;
	const char Error_Calibration [] = "Error Sending Queue in Calibration";
	while (1) {
		number = USARTGetInt();
		switch ( number ) {
			case 0:
				status = xQueueSendToBack( xQueueStepCounter, &number, portMAX_DELAY );
				if ( status == pdFALSE ) prvUSARTPutString( Error_Calibration, strlen( Error_Calibration ) );
				break;
			default:
				break;
		}
		vTaskDelay( ( TickType_t ) 50 / portTICK_RATE_MS );
	}
}

static float fillVector( const SensorData *sensorArray, uint16_t *processedArray ) {
    float sumMagnitude = 0.0;

    for ( int i = 0; i < ARRAY_SIZE_PROCESSING; i++ ) {
        int16_t x = sensorArray[i].x;
        int16_t y = sensorArray[i].y;
        int16_t z = sensorArray[i].z;

        processedArray[i] = ( float ) sqrt( ( double )( x * x + y * y + z * z ) );

        sumMagnitude += processedArray[i];
    }

    const float media = calculateMean( processedArray );

    for ( int i = 0; i < ARRAY_SIZE_PROCESSING; i++ ) {
        processedArray[i] -= media;
    }

    return media;
}

static uint16_t countPeaks( const uint16_t *processedArray, float media ) {
    uint16_t step = 0;
    uint16_t max = processedArray[0];
    uint16_t min = processedArray[0];
    int firstLoopBreak = 0;

    for ( int i = 0; i < ARRAY_SIZE_PROCESSING; i++ ) {
        max = fmax( processedArray[i], max );
        min = fmin( processedArray[i], min );
    }

    float threshold = ( max + min ) / 2.0 + 1 * StandardDeviation( processedArray );

    for ( int i = 0; i < ARRAY_SIZE_PROCESSING - 1; i++ ) {
    	if ( fabs( processedArray [i] - processedArray[i+1] ) > DIFFERENCE_THRESHOLD ) {
    		firstLoopBreak = 1;
    		break;
    	}
    }

    if ( firstLoopBreak ) {
		for ( int i = 0; i < ARRAY_SIZE_PROCESSING - 1; i++ ) {
			if ( processedArray[i] > threshold && processedArray[i] > processedArray[i - 1]
											   && processedArray[i] > processedArray[i + 1] )
			{
				step++;
				if ( step == 2 ) break;
			}
		}
    }

    return step;
}

static float calculateMean( const uint16_t *array ) {
    float sum = 0.0;

    for ( int i = 0; i < ARRAY_SIZE_PROCESSING; i++ ) {
        sum += array[i];
    }

    return sum / ( float )ARRAY_SIZE_PROCESSING;
}

static float StandardDeviation( const uint16_t *array ) {
    float mean = calculateMean(array);

    float sumSquaredDiff = 0.0;
    for ( int i = 0; i < ARRAY_SIZE_PROCESSING; i++ ) {
        float diff = array[i] - mean;
        sumSquaredDiff += diff * diff;
    }

    float sampleStdDev = sqrt( sumSquaredDiff / ( ARRAY_SIZE_PROCESSING - 1 ) );

    return sampleStdDev;
}

/**************************************************************************************/

static void printSteps( uint16_t stepCounter ) {
	char buffer [15];
	if ( xSemaphoreTake( xSemaphoreKm, (TickType_t) 10 ) == pdTRUE ) {
		char bufferKm[10];
		sprintf( bufferKm, "Walk: %d ", (uint8_t) stepCounter * 0.8);
		xSemaphoreTake( xSemaphoreLCD, portMAX_DELAY );
		lcd_draw_string( 10, 70, bufferKm, branco , 1, preto);
		xSemaphoreGive( xSemaphoreLCD );
	}
	sprintf( buffer, "Steps: %u ", stepCounter );
	xSemaphoreTake( xSemaphoreLCD, portMAX_DELAY );
	lcd_draw_string( 10, 50, buffer, branco , 1, preto);
	xSemaphoreGive( xSemaphoreLCD );
}

static int InitializeQueue ( void ) {
	if( xQueueLiveData == 0 )
	{
		xQueueLiveData = xQueueCreate( 5, sizeof( LiveData ) );
	}
	else return 1;
	if( xQueueButton == 0 )
	{
		xQueueButton = xQueueCreate( 10, sizeof( uint16_t ) );
	}
	else return 1;
	if( xQueueStepCounter == 0 )
	{
		xQueueStepCounter = xQueueCreate( 10, sizeof( uint8_t ) );
	}
	else return 1;
	if( xQueueProcessData == 0 )
	{
		xQueueProcessData = xQueueCreate( 10, sizeof( SensorData ) * ARRAY_SIZE_PROCESSING );
	}
	else return 1;
	if( xQueueProxy == 0 )
	{
		xQueueProxy = xQueueCreate( 5, sizeof( SensorData ) );
	}
	else return 1;
	return 0;
}

static void mainMenu ( void ) {
	printTitle();
	char bufferTaskNumber[15], bufferBytes[20], bufferQueueSpace[17], bufferTick[25];

	sprintf(bufferTick, "Tickcount: %lu ", xTaskGetTickCount());
	sprintf(bufferTaskNumber, "N. Tasks: %d ", uxTaskGetNumberOfTasks() );
	sprintf(bufferBytes, "Stack Used LCD: %u ", uxTaskGetStackHighWaterMark( HandleLCD ) );
	sprintf(bufferQueueSpace, "S.A Queue: %d ", uxQueueSpacesAvailable( xQueueProcessData ) );

	xSemaphoreTake( xSemaphoreLCD, portMAX_DELAY );
	lcd_draw_string( 10, 70, bufferTick, branco, 1, preto );
	lcd_draw_string( 10, 90, bufferTaskNumber, branco, 1, preto );
	lcd_draw_string( 10, 110, bufferBytes, branco, 1, preto );
	lcd_draw_string( 10, 130, bufferQueueSpace, branco, 1, preto );
	xSemaphoreGive( xSemaphoreLCD );

}

static void secondMenu ( void ) {
	printTitle();
	xSemaphoreGive( xSemaphoreKm );
}

static void printTitle ( void ) {
	xSemaphoreTake( xSemaphoreLCD, portMAX_DELAY );
	lcd_draw_string( 30, 20, "Step Counter", verde, 1, preto );
	xSemaphoreGive( xSemaphoreLCD );
}
