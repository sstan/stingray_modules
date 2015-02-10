#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


#include <avr/io.h>
#include <avr/interrupt.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* serial interface include file. */
#include "serial.h"
#include "motor_pwm.h"
#include "o_enc.h"
#include "tc_tools.h"

/*-----------------------------------------------------------*/
/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort;

static void TaskBlinkRedLED(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
static void TaskServiceOpticalEncoder(void* pvParameters);
static void TaskOencInfo(void* pvParameters);

/*-----------------------------------------------------------*/
uint32_t timestamp_g[2];
uint32_t enc_cntr[2];

evt_callback_ptr_t encoder_evt_callback(uint8_t enc_id, uint32_t timestamp)
{
	timestamp_g[enc_id] = timestamp;
	enc_cntr[enc_id]++;

	if (PORTB) PORTB = 0;
	else PORTB |= 1 << 7;

}


/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{
	tc_module_init(TC_MODULE_4);
	tc_module_init(TC_MODULE_5);

	DDRB  |= _BV(DDB7);

	enc_init(ENC_ID_ENCODER_LEFT, &encoder_evt_callback);
	enc_init(ENC_ID_ENCODER_RIGHT, &encoder_evt_callback);

	servo_start(MOTOR_PWM_LEFT, 3000);
	servo_start(MOTOR_PWM_RIGHT, 2690);
	servo_start(MOTOR_PWM_CENTER, 2000);

    // turn on the serial port for debugging or for other USART reasons.
	// serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)
	xSerialPort = xSerialPortInitMinimal(USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX);

	avrSerialPrint_P(PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

	/*
    xTaskCreate(
		TaskBlinkRedLED
		,(const portCHAR *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
		,256				// Tested 9 free @ 208
		,NULL
		,3
		,NULL );
		*/

    xTaskCreate(
    		TaskServiceOpticalEncoder
    		,(const portCHAR *)"oencs"
    		,100
    		,NULL
    		,3
    		,NULL);

    xTaskCreate(
        		TaskOencInfo
        		,(const portCHAR *)"oencInfo"
        		,256
        		,(void*) NULL
        		,3
        		,NULL);

    // needs heap_1 or heap_2 for this function to succeed.
	avrSerialPrintf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"), xPortGetFreeHeapSize());

	vTaskStartScheduler();

	avrSerialPrint_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...
}

/*-----------------------------------------------------------*/

static void TaskOencInfo(void* oenc_lh)
{


	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();




	while(1)
	{

		xSerialPrintf_P(PSTR("timestamp_g[0]: %u.\r\n"), timestamp_g[0]);
		xSerialPrintf_P(PSTR("timestamp_g[1]: %u.\r\n"), timestamp_g[1]);

		xSerialPrintf_P(PSTR("enc_cntr[0]: %u.\r\n"), enc_cntr[0]);
		xSerialPrintf_P(PSTR("enc_cntr[1]: %u.\r\n"), enc_cntr[1]);

		xSerialPrintf_P(PSTR("Pulse width (cycles): %d.\r\n"),
				servo_pulse_width_get(MOTOR_PWM_LEFT));

		xSerialPrintf_P(PSTR("***********************\r\n"));

		vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) );
	}
}

/*-----------------------------------------------------------*/

static void TaskServiceOpticalEncoder(void* pvParameters)
{
	(void) pvParameters;

	UBaseType_t ch;

	uint32_t pw;
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();


	while(1)
	    {

			vTaskDelayUntil( &xLastWakeTime, ( 24 / portTICK_PERIOD_MS ) );


			while (xSerialAvailableChar(&xSerialPort) >= 1)
			{
				xSerialGetChar(&xSerialPort, &ch);

				pw = servo_pulse_width_get(MOTOR_PWM_LEFT);

				if (ch == 'p')
				{
					servo_pulse_width_set(MOTOR_PWM_LEFT, pw + 1);
				}
				else if (ch == 'o')
				{
					servo_pulse_width_set(MOTOR_PWM_LEFT, pw - 1);
				}
				else if (ch == ' ')
				{
					servo_pulse_width_set(MOTOR_PWM_LEFT, 2640);
				}
			}

	    }
}

/*-----------------------------------------------------------*/

static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;
    TickType_t xLastWakeTime;

    const float target_speed = 12.0;


	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	DDRB |= _BV(DDB7);

	xSerialPrintf_P(PSTR("target speed: %u.\r\n"), (int) target_speed);

	while(1)
    {

		vTaskDelayUntil( &xLastWakeTime, ( 24 / portTICK_PERIOD_MS ) );
    }
}

/*-----------------------------------------------------------*/


void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    portCHAR *pcTaskName )
{

	DDRB  |= _BV(DDB7);
	PORTB |= _BV(PORTB7);       // main (red PB7) LED on. Mega main LED on and die.
	while(1);
}

/*-----------------------------------------------------------*/
