////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

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

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{
	struct encoder_state* oenc_left_handle = NULL;

	enc_init(OPTICAL_ENCODER_LEFT);

	oenc_left_handle = enc_get_handle(OPTICAL_ENCODER_LEFT);

	/* Will initialize TC module 1. */

	motor_pwm_tc_init();

	/* MOTOR_PWM_LEFT
	 * PB5 ( OC1A/PCINT5 )	Digital pin 11 (PWM)
	 *
	 */

	motor_pwm_start(MOTOR_PWM_LEFT, 3000);

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
        		,(void*) oenc_left_handle
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
	struct encoder_state* oenc_left = (struct encoder_state*) oenc_lh;

	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();



	while(1)
	{
		xSerialPrintf_P(PSTR("counter: %u.\r\n"), oenc_left->counter);
		xSerialPrintf_P(PSTR("period: %u.\r\n"), oenc_left->period);
		xSerialPrintf_P(PSTR("state: %u.\r\n"), oenc_left->state);
		xSerialPrintf_P(PSTR("last ICR val: %u.\r\n"), oenc_left->last_icr_val);
		xSerialPrintf_P(PSTR("TIFR: %x.\r\n"), TIFR4);

		xSerialPrintf_P(PSTR("ICR1: %u.\r\n"), ICR1);
		xSerialPrintf_P(PSTR("TCCR1A: %x.\r\n"), TCCR1A);
		xSerialPrintf_P(PSTR("TCCR1B: %x.\r\n"), TCCR1B);


		xSerialPrintf_P(PSTR("Pulse width (cycles): %d.\r\n"),
				motor_pwm_pulse_width_get(MOTOR_PWM_LEFT));

		xSerialPrintf_P(PSTR("***********************\r\n"));

		vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) );
	}
}

/*-----------------------------------------------------------*/

static void TaskServiceOpticalEncoder(void* pvParameters)
{
	(void) pvParameters;

	UBaseType_t ch;

	uint16_t pw;
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();


	while(1)
	    {
			enc_compute_tr_period(OPTICAL_ENCODER_LEFT);
			vTaskDelayUntil( &xLastWakeTime, ( 24 / portTICK_PERIOD_MS ) );


			while (xSerialAvailableChar(&xSerialPort) >= 1)
			{
				xSerialGetChar(&xSerialPort, &ch);

				pw = motor_pwm_pulse_width_get(MOTOR_PWM_LEFT);

				if (ch == 'p')
				{
					motor_pwm_pulse_width_set(MOTOR_PWM_LEFT, pw + 1);
				}
				else if (ch == 'o')
				{
					motor_pwm_pulse_width_set(MOTOR_PWM_LEFT, pw - 1);
				}
				else if (ch == ' ')
				{
					motor_pwm_pulse_width_set(MOTOR_PWM_LEFT, 2640);
				}
			}

	    }
}

/*-----------------------------------------------------------*/

static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;
    TickType_t xLastWakeTime;
    int current_speed;
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
