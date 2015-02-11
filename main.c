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

static void TaskInfo(void* arg);
static void TaskControl(void* enc_id_p);

uint32_t delta_ts_g[2];

/*-----------------------------------------------------------*/

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{
	tc_module_init(TC_MODULE_4);
	tc_module_init(TC_MODULE_5);

	DDRB  |= _BV(DDB7);

	enc_init(ENC_ID_ENCODER_LEFT);
	enc_init(ENC_ID_ENCODER_RIGHT);

	servo_start(MOTOR_PWM_LEFT, 3000);
	servo_start(MOTOR_PWM_RIGHT, 2690);
	//servo_start(MOTOR_PWM_CENTER, 2000);

    // turn on the serial port for debugging or for other USART reasons.
	// serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)
	xSerialPort = xSerialPortInitMinimal(USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX);

	avrSerialPrint_P(PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
    		TaskControl
    		,(const portCHAR *)"controlLeft"
    		,200
    		,(void*) ENC_ID_ENCODER_LEFT
    		,3
    		,NULL);

    xTaskCreate(
    		TaskControl
    		,(const portCHAR *)"controlRight"
    		,200
    		, (void*) ENC_ID_ENCODER_RIGHT
    		,3
    		,NULL);

    xTaskCreate(
    		TaskInfo
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

static void TaskInfo(void* arg)
{
	(void*) arg;

	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{

		xSerialPrintf_P(PSTR("delta_ts_g[0]: %u.\r\n"), delta_ts_g[0]/2000);
		xSerialPrintf_P(PSTR("delta_ts_g[1]: %u.\r\n"), delta_ts_g[1]/2000);

		xSerialPrintf_P(PSTR("enc_cntr[0]: %u.\r\n"), enc_get_count(0));
		xSerialPrintf_P(PSTR("enc_cntr[1]: %u.\r\n"), enc_get_count(1));

		xSerialPrintf_P(PSTR("Pulse width (cycles): %d.\r\n"),
				servo_pulse_width_get(MOTOR_PWM_LEFT));

		xSerialPrintf_P(PSTR("***********************\r\n"));

		vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) );
	}
}

/*-----------------------------------------------------------*/

static void TaskControl(void* enc_id_p)
{
	uint32_t ts[2];
	uint32_t delta_ts;
	uint8_t enc_id = (uint8_t)enc_id_p;

	uint32_t pw;

	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();


	while(1)
	    {
			if (enc_read_timestamp(enc_id, &ts[0]) == 0)
			{
				vTaskDelayUntil( &xLastWakeTime, (500/portTICK_PERIOD_MS));
				continue;
			}
			if (enc_read_timestamp(enc_id, &ts[1]) == 0)
			{
				vTaskDelayUntil( &xLastWakeTime, (500/portTICK_PERIOD_MS));
				continue;
			}

			if (ts[1] > ts[0])
			delta_ts_g[enc_id] = delta_ts = ts[1] - ts[0];
	    }
}


void vApplicationStackOverflowHook( TaskHandle_t xTask,
portCHAR *pcTaskName )
{
DDRB |= _BV(DDB7);
PORTB |= _BV(PORTB7); // main (red PB7) LED on. Mega main LED on and die.
while(1);
}

