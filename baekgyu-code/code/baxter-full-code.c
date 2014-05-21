/*
    FreeRTOS V7.3.0 - Copyright (C) 2012 Real Time Engineers Ltd.

    FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT 
    http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!
    
    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    
    http://www.FreeRTOS.org - Documentation, training, latest versions, license 
    and contact details.  
    
    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell 
    the code with commercial support, indemnification, and middleware, under 
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under 
    the SafeRTOS brand: http://www.SafeRTOS.com.
*/

/*
	NOTE : Tasks run in System mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/

/*
 * This demo includes a (basic) USB mouse driver and a WEB server.  It is
 * targeted for the AT91SAM7X EK prototyping board which includes a small
 * joystick to provide the mouse inputs.  The WEB interface provides some basic
 * interactivity through the use of a check box to turn on and off an LED.
 *
 * main() creates the WEB server, USB, and a set of the standard demo tasks
 * before starting the scheduler.  See the online FreeRTOS.org documentation 
 * for more information on the standard demo tasks.  
 *
 * LEDs D1 to D3 are controlled by the standard 'flash' tasks - each will 
 * toggle at a different fixed frequency.
 *
 * A tick hook function is used to monitor the standard demo tasks - with LED
 * D4 being used to indicate the system status.  D4 toggling every 5 seconds
 * indicates that all the standard demo tasks are executing without error.  The
 * toggle rate increasing to 500ms is indicative of an error having been found
 * in at least one demo task.
 *
 * See the online documentation page that accompanies this demo for full setup
 * and usage information.
 */

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo application includes. */
#include "partest.h"
//#include "USBSample.h"
//#include "uIP_Task.h"
//#include "BlockQ.h"
//#include "blocktim.h"
//#include "flash.h"
//#include "QPeek.h"
//#include "dynamic.h"

/*My include*/
#include "ParTest/pwm.h"
extern int Pwm_Init( void );

/* Priorities for the demo application tasks. */
/*#define mainUIP_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainUSB_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainFLASH_PRIORITY                  ( tskIDLE_PRIORITY + 2 )
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY ) 
*/
/* The task allocated to the uIP task is large to account for its use of the
sprintf() library function.  Use of a cut down printf() library would allow
the stack usage to be greatly reduced. */
//#define mainUIP_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 6 )

/* The LED toggle by the tick hook should an error have been found in a task. */
#define mainERROR_LED						( 3 )

/*-----------------------------------------------------------*/

/*
 * Configure the processor for use with the Atmel demo board.  Setup is minimal
 * as the low level init function (called from the startup asm file) takes care
 * of most things.
 */
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/
//My definition
int LED_TOGGLE_STAT = 0;
#define myLED_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )
void vMyLEDTask(void*);

//Task declaration
void vPITask(void*);
void vEmptyRsvDetectionTask(void*);
void vPumpMotorCtrTask(void*);
void vAlarmCtrTask(void*);
void vBolusReqTask(void*);

//Task configuration
const portTickType xPeriod_PITask = 100 / portTICK_RATE_MS;
const portTickType xPeriod_EmptyRsvDetectionTask = 1000 / portTICK_RATE_MS;
const portTickType xPeriod_PumpMotorCtrTask = 300 / portTICK_RATE_MS;
const portTickType xPeriod_AlarmCtrTask = 500 / portTICK_RATE_MS;
const portTickType xPeriod_BolusReqTask = 500 / portTICK_RATE_MS;

//Priority is assigned using rate-monotonic scheduling algorithm
#define PITask_PRIORITY			( tskIDLE_PRIORITY + 5 )
#define EmptyRsvDetectionTask_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define PumpMotorCtrTask_PRIORITY		( tskIDLE_PRIORITY + 6 )
#define AlarmCtrTask_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define BolusReqTask_PRIORITY    ( tskIDLE_PRIORITY + 4 )

//Queue declaration (Non-Blocking)
#define pollq_BolusReq_QUEUE_SIZE (10)
#define pollq_EmptyRsv_QUEUE_SIZE (10)
#define pollqNO_DELAY			( ( portTickType ) 0 )

/* Structure used to pass parameters to the blocking queue tasks. */
typedef struct BLOCKING_QUEUE_PARAMETERS
{
	xQueueHandle xQueue;					/*< The queue to be used by the task. */
	portTickType xBlockTime;				/*< The block time to use on queue reads/writes. */
	volatile short *psCheckVariable;	/*< Incremented on each successful cycle to check the task is still running. */
} xBlockingQueueParameters;

xBlockingQueueParameters *pxQueueT1R, *pxQueueT1S, *pxQueueT2R, *pxQueueT2S;
const portTickType xBlockTime = ( portTickType ) 2000 / portTICK_RATE_MS;
const portTickType xDontBlock = ( portTickType ) 0;

const unsigned portBASE_TYPE uxQueueSize1 = 1, uxQueueSize5 = 5;

/*Polled Queue*/
static xQueueHandle xQueueT3;
static xQueueHandle xQueueT4;

/*Event ID*/
#define Evt_BolusReq 1
#define Evt_EmptyRsv 2
#define Evt_NonEmptyRsv 3

/*-----------------------------------------------------------*/


/*
 * Starts all the other tasks, then starts the scheduler.
 */
int main( void )
{
	/*FIFO queue declaration*/
	//static xQueueHandle xPolled_BolusReq_Queue;
	//static xQueueHandle xPolled_EmptyRsv_Queue;

	/* Setup any hardware that has not already been configured by the low
	level init routines. */
	prvSetupHardware();

	/* Start the task that handles the TCP/IP and WEB server functionality. */
    //xTaskCreate( vuIP_Task, "uIP", mainUIP_TASK_STACK_SIZE, NULL, mainUIP_PRIORITY, NULL );
	
	/* Also start the USB demo which is just for the SAM7. */
    //vStartUSBTask( mainUSB_PRIORITY );
	
	/* Start the standard demo tasks. */
	//vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
    //vCreateBlockTimeTasks();
    //vStartLEDFlashTasks( mainFLASH_PRIORITY );
    //vStartGenericQueueTasks( mainGEN_QUEUE_TASK_PRIORITY );
    //vStartQueuePeekTasks();
    //vStartDynamicPriorityTasks();

	/*T1 queue initialization*/
	/* First create the structure used to pass parameters to the consumer tasks. */
	pxQueueT1R = ( xBlockingQueueParameters * ) pvPortMalloc( sizeof( xBlockingQueueParameters ) );
	/* Create the queue used by the first two tasks to pass the incrementing number.
		Pass a pointer to the queue in the parameter structure. */
	pxQueueT1R->xQueue = xQueueCreate( uxQueueSize5, ( unsigned portBASE_TYPE ) sizeof( unsigned short ) );
	/* The consumer is created first so gets a block time as described above. */
	pxQueueT1R->xBlockTime = xBlockTime;
	/* Create the structure used to pass parameters to the producer task. */
	pxQueueT1S = ( xBlockingQueueParameters * ) pvPortMalloc( sizeof( xBlockingQueueParameters ) );
	/* Pass the queue to this task also, using the parameter structure. */
	pxQueueT1S->xQueue = pxQueueT1R->xQueue;
	/* The producer is not going to block - as soon as it posts the consumer will
			wake and remove the item so the producer should always have room to post. */
	pxQueueT1S->xBlockTime = xDontBlock;

	/*T2 Queue initialization*/
	/* First create the structure used to pass parameters to the consumer tasks. */
	pxQueueT2R = ( xBlockingQueueParameters * ) pvPortMalloc( sizeof( xBlockingQueueParameters ) );
	/* Create the queue used by the first two tasks to pass the incrementing number.
		Pass a pointer to the queue in the parameter structure. */
	pxQueueT2R->xQueue = xQueueCreate( uxQueueSize5, ( unsigned portBASE_TYPE ) sizeof( unsigned short ) );
	/* The consumer is created first so gets a block time as described above. */
	pxQueueT2R->xBlockTime = xBlockTime;
	/* Create the structure used to pass parameters to the producer task. */
	pxQueueT2S = ( xBlockingQueueParameters * ) pvPortMalloc( sizeof( xBlockingQueueParameters ) );
	/* Pass the queue to this task also, using the parameter structure. */
	pxQueueT2S->xQueue = pxQueueT2R->xQueue;
	/* The producer is not going to block - as soon as it posts the consumer will
			wake and remove the item so the producer should always have room to post. */
	pxQueueT2S->xBlockTime = xDontBlock;


	/* Pass in the variable that this task is going to increment so we can check
		it is still running. */
	//pxQueueParameters2->psCheckVariable = &( sBlockingProducerCount[ 0 ] );

	/* Create the queue used by the producer and consumer. */
	//xPolled_BolusReq_Queue = xQueueCreate( pollq_BolusReq_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( unsigned short ) );
	//xPolled_EmptyRsv_Queue = xQueueCreate( pollq_EmptyRsv_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( unsigned short ) );

	xQueueT3 = xQueueCreate( pollq_BolusReq_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( unsigned short ) );
	xQueueT4 = xQueueCreate( pollq_EmptyRsv_QUEUE_SIZE, ( unsigned portBASE_TYPE ) sizeof( unsigned short ) );

	/* Create the queue*/
	//vQueueAddToRegistry( xPolled_BolusReq_Queue, ( signed char * ) "Poll_BolusReq_Queue" );
	//vQueueAddToRegistry( xPolled_EmptyRsv_Queue, ( signed char * ) "Poll_EmptyRsv_Queue" );


	/*My tasks declaration*/
	//xTaskCreate(vMyLEDTask, "myLED", configMINIMAL_STACK_SIZE*3, NULL, myLED_TASK_PRIORITY, NULL );

	xTaskCreate(vPITask, "PITask", configMINIMAL_STACK_SIZE*3, NULL, PITask_PRIORITY, NULL );

	//xTaskCreate(vEmptyRsvDetectionTask, "EmptyRsvDetectionTask", configMINIMAL_STACK_SIZE*3, &xPolled_EmptyRsv_Queue, EmptyRsvDetectionTask_PRIORITY, NULL );
	xTaskCreate(vEmptyRsvDetectionTask, "EmptyRsvDetectionTask", configMINIMAL_STACK_SIZE*3, NULL, EmptyRsvDetectionTask_PRIORITY, NULL );


	xTaskCreate(vPumpMotorCtrTask, "PumpMotorCtrTask", configMINIMAL_STACK_SIZE*3, NULL, PumpMotorCtrTask_PRIORITY, NULL );
	//xTaskCreate(vPumpMotorCtrTask, "PumpMotorCtrTask", configMINIMAL_STACK_SIZE*3, &xPolled_BolusReq_Queue, PumpMotorCtrTask_PRIORITY, NULL );


	xTaskCreate(vAlarmCtrTask, "AlarmCtrTask", configMINIMAL_STACK_SIZE*3, NULL, AlarmCtrTask_PRIORITY, NULL );
	//xTaskCreate(vAlarmCtrTask, "AlarmCtrTask", configMINIMAL_STACK_SIZE*3, &xPolled_EmptyRsv_Queue, AlarmCtrTask_PRIORITY, NULL );


	xTaskCreate(vBolusReqTask, "BolusReqTask", configMINIMAL_STACK_SIZE*3, NULL, BolusReqTask_PRIORITY, NULL );
	//xTaskCreate(vBolusReqTask, "BolusReqTask", configMINIMAL_STACK_SIZE*3, &xPolled_BolusReq_Queue, BolusReqTask_PRIORITY, NULL );

	/* Start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler. */
  	return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	//portDISABLE_INTERRUPTS();
	
	/* When using the JTAG debugger the hardware is not always initialised to
	the correct default state.  This line just ensures that this does not
	cause all interrupts to be masked at the start. */
	AT91C_BASE_AIC->AIC_EOICR = 0;
	
	/* Most setup is performed by the low level init function called from the
	startup asm file. */

	/* Enable the peripheral clock. */
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOB;
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_EMAC;

	/* Initialise the LED outputs for use by the demo application tasks. */
	vParTestInitialise();

	/* Initialize PWM signal*/
	vParTestSetLED(0, 0);
	//Pwm_Stop(0);
	//Pwm_Start(0);
	//Pwm_Set(0, 128);

	vParTestSetLED(1, 0);
	//For buzzer
	//Pwm_Start(1);
	//Pwm_Set(1, 128);

	/*Initialize joystick interrupt*/
	joystick_Interrupt_Init();

	testPIO_Output_Init();

	emptyRsv_Sensor_Init();

}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	//This function gets called every tick. Please refer to the original code.
}


//My code
void vMyLEDTask(void* pvParameters){

	const portTickType xDelay = 3000 / portTICK_RATE_MS;
	for(;;){
		//Task code
		if(LED_TOGGLE_STAT == 0){
			//Turn on LED DS2
			//vParTestSetLED(1, 1);
			LED_TOGGLE_STAT = 1;
			testPIO_Output_On();
		}else{
			//Turn off LED DS2
			//vParTestSetLED(1, 0);
			LED_TOGGLE_STAT = 0;
			testPIO_Output_Off();
		}

		vTaskDelay(xDelay);
	}
}

//A task for the Platform Independent Code (APeriodic version)
void vPITask(void* pvParameters){
	unsigned short usData = 0;
	for(;;){
		//Wait for item from queue.
		if( xQueueReceive( pxQueueT1R->xQueue, &usData, pxQueueT1R->xBlockTime ) == pdPASS )
		{
			//vParTestToggleLED(3);
			switch(usData){
			case Evt_BolusReq:
				// Send an event to T3 so that pump motor control task can process it.
				if( xQueueSend( *( ( xQueueHandle * ) &xQueueT3 ), ( void * ) &usData, pollqNO_DELAY ) != pdPASS )
				{
					// We should never find the queue full so if we get here there has been an error.
					//vParTestToggleLED(2);
				}
				else
				{
					//enqueuing successful.
					vParTestToggleLED(3);
				}
				break;
			case Evt_EmptyRsv :
				// Send an event to T4 so that alarm control task can process it.
				if( xQueueSend( *( ( xQueueHandle * ) &xQueueT4 ), ( void * ) &usData, pollqNO_DELAY ) != pdPASS )
				{
					// We should never find the queue full so if we get here there has been an error.
					//vParTestToggleLED(2);
				}
				else
				{
					//enqueuing successful.
					//vParTestToggleLED(3);
				}
				break;
			case Evt_NonEmptyRsv:
				// Send an event to T4 so that alarm control task can process it.
				if( xQueueSend( *( ( xQueueHandle * ) &xQueueT4 ), ( void * ) &usData, pollqNO_DELAY ) != pdPASS )
				{
					// We should never find the queue full so if we get here there has been an error.
					vParTestToggleLED(2);
				}
				else
				{
					//enqueuing successful.
					vParTestToggleLED(3);
				}
				break;
			default:
				break;
			}

		}else{
			//Error in reading queue
			//vParTestToggleLED(3);
		}
		//Perform action here.
		//vParTestToggleLED(3);
	}
}


//Using T1 queue, which is shared with BolusRequest Thread.
void vEmptyRsvDetectionTask(void* pvParameters){
	portTickType xLastWakeTime;
	unsigned short usValue = 0;

	signed portBASE_TYPE xError = pdFALSE;
	//Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
		//Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xPeriod_EmptyRsvDetectionTask);

		//Perform action here.
		//vParTestToggleLED(2);
		if(get_EmptyRsv_Val() == 1){ //if switch is closed, raise an alarm.
			//testPIO_Output_On();
			// Send an event on the queue without blocking.
			usValue = (unsigned short)Evt_EmptyRsv;
			if( xQueueSend( *( ( xQueueHandle * ) pxQueueT1S ), ( void * ) &usValue, pollqNO_DELAY ) != pdPASS )
			{
				// We should never find the queue full so if we get here there has been an error.
				xError = pdTRUE;
				//vParTestToggleLED(2);
			}
			else
			{
				//enqueuing successful.
			}
		}else{						//if switch is opened, continue infusion.
			//testPIO_Output_Off();
			// Send an event on the queue without blocking.
			usValue = (unsigned short)Evt_NonEmptyRsv;
			if( xQueueSend( *( ( xQueueHandle * ) pxQueueT1S ), ( void * ) &usValue, pollqNO_DELAY ) != pdPASS )
			{
				// We should never find the queue full so if we get here there has been an error.
				xError = pdTRUE;
				//vParTestToggleLED(2);
			}
			else
			{
				//enqueuing successful.
			}
		}
	}
}


void vPumpMotorCtrTask(void* pvParameters){
	portTickType xLastWakeTime;
	unsigned short usData = ( unsigned short ) 0;
	unsigned short usExpectedValue = ( unsigned short ) 3;
	signed portBASE_TYPE bolusStart = pdFALSE;
	signed portBASE_TYPE bolusInProgress = pdFALSE;
	signed portBASE_TYPE xError = pdFALSE;
	const short lockout_max = ( unsigned short ) 30;
	unsigned short lockout_counter = ( unsigned short ) 0;
	//Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
		//Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xPeriod_PumpMotorCtrTask);

		//Perform action here.
		//vParTestToggleLED(3);

		// Loop until the queue is empty.
		while( uxQueueMessagesWaiting( *( ( xQueueHandle * ) &xQueueT3 ) ) )
		{
			//vParTestToggleLED(3);
			if( xQueueReceive( *( ( xQueueHandle * ) &xQueueT3 ), &usData, pollqNO_DELAY ) == pdPASS )
			{
				//if received any event
				// Set the bolus req
				bolusStart = pdTRUE;
				//vParTestToggleLED(3);

				// Reset the temp buffer
				usData = ( unsigned short ) 0;
			}
		}

		//vParTestToggleLED(2);
		if(bolusStart == pdTRUE){
			//vParTestToggleLED(2);
			if(bolusInProgress == pdFALSE){
				//vParTestToggleLED(2);
				Pwm_Start(0);
				Pwm_Set(0, 128);  //TODO: should stop after some time
				lockout_counter = lockout_max;
				bolusInProgress = pdTRUE;
				//vParTestToggleLED(1);
			}
			bolusStart = pdFALSE;
		}

		//Decrease lockout counter.
		if(bolusInProgress == pdTRUE){
			if(lockout_counter > 0){
				lockout_counter--;
			}else{
				Pwm_Stop(0);
				lockout_counter = 0;
				bolusInProgress = pdFALSE;
			}
		}
		//Update bolusStart variable
		bolusStart = pdFALSE;
	}
}


//Queue connected to PI routine.
void vAlarmCtrTask(void* pvParameters){
	portTickType xLastWakeTime;
	signed portBASE_TYPE alarmStatus = pdFALSE;
	unsigned short usData = ( unsigned short ) 0;

	//Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
		//Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xPeriod_AlarmCtrTask);

		//Perform action here.

		// Loop until the queue is empty.(Dequeue policy: keep the most recent one)
		while( uxQueueMessagesWaiting( *( ( xQueueHandle * ) &xQueueT4 ) ) )
		{
			//vParTestToggleLED(3);
			if( xQueueReceive( *( ( xQueueHandle * ) &xQueueT4 ), &usData, pollqNO_DELAY ) == pdPASS )
			{
				//if received any event
				// Set the bolus req
				//alarmStatus = pdTRUE;
				//vParTestToggleLED(3);

				if( usData == Evt_EmptyRsv )
				{
					alarmStatus = pdTRUE;
					//vParTestToggleLED(3);
				}else if( usData == Evt_NonEmptyRsv ){
					alarmStatus = pdFALSE;
				}else{
				}
				// Reset the temp buffer
				usData = ( unsigned short ) 0;
			}
		}

		if(alarmStatus == pdTRUE){ //if alarm event is observed,
			testPIO_Output_On();
			Pwm_Start(1);
			//Pwm_Set(1, 128);
			Pwm_Set(1, 50);  //Raise an alarm. This works.
			Pwm_Stop(0); //Stop motor.
		}else{						//if switch is opened, continue infusion.
			testPIO_Output_Off();
			Pwm_Stop(1);
		}

		//vParTestToggleLED(0);
	}
}


void vBolusReqTask(void* pvParameters){
	portTickType xLastWakeTime;
	short sErrorEverOccurred = pdFALSE;
	unsigned short usData = ( unsigned short ) 0;
	//Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	signed portBASE_TYPE xError = pdFALSE;

	for(;;){
		//Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xPeriod_BolusReqTask);
		//Perform action here.
		if(get_PCB_Val() == 1){ //if switch is closed, start bolus request
			usData = (unsigned short)Evt_BolusReq;
			if( xQueueSend( pxQueueT1S->xQueue, ( void * ) &usData, pxQueueT1S->xBlockTime ) != pdPASS ) //this is non-blocking queue
			{
				sErrorEverOccurred = pdTRUE;
				//vParTestToggleLED(2);
			}
			else
			{
				/* We have successfully posted a message, so increment the variable
				used to check we are still running. */
				vParTestToggleLED(2);

			}
		}
		else{ //if switch is opened, no bolus request.

		}
		//vParTestToggleLED(3);
	}
}
