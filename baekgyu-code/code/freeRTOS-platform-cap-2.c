
/*
* Code patterns for FreeRTOS (We remove all error handling code for better understanding of code patterns).
 - Version 2
*/

/* Code pattern #1: Periodic task
*/

/* Declaration
*/

//Parameter declaration
const portTickType xPeriod_XTask = DEFAULT_PERIOD / portTICK_RATE_MS;	//Period
unsigned portBASE_TYPE XTask_PRIORITY = DEFAULT_PRIORITY; //Priority
unsigned portSHORT usStackDepth_XTask = DEFAULT_STACK_SIZE / STACK_VARIABLE_SIZE;	//Stack size

//Function declaration
void periodic_XTask_init( void );
void vXTask(void* pvParameters);

/* Task Initialization
*/
void periodic_XTask_init( void ){
	//Create task
	TaskCreate(vXTask, "XTask", usStackDepth_XTask, NULL, XTask_PRIORITY, NULL );
}


/* Task Execution
*/
void vXTask(void* pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		//Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xPeriod_XTask);
		//Perform action here.
		//(1)Read
		//(2)Compute
		//(3)Write
	}
}



/* Code pattern #2: Aperiodic task through blocking
*/

/* Declaration
*/
//Parameter declaration
unsigned portBASE_TYPE XTask_PRIORITY = DEFAULT_PRIORITY; //Priority
unsigned portSHORT usStackDepth_XTask = DEFAULT_STACK_SIZE / STACK_VARIABLE_SIZE;	//Stack size
static xQueueHandle xEvtQueueXTask;	//Event Queue for vXTask
const unsigned portBASE_TYPE xEvtQueueSizeXTask = DEFAULT_QUEUE_SIZE;	//Event queue size for XTask

//Function declaration
void aperiodic_XTask_init( void );
void vXTask(void* pvParameters);

/* Task Initialization
*/
void aperiodic_XTask_init( void ){
	//Create event queue
	xEvtQueueXTask = xQueueCreate( xEvtQueueSizeXTask, ( unsigned portBASE_TYPE ) sizeof( unsigned short ) );
	//Create task
	TaskCreate(vXTask, "XTask", usStackDepth_XTask, NULL, XTask_PRIORITY, NULL );
}


/* Task Execution
*/
void vXTask(void* pvParameters){
	for(;;){
		//Wait for the event. 
		//Blocking function interface: User should implement this function in a way it is unblocked if any of the inputs are triggerred.
		//Wait for item from queue.
		xQueueReceive( xEvtQueueXTask, &usData, portMAX_DELAY);	//Blocked until an event occurrs.
		//Perform action here.
		//(1)Read
		//(2)Compute
		//(3)Write		
	}
}

/* Code pattern #3: Data port
*/

/*Declaration
*/
//Parameter declaration
unsigned portBASE_TYPE DPORT_DPORTX;
xSemaphoreHandle xSemaphore_DPORTX;	//Mutex for the data port (FreeRTOS mutex follows priority inheritence protocol.)

//Function declaration
void DPORT_DPORTX_init( void );
unsigned portBASE_TYPE DPORT_Read_DPORTX( void );
unsigned portBASE_TYPE DPORT_Write_DPORTX( unsigned portBASE_TYPE val );

/* Port Initialization
*/

void DPORT_DPORTX_init( void ){
	DPORT_DPORTX = 0;		//Initialize the value
	xSemaphore_DPORTX = xSemaphoreCreateMutex();	//Initialize them using mutex
}

/* Data Port Read 
*/
unsigned portBASE_TYPE DPORT_Read_DPORTX( void ){
	unsigned portBASE_TYPE return_temp;
    xSemaphoreTake( xSemaphore, portMAX_DELAY );	//Blocked until it gets access
    return_temp = DPORT_DPORTX;
    xSemaphoreGive( xSemaphore );	//Release semaphore when it completes reading.
	return return_temp;

}

/* Data Port Write
*/
unsigned portBASE_TYPE DPORT_Write_DPORTX( unsigned portBASE_TYPE val ){
	xSemaphoreTake( xSemaphore, portMAX_DELAY );
    DPORT_DPORTX = val;
    xSemaphoreGive( xSemaphore );  
	return PDTRUE;
}



/* Code pattern #4: Event-Data port
*/

/*Declaration
*/
//Parameter declaration
static xQueueHandle xEDPort_EDPORTX;	//FIFO queue for the event data port EDPORTX
const unsigned portBASE_TYPE xQueueSizeEDPort_EDPORTX = DEFAULT_QUEUE_SIZE;	//Queue size for the event data port EDPORTX.
const unsigned portBASE_TYPE xDequeuePolicy = DEQUEUE_ONE_ITEM;		//Default dequeue policy is read one. {DEQUEUE_ONE_ITEM, DEQUEUE_ALL_ITEMS}
const unsigned portBASE_TYPE xQueue_EDPORTX_BlockMode = NONBLOCK;	//Default queue mode is non blocking {NONBLOCK, BLOCK} 
const unsigned portBASE_TYPE xQueue_EDPORTX_OverflowHandling = ERROR;	//Default queue overflow handling is error {DROP_OLDEST, DROP_NEWEST,ERROR}

//Function declaration
void EDPORT_EDPORTX_Init( void );
unsigned portBASE_TYPE EDPORT_ReadOne_EDPORTX( unsigned portBASE_TYPE* buf);
unsigned portBASE_TYPE EDPORT_ReadAll_EDPORTX( unsigned portBASE_TYPE* buf);
unsigned portBASE_TYPE EDPORT_Write_EDPORTX( unsigned portBASE_TYPE* buf);

/* Port Initialization
*/
void EDPORT_EDPORTX_Init( void ){
	//Create event queue
	xEDPort_EDPORTX = xQueueCreate( xQueueSizeEDPort_EDPORTX, ( unsigned portBASE_TYPE ) sizeof( unsigned short ) );
}

/* Event Data Port Read
   @buf: the buffer that stores item in the queue.
   @timeout: specify blocking/non-blocking/timed-blocking mode.
   @Return the size of item read.
*/

//Read one
unsigned portBASE_TYPE EDPORT_ReadOne_EDPORTX( unsigned portBASE_TYPE* buf){
	
	xQueueReceive( xEDPort_EDPORTX, &buf, xQueue_EDPORTX_BlockMode);	//Blocked until an event occurrs.
	return PDTRUE;
	
}

//Read all
unsigned portBASE_TYPE EDPORT_ReadAll_EDPORTX( unsigned portBASE_TYPE* buf){

	unsigned portBASE_TYPE item_count = 0;

	//Implement the concept of freeze.
	//Get the number of items in the queue.
	item_count = uxQueueMessagesWaiting( *( ( xQueueHandle * ) &xEDPort_EDPORTX ) );
	//Read these items from the queue.
	for(int i = 0 ; i < item_count ; i++){
		
		xQueueReceive( *( ( xQueueHandle * ) &xEDPort_EDPORTX ), buf + i*sizeof(unsigned portBASE_TYPE), xQueue_EDPORTX_BlockMode ) == pdPASS )
	
	}
	return item_count;
}

/* Event Data Port Send
*/

unsigned portBASE_TYPE EDPORT_Write_EDPORTX( unsigned portBASE_TYPE* buf){
	unsigned portBASE_TYPE result = xQueueSend( xEDPort_EDPORTX, ( void * ) buf, xQueue_EDPORTX_BlockMode );
	if(result == PDTRUE){
		return PDTRUE;
	}else
	{
		switch(xQueue_EDPORTX_OverflowHandling){
		case DROP_OLDEST:
			//Drop oldest and enqueue
			break;
		case DROP_NEWEST:
			//Drop newest and enqueue
			break;
		case ERROR:
			//Raise an exception
			break;
		default:
		}
		return PDFALSE;
	}	
}


