/* LET platform
*/
/* Code pattern #0: LET initialization
*/

/*
LET parameter declarartion
*/

LETaskSet* taskSets;		//Pointer to the task sets.
//const int LET_INTERVAL_MS = 10;
const int LET_READ_WCET = 5;		//in ms.
const int LET_WRITE_WCET = 5;
const int LET_COMPUTE_WCET = 40;
unsigned int LET_MODE = LET_READ;	//{LET_READ, LET_WRITE, LET_COMPUTE}
unsigned int LET_ROUND = 0;
void LET_TRIGGER(void);
void LET_TIMER_RESET(void (*ptrTriggerFunc)(void), unsigned int timeout);
/*
LET initialization
*/
void LET_INIT(){
	//intitializat LET.
	//Disable interrupt
	LET_MODE = LET_READ;
	//Reset the timer to trigger after OFFSET.
	LET_TIMER_RESET(LET_TRIGGER, OFFSET);
	//Enable intererupt
}
/*
LET trigger execution
*/

void LET_TRIGGER(void){ 
	switch (LET_MODE)
	{
	case LET_READ:
			//1. Disable interrupt
			//2. Reset the timer to READ_WCET and start the timer
			LET_TIMER_RESET(LET_TRIGGER, LET_READ_WCET);
			//3. Update the next mode
			LET_MODE = LET_COMPUTE;
			//4. Read from ports	//Assumption Reading from port should be done before READ_WCET 
			//5. Enable interrupts	
		break;	
	case LET_COMPUTE;
			//1. Let interrupt enabled
			//2. Reset the timer to the COMPUTE_WCET and start the timer
			LET_TIMER_RESET(LET_TRIGGER, LET_COMPUTE_WCET);
			//3. Update the next mode
			LET_MODE = LET_WRITE;
			//4. Compute tasks based on the input read in LET_READ
			LET_Schedule_Tasks(taskSets);	//Assumption: Computation of taskset should be done before COMPUTE_WCET
		break;
	case LET_WRITE:
			//1. Disable interrupt
			//2. Reset the timer to the WRITE_WCET
			LET_TIMER_RESET(LET_TRIGGER, LET_READ_WCET);
			//3. Update the next mode
			LET_MODE = LET_READ;
			//4. Write to ports
			//5. Enable interrupts
		break;
	}
}

/*void LET_TRIGGER_Xms(void){ 
	DisableInterrupts;
	//This is instantaneous communication; it takes logically zero time.
	//Write to ports: Write(ports, out(n-1))
	//Read from ports: in(n) = Read(ports)
	EnableInterrupts;
	//This is a scheduled computation.
	//Compute tasks using any scheduling algorithms
	LET_Schedule_Tasks(taskSets);
}*/

LETOUT* LET_Schedule_Tasks(LETIN* inputSet, LETaskSet* taskSets){
	//Platform dependent schedule.

	//Example
	for(int i = 0; i < numofTask(taskSets) ; i++){
		taskSets[i].taskFunc();
	}
}


/* Code pattern #1: Periodic task
*/

/* Parameter Declaration
*/
const int xPeriod_XTask = 500 / LET_INTERVAL_MS;	//Period
unsigned int elapsedRound_XTask = 0;					//Store the number of elapsed round since last execution.
//unsigned int XTask_PRIORITY = 5 //Priority
void vXTask(void* pvParameters);

/* Task Initialization
*/
void periodic_XTask_init(){
	//Create task	
	register_task(XTask, xPeriod_XTask, XTask_Priority);
}

/* Task Execution
*/
void vXTask(void* pvParameters){

	//only if the current round is the exeution round of XTask, perform actions
	if(elapsedRound_XTask == xPeriod_XTask){
		//Perform action here.
		//(1)temp_Read
		//(2)Compute
		//(3)temp_Write
		elapsedRound_XTask = 0;
	}	
}

/* Code pattern #2: Aperiodic task 
*/
unsigned int eventFlag_XTask = 0;	//This variable should be updated in the instantaneous phase. 
/* Task Execution
*/
void vXTask(void* pvParameters){

	//only if the reading has the event that invokes the aperiodic task, perform actions
	if(eventFlag_XTask){
		//Perform action here.
		//(1)Read
		//(2)Compute 
		//(3)Write
		eventFlag_XTask_XTask = 0;	//Reset the event flag.
	}	
}

/* Code pattern #3: Data port
*/

/*Declaration
*/
//Parameter declaration
unsigned int LET_DPORT_DPORTX;

//Function declaration
void LET_DPORT_DPORTX_init( void );
unsigned int LET_DPORT_Read_DPORTX( void );
unsigned int LET_DPORT_Write_DPORTX( unsigned int val );

/* Port Initialization
*/

void LET_DPORT_DPORTX_init( void ){
	LET_DPORT_DPORTX = 0;		//Initialize the value
}

/* Data Port Read 
*/
unsigned int LET_DPORT_Read_DPORTX( void ){
	unsigned int return_temp;
    return_temp = DPORT_DPORTX;
	return return_temp;

}

/* Data Port Write
*/
unsigned int LET_DPORT_Write_DPORTX( unsigned int val ){
    DPORT_DPORTX = val;
	return PDTRUE;
}

/* Code pattern #4: Event-Data port (EDPORTX)
*/

/*Declaration
*/
//Parameter declaration
static int LET_QueueID_EDPORTX;	//Queue ID
const unsigned int LET_QueueSize_EDPORTX = DEFAULT_QUEUE_SIZE;	//Queue size for the event data port EDPORTX.
const unsigned int LET_DequeuePolicy = DEQUEUE_ONE_ITEM;		//Default dequeue policy is read one. {DEQUEUE_ONE_ITEM, DEQUEUE_ALL_ITEMS}
const unsigned int LET_Queue_EDPORTX_OverflowHandling = ERROR;	//Default queue overflow handling is error {DROP_OLDEST, DROP_NEWEST,ERROR}

//Function declaration
void LET_EDPORT_EDPORTX_Init( void );
unsigned int LET_EDPORT_ReadOne_EDPORTX( unsigned int* buf);
unsigned int LET_EDPORT_ReadAll_EDPORTX( unsigned int* buf);
unsigned int LET_EDPORT_Write_EDPORTX( unsigned int* buf);

/* Port Initialization
*/
void LET_EDPORT_EDPORTX_Init( void ){
	//Create event queue
	create_queue(LET_QueueID_EDPORTX, LET_QueueSize_EDPORTX);
}

/* Event Data Port Read
   @buf: the buffer that stores item in the queue.
   @timeout: specify blocking/non-blocking/timed-blocking mode.
   @Return the size of item read.
*/

//Read one
unsigned int LET_EDPORT_ReadOne_EDPORTX( unsigned int* buf){
	iReadNonBlock(LET_QueueID_EDPORTX, buf);
	return true;
}

//Read all
unsigned int LET_EDPORT_ReadAll_EDPORTX( unsigned int* buf){

	unsigned int item_count = 0;

	//Implement the concept of freeze.
	//Get the number of items in the queue.
	item_count = getNumItemQueue(LET_QueueID_EDPORTX);
	//Read these items from the queue.
	for(int i = 0 ; i < item_count ; i++){
		iReadNonBlock(LET_QueueID_EDPORTX, buf + i*sizeof(unsigned int));	
	}
	return item_count;
}

/* Event Data Port Send
*/

unsigned int LET_EDPORT_Write_EDPORTX( unsigned int* buf){
	unsigned int result = iWriteNonBlock(LET_QueueID_EDPORTX, buf);	
	if(result == PDTRUE){
		return PDTRUE;
	}else
	{
		switch(LET_Queue_EDPORTX_OverflowHandling){
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