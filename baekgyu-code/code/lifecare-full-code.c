/*
 Interface Synthesis Demonstration
 - Platform: Lifecare 4100 PCA.
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <stdlib.h>           
#include "interface_header.h"      
#include "interface_policy.h"
#include "pd.h"   

#define false 0
#define true 1

/****************************
  Platform Independent Code
*****************************/
//GPCA State ID
#define Init 1
#define Infusion_NormalOperation 2
#define Alrm_LevelTwoHardwareFailure 8

//Platform Input/Output variables
int E_Pressed_Start = false;
int E_RequestBolus = false;
int E_LevelTwoAlarm = false;
int E_ClearAlarm = false;
int E_BolusFinish = false;

//Task declaration
void TASK_gpca2(void);		 //Compute output, and Write output
void TASK_input2(void);	  	 //Read input

//Transition handlers
int process_Init_Transitions(void);
int process_Infusion_NormalOperation_Transitions(void);
int process_Alrm_LevelTwoHardwareFailure_Transitions(void);

//Initialize GPCA State
int current_state = Init;        

/**********************************
    Platform Primitives
*************************************/

//Clock Primitives
void Set_GPCA_Clock(int val);
int Read_GPCA_Clock();

//Platform Dependent Motor Control Task
void TASK_Motor_Control(void); 

/************************************
 Platform Specific Parameter
************************************/

//Basal Infusion Parameter
int Basal_Rate = 500;
int Basal_TimeOut = 10000;

//Bolus Infusion Parameter
int Bolus_Rate = 100;
int Bolus_TimeOut = 3000;

//Alarm Sound Level
int WRN_Sound_Level = 0x04;
int ALM_Sound_Level = 0x02;

//UI related
int MSG_ChangeDoseRate = 0x01;
int MSG_ConfirmPauseInfusion = 0x02;
int MSG_StopInfusion = 0x03;
int MSG_ConfirmStopInfusion = 0x04;
int MSG_BolusGranted = 0x05;
int MSG_BolusDenied = 0x06; 

int MAX_PAUSED_T = 10;  


/************************************
 Code Schedule Parameter
************************************/
const int PI_CODE_FREQUENCY = 10;
//const int PI_CODE_FREQUENCY = 30;		//The number of counter between two PI code activation.
//const int PI_CODE_FREQUENCY = 500;		//The number of counter between two PI code activation.
int current_PI_CODE_count = 0;

const int PD_CODE_FREQUENCY = 10;		//The number of counter between two PD code activation (to read events sent from PI code).
int current_PD_CODE_count = 0;

const int TIMEOUT_MONITOR_FREQUENCY = 1;   //Timeout monitor frequency
int current_TIMEOUT_MONITOR_count = 0;

const int POLL_L2Alarm_FREQUENCY = 3;
//const int POLL_L2Alarm_FREQUENCY = 30; 
int current_POLL_L2Alarm_count = 0;

const int MOTOR_CONTROL_FREQUENCY = 1;  	//The frequency of the activation of the motor-control task.
int current_MOTOR_CONTROL_FREQUENCY_count = 0;  

static void delay(void) {
  volatile unsigned long i;
  for(i=0;i<MOTOR_STEP_DELAY;i++);
}    

/************************************
 Interface Related Declaration
************************************/
void iEXP_Handler(int exp_id);

/************************************
 Platform Hardware Initialization
************************************/

void interrupt ISR_PI_TIMER1_Overflow(void);

/************************************
 Main function
************************************/
void main(void) {   

	///Hardware Initialization.  
	Hardware_Init();

	///Initialize the input buffer.	 
	initialize_interface_buffer();
  
	EnableInterrupts;

	//GPCA model execution code
	while(1){    
		TASK_Motor_Control();       		
	}
  
  /* please make sure that you never leave this function */
}

/************************************
 TASK functions
************************************/


void TASK_Motor_Control(void){

  //Motor Control (Stepper motor)
  if(MOTOR_State == 1){
  
    if(MAX_PUMP_STROKE_COUNT > pump_stroke_counter){
      
      if(next_state > (NUM_OF_STATES - 1)){
        
        next_state = 0;
      }
      
      PORTA = state_array[next_state];
      delay();
      next_state++;
      pump_stroke_counter++;
    }else{
	  //if bolus is finished, generate [BolusFinish] event
	  if(iWriteNonBlock(ID_INPUT_INTERFACE, QID_BolusFin, 'f') <= 0){
    	iEXP_Handler(EXP_ID_BUF_OVERFLOW);			//In case of buffer overflow, call the exception handler.     	
      }else{
    	//Success writing
      }
	  
      MOTOR_State = 0;
      pump_stroke_counter = 0;

    }
  }else{
    
  }  
}  

//These functions will be running inside the PI ISR
void TASK_gpca2(void){

	switch(current_state){
	case Init :
		current_state = process_Init_Transitions();
		break;
	case Infusion_NormalOperation :
		current_state = process_Infusion_NormalOperation_Transitions();
		break;
	case Alrm_LevelTwoHardwareFailure :
		current_state = process_Alrm_LevelTwoHardwareFailure_Transitions();
		break; 
	}		 

}

void TASK_input2(void){
	char temp_char;
	
	//Initialize the conditions
	E_RequestBolus = false;
    E_BolusFinish =false;
    E_LevelTwoAlarm = false;		//Complement of E_ClearAlarm
    E_ClearAlarm = false;
	
	temp_char = 0;
    
    //Read bolus request from Input Buffer
    while(iReadNonBlock(ID_INPUT_INTERFACE, QID_BolusReq, &temp_char) > 0){
    	//At least one event is detected, E_RequestBolus is set to true.
    	if(temp_char != 0){
    		E_RequestBolus = true;     		
    	}    	
    	temp_char = 0;    	
    }
    temp_char = 0;

	//Read Level Two Alarms from Input Buffer
	while(iReadNonBlock(ID_INPUT_INTERFACE, QID_LevelTwoAlarm, &temp_char) > 0){
		//At least one event is detected, E_LevelTwoAlarm is set to true.
		if(temp_char != 0){
    		E_LevelTwoAlarm = true; 
    		//E_ClearAlarm = false;		
    	}    	
    	temp_char = 0;	
	}
	
	//Read Bolus Finish Signal from Input Buffer
	while(iReadNonBlock(ID_INPUT_INTERFACE, QID_BolusFin, &temp_char) > 0){
		//At least one event is detected, E_BolusFinish is set to true.
		if(temp_char != 0){
    		E_BolusFinish = true;      		 		
    	}    	
    	temp_char = 0;	
	}
	
	while(iReadNonBlock(ID_INPUT_INTERFACE, QID_ClearAlarm, &temp_char) > 0){
		//At least one event is detected, E_ClearAlarm is set to false.
		if(temp_char != 0){		    	
    		E_ClearAlarm = true;		
    	}    	
    	temp_char = 0;	
	}			  	
	temp_char = 0;		     
}

/********************************************
	Transition Handlers of GPCA state
	- The codes can be automatically generated with actuation primitives. 
	- But the body of actuation primitives needs to be implemented by platforms.
	e.g.) The body of ACT_PumpMotor_Run() needs to be implemented by platforms.
	- The body of Read_GPCA_Clock() and Set_GPCA_Clock(val) needs to be implemented by platforms.
*********************************************/
int process_Init_Transitions(){
	if(E_RequestBolus == true){

		if(iWriteNonBlock(ID_OUTPUT_INTERFACE, QID_BolusStart, 's') <= 0){
  			iEXP_Handler(EXP_ID_BUF_OVERFLOW);			//In case of buffer overflow, call the exception handler.
	  	}else{
	  	}

		//Reset the input variable
		E_RequestBolus = false;

		//Return a new state
		return Infusion_NormalOperation;
	}

	return Init;
}


int process_Infusion_NormalOperation_Transitions(){
	if(E_RequestBolus == true){    

		if(iWriteNonBlock(ID_OUTPUT_INTERFACE, QID_BolusStart, 's') <= 0){
  			iEXP_Handler(EXP_ID_BUF_OVERFLOW);			//In case of buffer overflow, call the exception handler.
	  	}else{
	  	}

		//Reset the input variable
		E_RequestBolus = false;
		
		//Return a new state
		return Infusion_NormalOperation;
	}


	if(E_LevelTwoAlarm == true ){

		if(iWriteNonBlock(ID_OUTPUT_INTERFACE, QID_RaiseAlarm, 'a') <= 0){
  			iEXP_Handler(EXP_ID_BUF_OVERFLOW);			//In case of buffer overflow, call the exception handler.
	  	}else{
	  	}

		//Reset the input variable
		E_LevelTwoAlarm = false;

		//Return a new state
		return Alrm_LevelTwoHardwareFailure;
	}
	
	if(E_BolusFinish == true){	
		if(iWriteNonBlock(ID_OUTPUT_INTERFACE, QID_BolusProcessed, 'p') <= 0){
  			iEXP_Handler(EXP_ID_BUF_OVERFLOW);			//In case of buffer overflow, call the exception handler.
	  	}else{
	  	}
		
		//Return a new state
		return Init;  		
	} 	
	return Infusion_NormalOperation; 
}


int process_Alrm_LevelTwoHardwareFailure_Transitions(void){
  
  if(E_ClearAlarm == true){
 
    if(iWriteNonBlock(ID_OUTPUT_INTERFACE, QID_SilenceAlarm, 's') <= 0){
		iEXP_Handler(EXP_ID_BUF_OVERFLOW);			//In case of buffer overflow, call the exception handler.
  	}else{
  	}       
    E_ClearAlarm = false;
    
    //Return a new state
	return Init;
     
  }	  
  return Alrm_LevelTwoHardwareFailure;   
}




//Interrupt-Based Primitives
#pragma CODE_SEG __NEAR_SEG NON_BANKED
__interrupt VectorNumber_Vportp void ISR_PCB_PRESSED(void){       
  
  DisableInterrupts;
  
  //When the patient controlled button is pressed
  if((PIFP && DDRP_DDRP7_MASK) > 0)   {
  	PIFP = PIFP | DDRP_DDRP7_MASK;       //Clear interrupt flag   
    
    //Insert the input event to Input Buffer
  	if(iWriteNonBlock(ID_INPUT_INTERFACE, QID_BolusReq, 'b') <= 0){
  		iEXP_Handler(EXP_ID_BUF_OVERFLOW);			//In case of buffer overflow, call the exception handler.
  	}else{
		if(OnBoard_LED1_State == 0){	  //In case of success, toggle LED 2
	   		LED_Onboard_On(1);  	    
	  	}else{
	   	 	LED_Onboard_Off(1);       
	  	}
  	}  
  }						    
  EnableInterrupts;		    
}

//Platform-Independent Code Interrupt Service Routine
#pragma CODE_SEG __NEAR_SEG NON_BANKED
__interrupt VectorNumber_Vtimch1 void ISR_PI_TIMER1_Overflow(void){ 

  char temp_char;      
  
  DisableInterrupts;  
  
  //Increase counters.
  current_PI_CODE_count++;
  current_PD_CODE_count++;
  current_TIMEOUT_MONITOR_count++;
  current_MOTOR_CONTROL_FREQUENCY_count++;
  current_POLL_L2Alarm_count++;
  
  
  //Execute associate code
  if(current_PI_CODE_count == PI_CODE_FREQUENCY){
	  TASK_input2(); //Process Input 
	    
	  TASK_gpca2();  //Process Transitions
	  
	  current_PI_CODE_count = 0;	
  }
  
  //PD code to read events from Output Interface
  if(current_PD_CODE_count == PD_CODE_FREQUENCY){
  	//Read bolus start from Output Buffer
    while(iReadNonBlock(ID_OUTPUT_INTERFACE, QID_BolusStart, &temp_char) > 0){
    	//At least one event is detected, perform the platform-dependent action.
    	if(temp_char != 0){
    		ACT_PumpMotor_Run(Bolus_Rate, Bolus_TimeOut);     		
    	}    	
    	temp_char = 0;    	
    }
    temp_char = 0;
    
    //Read bolus processed from Output Buffer
    while(iReadNonBlock(ID_OUTPUT_INTERFACE, QID_BolusProcessed, &temp_char) > 0){
    	//At least one event is detected, perform the platform-dependent action.
    	if(temp_char != 0){
    		//Do platform-dependent action for BolusProcessed!  
    		ACT_PumpMotor_Stop();  		
    	}    	
    	temp_char = 0;    	
    }
    temp_char = 0;
    
    //Read raise alarm from Output Buffer
    while(iReadNonBlock(ID_OUTPUT_INTERFACE, QID_RaiseAlarm, &temp_char) > 0){
    	//At least one event is detected, perform the platform-dependent action.
    	if(temp_char != 0){
    		//Do platform-dependent action for RaiseAlarm!  
    		ACT_Buzzer_On(ALM_Sound_Level);
			ACT_PumpMotor_Stop();
			ACT_AlarmLED_On();  		
    	}    	
    	temp_char = 0;    	
    }
    temp_char = 0;
    
    //Read Silence Alarm from Output Buffer
    while(iReadNonBlock(ID_OUTPUT_INTERFACE, QID_SilenceAlarm, &temp_char) > 0){
    	//At least one event is detected, perform the platform-dependent action.
    	if(temp_char != 0){
    		//Do platform-dependent action for RaiseAlarm!  
  		    ACT_Buzzer_Off();
    		ACT_AlarmLED_Off(); 
    	}    	
    	temp_char = 0;    	
    }
    temp_char = 0;	   
  
  	current_PD_CODE_count = 0;	
  }		  
  
  if(current_TIMEOUT_MONITOR_count == TIMEOUT_MONITOR_FREQUENCY){
	  
	//Call handler
	Timeout_Exception_Monitor();
	
	current_TIMEOUT_MONITOR_count = 0;
  }	    
  
  //Sampling for Empty Reservoir switch AND Drug loaded switch. 
  if(current_POLL_L2Alarm_count == POLL_L2Alarm_FREQUENCY){
  
  	//In case of L2Alarm sampling event, toggle LED 1
  	if(OnBoard_LED1_State == 0){	  
 		LED_Onboard_On(1);  	    
  	}else{
   	 	LED_Onboard_Off(1);       
  	}	    	
    
    if(GET_empty_drug_sw_status() == 0 && GET_drug_load_sw_status() == 1){
    	//Insert the input event to Input Buffer
	  	if(iWriteNonBlock(ID_INPUT_INTERFACE, QID_ClearAlarm, 'c') <= 0){  
	  		iEXP_Handler(EXP_ID_BUF_OVERFLOW);			//In case of buffer overflow, call the exception handler.
	  	}else{
	  	}
	    
    }else{					  //implies Drug is empty OR Drug is unloaded (Alarming condition)
    	//Insert the input event to Input Buffer
	  	if(iWriteNonBlock(ID_INPUT_INTERFACE, QID_LevelTwoAlarm, 'e') <= 0){
	  		iEXP_Handler(EXP_ID_BUF_OVERFLOW);			//In case of buffer overflow, call the exception handler.
	  	}else{
	  	}	      	
    }		       	
  	current_POLL_L2Alarm_count = 0;	    	
  }
  
  //Clear Interrupt Flag.
  TFLG1 = TFLG1 | 0x02;
  
  EnableInterrupts;
  
} 

/**********************************************
   Interface Exception Handler Implementation
***********************************************/
void iEXP_Handler(int exp_id){
	if(exp_id == EXP_ID_BUF_OVERFLOW){
		if(OnBoard_LED2_State == 0){
    		LED_Onboard_On(2);
    
  		}else{
    		LED_Onboard_Off(2);       
  		}
	}else if(exp_id == EXP_ID_EVT_TIMEOUT){
		//Beep
		/*Alarm_Buzzer_On(); 
		delay(); 
		Alarm_Buzzer_Off();*/
		if(OnBoard_LED4_State == 0){
    		LED_Onboard_On(4);
    
  		}else{
    		LED_Onboard_Off(4);       
  		}	  	
	}
}	  

/************************************
Clock Primitive Implementation
************************************/
void Set_GPCA_Clock(int val){

  //Platform needs to implement the body.
  
  return; 
}

int Read_GPCA_Clock(void){
  
  int val = 0;
  
  //Platform needs to implement the body.
  
  return val;
  
}

