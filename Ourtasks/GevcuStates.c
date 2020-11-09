/******************************************************************************
* File Name          : GevcuStates.c
* Date First Issued  : 07/01/2019
* Description        : States in Gevcu function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"
#include "LEDTask.h"
#include "GevcuEvents.h"
#include "GevcuStates.h"
#include "GevcuTask.h"
#include "calib_control_lever.h"
#include "contactor_control.h"
#include "yprintf.h"
#include "lcdprintf.h"
#include "gevcu_idx_v_struct.h"
#include "morse.h"
#include "adcparamsinit.h"
#include "lcdmsg.h"
#include "dmoc_control.h"
#include "control_law_v1.h"
#include "LcdTask.h"
#include "LcdmsgsetTask.h"
#include "stepper_items.h"

#define GEVCULCDMSGDELAY 32 // Minimum number of time ticks between LCD msgs
#define GEVCULCDMSGLONG (128*30) // Very long delay

extern struct LCDI2C_UNIT* punitd4x20; // Pointer LCDI2C 4x20 unit

enum GEVCU_INIT_SUBSTATEA
{
	GISA_OTO,  
	GISA_WAIT,
};

/* Flag queue LCD msg only once. defaultTask will call these lcdprintf functions. */
static uint8_t msgflag = 0; // 0 = send; 1 = don't send
static uint8_t msgslow = 0; // Counter for pacing repeated msgs

/* *************************************************************************
 * void payloadfloat(uint8_t *po, float f);
 *	@brief	: Convert float to bytes and load into payload
 * @param	: po = pointer to payload byte location to start (Little Endian)
 * *************************************************************************/
void payloadfloat(uint8_t *po, float f)
{
	union FF
	{
		float f;
		uint8_t ui[4];
	}ff;
	ff.f = f; 

	*(po + 0) = ff.ui[0];
	*(po + 1) = ff.ui[1];
	*(po + 2) = ff.ui[2];
	*(po + 3) = ff.ui[3];
	return;
}

/* *************************************************************************
 * void GevcuStates_GEVCU_INIT(void);
 * @brief	: Initialization sequence: One Time Only
 * *************************************************************************/
//  20 chars will over-write all display chars from previous msg:       12345678901234567890
//static void lcdmsg1   (void)             {lcdprintf (&gevcufunction.pbuflcd3,GEVCUTSK,0,"GEVCU_INT           ");}
static void lcdi2cmsg1(union LCDSETVAR u){lcdi2cputs(&punitd4x20,           GEVCUTSK,0,"GEVCU_INT           ");}

//static void lcdmsg2    (void)             {lcdprintf (&gevcufunction.pbuflcd3,GEVCUTSK, 0,"SWITCH TO SAFE      ");}// LCD uart
static void lcdi2cmsg2a(union LCDSETVAR u){lcdi2cputs(&punitd4x20,           GEVCUTSK, 0,"SWITCH TO SAFE      ");}// LCD i2c

 /* LCDI2C 4x20 msg. */
static struct LCDMSGSET lcdi2cfunc;

void GevcuStates_GEVCU_INIT(void)
{	
//	void (*ptr2)(void); // Pointer to queue LCD msg
	struct SWITCHPTR* p;

	uint8_t loopctr = 0;

	switch (gevcufunction.substateA)
	{
	case GISA_OTO: // Cycle Safe/Active sw.

		/* Wait for task that instantiates the LCD display. */
		while ((punitd4x20 == NULL) && (loopctr++ < 10)) osDelay(10);
  		if (punitd4x20 == NULL) morse_trap(2326);

		msgflag = 0; // One-msg flag, JIC

		/* Wait for calib_control_lever.c to complete calibrations. */
		if (flag_clcalibed == 0) 
			break;

		/* Queue LCD msg to be sent once. */
		if (msgflag == 0)
		{ 
			msgflag = 1; // Don't keep banging away with the same msg

			// Msg on UART LCD
//			ptr2 = &lcdmsg1; // LCD msg pointer
//			xQueueSendToBack(lcdmsgQHandle,&ptr2,0);

			// LCD I2C unit msg
			lcdi2cfunc.ptr = lcdi2cmsg1;
			// Place ptr to struct w ptr 
		 	if (LcdmsgsetTaskQHandle != NULL)
	  	  		xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
		}

		/* Update LED with SAFE/ACTIVE switch status. */
		p = gevcufunction.psw[PSW_PR_SAFE];
		gevcufunction.safesw_prev = p->db_on;
		switch (p->db_on)
		{
		case 1:
	      led_safe.mode = 0; // LED_SAFE off
			break;
			
		case 2:
	      led_safe.mode = 1; // LED_SAFE on
			break;
		}
		xQueueSendToBack(LEDTaskQHandle,&led_safe,portMAX_DELAY);
		msgflag = 0; // Allow next LCD msg to be sent once
		gevcufunction.substateA = GISA_WAIT;
		break;

	case GISA_WAIT: // More OTO to do here?
		if (gevcufunction.psw[PSW_PR_SAFE]->db_on == SWP_OPEN )
		{ // Here SAFE/ACTIVE switch is in ACTIVE position
			if (msgflag == 0)
			{ 
				msgflag = 1; // Don't keep banging away with the same msg
				lcdi2cfunc.ptr = lcdi2cmsg2a;
				if (LcdmsgsetTaskQHandle != NULL)
	 		   		xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
			}
			break;
		}

		/* Transition into safe mode,. */
		msgflag = 0; // Allow next LCD msg to be sent once
		msgslow = 255; // Pacing counter 
		gevcufunction.state = GEVCU_SAFE_TRANSITION;
		break;

		default:
			break;
	}
	return;
}
/* *************************************************************************
 * void GevcuStates_GEVCU_SAFE_TRANSITION(void);
 * @brief	: Peace and quiet, waiting for hapless Op.
 * *************************************************************************/
//  20 chars will over-write all display chars from previous msg:             12345678901234567890
static void lcdi2cmsg3a(union LCDSETVAR u){lcdi2cputs(&punitd4x20,GEVCUTSK,0,"GEVCU_SAFE_TRANSITIO");}
//static void lcdi2cmsg3b(union LCDSETVAR u){lcdi2cputs(&punitd4x20,GEVCUTSK,0,"WAIT CONTACTOR OPEN ");}
//static void lcdi2cmsg3c(union LCDSETVAR u){lcdi2cputs(&punitd4x20,GEVCUTSK,0,"CONTACTOR NO-RESPONS");}
//static void lcdi2cmsg3d(union LCDSETVAR u){lcdi2cputs(&punitd4x20,GEVCUTSK,0,"CONTACTOR NOT INITed");}

void GevcuStates_GEVCU_SAFE_TRANSITION(void)
{
//	void (*ptr2)(void) = &lcdmsg3; // LCD msg pointer

	if (msgflag == 0)
	{ 
		msgflag = 1; // Don't keep banging away with the same msg
//		xQueueSendToBack(lcdmsgQHandle,&ptr2,0);

		// Repeat msg on LCD I2C unit
		lcdi2cfunc.ptr = lcdi2cmsg3a; 
		 if (LcdmsgsetTaskQHandle != NULL)
	    	xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
	}
      led_safe.mode    = LED_BLINKFAST; // LED_SAFE blinking
		led_arm_pb.mode  = LED_OFF; // ARM Pushbutton LED
		led_arm.mode     = LED_OFF; // ARM LED
		led_prep_pb.mode = LED_OFF; // PREP Pushbutton LED
		led_prep.mode    = LED_OFF; // PREP LED
	xQueueSendToBack(LEDTaskQHandle, &led_safe   ,portMAX_DELAY);
	xQueueSendToBack(LEDTaskQHandle, &led_arm_pb ,portMAX_DELAY);
	xQueueSendToBack(LEDTaskQHandle, &led_arm    ,portMAX_DELAY);
	xQueueSendToBack(LEDTaskQHandle, &led_prep_pb,portMAX_DELAY);
	xQueueSendToBack(LEDTaskQHandle, &led_prep   ,portMAX_DELAY);

	/* Request contactor to DISCONNECT. */
	cntctrctl.req = CMDRESET;


//	if (cntctrctl.nrflag != 0)
//	{ // Here, contactor is not responding
//		msgslow += 1;
//		if (msgslow >= 48)
//		{ 
//			msgslow = 0;
//			lcdi2cfunc.ptr = lcdi2cmsg3c; 
  // 			xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
   	//	}
	//	return;
//	}

//	if ((cntctrctl.cmdrcv & 0xf) == OTOSETTLING)
//	{ // Waiting for contactor initialization "settling"
//		msgslow += 1;
//		if (msgslow >= 48)
//		{ 
//			msgslow = 0;
//			lcdi2cfunc.ptr = lcdi2cmsg3d; 
  // 			xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
   	//	}
	//	return;
//	}


//	/* Wait until contactor shows DISCONNECTED state. */
//	if ((cntctrctl.cmdrcv & 0xf) != DISCONNECTED)
//	{ // LCD msg here?
//		if (msgflag == 1)
//		{
//			if (LcdmsgsetTaskQHandle != NULL) 
//			{
//				lcdi2cfunc.ptr = lcdi2cmsg3b; 
//	    		xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
//	    		msgflag = 2;
//	    	}
//	    }
//		return;
//	}


	/* Assure disable stepper controller states. */
	HAL_GPIO_WritePin(EN_port,EN_pin, GPIO_PIN_RESET); // Disable Stepper motor

	/* Default stepper motor controller direction. */
	HAL_GPIO_WritePin(DR_port,DR_pin, GPIO_PIN_RESET); 


	msgflag = 0; // Send LCD msg once

   led_safe.mode    = LED_ON;
	xQueueSendToBack(LEDTaskQHandle, &led_safe   ,portMAX_DELAY);

	msgflag = 0; // Allow next LCD msg to be sent once

	gevcufunction.state = GEVCU_SAFE;
	return;
}
/* *************************************************************************
 * void GevcuStates_GEVCU_SAFE(void);
 * @brief	: Peace and quiet, waiting for hapless Op.
 * *************************************************************************/
//  20 chars will over-write all display chars from previous msg:       12345678901234567890
//static void lcdmsg4   (void)             {lcdprintf (&gevcufunction.pbuflcd3,GEVCUTSK,0,"GEVCU_SAFE          ");}
static void lcdi2cmsg4(union LCDSETVAR u){lcdi2cputs(&punitd4x20,           GEVCUTSK,0,"GEVCU_SAFE          ");}

void GevcuStates_GEVCU_SAFE(void)
{
//	void (*ptr2)(void) = &lcdmsg4; // LCD msg pointer

	if (msgflag == 0)
	{ 
		msgflag = 1; // Don't keep banging away with the same msg

//		xQueueSendToBack(lcdmsgQHandle,&ptr2,0);

		// Repeat msg on LCD I2C unit
		lcdi2cfunc.ptr = lcdi2cmsg4;
		// Place ptr to struct w ptr 
		 if (LcdmsgsetTaskQHandle != NULL)
	    	xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
	}
		
	if (gevcufunction.psw[PSW_PR_SAFE]->db_on == SWP_OPEN )
	{ // Here SAFE/ACTIVE switch is in ACTIVE position
		msgflag = 0; // Allow next LCD msg to be sent once
		gevcufunction.state = GEVCU_ACTIVE_TRANSITION;

		led_safe.mode = LED_OFF; // LED_SAFE off
		xQueueSendToBack(LEDTaskQHandle,&led_safe,portMAX_DELAY);

		led_prep.mode = LED_BLINKFAST; // PREP Pushbutton LED fast blink mode
		xQueueSendToBack(LEDTaskQHandle,&led_prep,portMAX_DELAY);

		/* Request contactor to CONNECT. */
//		cntctrctl.req = CMDCONNECT;

		/* Set the last received contactor response to bogus. */
//		cntctrctl.cmdrcv = 0x8f; // Connect cmd w bogus response code
		return;
	}
	return;
}
/* *************************************************************************
 * void GevcuStates_GEVCU_ACTIVE_TRANSITION(void);
 * @brief	: Contactor & DMOC are ready. Keep fingers to yourself.
 * *************************************************************************/
//  20 chars will over-write all display chars from previous msg:       12345678901234567890
//static void lcdmsg5   (void)             {lcdprintf (&gevcufunction.pbuflcd3,GEVCUTSK,0,"GEVCU_ACTIVE_TRANSIT");}
static void lcdi2cmsg5(union LCDSETVAR u){lcdi2cputs(&punitd4x20,           GEVCUTSK,0,"GEVCU_PREP TRANSITIO");}

void GevcuStates_GEVCU_ACTIVE_TRANSITION(void)
{
//	void (*ptr2)(void) = &lcdmsg5; // LCD msg pointer

	if (msgflag == 0)
	{ 
		msgflag = 1; // Don't keep banging away with the same msg
//		xQueueSendToBack(lcdmsgQHandle,&ptr2,0);

				// Repeat msg on LCD I2C unit
		lcdi2cfunc.ptr = lcdi2cmsg5;
		// Place ptr to struct w ptr 
		 if (LcdmsgsetTaskQHandle != NULL)
	    	xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
	}

	if (gevcufunction.psw[PSW_PR_SAFE]->db_on == SWP_CLOSE)
	{ // Here SAFE/ACTIVE switch is now in SAFE (CLOSE) position
		/* Go back into safe mode,. */
		msgflag = 0; // Allow next LCD msg to be sent once
		gevcufunction.state = GEVCU_SAFE_TRANSITION;
		return;
	}
	
	/* Wait for CONNECTED. */
//	if ((cntctrctl.cmdrcv & 0xf) != CONNECTED)
//	{ // Put a stalled loop timeout here?
//		cntctrctl.req = CMDCONNECT;
//		return;
//	}

	/* Contactor connected. */

	led_prep_pb.mode = LED_OFF; // PREP Pushbutton off
	xQueueSendToBack(LEDTaskQHandle,&led_prep,portMAX_DELAY);

	led_prep.mode = LED_ON; // PREP state led on
	xQueueSendToBack(LEDTaskQHandle,&led_prep,portMAX_DELAY);

	led_arm_pb.mode = LED_BLINKFAST; // ARM Pushbutton LED fast blink mode
	xQueueSendToBack(LEDTaskQHandle,&led_arm_pb,portMAX_DELAY);

	msgflag = 0; // Allow next LCD msg to be sent once
	gevcufunction.state = GEVCU_ACTIVE;
	return;
}
/* *************************************************************************
 * void GevcuStates_GEVCU_ACTIVE(void);
 * @brief	: Contactor & DMOC are ready. Keep fingers to yourself.
 * *************************************************************************/
//  20 chars will over-write all display chars from previous msg:       12345678901234567890
//static void lcdmsg6   (void)             {lcdprintf (&gevcufunction.pbuflcd3,GEVCUTSK,0,"GEVCU_ACTIVE        ");}
static void lcdi2cmsg6(union LCDSETVAR u){lcdi2cputs(&punitd4x20,           GEVCUTSK,0,"GEVCU_PREP          ");}


void GevcuStates_GEVCU_ACTIVE(void)
{
//	void (*ptr2)(void) = &lcdmsg6; // LCD msg pointer

	if (msgflag == 0)
	{ 
		msgflag = 1; // Don't keep banging away with the same msg
//		xQueueSendToBack(lcdmsgQHandle,&ptr2,0);
		// Repeat msg on LCD I2C unit
		lcdi2cfunc.ptr = lcdi2cmsg6;
		// Place ptr to struct w ptr 
		 if (LcdmsgsetTaskQHandle != NULL)
	    	xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
	}

	/* Wait for PREP pushbutton to be pressed (toggle logic). */	
	if (gevcufunction.psw[PSW_PB_PREP]->db_on != SW_CLOSED)
	{ // Here, pushbutton is open
		gevcufunction.pbprep_prev = SW_OPEN; 
	}
	else
	{ // Here pushbutton is closed
		if (gevcufunction.pbprep_prev != SW_CLOSED)
		{ // Here pushbutton was previously open
			gevcufunction.pbprep_prev = SW_CLOSED; 
	
			HAL_GPIO_WritePin(EN_port,EN_pin, GPIO_PIN_SET); // Enable Stepper motor

			led_arm_pb.mode = LED_ON; // ARM Pushbutton LED
			xQueueSendToBack(LEDTaskQHandle,&led_arm_pb,portMAX_DELAY);

			led_prep.mode = LED_OFF; // PREP state LED
			xQueueSendToBack(LEDTaskQHandle,&led_prep,portMAX_DELAY);

			msgflag = 0; // Allow next LCD msg to be sent once
			gevcufunction.state = GEVCU_ARM_TRANSITION;
		}
	}
	return;
}
/* *************************************************************************
 * void GevcuStates_GEVCU_ARM_TRANSITION(void);
 * @brief	: Do everything needed to get into state
 * *************************************************************************/
//  20 chars will over-write all display chars from previous msg:       12345678901234567890
//static void lcdmsg7   (void)             {lcdprintf (&gevcufunction.pbuflcd3,GEVCUTSK,0,"ARM: MOVE CL ZERO   ");}
//static void lcdi2cmsg7(union LCDSETVAR u){lcdi2cputs(&punitd4x20,           GEVCUTSK,0,"ARM: MOVE CL ZERO   ");}

//static void lcdmsg8   (void){lcdprintf (&gevcufunction.pbuflcd3,GEVCUTSK,0,"GEVCU_ARM           ");}
static void lcdi2cmsg8(union LCDSETVAR u){lcdi2cputs(&punitd4x20,           GEVCUTSK,0,"GEVCU_ARM           ");}

void GevcuStates_GEVCU_ARM_TRANSITION(void)
{
//	void (*ptr2)(void); // Pointer to queue LCD msg
#ifdef USEMOVECLTOZEROLOGIC
		/* Make sure Op has CL in zero position. */
		if (clfunc.curpos > 0)
		{
			if (msgflag == 0)
			{ 
				msgflag = 1; // Don't keep banging away with the same msg
//				ptr2 = &lcdmsg7; // LCD msg pointer
//				xQueueSendToBack(lcdmsgQHandle,&ptr2,0);
						// Repeat msg on LCD I2C unit
				lcdi2cfunc.ptr = lcdi2cmsg7;
				// Place ptr to struct w ptr 
		 		if (LcdmsgsetTaskQHandle != NULL)
	    		xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
			}
			return;
		}
#endif		

//		ptr2 = &lcdmsg8; // LCD msg pointer
//		xQueueSendToBack(lcdmsgQHandle,&ptr2,0);

		// Repeat msg on LCD I2C unit
		lcdi2cfunc.ptr = lcdi2cmsg8;
		// Place ptr to struct w ptr 
		 if (LcdmsgsetTaskQHandle != NULL)
	    	xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);

		led_arm_pb.mode = LED_OFF; // ARM Pushbutton LED
		xQueueSendToBack(LEDTaskQHandle,&led_arm_pb,portMAX_DELAY);

		led_prep.mode = LED_OFF; // PREP state LED
		xQueueSendToBack(LEDTaskQHandle,&led_prep,portMAX_DELAY);

		led_arm.mode = LED_ON; // ARM state LED
		xQueueSendToBack(LEDTaskQHandle,&led_arm,portMAX_DELAY);

		msgflag = 0; // Allow next LCD msg to be sent once
		gevcufunction.state = GEVCU_ARM;
		return;
}
/* *************************************************************************
 * void GevcuStates_GEVCU_ARM(void);
 * @brief	: Contactor & DMOC are ready. Keep fingers to yourself.
 * *************************************************************************/
static void lcdi2cmsg10(union LCDSETVAR u){lcdi2cputs(&punitd4x20,LCDLEVELWIND,0,"LW nomode     ");}
static void lcdi2cmsg11(union LCDSETVAR u){lcdi2cputs(&punitd4x20,LCDLEVELWIND,0,"LW OFF        ");}
static void lcdi2cmsg12(union LCDSETVAR u){lcdi2cputs(&punitd4x20,LCDLEVELWIND,0,"LW CENTER     ");}
static void lcdi2cmsg13(union LCDSETVAR u){lcdi2cputs(&punitd4x20,LCDLEVELWIND,0,"LW TRACK      ");}

#define USETHEPREPSWITCHTOBREAKSTATE 

void GevcuStates_GEVCU_ARM(void)
{
#ifdef USETHEPREPSWITCHTOBREAKSTATE

	/* Pressing PREP toggles back to ACTIVE state. */
	if (gevcufunction.psw[PSW_PB_PREP]->db_on != SW_CLOSED)
	{ // Here, pushbutton is open
		gevcufunction.pbprep_prev = SW_OPEN; 
	}
	else
	{ // Here pushbutton is closed
		if (gevcufunction.pbprep_prev != SW_CLOSED)
		{ // Here pushbutton was previously open
			gevcufunction.pbprep_prev = SW_CLOSED; 

			led_prep.mode = LED_ON; // PREP state led on
			xQueueSendToBack(LEDTaskQHandle,&led_prep,portMAX_DELAY);

			led_arm.mode = LED_OFF; // ARM state LED
			xQueueSendToBack(LEDTaskQHandle,&led_arm,portMAX_DELAY);

			/* Return stepper to disabled, safe state. */
			HAL_GPIO_WritePin(EN_port,EN_pin, GPIO_PIN_RESET); // Disable Stepper motor


			/* Set DMOC torque and integrator to zero. */
			control_law_v1_reset();

			// Set desired speed CAN msg payload to zero.
			payloadfloat(&gevcufunction.canmsg[CID_GEVCUR_CTL_LAWV1].can.cd.uc[0],0);

			/* Be sure to update LCD msg. */
			msgflag = 0;

			gevcufunction.state = GEVCU_ACTIVE_TRANSITION;
			return;		
		}
	}

	gevcufunction.stepperclpos = clfunc.curpos; // Save current position 
	#endif

#ifndef USETHEPREPSWITCHTOBREAKSTATE
	/* Pressing PREP toggles freezing CL position. */
	if (gevcufunction.psw[PSW_PB_PREP]->db_on == SW_CLOSED)
	{
		if (gevcufunction.stepperclposswprev != SW_CLOSED)
		{ // Switch went from open to closed
			gevcufunction.stepperclposswprev = SW_CLOSED;
			gevcufunction.stepperclpostoggle ^= 0x1;
			gevcufunction.stepperclpos = clfunc.curpos; // Save current position
		}
	}
	else
	{ // Here, switch released
		gevcufunction.stepperclposswprev = SW_OPEN;
	}

	/* When toggle not ON, update continuously. */
	if (gevcufunction.stepperclpostoggle == 0)
	{ // Update postiion sent to stepper
		led_prep.mode = LED_ON; // PREP state led on
		xQueueSendToBack(LEDTaskQHandle,&led_prep,portMAX_DELAY);		
		gevcufunction.stepperclpos = clfunc.curpos; // Save current position
	}
	else
	{ // Don't update position sent to stepper
		led_prep.mode = LED_OFF; // PREP state led on
		xQueueSendToBack(LEDTaskQHandle,&led_prep,portMAX_DELAY);				
	}
#endif	

	/* When ZODOMTR (retrieve) switch pressed toggle DR (direction) line */
	if (gevcufunction.psw[PSW_ZODOMTR]->db_on == SW_CLOSED)
	{ 
		if (gevcufunction.stepperdrswprev != SW_CLOSED)
		{
			gevcufunction.stepperdrswprev = SW_CLOSED;
			gevcufunction.stepperdrtoggle ^= 0x1;
		}
	}
	else
	{ // Here switch released
		gevcufunction.stepperdrswprev = SW_OPEN;
	}

	if (gevcufunction.stepperdrtoggle == 0)
	{
		led_retrieve.mode = LED_OFF;
		xQueueSendToBack(LEDTaskQHandle,&led_retrieve,portMAX_DELAY);
		HAL_GPIO_WritePin(Stepper__DR__direction_GPIO_Port,Stepper__DR__direction_Pin,GPIO_PIN_SET);
		stepper_items_clupdate(0,gevcufunction.stepperclpos);
	}
	else
	{
		led_retrieve.mode = LED_ON;
		xQueueSendToBack(LEDTaskQHandle,&led_retrieve,portMAX_DELAY);
		HAL_GPIO_WritePin(Stepper__DR__direction_GPIO_Port,Stepper__DR__direction_Pin,GPIO_PIN_SET);
		stepper_items_clupdate(1,gevcufunction.stepperclpos);

	}

	/* Pushbutton to simulate stepper limit switch. */
	if (gevcufunction.psw[PSW_ZTENSION]->db_on == SW_CLOSED)
	{
		gevcufunction.stepperlmbit = LMBIT;
	}
	else
	{
		gevcufunction.stepperlmbit = 0;	
	}

	/* Pushbutton to simuilate Levelwind OFF-CENTER-TRACK */
	if (gevcufunction.psw[PSW_PB_ARM]->db_on != SW_CLOSED)
	{ // Here, pushbutton is open
		gevcufunction.pbarm_prev = SW_OPEN; 
	}
	else
	{ // Here pushbutton is closed
		if (gevcufunction.pbarm_prev != SW_CLOSED)
		{ // Here pushbutton was previously open
			gevcufunction.pbarm_prev = SW_CLOSED; 
			// Advance levelwind mode (sequence: 01, 10, 11)
			gevcufunction.levelwindmode = (gevcufunction.levelwindmode + 1) & 0x3;
			if (gevcufunction.levelwindmode == 0) gevcufunction.levelwindmode = 1;

			/* LCD message. */
			switch (gevcufunction.levelwindmode)
			{
			case 0: lcdi2cfunc.ptr = lcdi2cmsg10; break;
			case 1: lcdi2cfunc.ptr = lcdi2cmsg11; break;
			case 2: lcdi2cfunc.ptr = lcdi2cmsg12; break;
			case 3: lcdi2cfunc.ptr = lcdi2cmsg13; break;
			}
			// Place ptr to struct w ptr 
	 		if (LcdmsgsetTaskQHandle != NULL)
    			xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc, 0);
		}
	}


	/* Compute torque request when the GevcuEvents_04 handling of the
      timer notification called dmoc_control_time, and dmoc_control_time
      set the sendflag, indicating it is time to send the three CAN msgs. The
		sending is executed via the call in GevcuUpdates to dmoc_control_CANsend,
      which sends the msgs "if" the Gevcu state is ARM. dmoc_control_CANsend 
		resets the sendflag.
		Net-- a new torque request is only computed when it is needed.
 	*/
	if (dmocctl[DMOC_SPEED].sendflag != 0)
	{
//		control_law_v1_calc(&dmocctl[DMOC_SPEED]); // Version 1: PI Loop

		/* Setup and send CAN msgs to dmoc in GevcuUpdates.c. */
	}

	return;
}

