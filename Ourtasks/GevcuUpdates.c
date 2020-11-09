/******************************************************************************
* File Name          : GevcuUpdates.c
* Date First Issued  : 07/02/2019
* Description        : Update outputs in Gevcu function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "ADCTask.h"
#include "adctask.h"

#include "GevcuTask.h"
#include "gevcu_idx_v_struct.h"
#include "CanTask.h"
#include "gevcu_msgs.h"
#include "contactor_control.h"
#include "dmoc_control.h"
#include "calib_control_lever.h"
#include "control_law_v1.h"
#include "stepper_items.h"
#include "spiserialparallelSW.h"

#include "morse.h"

/* From 'main.c' */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static void payloadfloat(uint8_t *po, float f)
{
	union FFUI
	{
		float f;
		uint8_t u8[4];
	}ffui;
	ffui.f = f;

	*(po + 0) = ffui.u8[0];
	*(po + 1) = ffui.u8[1];
	*(po + 2) = ffui.u8[2];
	*(po + 3) = ffui.u8[3];
	return;
}

/* *************************************************************************
 * void GevcuUpdates(void);
 * @brief	: Update outputs based on bits set
 * *************************************************************************/
void GevcuUpdates(void)
{
	/* Contactor keepalive/command msg sending. */
	contactor_control_CANsend();
	
	/* DMOC CAN msg sending. */
	if (gevcufunction.state == GEVCU_ARM)
	{
		dmocctl[DMOC_SPEED].dmocopstate = DMOC_ENABLE;
	}
	else
	{
		dmocctl[DMOC_SPEED].dmocopstate = DMOC_DISABLED;
	}

	/* sendflag is set in GevcuEvents with timer tick call to dmoc_control_time. */
	if (dmocctl[DMOC_SPEED].sendflag != 0)
	{ // Here dmoc CMD1,2,3 are scheduled to be sent
			// Queue CAN msg with desired speed
		// Load desired speed (float) into payload starting at [0] 
		payloadfloat(&gevcufunction.canmsg[CID_GEVCUR_CTL_LAWV1].can.cd.uc[0],clv1.dsrdspd);
		// Load control law integrator value
		payloadfloat(&gevcufunction.canmsg[CID_GEVCUR_CTL_LAWV1].can.cd.uc[4],clv1.intgrtr);
		// Queue CAN msg for sending
		xQueueSendToBack(CanTxQHandle,&gevcufunction.canmsg[CID_GEVCUR_CTL_LAWV1],4);
	}

	/* Send required suite of three dmoc command msgs, and reset send flag. */
	dmoc_control_CANsend(&dmocctl[DMOC_SPEED]); 

	/* Queue GEVCUr keep-alive status CAN msg */
	if ((gevcufunction.outstat & CNCTOUT05KA) != 0)
	{
		gevcufunction.outstat &= ~CNCTOUT05KA;	
	}

	if (stepperstuff.CANsend != 0)
	{
		stepperstuff.CANsend = 0;
		/* Send CAN msg to drum node with CL position and DR, EN bits. */

		// Load float with CL position
	/* When toggle not ON, update continuously. */
		if (gevcufunction.stepperclpostoggle != 0)
		{ // Toggled state: freeze minimum
			if (clfunc.curpos > gevcufunction.stepperclpos)
			{
				payloadfloat(&gevcufunction.canmsg[CID_GEVCUR_TST_STEPCMD].can.cd.uc[1],clfunc.curpos);
			}
			else
			{
				payloadfloat(&gevcufunction.canmsg[CID_GEVCUR_TST_STEPCMD].can.cd.uc[1],gevcufunction.stepperclpos);
			}
		}
		else
		{ // Not toggled
			payloadfloat(&gevcufunction.canmsg[CID_GEVCUR_TST_STEPCMD].can.cd.uc[1],gevcufunction.stepperclpos);			
		}

/* CAN msg: cid_drum_tst_stepcmd: payload[0] bit definitions. 
 CAN msg: cid_drum_tst_stepcmd: payload[0] bit definitions. 
#define DRBIT 0x01 // (1) Bit mask Direction output pin: 0 = low; 1 = high
#define ENBIT 0x02 // (2) Bit mask Enable output pin: 0 = low; 1 = high
#define LMBIT 0x04 // (3) Bit mask Limit switch simulation
#define IXBIT 0x08 // (4) Bit mask Indexing command
#define ZTBIT 0x10 // (5) Bit mask PB State: Zero Tension
#define ZOBIT 0x20 // (6) Bit mask PB State: Zero Odometer
#define ARBIT 0x40 // (7) Bit mask PB State: ARM
#define PRBIT 0x80 // (8) Bit Mask PB State: PREP
 Notes of above bit usage--
(1) CP PB processed: Zero Odometer TOGGLES direction minus sign on LCD
(2) CP SAFE/ACTIVE: Bit sets when in CP goes into ARM state
(3) CP PB: Zero Tension PB state simulates limit switch
(4) CP PB: ARM PB state simulates CP begin indexing command
(5) CP PB state: Zero Tension (CP toggles direction)
(6) CP PB state: Zero Odometer
(7) CP PB state: ARM
(8) CP PB state: Prep (CP toggles freeze of CL setting)
*/		
		// Load byte with enable, direction, limit sw, index, command bits
		gevcufunction.stepperenbit = HAL_GPIO_ReadPin(EN_port,EN_pin); 

		if (gevcufunction.psw[PSW_ZTENSION]->db_on == SW_CLOSED) 
			gevcufunction.stepperlmbit = LMBIT;

		if (gevcufunction.psw[PSW_PB_ARM]->db_on == SW_CLOSED)
			gevcufunction.stepperixbit = IXBIT;

		gevcufunction.canmsg[CID_GEVCUR_TST_STEPCMD].can.cd.uc[0] = (
			(gevcufunction.stepperdrtoggle & 0x1) |   /* DRBIT */
			(gevcufunction.stepperenbit << 1)     |   /* ENBIT */
		    (gevcufunction.stepperlmbit)          |   /* LMBIT */
   		    (gevcufunction.stepperixbit)          );  /* IXBIT */

		/* Load byte with pushbuttons state. */
		if (gevcufunction.psw[PSW_ZTENSION]->db_on == SW_CLOSED) 
			gevcufunction.canmsg[CID_GEVCUR_TST_STEPCMD].can.cd.uc[0] |= ZTBIT;

		if (gevcufunction.psw[PSW_ZODOMTR]->db_on == SW_CLOSED)
			gevcufunction.canmsg[CID_GEVCUR_TST_STEPCMD].can.cd.uc[0] |= ZOBIT;

		if (gevcufunction.psw[PSW_PB_ARM]->db_on == SW_CLOSED)
			gevcufunction.canmsg[CID_GEVCUR_TST_STEPCMD].can.cd.uc[0] |= ARBIT;

		if (gevcufunction.psw[PSW_PB_PREP]->db_on == SW_CLOSED)
			gevcufunction.canmsg[CID_GEVCUR_TST_STEPCMD].can.cd.uc[0] |= PRBIT;

		// Queue CAN msg for sending
		xQueueSendToBack(CanTxQHandle,&gevcufunction.canmsg[CID_GEVCUR_TST_STEPCMD],4);


		/* =========== CPSWSV1_1 ============= */
		gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[0] = 0; // Status = green, no issues
		/*
		  bit 7 – SAFE/ACTIVE (R-S toggle)
         0  = safe
         1 = active
 bit 6 - (pushbutton)  Arm 
 bit 5 – (pushbutton) Retrieve
 bit 4 – (pushbutton) Zero Tension
 bit 3 – (pushbutton) Zero Odometer  
 bit 2 – (pushbutton) Apply brake
 bit 1 – (pushbutton) Actuate guillotine
 bit 0 – (pushbutton) Emergency
*/

		// SAFE/ACTIVE switch
		if (gevcufunction.psw[PSW_PR_SAFE]->db_on == SWP_OPEN )
		{ // Here SAFE/ACTIVE switch is in ACTIVE position
			gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[1] = 1;
		}
		else
		{	
			gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[1] = 0;
		}			
		// Debounced pushbuttons
		if (gevcufunction.psw[PSW_PB_ARM]->db_on == SW_CLOSED)
			gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[1] |= (1<<6);

		if (gevcufunction.psw[PSW_PB_PREP]->db_on == SW_CLOSED)
			gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[1] |= (1<<5);

		if (gevcufunction.psw[PSW_ZTENSION]->db_on == SW_CLOSED) 
			gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[1] |= (1<<4);

		if (gevcufunction.psw[PSW_ZODOMTR]->db_on == SW_CLOSED)
			gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[1] |= (1<<3);


		gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[2] = (gevcufunction.levelwindmode << 6); // Levelwind Mode 2 bits; Drum #1 selected
		gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[3] = 1; // Drum #1 active
		gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[4] = 0; // spare
		gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[5] = 0; // spare
		gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[6] = 0; // spare
		gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1].can.cd.uc[7] = 0; // spare
				// Queue CAN msg for sending
		xQueueSendToBack(CanTxQHandle,&gevcufunction.canmsg[CID_GEVCUR_HB_CBSWSV1],4);
		/* ========== CPSWSCLV1 ============== */

	}

	/* Reset new & various flags. */
	gevcufunction.evstat &= ~(
		EVSWTIM1TICK | /* Timer tick */
		EVNEWADC       /* new ADC readings */
		);

	return;
}

	
