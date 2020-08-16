/******************************************************************************
* File Name          : stepper_items.c
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/
/*
branch: inversion

08/10/2020 - pins added to Control Panel for stepper testing

Control lines: Output pin drives FET gate, open drain to controller opto-isolator
PE5  - TIM9CH1 Stepper Pulse: PU (TIM1 Break shared interrupt vector)
   Interuupt vector: TIM1_BRK_TIM9_IRQHandler
PB0  - Direction: DR 
PB1  - Enable: EN  

Limit switches: resistor pullup to +5v. Contact closes to gnd
   Interrupt vector: EXTI15_10_IRQHandler (common to PE10-PE15)
PE10 - EXTI10 Inside  Limit switch: NO contacts (switch connects to gnd)
PE11 - EXTI11 Inside  Limit switch: NC contacts (switch connects to gnd)
PE12 - EXTI12 Outside Limit switch: NO contacts (switch connects to gnd)
PE13 - EXTI13 Outside Limit switch: NC contacts (switch connects to gnd)
PE14 - EXTI14 Index   Limit switch: NO contacts (switch connects to gnd)
PE15 - EXTI15 Index   Limit switch: NC contacts (switch connects to gnd)


*/
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "stm32f4xx_hal.h"
#include "morse.h"
#include "yprintf.h"
#include "main.h"
#include "stepper_items.h"
#include "calib_control_lever.h"

#define TIM2CNTRATE 84000000   // TIM2 counter rate (Hz)
#define UPDATERATE 100000      // 100KHz interrupt/update rate

#define TIM3CNTRATE 84000000   // TIM3 counter rate (Hz)
#define UPDATERATE 100000      // 100KHz interrupt/update rate
#define TIM3DUR  (TIM3CNTRATE/UPDATERATE) // 1680 counts per interrupt

#define TIM9CNTRATE 168000000 // TIM9 counter rate (Hz)


TIM_TypeDef  *pT9base; // Register base address 
TIM_HandleTypeDef *ptimlocal;

/* Struct with all you want to know. */
struct STEPPERSTUFF stepperstuff;

static TIM_HandleTypeDef *ptim9;

//uint32_t stepper_pu1_inc = ((168*1000000)/1);

/* *************************************************************************
 * void stepper_idx_v_struct_hardcode_params(void);
 * 
 * @brief	: Initialization
 * *************************************************************************/
void stepper_idx_v_struct_hardcode_params(void)
{
	/*
	clfactor: 
	APB2 (TIM9) runs at 84 MHz so it counts at 168 MHz. If max stepper speed is 
	3000 rpm (50 rps) and there are 2000 microsteps per rev, the output pulse
	rate is 100,000 per sec. Since the output pin is toggled the interrupt
	rate would be 200,000 per sec, or 5 us between interrupts.  At 168 counts
	per microsecond the OC increment would be 840. Since the CL position is
	range is 0 - 100.0, that at max speed, the OC increment would be
	1.0/(840000 * CLposition)

	*/

	stepperstuff.clfactor = (1.0/84000.0); 
	stepperstuff.zerohold = 0;
	stepperstuff.ocinc    = 42000000;//0x7FFFFFFF-101; // 50 sec per step (almost zero)
	stepperstuff.ocnxt    = 42000000;//0x7FFFFFFF-101; // 50 sec per step (almost zero)
	stepperstuff.hicnt    =	0;
	stepperstuff.ledctr   =	0;
	stepperstuff.ocext.ui =	0;

	return;
}

/* *************************************************************************
 * void stepper_items_init(TIM_HandleTypeDef *phtim);
 * phtim = pointer to timer handle
 * @brief	: Initialization of channel increment
 * *************************************************************************/
void stepper_items_init(TIM_HandleTypeDef *phtim)
{
	/* Save locally for faster calling. */
	ptimlocal = phtim;

	/* Initialize parameters. */
	stepper_idx_v_struct_hardcode_params();

	/* CH1 - PU (Stepper pulse line). */
	phtim->Instance->SMCR = 0; // Slave mode control register
	phtim->Instance->CCR1 = phtim->Instance->CNT + 10000; //
	phtim->Instance->DIER   = 0x2; // Enable OC interrupt CH1 *ONLY*
	phtim->Instance->EGR   |= (1<<1);     // Generate Event for OC CH1
	phtim->Instance->CCMR1  = (0X3 << 4); // Toggle OC1
	phtim->Instance->CCER   = 1;	      // CH1 pin active output
	phtim->Instance->ARR    = 0xffff;     // Auto-reload reg: max

	pT9base = phtim->Instance;

	/* Start timer. */
	phtim->Instance->CR1 = 1;

	return;
}
/* *************************************************************************
 * void stepper_items_clupdate(uint8_t dr);
 * @param 	: dr = direction: 0 = forward, not 0 = reverse
 * @brief	: Initialization of channel increment
 * *************************************************************************/
void stepper_items_clupdate(uint8_t dr)
{
	int32_t  ntmp;
	uint32_t itmp;

	stepperstuff.speedcmdf = clfunc.curpos * stepperstuff.clfactor;
	if (clfunc.curpos > 0)
	{
		stepperstuff.ocnxt = 1.0/stepperstuff.speedcmdf;
		stepperstuff.zerohold = 0;

	}
	else
	{
		stepperstuff.ocnxt = 0x7FFFFFFF-101; // 50 sec per step (almost zero)
		stepperstuff.zerohold = 1;
	}

	/* Are we increasing speed: reducing timer duration. */
	ntmp = (stepperstuff.ocnxt - stepperstuff.ocinc);
	if (ntmp < -1000)
	{
		itmp = ptimlocal->Instance->CCR1 - ptimlocal->Instance->CNT;
		if (itmp > 1000)
		{
			ptimlocal->Instance->CCR1 += (stepperstuff.ocinc & 0xffff); // Set OC 16b register
			stepperstuff.hicnt = (stepperstuff.ocinc >> 16); // Set countdown ctr
		}

	}

	stepperstuff.ocinc = stepperstuff.ocnxt;

	return;
}

/*#######################################################################################
 * ISR routine for timer (TIM9) generating OC pulses (inversion branch)
 *####################################################################################### */
void stepper_items_IRQHandler(TIM_HandleTypeDef *phtim)
{
	__attribute__((__unused__))unsigned int temp;	// Dummy for readback of hardware registers


		phtim->Instance->SR = ~0x02; // Reset CH1 OC flag

		if (stepperstuff.hicnt == 0)
		{ // Here, begin last overflow cycle
			stepperstuff.hicnt -= 1;

			// Set pin to toggle for CH1 OC upon next OC
			phtim->Instance->CCMR1 |= (0x3 << 4);
		}
		else if (stepperstuff.hicnt < 0)
		{ // Here, hicnt == -1. 

/* LED toggling rate reduction. */				
		stepperstuff.ledctr += 1;
		if (stepperstuff.ledctr > 250)
		{
			stepperstuff.ledctr	= 0;
			HAL_GPIO_TogglePin(GPIOD, LED_GREEN_Pin);		
		}

			// Reload next OC time 
			phtim->Instance->CCR1 += (stepperstuff.ocinc & 0xffff); // Set OC 16b register
			stepperstuff.hicnt = (stepperstuff.ocinc >> 16); // Set countdown ctr
			if (stepperstuff.hicnt == 0)
			{ // Here, multiple cycles are not needed 
				// Set pin to toggle for CH1 OC upon next OC
				phtim->Instance->CCMR1 |= (0x3 << 4);
			}
			else
			{ // Here multiple cycles are needed.
				// Set pin for CH1 OC to do nothing upon next OC event
				phtim->Instance->CCMR1 &= ~(0x7 << 4);
			}
		}
	
		stepperstuff.hicnt -= 1;

	/* Override output setup for zero situation. */
	if (stepperstuff.zerohold != 0)
	{	// Set pin for CH1 OC to do nothing upon next OC event
		phtim->Instance->CCMR1 &= ~(0x7 << 4);
	}

	temp = phtim->Instance->SR; // Readback to avoid tailchaining
	return;
}


