/******************************************************************************
* File Name          : stepper_items.c
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/
/*

inversion
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
#define PULSEWIDTHCNT (TIM2CNTRATE/50000) // 5 us 

/* Struct with all you want to know. */
struct STEPPERSTUFF stepperstuff;

TIM_HandleTypeDef *ptimlocal;

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
	APB1 (TIM2) runs at 42 MHz, so TIM2 max speed counts at 84 MHz. If the
	max stepper rate is 40 KHz, that is 25 us per step. Since the output pin
	toggles upon each output catpure, the duration between oc interrupts is
	1/2, or 12.5 us. At 84 MHz 12.5 us is a count of 1050.

	Since the CL position is 0.0 - 100.0, at a CL setting of 100.0 the factor
	is (1050 * 100). 
	*/

	stepperstuff.clfactor = (1E-7); //(1.0/(1050*100));
	stepperstuff.zerohold = 0;
	stepperstuff.ocinc    = 42000000;//0x7FFFFFFF-101; // 50 sec per step (almost zero)
	stepperstuff.ocnxt    = 42000000;//0x7FFFFFFF-101; // 50 sec per step (almost zero)
	stepperstuff.ocupd   = (TIM2CNTRATE/UPDATERATE); // Number of timer ticks for update

	return;
}

/* *************************************************************************
 * void stepper_items_init(TIM_HandleTypeDef *phtim2);
 * phtim2 = pointer to timer handle
 * @brief	: Initialization of channel increment
 * *************************************************************************/
void stepper_items_init(TIM_HandleTypeDef *phtim2)
{
	/* Save locally for faster calling. */
	ptimlocal = phtim2;

	/* Initialize parameters. */
	stepper_idx_v_struct_hardcode_params();

	/* Channel 2 - PU (Stepper pulse line). */
	phtim2->Instance->CCR2 = phtim2->Instance->CNT + stepperstuff.ocupd; //
//	HAL_TIM_OC_Start_IT(phtim2, TIM_CHANNEL_2);
	phtim2->Instance->DIER   = (1<<2); // Enable OC interrupt CH2
	phtim2->Instance->EGR   |= (1<<2); // Generate Event for OC CH2
	phtim2->Instance->CCMR1  = (0X3 << 12); // Toggle OC2
	phtim2->Instance->CCMR1  = 

	/* Channel 1 - Internal update. (No output pin). */
//	phtim2->Instance->CCR1 = phtim2->Instance->CNT + stepperstuff.oc1inc; // Update channel
//	HAL_TIM_OC_Start_IT(phtim2, TIM_CHANNEL_1);

	//HAL_TIM_Base_Start_IT(phtim2);
	phtim2->Instance->CR1 |= 1;

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
	}
	else
	{
		stepperstuff.ocnxt = 84000000;//0x7FFFFFFF; // 50 sec per step (almost zero)
	}

	/* Are we increasing speed: reducing timer duration. */
	ntmp = (stepperstuff.ocnxt - stepperstuff.ocinc);
	if (ntmp < -1000)
	{
		itmp = ptimlocal->Instance->CCR2 - ptimlocal->Instance->CNT;
		if (itmp > 1000)
		{
			ptimlocal->Instance->CCR2 = ptimlocal->Instance->CNT + stepperstuff.ocnxt;
		}

	}

	stepperstuff.ocinc = stepperstuff.ocnxt;

	return;
}


/*#######################################################################################
 * ISR routine for TIM2
 *####################################################################################### */
void stepper_items_IRQHandler(TIM_HandleTypeDef *phtim2)
{
	__attribute__((__unused__))int temp;
	int32_t tmp;

	/* Next Channel 2 iinterrupt. */
	phtim2->Instance->SR = ~(0x1F);	// Reset CH2 flag

	/* Increment OC CH2 for next interrupt. */
	phtim2->Instance->CCR2 += stepperstuff.ocinc;

HAL_GPIO_TogglePin(GPIOD, LED_GREEN_Pin);


	temp = phtim2->Instance->SR;	// Avoid any tail-chaining
	return;
}
