/******************************************************************************
* File Name          : stepper_items.h
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/

#ifndef __STEPPER_ITEMS
#define __STEPPER_ITEMS

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stepper_items.h"

/* Port and pin numbers for stepper controller. */
#define PU_port  GPIOA      // Pulse
#define PU_pin   GPIO_PIN_5 // Pulse
#define DR_port  GPIOB      // Direction
#define DR_pin   GPIO_PIN_0 // Direction
#define EN_port  GPIOB      // Enable
#define EN_pin   GPIO_PIN_1 // Enable
#define LMIN_port  GPIOE       // Limit switch inner
#define LMIN_pin   GPIO_PIN_5  // Limit switch inner
#define LMOUT_port GPIOE       // Limit switch outside
#define LMOUT_pin  GPIO_PIN_10 // Limit switch outside


struct STEPPERSTUFF
{
	int64_t  position;	// Step count of position
	float	 clfactor;	// Constant to compute oc duration at CL = 100.0
	float    speedcmdf;
	int32_t  speedcmdi;	// Commanded speed (signed)
	uint32_t ocinc;     // Current output capture increment
	uint32_t oc1inc;    // CH1 update output capture increment
	uint32_t ocupd;     // CH2 update increment (100KHz)
	uint32_t ocnxt;     // Next oc increment
	uint8_t  zerohold;  // 0 = no OC pulses; not zero = running
	int64_t  accum1;    // Position accumulator
};

/* *************************************************************************/
 void stepper_items_init(TIM_HandleTypeDef *phtim2);
 /* phtim2 = pointer to timer handle
 * @brief	: Initialization of channel increment
 * *************************************************************************/
 void stepper_items_clupdate(uint8_t dr);
 /* @param 	: dr = direction: 0 = forward, not 0 = reverse
  * @brief	: Initialization of channel increment
 * *************************************************************************/
 void stepper_items_IRQHandler(TIM_HandleTypeDef *phtim2);


 extern struct STEPPERSTUFF stepperstuff;

#endif
