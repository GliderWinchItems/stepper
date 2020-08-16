/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "SerialTaskSend.h"
#include "stm32f4xx_hal_pcd.h"
//#include "usbd_cdc_if.h"
#include "cdc_txbuff.h"
#include "CanTask.h"
#include "can_iface.h"
#include "canfilter_setup.h"
#include "stm32f4xx_hal_can.h"
#include "getserialbuf.h"
#include "stackwatermark.h"
#include "yprintf.h"
#include "gateway_comm.h"
#include "gateway_CANtoPC.h"
#include "DTW_counter.h"
#include "SerialTaskReceive.h"
#include "yscanf.h"
#include "adctask.h"
#include "ADCTask.h"
#include "adcparams.h"
#include "adcparamsinit.h"
#include "gateway_PCtoCAN.h"
#include "morse.h"
#include "MailboxTask.h"
#include "GatewayTask.h"
#include "iir_f2.h"
#include "spiserialparallelSW.h"
#include "cdc_txbuff.h"
#include "SpiOutTask.h"
#include "BeepTask.h"
#include "lcdprintf.h"
#include "LEDTask.h"
#include "led_chasing.h"
#include "calib_control_lever.h"
#include "lcdmsg.h"
#include "dmoc_control.h"
#include "control_law_v1.h"

#include "LcdTask.h"
#include "lcd_hd44780_i2c.h"
#include "LcdmsgsTask.h"
#include "LcdmsgsetTask.h"
#include "stepper_items.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void* verr[8];
uint32_t verrx = 0;
__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __get_SP(void) 
{ 
  register uint32_t result; 

  __ASM volatile ("MOV %0, SP\n" : "=r" (result) ); 
  return(result); 
} 

uint32_t timectr = 0;
struct CAN_CTLBLOCK* pctl0;	// Pointer to CAN1 control block
struct CAN_CTLBLOCK* pctl1;	// Pointer to CAN2 control block

uint32_t debugTX1b;
uint32_t debugTX1b_prev;

uint32_t debugTX1c;
uint32_t debugTX1c_prev;

uint32_t debug03;
uint32_t debug03_prev;

extern osThreadId SerialTaskHandle;
extern osThreadId CanTxTaskHandle;
extern osThreadId CanRxTaskHandle;
extern osThreadId SerialTaskReceiveHandle;
extern osThreadId GatewayTaskHandle;

uint16_t m_trap = 450; // Trap codes for MX Init() and Error Handler

uint8_t canflag;
uint8_t canflag1;
uint8_t canflag2;

uint8_t lcdflag = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
osTimerId defaultTaskTimerHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void const * argument);
void CallbackdefaultTaskTimer(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	BaseType_t ret;	   // Used for returns from function calls
	osMessageQId Qidret; // Function call return
	osThreadId Thrdret;  // Return from thread create

// Debug: Clear heap area
//uint32_t* pclr = (uint32_t*)(0x2000bb80);
//while (pclr < (uint32_t*)(0x2000bb80 + 32768)) *pclr++ = 0x66666666;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	DTW_counter_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	/* Add bcb circular buffer to SerialTaskSend for uart6 -- LCD */
	#define NUMCIRBCB6  16 // Size of circular buffer of BCB for uart6
	ret = xSerialTaskSendAdd(&HUARTLCD, NUMCIRBCB6, 1); // dma
	if (ret < 0) morse_trap(1); // Panic LED flashing

	/* Add bcb circular buffer to SerialTaskSend for usart2 -- gateway */
	#define NUMCIRBCB2  16 // Size of circular buffer of BCB for usart2
	ret = xSerialTaskSendAdd(&huart2, NUMCIRBCB2, 1); // dma
	if (ret < 0) morse_trap(2); // Panic LED flashing

	/* Add bcb circular buffer to SerialTaskSend for usart3 -- PC monitor */
	#define NUMCIRBCB3  16 // Size of circular buffer of BCB for usart3
	ret = xSerialTaskSendAdd(&HUARTMON, NUMCIRBCB3, 1); // dma
	if (ret < 0) morse_trap(14); // Panic LED flashing

	/* Setup TX linked list for CAN  */
   // CAN1 (CAN_HandleTypeDef *phcan, uint8_t canidx, uint16_t numtx, uint16_t numrx);
	pctl0 = can_iface_init(&hcan1, 0, 32, 32);
	if (pctl0 == NULL) morse_trap(7); // Panic LED flashing
	if (pctl0->ret < 0) morse_trap(77);

	// CAN 2
#ifdef CONFIGCAN2
	pctl1 = can_iface_init(&hcan2, 1,32, 64);
	if (pctl1 == NULL) morse_trap(8); // Panic LED flashing
	if (pctl1->ret < 0) morse_trap(88);
#endif

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

/*
DiscoveryF4 LEDs --
 GPIOD, GPIO_PIN_12 GREEN
 GPIOD, GPIO_PIN_13 ORANGE
 GPIOD, GPIO_PIN_14 RED
 GPIOD, GPIO_PIN_15 BLUE
*/
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of defaultTaskTimer */
  osTimerDef(defaultTaskTimer, CallbackdefaultTaskTimer);
  defaultTaskTimerHandle = osTimerCreate(osTimer(defaultTaskTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

	/* defaultTask timer for pacing defaultTask output. */
//	ret = xTimerChangePeriod( defaultTaskTimerHandle  ,pdMS_TO_TICKS(64),0);

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 384);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
/* =================================================== */
  /* init code for USB_DEVICE */

//taskENTER_CRITICAL();
  //MX_USB_DEVICE_Init();
  //osDelay(0);
//taskEXIT_CRITICAL();


	/* Create serial task (priority) */
	// Task handle "osThreadId SerialTaskHandle" is global
	Thrdret = xSerialTaskSendCreate(1);	// Create task and set Task priority
	if (Thrdret == NULL) morse_trap(225);

	/* Create serial receiving task. */
	ret = xSerialTaskReceiveCreate(1);
	if (ret != pdPASS) morse_trap(224);

	/* Setup semaphore for yprint and sprintf et al. */
	yprintf_init();

#ifdef USEUSBFORCANMSGS
	/* USB-CDC buffering */
	#define NUMCDCBUFF 4	// Number of CDC task local buffers
	#define CDCBUFFSIZE 64*4	// Best buff size is multiples of usb packet size
	struct CDCBUFFPTR* pret;
	pret = cdc_txbuff_init(NUMCDCBUFF, CDCBUFFSIZE); // Setup local buffers
	if (pret == NULL) morse_trap(223);
	
	/* USB-CDC queue and task creation */
	Qidret = xCdcTxTaskSendCreate(3); // arg = Priority level
	if (Qidret < 0) morse_trap(221); // Panic LED flashing
#endif

  /* definition and creation of CanTxTask - CAN driver TX interface. */
  Qidret = xCanTxTaskCreate(2, 64); // CanTask priority, Number of msgs in queue
	if (Qidret < 0) morse_trap(220); // Panic LED flashing

  /* definition and creation of CanRxTask - CAN driver RX interface. */
  /* The MailboxTask takes care of the CANRx                         */
//  Qidret = xCanRxTaskCreate(1, 32); // CanTask priority, Number of msgs in queue
//	if (Qidret < 0) morse_trap(6); // Panic LED flashing

	/* Setup CAN hardware filters to default to accept all ids. */
	HAL_StatusTypeDef Cret;
	Cret = canfilter_setup_first(0, &hcan1, 15); // CAN1
	if (Cret == HAL_ERROR) morse_trap(219);

#ifdef CONFIGCAN2
	Cret = canfilter_setup_first(1, &hcan2, 15); // CAN2
	if (Cret == HAL_ERROR) morse_trap(217);
#endif

	/* Remove "accept all" CAN msgs and add specific id & mask, or id here. */
	// See canfilter_setup.h

	/* Create MailboxTask */
	xMailboxTaskCreate(2); // (arg) = priority

	/* Create GatewayTask */
	xGatewayTaskCreate(2); // (arg) = priority

	/* GEVCUr state machine. */
	Thrdret = xGevcuTaskCreate(2); // (arg) = priority
	if (Thrdret == NULL) morse_trap(216);

	/* Create Mailbox control block w 'take' pointer for each CAN module. */
	struct MAILBOXCANNUM* pmbxret;
	// (CAN1 control block pointer, size of circular buffer)
	pmbxret = MailboxTask_add_CANlist(pctl0, 48);
	if (pmbxret == NULL) morse_trap(215);

#ifdef CONFIGCAN2
	// (CAN2 control block pointer, size of circular buffer)
	pmbxret = MailboxTask_add_CANlist(pctl1, 48); // Use default buff size
	if (pmbxret == NULL) morse_trap(214);
#endif

	/* Further initialization of mailboxes takes place when tasks start */

	/* Select interrupts for CAN1 */
	HAL_CAN_ActivateNotification(&hcan1, \
		CAN_IT_TX_MAILBOX_EMPTY     |  \
		CAN_IT_RX_FIFO0_MSG_PENDING |  \
		CAN_IT_RX_FIFO1_MSG_PENDING    );

	/* Select interrupts for CAN2 */
#ifdef CONFIGCAN2
	HAL_CAN_ActivateNotification(&hcan2, \
		CAN_IT_TX_MAILBOX_EMPTY     |  \
		CAN_IT_RX_FIFO0_MSG_PENDING |  \
		CAN_IT_RX_FIFO1_MSG_PENDING    );
#endif

	/* Switch logic from queue loaded by Spi interrupts. */
//	Thrdret = xSwitchTaskCreate(1);
//	if (Thrdret == NULL) morse_trap(20); // Panic LED flashing

	/* Spi shift register task. */
	Thrdret = xSpiOutTaskCreate(1);
	if (Thrdret == NULL) morse_trap(213); // Panic LED flashing

	/* Beeper task (taskpriority, beepqsize) */
	Thrdret = xBeepTaskCreate(0, 32);
	if (Thrdret == NULL) morse_trap(212); // Panic LED flashing

	/* LED task (taskpriority, max queued items) .*/
	Thrdret = xLEDTaskCreate(0, 32);
	if (Thrdret == NULL) morse_trap(211); // Panic LED flashing

	/* ADC summing, calibration, etc. */
	xADCTaskCreate(3); // (arg) = priority

	/* Start SPI for switch/led shift register. */
	if (spiserialparallel_init(&hspi2) != HAL_OK) morse_trap(49);

	/* Queue for running LCD out of defaultTask. */
	Qidret = lcdmsg_init(16);
	if (Qidret < 0) morse_trap(210); // Panic LED flashing

  /* LCD I2C: message handling. (taskpriority, number buffers). */
  osThreadId retThrd =  xLcdTaskCreate(0,16);
  if (retThrd == NULL) morse_trap(123);

  /* LCD I2C: message generation (calls to 'printf or'puts). 
  'printf and overrun stalls take place in this task.  
  (task priority, number in queue of pointer|union) */
  retThrd= xLcdmsgsetTaskCreate(0, 16);
  if (retThrd == NULL) morse_trap(125);


  /* init code for USB_DEVICE */
//taskENTER_CRITICAL();
  //MX_USB_DEVICE_Init();
//taskEXIT_CRITICAL();

/* =================================================== */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
	m_trap = 457; // morse_trap(457);
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	m_trap = 456; // morse_trap(456);
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
	m_trap = 455; // morse_trap(455);
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
	m_trap = 454; // morse_trap(454);
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
	m_trap = 453; // morse_trap(453);
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 35000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 15700;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 0xffff;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
	m_trap = 452; // morse_trap(452);
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
	m_trap = 451; // morse_trap(451);
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */
	m_trap = 458; // morse_trap(458);
  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Beeper_Drive_GPIO_Port, Beeper_Drive_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Stepper__DR__direction_Pin|Stepper__MF_not_enable_Pin|SPI2_NSS__CK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_RED_Pin|LED_ORANGE_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Beeper_Drive_Pin */
  GPIO_InitStruct.Pin = Beeper_Drive_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Beeper_Drive_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Stepper__DR__direction_Pin */
  GPIO_InitStruct.Pin = Stepper__DR__direction_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Stepper__DR__direction_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Stepper__MF_not_enable_Pin */
  GPIO_InitStruct.Pin = Stepper__MF_not_enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Stepper__MF_not_enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : StepLiimitSw_inside_NO_Pin StepLimitSw_inside_NC_Pin StepLimitSw_outside_NO_Pin */
  GPIO_InitStruct.Pin = StepLiimitSw_inside_NO_Pin|StepLimitSw_inside_NC_Pin|StepLimitSw_outside_NO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : StepLimitSw_outside_NC_Pin */
  GPIO_InitStruct.Pin = StepLimitSw_outside_NC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(StepLimitSw_outside_NC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : StepLimitSw_index_NO_Pin StepLimitSw_index_NC_Pin */
  GPIO_InitStruct.Pin = StepLimitSw_index_NO_Pin|StepLimitSw_index_NC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_NSS__CK_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS__CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SPI2_NSS__CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin LED_ORANGE_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|LED_ORANGE_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
// LCD msg: Main 
//      
static struct LCDMSGSET lcdi2cfunc1;
  #ifdef TWOCALLSWITHONEARGUMENT  
static struct LCDMSGSET lcdi2cfunc2;
static struct LCDMSGSET lcdi2cfunc3;
  #else
static struct LCDMSGSET lcdi2cfunc4;
  #endif
//                                                                       "12345678901234567890"
static void lcdi2cmsgm1 (union LCDSETVAR u){lcdi2cputs  (&punitd4x20,0,0,"stepINV  20200815 02");}
  #ifdef TWOCALLSWITHONEARGUMENT  
static void lcdi2cmsgM1a(union LCDSETVAR u){lcdi2cprintf(&punitd4x20,DMOCSPDTQ, 0,"S%6i  ",   u.u32);}
static void lcdi2cmsgM1b(union LCDSETVAR u){lcdi2cprintf(&punitd4x20,DMOCSPDTQ, 9,"T%6.1f  ",u.f);}
  #else
static void lcdi2cmsgM1c(union LCDSETVAR u){ 
  if ((lcdcontext & LCDX_CNTR) == 0) // Skip if Contactor is in Fault: do not overwrite Contactor error msg
  lcdi2cprintf(&punitd4x20,DMOCSPDTQ, 0,"S%6i   T%6.1f  ",u.u32two[0],u.ftwo[1]);}
  #endif
static struct LCDMSGSET lcdi2cfunc5;
static void lcdi2cmsgMstep(union LCDSETVAR u){lcdi2cprintf(&punitd4x20,DMOCSPDTQ, 0,"SPS%6i RPM %6.1f  ",u.u32two[0],u.ftwo[1]);}


// LCD UART msg
//struct SERIALSENDTASKBCB* pbuflcd;


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
// ################################################################################
// ######### DEFAULT TASK #########################################################
// ################################################################################
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

osDelay(0); // Debugging HardFault

/* Select code for testing/monitoring by uncommenting #defines */
//#define DISPLAYSTACKUSAGEFORTASKS
//#define SHOWEXTENDEDSUMSOFADCRAWREADINGS
//#define SHOWSUMSOFADCRAWREADINGS
//#define SHOWINCREASINGAVERAGEOFADCRAWREADINGS
//#define SHOWSERIALPARALLELSTUFF
//#define STARTUPCHASINGLEDS
//#define TESTBEEPER
//#define SENDCANTESTMSGSINABURST
//#define SHOWADCCOMMONCOMPUTATIONS
//#define DMOCTESTS
//#define CONTROLV1DEBUG
#define STEPPERSHOW

	#define DEFAULTTSKBIT00	(1 << 0)  // Task notification bit for sw timer: stackusage
	#define DEFAULTTSKBIT01	(1 << 1)  // Task notification bit for sw timer: something else

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	/* notification bits processed after a 'Wait. */
	uint32_t noteused = 0;

	struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&HUARTMON,96);
	if (pbuf1 == NULL) morse_trap(11);

	struct SERIALSENDTASKBCB* pbuf3 = getserialbuf(&HUARTMON,96);
	if (pbuf3 == NULL) morse_trap(13);

	struct SERIALSENDTASKBCB* pbuf2 = getserialbuf(&HUARTMON,96);
	if (pbuf2 == NULL) morse_trap(12);

	struct SERIALSENDTASKBCB* pbuf4 = getserialbuf(&HUARTMON,96);	
	if (pbuf4 == NULL) morse_trap(12);

  /* LCD I2C msgs used in this task. */
  // Wait for LcdTask to instantiate 4x20 #1 unit. */
  uint8_t loopctr = 0;
  while ((punitd4x20 == NULL) && (loopctr++ < 10)) osDelay(10);
      if (punitd4x20 == NULL) morse_trap(2326);

  // Initial "splash:" msg on line 1 of lcdi2c 4x20 #1 display
  lcdi2cfunc1.ptr = lcdi2cmsgm1;

  if (LcdmsgsetTaskQHandle == NULL) morse_trap(2323);
    xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc1, 0);       

// STEPPERSHOW     
  lcdi2cfunc5.ptr = lcdi2cmsgMstep;

  // Get buffers and partially initialize for later use.
#ifdef TWOCALLSWITHONEARGUMENT  
  lcdi2cfunc2.ptr = lcdi2cmsgM1a;
  lcdi2cfunc3.ptr = lcdi2cmsgM1b;
#else
// One call with two arguments
  lcdi2cfunc4.ptr = lcdi2cmsgM1c;
#endif

#ifdef DISPLAYSTACKUSAGEFORTASKS
	int ctr = 0; // Running count
	uint32_t heapsize;
	uint32_t showctr = 0;
	uint32_t t1_DSUFT; // DTW time  
	uint32_t t2_DSUFT;
#endif

#ifdef SENDCANTESTMSGSINABURST
	/* Test CAN msg */
	struct CANTXQMSG testtx;
	testtx.pctl = pctl0;
	testtx.can.id = 0xc2200000;
	testtx.can.dlc = 8;
	int i;
	for (i = 0; i < 8; i++)
		testtx.can.cd.uc[i] = 0x30 + i;
	testtx.maxretryct = 8;
	testtx.bits = 0;
#endif


HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); // BLUE LED


#ifdef SHOWSUMSOFADCRAWREADINGS
extern uint32_t adcsumdb[ADC1IDX_ADCSCANSIZE]; // debug
extern uint32_t adcdbctr;    // debug
uint32_t adcdbctr_prev = adcdbctr;
uint16_t hdrctr = 999;
#endif


#ifdef SHOWEXTENDEDSUMSOFADCRAWREADINGS
uint16_t idx_xsum_prev = 0;
uint16_t hdrctr2 = 999;
#endif


#ifdef SHOWINCREASINGAVERAGEOFADCRAWREADINGS
float  fxxsum[ADC1IDX_ADCSCANSIZE];
float ftmp;
uint64_t xxsum[ADC1IDX_ADCSCANSIZE];
uint32_t ravectr = 0;
float frecip;
uint16_t idx_xxsum_prev = 0;
uint16_t hdrctr3 = 999;
uint8_t ix;
for (ix = 0; ix < ADC1IDX_ADCSCANSIZE; ix++) xxsum[ix] = 0;
#endif

#ifdef  SHOWSERIALPARALLELSTUFF
uint32_t spispctr_prev = 0;
#endif

#ifdef STARTUPCHASINGLEDS
#include "calib_control_lever.h"
struct LEDREQ spiledx;
spiledx.bitnum = 0; // Start with bit 0
struct LEDREQ spiledx_prev;
spiledx_prev.bitnum = 15; // Sart with previous bit 15
uint8_t chasectr = 0; // Counter for slowing down output rate
#endif

#ifdef TESTBEEPER
struct BEEPQ beepqtest;
beepqtest.duron  = 100;
beepqtest.duroff = 50;
beepqtest.repct  = 2;
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  
#endif

#ifdef CONTROLV1DEBUG
uint16_t hdrctr = 0;

#endif

/* Purpose: flag to have other task wait before 'for' loop. */
lcdflag = 1;

/* Countdown timer for various display speeds. */
uint16_t slowtimectr = 0; // Approx 1/sec
uint16_t medtimectr = 0;  // Approx 8/sec

uint8_t ratepace = 0;

//osDelay(1);
	xTimerChangePeriod( defaultTaskTimerHandle  ,pdMS_TO_TICKS(64),0);
// ===== BEGIN FOR LOOP ==============================

	for ( ;; )
	{
		xTaskNotifyWait(noteused, 0, &noteval, portMAX_DELAY);
		noteused = 0;
		if ((noteval & DEFAULTTSKBIT00) != 0)
		{
			noteused |= DEFAULTTSKBIT00;
// ================= Higest rate =======================================
			/* Check each LED until CL calibration ends. */
			led_chasing();

    ratepace += 1;
    if (ratepace > 2) // Slow down LCD output rate
    {
      ratepace = 0;

			/* Update CL position on LCD after CL calibration ends. */
			lcdout();

			/* Generate LCD msg for speed actual and commanded torque. */
			if (flag_clcalibed != 0)
			{
//HAL_GPIO_TogglePin(GPIOD, LED_GREEN_Pin); // GREEN
#ifndef STEPPERSHOW
#ifdef TWOCALLSWITHONEARGUMENT 
        // LCD I2C: two values requires two calls
        lcdi2cfunc2.u.u32 = dmocctl[DMOC_SPEED].speedact; // Value that is passed to function 
        if (LcdmsgsetTaskQHandle != NULL)
            xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc2, 0);

        lcdi2cfunc3.u.f = dmocctl[DMOC_SPEED].ftorquereq; // Value that is passed to function 
        if (LcdmsgsetTaskQHandle != NULL)
            xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc3, 0);
#else
        lcdi2cfunc4.u.u32two[0] = dmocctl[DMOC_SPEED].speedact; // Value that is passed to function 
        lcdi2cfunc4.u.ftwo[1]   = dmocctl[DMOC_SPEED].ftorquereq; // Value that is passed to function 
        if (LcdmsgsetTaskQHandle != NULL)
            xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc4, 0);

#endif          
#endif          
			}

			/* LCD output from queue pointers. */
			lcdmsg_poll();

#ifdef STEPPERSHOW
     lcdi2cfunc5.u.ftwo[1] = stepperstuff.speedcmdf*(84000*30);
     if (stepperstuff.ocinc > 99999 )
      lcdi2cfunc5.u.u32two[0] = 99999;
     else      
      lcdi2cfunc5.u.u32two[0] = stepperstuff.ocinc;

    yprintf(&pbuf4,"\n\r%10u RPM 0.3f",stepperstuff.ocinc,lcdi2cfunc5.u.ftwo[1]);
    if (LcdmsgsetTaskQHandle != NULL)
       xQueueSendToBack(LcdmsgsetTaskQHandle, &lcdi2cfunc5, 0);


#endif      
    }

#ifdef DMOCTESTS
	yprintf(&pbuf1,"\n\rSTATE:dmoc:act: %X new: %X : rep: %X :op: %X",
         dmocctl[DMOC_SPEED].dmocstateact,
         dmocctl[DMOC_SPEED].dmocstatenew,
         dmocctl[DMOC_SPEED].dmocstaterep,
         dmocctl[DMOC_SPEED].dmocopstate);
#endif

#ifdef CONTROLV1DEBUG
	hdrctr += 1;
	if (hdrctr > 16)
	{
		hdrctr = 0;
		yprintf(&pbuf4,"\n\rdesired   act  err intgrtr torqreq state");
	}

	yprintf(&pbuf2,"\n\r%7.1f %5i %4.1f %7.1f %7.1f  %01X",
		clv1.dsrdspd,	                 /* Desired speed */
		dmocctl[DMOC_SPEED].speedact,   /* Actual speed */
		clv1.spderr,	                 /* Speed error   */
		clv1.intgrtr,                   /* PI integrator */
		dmocctl[DMOC_SPEED].ftorquereq, /* Torque requested */
		dmocctl[DMOC_SPEED].dmocopstate);/* DMOC state actual*/
#endif
	
// ================== SLOW ==============================================
/* Countdown timer notifications. */
			slowtimectr += 1;
			if (slowtimectr >= 16)
			{
				slowtimectr = 0;

#ifdef DISPLAYSTACKUSAGEFORTASKS
			/* Display the amount of unused stack space for tasks. */
t1_DSUFT = DTWTIME;
			showctr += 1; 
/* 'for' is to test doing all scans at one timer tick. */
for (showctr = 0; showctr < 13; showctr++)
{
				switch (showctr)
				{
/* Cycle through the tasks. */
case  0: stackwatermark_show(defaultTaskHandle,&pbuf1,"defaultTask--");break;
case  1: stackwatermark_show(SerialTaskHandle ,&pbuf2,"SerialTask---");break;
case  2: stackwatermark_show(CanTxTaskHandle  ,&pbuf3,"CanTxTask----");break;
case  3: stackwatermark_show(MailboxTaskHandle,&pbuf4,"MailboxTask--");break;
case  4: stackwatermark_show(ADCTaskHandle    ,&pbuf1,"ADCTask------");break;
case  5: stackwatermark_show(SerialTaskReceiveHandle,&pbuf2,"SerialRcvTask");break;
case  6: stackwatermark_show(GatewayTaskHandle,&pbuf3,"GatewayTask--");break;
case  7: stackwatermark_show(CdcTxTaskSendHandle,&pbuf4,"CdcTxTask----");break;
case  8: stackwatermark_show(SpiOutTaskHandle, &pbuf1,"SpiOutTask---");break;
case  9: stackwatermark_show(GevcuTaskHandle,  &pbuf2,"GevcuTask----");break;
case 10: stackwatermark_show(BeepTaskHandle,   &pbuf3,"BeepTask-----");break;
case 11: stackwatermark_show(LEDTaskHandle,    &pbuf4,"LEDTask------");break;

case 12:	heapsize = xPortGetFreeHeapSize(); // Heap usage (and test fp working.
			yprintf(&pbuf1,"\n\rGetFreeHeapSize: total: %i free %i %3.1f%% used: %i",configTOTAL_HEAP_SIZE, heapsize,\
				100.0*(float)heapsize/configTOTAL_HEAP_SIZE,(configTOTAL_HEAP_SIZE-heapsize)); break;
default: showctr=0; yprintf(&pbuf1,"\n\r%4i Unused Task stack space--", ctr++); break;
				}
}
t2_DSUFT = DTWTIME;
yprintf(&pbuf2,"\n\rDTW DUR: %d",t2_DSUFT - t1_DSUFT);


#endif

#ifdef TESTBEEPER
			xQueueSendToBack(BeepTaskQHandle,&beepqtest,portMAX_DELAY);

#endif

#ifdef  SENDCANTESTMSGSINABURST
			/* ==== CAN MSG sending test ===== */
			/* Place test CAN msg to send on queue in a burst. */
			/* Note: an odd makes the LED flash since it toggles on each msg. */
uint16_t ib;
			for (ib = 0; ib < 7; ib++)
				xQueueSendToBack(CanTxQHandle,&testtx,portMAX_DELAY);
#endif

#ifdef DEBUGGINGCDCREADINGFROMPC
/* Debugging CDC reading from PC */
extern uint32_t dbuggateway1;
extern uint32_t dbcdcrx;
extern uint32_t dblen;
extern uint32_t cdcifctr;
extern uint32_t dbrxbuff;
yprintf(&pbuf2,"\n\rdbuggateway1: %d dbcdcrx: %d dblen: %d cdcifctr: %d dbrxbuff: %d", dbuggateway1,dbcdcrx,dblen,cdcifctr,dbrxbuff);
#endif
  			}
// ================= Moderate ===================================
/* Countdown timer notifications. */
			medtimectr += 1;
			if (medtimectr >= 4)
			{
				medtimectr = 0;
			
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); // BLUE LED

#ifdef SHOWSERIALPARALLELSTUFF
			/* SPI serial-parallel hw testing. */
			yprintf(&pbuf3,"\n\rspi ctr: %4d wr: %04X rd: %04X",(spispctr - spispctr_prev),spisp_wr[0].u16,spisp_rd[0].u16);
			spispctr_prev = spispctr; // Running count of spi interrupts

//			yprintf(&pbuf2,"\tcurpos %5.1f %5d %5d",clfunc.curpos,adc1.chan[0].sum,adc1.abs[0].adcfil);

//extern struct SWITCHPTR swpair_safeactive;
extern struct SWITCHPTR* psw_cl_fs_no;
extern struct SWITCHPTR* psw_cl_rst_n0;
extern uint16_t sr1;
extern uint16_t srdiff1;
			yprintf(&pbuf4,"\tcurpos %5.1f %d %d %d %d %d %d  %d %d %04X %04X",clfunc.curpos,
					gevcufunction.psw[PSW_ZTENSION]->db_on,
					gevcufunction.psw[PSW_ZODOMTR]->db_on,
					gevcufunction.psw[PSW_PB_PREP]->db_on,
					gevcufunction.psw[PSW_PB_ARM]->db_on,
					psw_cl_fs_no->db_on,
					psw_cl_rst_n0->db_on,
					gevcufunction.psw[PSW_PR_SAFE]->db_on,
					gevcufunction.psw[PSW_PR_SAFE]->on,
					sr1, srdiff1);
#endif

//#define SHOWMAILBOXGEVCU07
#ifdef SHOWMAILBOXGEVCU07
extern struct MAILBOXCAN* pdbg07mbx;
yprintf(&pbuf3,"\n\rCONT %4d %08X %02X",pdbg07mbx->ctr,pdbg07mbx->ncan.can.id,pdbg07mbx->ncan.can.cd.uc[0]);
#endif

#ifdef STARTUPCHASINGLEDS
				if (flag_clcalibed == 0)
				{
					chasectr += 1;
					if (chasectr > 2)
					{
				chasectr = 0;
				/* Send a lit LED down the row, over and over. */
				spiledx.mode = LED_ON; // Turn current LED on
				xQueueSendToBack(LEDTaskQHandle,&spiledx,portMAX_DELAY);

				spiledx_prev.mode = LED_OFF; // Turn previous LED off
				xQueueSendToBack(LEDTaskQHandle,&spiledx_prev,portMAX_DELAY);
				
				spiledx_prev = spiledx; // Update previous
				spiledx.bitnum += 1;    // Advance new
				if (spiledx.bitnum > 15) spiledx.bitnum = 0;
					}
				}
				// When Calibration complete turn off the lit LED
				if (flag_clcalibed == 1)
				{
			flag_clcalibed = 2;
			spiledx_prev.mode = LED_OFF; // Turn previous LED off
			xQueueSendToBack(LEDTaskQHandle,&spiledx_prev,portMAX_DELAY);
				}
#endif

#ifdef SHOWADCCOMMONCOMPUTATIONS
uint32_t dmact_prev = adcommon.dmact;
extern volatile uint32_t adcdbg2;
			yprintf(&pbuf4,"\n\rADC: Vdd: %7.4f %8.4f   Temp: %6.1f %6.1f %i",adcommon.fvdd,adcommon.fvddfilt,adcommon.degC,adcommon.degCfilt,(adcommon.dmact-dmact_prev));
			dmact_prev = adcommon.dmact;

//			yprintf(&pbuf4,"\n\rInternal ref:   %d %d %d\n\r",adc1.chan[ADC1IDX_INTERNALVREF].sum/ADC1DMANUMSEQ, adcommon.ivdd, adcdbg2);

			yprintf(&pbuf1,"\n\rCalibrated: vref: %6.4f",adc1.common.fvref);
#endif

#ifdef SHOWSUMSOFADCRAWREADINGS
				hdrctr += 1;
				if (hdrctr >= 16) // Periodic print header
				{
					hdrctr = 0;
yprintf(&pbuf1,"\n\r     count           CL     12V    5V    spare  temp   vref");
				}
yprintf(&pbuf2,"\n\rctr: %5d adcsum: %6d %6d %6d %6d %6d %6d",(adcdbctr-adcdbctr_prev),adcsumdb[0],adcsumdb[1],adcsumdb[2],adcsumdb[3],adcsumdb[4],adcsumdb[5]);
		adcdbctr_prev = adcdbctr;
#endif

#ifdef SHOWEXTENDEDSUMSOFADCRAWREADINGS
				if (adc1.idx_xsum != idx_xsum_prev)
				{
					hdrctr2 += 1;
					if (hdrctr2 >= 16) // Periodic print header
					{
						hdrctr2 = 0;
yprintf(&pbuf3,"\n\r     count           CL     12V    5V    spare  temp   vref");
				}
				idx_xsum_prev = adc1.idx_xsum;
				yprintf(&pbuf3,"\n\rctr: %5d  exsum: %6.0f %6.0f %6.0f %6.0f %6.0f %6.0f",hdrctr2,
					(float)adc1.chan[0].xsum[1]*(1.0/(float)ADCEXTENDSUMCT),
					(float)adc1.chan[1].xsum[1]*(1.0/(float)ADCEXTENDSUMCT),
					(float)adc1.chan[2].xsum[1]*(1.0/(float)ADCEXTENDSUMCT),
					(float)adc1.chan[3].xsum[1]*(1.0/(float)ADCEXTENDSUMCT),
					(float)adc1.chan[4].xsum[1]*(1.0/(float)ADCEXTENDSUMCT),
					(float)adc1.chan[5].xsum[1]*(1.0/(float)ADCEXTENDSUMCT)	);
				}
#endif

#ifdef SHOWINCREASINGAVERAGEOFADCRAWREADINGS
				if (adc1.idx_xsum != idx_xxsum_prev)
				{
					hdrctr3 += 1;
					if (hdrctr3 >= 16) // Periodic print header
					{
						hdrctr3 = 0;
yprintf(&pbuf3,"\n\r     count           CL     12V    5V    spare  temp   vref");
					}
					idx_xxsum_prev = adc1.idx_xsum;
					ravectr += 1;
					frecip =  (1.0/((float)ADCEXTENDSUMCT * (float)ravectr));
	
					for (ix = 0; ix < ADC1IDX_ADCSCANSIZE; ix++)
					{
						xxsum[ix] += adc1.chan[ix].xsum[1];	
						ftmp = xxsum[ix];
						fxxsum[ix] = ftmp * frecip;
					}
yprintf(&pbuf4,"\n\rctr: %5d incave: %6.0f %6.0f %6.0f %6.0f %6.0f %6.0f",
		ravectr,fxxsum[0],fxxsum[1],fxxsum[2],fxxsum[3],fxxsum[4],fxxsum[5]);
				}
#endif
			}	
	  	}
	}
  /* USER CODE END 5 */ 
}

/* CallbackdefaultTaskTimer function */
void CallbackdefaultTaskTimer(void const * argument)
{
  /* USER CODE BEGIN CallbackdefaultTaskTimer */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (defaultTaskHandle != NULL)
	{
		xTaskNotifyFromISR(defaultTaskHandle, 
			DEFAULTTSKBIT00,	/* 'or' bit assigned to buffer to notification value. */
			eSetBits,      /* Set 'or' option */
			&xHigherPriorityTaskWoken ); 
	}

  /* USER CODE END CallbackdefaultTaskTimer */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM12 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM12) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	morse_trap(m_trap); // Flash error code set in earlier MX Init()'s

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
morse_trap(222);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
