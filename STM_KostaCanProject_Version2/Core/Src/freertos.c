/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fnd_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern CAN_HandleTypeDef hcan;
extern uint8_t_LED_0F[29];
uint8_t Velocity;

CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header

uint8_t canData[8];  //CAN Bus Receive Buffer
uint8_t canRX[8];
uint8_t csend[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
		  canfil.FilterBank = 0;
		  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
		  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
		  canfil.FilterIdHigh = 0;
		  canfil.FilterIdLow = 0;
		  canfil.FilterMaskIdHigh = 0;
		  canfil.FilterMaskIdLow = 0;
		  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
		  canfil.FilterActivation = ENABLE;
		  canfil.SlaveStartFilterBank = 14;

		  txHeader.DLC = 8;
		  txHeader.IDE = CAN_ID_STD;
		  txHeader.RTR = CAN_RTR_DATA;
		  txHeader.StdId = 0x77;
		  //txHeader.ExtId = 0x02;
		  txHeader.TransmitGlobalTime = DISABLE;

		  HAL_CAN_ConfigFilter(&hcan,&canfil);
		  HAL_CAN_Start(&hcan);
		  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

		  /* Infinite loop */
		  for(;;)
		  {
			  Velocity = canData[1];
			  uint8_t tmp = Velocity;
			  if (canData[0] >= 0 && canData[0] <=10) tmp = 0;
			  else if (canData[0] > 10 && canData[0] <= 20) tmp = 10;
			  else if (canData[0] > 20 && canData[0] <= 30) tmp = 15;
			  else if (canData[0] > 30 && canData[0] <= 40) tmp = 20;
			  else if (canData[0] > 40 && canData[0] <= 50) tmp = 25;
			  else{
				  continue;
			  }

			  uint8_t csend[] = {canData[0], tmp, 0, 0, 0, 0 , 0 , 0};
			  HAL_CAN_AddTxMessage(&hcan,&txHeader,csend,&canMailbox);

			  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

			  osDelay(1000);
		  }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	//functionIdle();

  /* Infinite loop */
  for(;;)
  {

	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	  osDelay(1000);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX);
	for(int i = 0; i<2; i++){
		canData[i] = canRX[i];
	}
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);


	//if (rxHeader.StdId == 0x0F6) {
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	//}

}



/* USER CODE END Application */

