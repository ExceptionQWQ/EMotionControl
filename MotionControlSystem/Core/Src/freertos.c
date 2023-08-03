/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include "key.h"
#include "servo.h"
#include "usart.h"
#include "screen.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




enum MODE_STATE{
    MODE_STATE_NONE,
    MODE_STATE_CALI_A_X,
    MODE_STATE_CALI_A_Y,
    MODE_STATE_CALI_C_X,
    MODE_STATE_CALI_C_Y,
    MODE_STATE_RESET,
    MODE_STATE_BORDER,
};

int mode_state = MODE_STATE_NONE;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for modeSelection */
osThreadId_t modeSelectionHandle;
const osThreadAttr_t modeSelection_attributes = {
  .name = "modeSelection",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



void EchoModeSelectOk()
{
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ModeSelection(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of modeSelection */
  modeSelectionHandle = osThreadNew(ModeSelection, NULL, &modeSelection_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ModeSelection */


void NodeBorder()
{
    for (double x = -0.25; x <= 0.25; x += 0.01) {
        struct BallCoord ballCoord = CalcScreenServoCoord(x, 0.25);
        struct ServoPos servoPos = CalcServoPos(ballCoord);
        servoXPos = servoPos.x; servoYPos = servoPos.y;
        UpdateScreen();
        osDelay(30);
    }
    for (double y = 0.25; y >= -0.25; y -= 0.01) {
        struct BallCoord ballCoord = CalcScreenServoCoord(0.25, y);
        struct ServoPos servoPos = CalcServoPos(ballCoord);
        servoXPos = servoPos.x; servoYPos = servoPos.y;
        UpdateScreen();
        osDelay(30);
    }
    for (double x = 0.25; x >= -0.25; x -= 0.01) {
        struct BallCoord ballCoord = CalcScreenServoCoord(x, -0.25);
        struct ServoPos servoPos = CalcServoPos(ballCoord);
        servoXPos = servoPos.x; servoYPos = servoPos.y;
        UpdateScreen();
        osDelay(30);
    }
    for (double y = -0.25; y <= 0.25; y += 0.01) {
        struct BallCoord ballCoord = CalcScreenServoCoord(-0.25, y);
        struct ServoPos servoPos = CalcServoPos(ballCoord);
        servoXPos = servoPos.x; servoYPos = servoPos.y;
        UpdateScreen();
        osDelay(30);
    }

}

/**
* @brief Function implementing the modeSelection thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModeSelection */
void ModeSelection(void *argument)
{
  /* USER CODE BEGIN ModeSelection */



  /* Infinite loop */
  for(;;)
  {
      ReadKeyBit();
      UpdateScreen();
      switch (mode_state) {
          case MODE_STATE_NONE:
              if (IsModeChange()) {
                  EchoModeSelectOk();
                  if (bit0 == 0 && bit1 == 0) {
                      mode_state = MODE_STATE_CALI_A_X;
                  } else if (bit0 == 1 && bit1 == 0) {
                      mode_state = MODE_STATE_RESET;
                  } else if (bit0 == 0 && bit1 == 1) {
                      mode_state = MODE_STATE_BORDER;
                  }
              }
              break;

          case MODE_STATE_BORDER:
              NodeBorder();
              mode_state = MODE_STATE_NONE;
              break;
      }
    osDelay(1);
  }
  /* USER CODE END ModeSelection */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

