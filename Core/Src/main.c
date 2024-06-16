/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int a;
void runAndWait();
void adjustSpeedScales();
void setNextInterruptInterval();
#define NUM_STEPPERS 4
// PA2 - STEP, PA3 - DIR
#define X_DIR_PIN GPIO_PIN_3
#define X_STEP_PIN GPIO_PIN_2
#define X_DIR_PORT GPIOA
#define X_STEP_PORT GPIOA

// PA7 - STEP, PA6 - DIR
#define Y_DIR_PIN GPIO_PIN_6
#define Y_STEP_PIN GPIO_PIN_7
#define Y_DIR_PORT GPIOA
#define Y_STEP_PORT GPIOA

#define BalanceX_DIR_PIN GPIO_PIN_10
#define BalanceX_STEP_PIN GPIO_PIN_11
#define BalanceX_DIR_PORT GPIOB
#define BalanceX_STEP_PORT GPIOB

#define BalanceY_DIR_PIN GPIO_PIN_12
#define BalanceY_STEP_PIN GPIO_PIN_11
#define BalanceY_DIR_PORT GPIOA
#define BalanceY_STEP_PORT GPIOA

#define TIMER1_INTERRUPTS_ON HAL_TIM_Base_Start_IT(&htim2);
#define TIMER1_INTERRUPTS_OFF HAL_TIM_Base_Stop_IT(&htim2);

unsigned long OCR1A;

struct stepperInfo {
  // externally defined parameters
  float acceleration;
  unsigned long minStepInterval;  // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();

  // derived parameters
  unsigned int c0;    // step interval for first step, determines acceleration
  long stepPosition;  // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  int dir;                        // current direction of movement, used to keep track of position
  unsigned int totalSteps;        // number of steps requested for current movement
  int movementDone;               // 1 if the current movement has been completed (used by main program to wait for completion)
  unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)
  unsigned long estStepsToSpeed;  // estimated steps required to reach max speed
  unsigned long estTimeForMove;   // estimated time (interrupt ticks) required to complete movement
  unsigned long rampUpStepTime;
  float speedScale;  // used to slow down this motor to make coordinated movement with other motors

  // per iteration variables (potentially changed every interrupt)
  unsigned int n;          // index in acceleration curve, used to calculate next interval
  float d;                 // current interval length
  unsigned long di;        // above variable truncated
  unsigned int stepCount;  // number of steps completed in current movement
};

void xStep() {
    HAL_GPIO_WritePin(X_STEP_PORT, X_STEP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(X_STEP_PORT, X_STEP_PIN, GPIO_PIN_RESET);
}
void xDir(int dir) {
  HAL_GPIO_WritePin(X_DIR_PORT, X_DIR_PIN, dir);
}

void yStep() {
    HAL_GPIO_WritePin(Y_STEP_PORT, Y_STEP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Y_STEP_PORT, Y_STEP_PIN, GPIO_PIN_RESET);
}
void yDir(int dir) {
  HAL_GPIO_WritePin(Y_DIR_PORT, Y_DIR_PIN, dir);
}

void Balance_XStep() {
    HAL_GPIO_WritePin(BalanceX_STEP_PORT, BalanceX_STEP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BalanceX_STEP_PORT, BalanceX_STEP_PIN, GPIO_PIN_RESET);
}
void Balance_XDir(int dir) {
  HAL_GPIO_WritePin(BalanceX_DIR_PORT, BalanceX_DIR_PIN, dir);
}

void Balance_YStep() {
    HAL_GPIO_WritePin(BalanceY_STEP_PORT, BalanceY_STEP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BalanceY_STEP_PORT, BalanceY_STEP_PIN, GPIO_PIN_RESET);
}
void Balance_YDir(int dir) {
  HAL_GPIO_WritePin(BalanceY_DIR_PORT, BalanceY_DIR_PIN, dir);
}

void resetStepperInfo(struct stepperInfo *si) {
  si->n = 0;
  si->d = 0;
  si->di = 0;
  si->stepCount = 0;
  si->rampUpStepCount = 0;
  si->rampUpStepTime = 0;
  si->totalSteps = 0;
  si->stepPosition = 0;
  si->movementDone = 0;
}



struct stepperInfo steppers[NUM_STEPPERS];
void resetStepper(struct stepperInfo *si) {
  si->c0 = si->acceleration;
  si->d = si->c0;
  si->di = si->d;
  si->stepCount = 0;
  si->n = 0;
  si->rampUpStepCount = 0;
  si->movementDone = 0;
  si->speedScale = 1;

  float a = si->minStepInterval / (float)si->c0;
  a *= 0.676;

  float m = ((a * a - 1) / (-2 * a));
  float n = m * m;

  si->estStepsToSpeed = n;
}

unsigned char remainingSteppersFlag = 0;

float getDurationOfAcceleration(struct stepperInfo *s, unsigned int numSteps) {

//		return s->c0 * sqrt(1.138 * numSteps + numSteps);
  float d = s->c0;
  float totalDuration = 0;
  for (unsigned int n = 1; n < numSteps; n++) {
    d = d - (2 * d) / (4 * n + 1);
    totalDuration += d;
  }
  return totalDuration;
}

void prepareMovement(int whichMotor, long steps) {
  struct stepperInfo *si = &steppers[whichMotor];
  si->dirFunc(steps < 0 ? 1 : 0);
  si->dir = steps > 0 ? 1 : -1;
  si->totalSteps = labs(steps);
  resetStepper(si);

  remainingSteppersFlag |= (1 << whichMotor);

  unsigned long stepsAbs = labs(steps);

  if ((2 * si->estStepsToSpeed) < stepsAbs) {
    // there will be a period of time at full speed
    unsigned long stepsAtFullSpeed = stepsAbs - 2 * si->estStepsToSpeed;
    float accelDecelTime = getDurationOfAcceleration(si, si->estStepsToSpeed);
    si->estTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * si->minStepInterval;
  } else {
    // will not reach full speed before needing to slow down again
    float accelDecelTime = getDurationOfAcceleration(si, stepsAbs / 2);
    si->estTimeForMove = 2 * accelDecelTime;
  }
}

unsigned char nextStepperFlag = 0;


void runAndWait() {
  adjustSpeedScales();
  setNextInterruptInterval();
  TIMER1_INTERRUPTS_ON
  while (remainingSteppersFlag);
  remainingSteppersFlag = 0;
  nextStepperFlag = 0;
}

void adjustSpeedScales() {
  float maxTime = 0;

  for (int i = 0; i < NUM_STEPPERS; i++) {
    if (!((1 << i) & remainingSteppersFlag))
      continue;
    if (steppers[i].estTimeForMove > maxTime)
      maxTime = steppers[i].estTimeForMove;
  }

  if (maxTime != 0) {
    for (int i = 0; i < NUM_STEPPERS; i++) {
      if (!((1 << i) & remainingSteppersFlag))
        continue;
      steppers[i].speedScale = maxTime / steppers[i].estTimeForMove;
    }
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int last_pos1;
int current_pos1;
int deltaA;
void RunIK(int x) {
    current_pos1 = x - last_pos1;
	
    deltaA = (current_pos1 *200);
		prepareMovement(1, deltaA);
		
    runAndWait();
		
    last_pos1 = x;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//  steppers[0].dirFunc = Balance_YDir;
//  steppers[0].stepFunc = Balance_YStep;
//  steppers[0].acceleration = 3000;
//  steppers[0].minStepInterval = 1;
//
  steppers[1].dirFunc = xDir;
  steppers[1].stepFunc = xStep;
  steppers[1].acceleration = 3000;
  steppers[1].minStepInterval = 1;
//
//  steppers[2].dirFunc = yDir;
//  steppers[2].stepFunc = yStep;
//  steppers[2].acceleration = 3000;
//  steppers[2].minStepInterval = 1;
////
//  steppers[3].dirFunc = Balance_XDir;
//  steppers[3].stepFunc = Balance_XStep;
//  steppers[3].acceleration = 3000;
//  steppers[3].minStepInterval = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		RunIK(a);
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7
                           PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		
		if (htim->Instance== htim2.Instance){
    unsigned int tmpCtr = OCR1A;
    OCR1A = 1;
    __HAL_TIM_SET_AUTORELOAD(&htim2, OCR1A);

    for (int i = 0; i < NUM_STEPPERS; i++) {
      if (!((1 << i) & remainingSteppersFlag))
        continue;

      if (!(nextStepperFlag & (1 << i))) {
        steppers[i].di -= tmpCtr;
        continue;
      }

      volatile struct stepperInfo *s = &steppers[i];

      if (s->stepCount < s->totalSteps) {
        s->stepFunc();
        s->stepCount++;
        s->stepPosition += s->dir;
        if (s->stepCount >= s->totalSteps) {
          s->movementDone = 1;
          remainingSteppersFlag &= ~(1 << i);
        }
      }

      if (s->rampUpStepCount == 0) {
        s->n++;
        s->d = s->d - (2 * s->d) / (4 * s->n + 1);
        if (s->d <= s->minStepInterval) {
          s->d = s->minStepInterval;
          s->rampUpStepCount = s->stepCount;
        }
        if (s->stepCount >= s->totalSteps / 2) {
          s->rampUpStepCount = s->stepCount;
        }
        s->rampUpStepTime += s->d;
      } else if (s->stepCount >= s->totalSteps - s->rampUpStepCount) {
        s->d = (s->d * (4 * s->n + 1)) / (4 * s->n + 1 - 2);
        s->n--;
      }

      s->di = s->d * s->speedScale;  // integer
    }

    setNextInterruptInterval();

    __HAL_TIM_SET_COUNTER(&htim2, 0);
		}
}

int movementComplete;
void setNextInterruptInterval() { 
  movementComplete = 1;
  unsigned long mind = 7999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if (((1 << i) & remainingSteppersFlag) && steppers[i].di < mind) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if (!steppers[i].movementDone)
      movementComplete = 0;
    if (((1 << i) & remainingSteppersFlag) && steppers[i].di == mind)
      nextStepperFlag |= (1 << i);
  }

  if (remainingSteppersFlag == 0) {
    TIMER1_INTERRUPTS_OFF
    OCR1A = 1;
    __HAL_TIM_SET_AUTORELOAD(&htim2, OCR1A);
  }
  OCR1A = (uint32_t)mind;
  __HAL_TIM_SET_AUTORELOAD(&htim2, OCR1A);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
