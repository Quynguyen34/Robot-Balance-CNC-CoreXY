/*
 * motion_controll.c
 *
 *  Created on: Dec 11, 2023
 *      Author: Wis
 */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include "stdlib.h"
#include "usart.h"
#include "motion_controll.h"
#include "AccelStepper.h"
/* USER CODE END Includes */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) > (b) ? (b) : (a))

/* USER CODE END PM */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
	float pos_x;
	float pos_y;
}coreXY_pos_t;

extern Acceleration_t Stepper1;
extern Acceleration_t Stepper2;
extern Acceleration_t Stepper3;
extern Acceleration_t Stepper4;

//extern S_curve_t Stepper1;
//extern S_curve_t Stepper2;
//extern S_curve_t Stepper3;
//extern S_curve_t Stepper4;

coreXY_pos_t coreXY_pos;
coreXY_pos_t current_pos;
float x ;
float y ;
float pos_1 = 0;
float pos_2 = 0;
float pos_3 = 0;
float pos_4 = 0;

int set_pos1 = 0;
int set_pos2 = 0;
int set_pos3 = 0;
int set_pos4 = 0;

float pos1dot = 0;
float pos2dot = 0;
float pos3dot = 0;
float pos4dot = 0;

float accel1 = 0;
float accel2 = 0;
float accel3 = 0;
float accel4 = 0;
float t = 1;

float jerk1,jerk2,jerk3,jerk4;

int deltaA = 0;
int deltaB = 0;
int BalanceA = 0;
int BalanceB = 0;

uint8_t array[10];
uint8_t index = 0;

int maxAB(int a, int b) {
    return max(a, b);
}
int maxCD(int c, int d) {
    return max(c,d);
}

/*
void HOME(void){
	static uint8_t tmp = 0;
}
*/

void MoveToPos(float x, float y){
	set_pos1 = x - pos_1;
	set_pos2 = y - pos_2;
	
	set_pos3 = set_pos1*12.5;
	set_pos4 = set_pos2*12.5;
	
	deltaA = set_pos1*156.25 + set_pos2*156.25;
	deltaB = -(set_pos1*156.25 - set_pos2*156.25);
	
	BalanceA = set_pos3;
	BalanceB = set_pos4;
		
	pos1dot = fabs(deltaA/t);
	pos2dot = fabs(deltaB/t);
	pos3dot = fabs(set_pos3/t);
	pos4dot = fabs(set_pos4/t);

	accel1 = fabs(pos1dot/t);
	accel2 = fabs(pos2dot/t);
	accel3 = fabs(pos3dot/t);
	accel4 = fabs(pos4dot/t);

	jerk1 = fabs(pos1dot/t);
	jerk2 = fabs(pos2dot/t);
	jerk3 = fabs(pos3dot/t);
	jerk4 = fabs(pos4dot/t);

	if(Stepper1.run_state != 1 && Stepper2.run_state != 1 && Stepper3.run_state != 1 && Stepper4.run_state != 1){
		Accel_Stepper_Move(&Stepper1, deltaA, accel1, jerk1, pos1dot);
		Accel_Stepper_Move(&Stepper2, deltaB, accel2, jerk2, pos2dot);
		Accel_Stepper_Move(&Stepper3, BalanceA, accel3, jerk3, pos3dot);
		Accel_Stepper_Move(&Stepper4, BalanceB, accel4, jerk4, pos4dot);
		pos_1 = x;
		pos_2 = y;
		pos_3 = x;
		pos_4 = y;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t received_data[2]; 

    if (huart->Instance == USART1) {
        static uint8_t index = 0;

        received_data[index++] = array[0];

        if (index >= 2) {
            float x = (float)received_data[0];
            float y = (float)received_data[1];
            MoveToPos(x, y);
            index = 0;
        }

        HAL_UART_Receive_IT(&huart1, &array[0], 1);
    }
}

void StartDefaultTask(void)
{
	 Accel_Stepper_SetPin(&Stepper1, step_1_GPIO_Port, step_1_pin, dir_1_GPIO_Port, dir_1_pin);
	 Accel_Stepper_SetPin(&Stepper2, step_2_GPIO_Port, step_2_pin, dir_2_GPIO_Port, dir_2_pin);
	 Accel_Stepper_SetPin(&Stepper3, step_3_GPIO_Port, step_3_pin, dir_3_GPIO_Port, dir_3_pin);
	 Accel_Stepper_SetPin(&Stepper4, step_4_GPIO_Port, step_4_pin, dir_4_GPIO_Port, dir_4_pin);
	 Accel_Stepper_SetTimer(&Stepper1, &htim1);
	 Accel_Stepper_SetTimer(&Stepper2, &htim2);
	 Accel_Stepper_SetTimer(&Stepper3, &htim3);
	 Accel_Stepper_SetTimer(&Stepper4, &htim4);
	 x = 0;
	 y = 0;
	 pos_1 = 0;
	 pos_2 = 0;
	 pos_3 = 0;
	 pos_4 = 0;
	
	 for(;;)
	 {
     HAL_UART_Receive_IT(&huart1, &array[0], 1); 
	 }
}




