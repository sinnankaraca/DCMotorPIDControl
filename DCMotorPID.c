/*******************************************************************************
 * File Name          : DCMotorPID.c
 * Description        : The code for PID control of DC motor speed and position.
 *						DC motor has driven by PWM signal to motor driver.
 *						Feedback signal are taken from encoder singals.
 *						Rpm has been calculated with encoder signals with 100ms
 *						sampling rate.
 * Author:              Group 1
 *						Sinan KARACA
 *						Mohammed Al Bunde
 * Date:                12.09.2021
 ******************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "common.h"
#include "main.h"
#include "math.h"

 //Necessary function for PWM running
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void speedTime(void);

//Global variable declerations
uint32_t speedPwm;
uint32_t tempCount;
int32_t error;
int32_t calculatedPwm = 0;
int32_t integral = 0;
int32_t derivative = 0;
int32_t lastError;
uint32_t mode;
int32_t calcDif;
uint32_t tempPwmInInterrupt;


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef tim11;
TIM_HandleTypeDef tim3;
TIM_Encoder_InitTypeDef encoderConfig;


// Initialise the two timers
// Initialise the GPIO for pwm output of 3 channel
ParserReturnVal_t glow(int action) {

	//Init TIM3 counter for position control
	TIM3->CNT = 0;

	HAL_StatusTypeDef rc;

	fetch_uint32_arg(&mode);
	fetch_uint32_arg(&speedPwm);

	//Mode 0 for speed control
	//Mode 1 for position control
	//Mode 0 will change motor only for CW
	//Mode 1 will change the motor both sides
	//Inside interrupt function it will change CW or CCW
	//There are only 2 modes
	if (mode == 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
		if (speedPwm > 1000) {
			printf("Speed between 0-1000!!");
		}
	}
	else if (mode == 1) {
		if (speedPwm > 360) {
			printf("Angle between 0-360!!");
		}


	}
	else if (mode > 1) {

		printf("Check mode between 0-1");
		return CmdReturnBadParameter1;
	}



	//Change the speed by manipulating the TIM11 period

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* initialize GPIO pins PA6 and PA7 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* Configure these timer pins mode using
	HAL_GPIO_Init() */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = 2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	__HAL_RCC_TIM3_CLK_ENABLE();
	tim3.Instance = TIM3;
	tim3.Init.Prescaler = 0;
	tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim3.Init.Period = 0xffff;
	tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim3.Init.RepetitionCounter = 0;
	rc = HAL_TIM_Base_Init(&tim3);
	if (rc != HAL_OK) {
		printf("Failed to initialize Timer 3 Base, “ ”rc=%u\n", rc);
		return CmdReturnBadParameter1;
	}

	encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	encoderConfig.IC1Polarity = 0;
	encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	encoderConfig.IC1Prescaler = 0;
	encoderConfig.IC1Filter = 3;
	encoderConfig.IC2Polarity = 0;
	encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	encoderConfig.IC2Prescaler = 0;
	encoderConfig.IC2Filter = 3;

	rc = HAL_TIM_Encoder_Init(&tim3, &encoderConfig);
	if (rc != HAL_OK) {
		printf("Failed to init Timer 3 Encoder," "rc=%u\n", rc);
		return CmdReturnBadParameter1;
	}

	rc = HAL_TIM_Encoder_Start_IT(&tim3, TIM_CHANNEL_1);
	if (rc != HAL_OK) {
		printf("Failed to start Timer 3 Encoder," "rc=%u\n", rc);
		return CmdReturnBadParameter1;
	}
	rc = HAL_TIM_Encoder_Start_IT(&tim3, TIM_CHANNEL_2);
	if (rc != HAL_OK) {
		printf("Failed to start Timer 3 Encoder," "rc=%u\n", rc);
		return CmdReturnBadParameter1;
	}

	__GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = (GPIO_PIN_8 | GPIO_PIN_9);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	__HAL_RCC_TIM1_CLK_ENABLE();
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	__HAL_RCC_TIM11_CLK_ENABLE();
	tim11.Instance = TIM11;
	tim11.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 10000 - 1;
	tim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim11.Init.Period = 999;
	tim11.Init.ClockDivision =
		TIM_CLOCKDIVISION_DIV1;
	tim11.Init.RepetitionCounter = 0;

	HAL_TIM_Base_Init(&tim11);

	HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 10, 0U);
	HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

	HAL_TIM_Base_Start_IT(&tim11);

	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
		!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
		!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
		!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
		!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
		!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);

	//Enable 3 channels
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	return CmdReturnOk;
}


ADD_CMD("pwm", glow, "Mode<0-1> SpeedAngle");



// FUNCTION      : HAL_TIM_MspPostInit()
// DESCRIPTION   : Initialise the 3 output channel pins
//		   In this lab, only one PWM output has been used.
// PARAMETERS    : TIM_HandleTypeDef -- Take the TIM1 as input
// RETURNS       : None

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (htim->Instance == TIM1) {

		__HAL_RCC_GPIOA_CLK_ENABLE();
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	}

}


// FUNCTION      : monitorSpeed()
// DESCRIPTION   : Shows the DC motor speed in RPM
// PARAMETERS    : int : Comes from terminal
// USAGE	     : speed
// RETURNS       : ParserReturnVal_t and display the speed

ParserReturnVal_t speed(int action) {

	printf("rpm : %ld\n", tempCount);
	return CmdReturnOk;

}

ADD_CMD("Speed", speed, "Show Speed!");



// FUNCTION      : TIM1_TRG_COM_TIM11_IRQHandler()
// DESCRIPTION   : Interrupt handler function for TIM11
//		   Used to measure time
// PARAMETERS    : None
// RETURNS       : None

void TIM1_TRG_COM_TIM11_IRQHandler(void) {

	HAL_TIM_IRQHandler(&tim11);

}

// FUNCTION      : TIM3_IRQHandler()
// DESCRIPTION   : Interrupt handler function for TIM3
//		   PWM interrupt function
// PARAMETERS    : None
// RETURNS       : None

void TIM3_IRQHandler(void) {

	HAL_TIM_IRQHandler(&tim3);

}


// FUNCTION      : HAL_TIM_PeriodElapsedCallback()
// DESCRIPTION   : It changes the duty cycle percentage for each interrupt cycle - 100 ms.
// PARAMETERS    : TIM_HandleTypeDef ---- TIM11 
// RETURNS       : None

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {

	if (htim == &tim11) {
		if (mode == 0) {

			tempCount = TIM3->CNT;


			// Each turn produce 120 pulses from encoder
			// Sampling 100 ms
			// For rotation per minute : 60 second = 100 ms x 600
			// Per rotation is 120 pulse so 600 / 5 times counter will give rpm
			tempCount = tempCount * 5;

			// Make cnt 0 for next measurement
			TIM3->CNT = 0;

			//Calculate PID values
			// P
			error = speedPwm - tempCount;
			// D
			derivative = error - lastError;
			// I
			integral = integral + error;


			// Sum PID values
			// Kp = 0.1 Kd = 0.6 KI = 0.05
			calculatedPwm = error * 0.01 + derivative * 0.6 + integral * 0.01;



			//Update the pwm value
			//Check if value reached maximum
			if (calculatedPwm > 750) {
				htim1.Instance->CCR1 = 750;
			}
			else {
				htim1.Instance->CCR1 = calculatedPwm;
			}


			// Get the last error for next state
			lastError = error;



		}
		else if (mode == 1) {

			calcDif = speedPwm - tempCount;

			tempCount = TIM3->CNT;

			tempCount = tempCount * 3;

			//Calculate PID values
			// P
			error = speedPwm - tempCount;
			// D
			derivative = error - lastError;
			// I
			integral = integral + error;

			if (TIM3->CNT > 65000) {
				TIM3->CNT = 0;
			}

			if (calcDif > 3) {

				//Change rotation to CW
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);

				//Kp : 0.1 Kd : 0.6 KI : 0.05
				calculatedPwm = error * 0.1 + derivative * 0.6 + integral * 0.05;


				if (calculatedPwm > 150) {
					tempPwmInInterrupt = 150;
				}
				else {
					tempPwmInInterrupt = calculatedPwm;
				}


			}
			else if (calcDif < 3) {

				//Change rotation to CCW
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);

				//Kp : 0.1 Kd : 0.6 KI : 0.05
				calculatedPwm = error * 0.1 + derivative * 0.6 + integral * 0.05;


				//Limit the speed between 0-150
				if (calculatedPwm < -150) {
					tempPwmInInterrupt = 150;
				}
				else {
					tempPwmInInterrupt = -1 * calculatedPwm;
				}

			}
			else {

				//If it reaches to poisiton stop the turning.
				tempPwmInInterrupt = 0;

			}

			htim1.Instance->CCR1 = tempPwmInInterrupt;

		}

	}

}


