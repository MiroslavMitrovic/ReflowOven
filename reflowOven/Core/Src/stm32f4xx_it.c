/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "variables.h"
#include "functions.h"
#include <stdio_ext.h>
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
const float  alpha=0.1;
static float avg_temp=-100;
volatile float *p_temperature=&temperature;
volatile uint32_t rate_counter;
uint16_t ReflowIndex=0;
uint8_t ReflowEnable;
uint16_t PhaseIndex[5];
char ConsoleMSG[20];
FLAGS Flags;
uint8_t State;
uint8_t FlagBank1;
uint8_t FlagBank2;
 uint16_t counter_us2;
 uint16_t counter_us;
 uint16_t counterBank1;
 uint16_t counterBank2;
volatile float32_t pid_error;
volatile uint32_t PidKorekcija;
volatile uint16_t PIDBank1;
volatile uint16_t PIDBank2;
float32_t PidCorr;
uint32_t PidCorrLim;
 uint8_t ReflowCurve[4000];
 arm_pid_instance_f32 PID;
 extern uint8_t PIDFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

	if((__HAL_GPIO_EXTI_GET_FLAG(ZeroCrossingPin_Pin))	)
	{
		//Test output pin for zero crossing
		HAL_GPIO_TogglePin(GPIOTestPin_GPIO_Port, GPIOTestPin_Pin);

		//PWM activation of both heater banks
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		PIDFlag=1;


	}
	else
	{
		//Do nothing
		//	FlagBank1=0;
		//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	}


  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	counter3++;
	counter4++;
	counter5++;
	rate_counter++;




	if(200==counter3)
	{
		counter3=0;
		readTemperatureData(p_temperature);
		if(avg_temp==-100)
		{
			avg_temp=(*p_temperature);
		}
		else
		{

		}
		avg_temp=alpha*(*p_temperature)+(1-alpha)*avg_temp;
		(*p_temperature)=avg_temp;
	}
	else
	{

	}
	if(500==counter5)
	{
		if(1==ReflowEnable)
		{
			//TODO need to perform modification of PID controller in order to get half power
			//Error
			//pid_error =temperature  -(float32_t)ReflowCurve[ReflowIndex];
			//Error for power limitation
			pid_error =(float32_t)ReflowCurve[ReflowIndex]-temperature;
			//Correction
			PidCorr = arm_pid_f32(&PID, pid_error);
			PidCorrLim=(uint32_t)PidCorr;
			//Correction limits bank1-set value
			if (PidCorrLim > 750)
			{
				PIDBank1 = 750;
			}
			else
			{

			}
			//Correction limits bank2-set value
			if(PidCorrLim>750)
			{
				PIDBank2 =750;
			}
			else
			{

			}
			if (PidCorrLim < 0)
			{
				PIDBank1 = 0;
				PIDBank2 = 0;
			}
			else
			{

			}
			if( (0<=PidCorrLim) && (450>=PidCorrLim)	)
			{
				PIDBank1 = PidCorrLim;
				PIDBank2 = PidCorrLim;
			}
			else
			{

			}
			if( (0<=PidCorrLim) && (750>=PidCorrLim)	)
			{
				PIDBank1 = PidCorrLim;
			}
			else
			{

			}
			//P Control without power limitation
			//TIM3->CCR2=999-PIDBank1;
			//TIM3->CCR3=450-PIDBank2;
			//P Control with power limitation
			TIM3->CCR2=PIDBank1;
			TIM3->CCR3=PIDBank2;
			if((TIM3->CCR2>0) || (TIM3->CCR3>0))
			{
				HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin,GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin,GPIO_PIN_RESET);
			}

			if(	(ReflowIndex >= (PhaseIndex[0]+10)	)	&&	(ReflowIndex < PhaseIndex[1])	 )
			{
				sprintf(ConsoleMSG,"HEAT UP");
				Flags.initComplete=TRUE;
				State=Preheat;
			}
			else
			{
				//do nothing.
			}
			if(ReflowIndex == PhaseIndex[1])
			{
				sprintf(ConsoleMSG,"SOAK");
				Flags.preheatComplete=TRUE;
				State=Soak;
			}
			else
			{
				//do nothing.
			}
			if(ReflowIndex == PhaseIndex[2])
			{
				sprintf(ConsoleMSG,"HEAT UP");
				Flags.preheatComplete=FALSE;
				Flags.soakComplete=TRUE;
				State=Preheat;
			}
			else
			{
				//do nothing.
			}
			if(ReflowIndex == PhaseIndex[3])
			{
				sprintf(ConsoleMSG,"REFLOW");
				Flags.preheatComplete=TRUE;
				State=Reflow;
			}
			else
			{
				//do nothing.
			}
			if(ReflowIndex == PhaseIndex[4])
			{
				HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
				HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin,GPIO_PIN_RESET);
				sprintf(ConsoleMSG,"COOL DOWN");
				Flags.reflowComplete=TRUE;
				State=Cooldown;
			}
			else
			{
				//do nothing.
			}
			if (PhaseIndex[5]==ReflowIndex)
			{
				sprintf(ConsoleMSG,"FINISHED");
				Flags.cooldownComplete=TRUE;
				State=Finish;
				ReflowEnable = 0;
				PIDFlag=0;

			}
			else
			{
				//do nothing.
			}

		}
		else
		{
			ReflowIndex = 0;
		}

		counter5=0;
		ReflowIndex++;
	}
	else
	{

	}


  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
