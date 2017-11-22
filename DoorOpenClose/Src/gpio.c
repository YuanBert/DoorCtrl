/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
extern uint8_t gVerticalLimitFlag;
extern uint8_t gHorizontalLimitFlag;
extern uint8_t gMotorDir;
extern uint8_t gMotorState;

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MotorBRKPin_GPIO_Port, MotorBRKPin_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MotorENPin_Pin|MotorFRPin_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = MotorBRKPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MotorBRKPin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = MotorENPin_Pin|MotorFRPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = HorizontalLimitInttrupt_Pin|VerticalLimitInttrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void BSP_Motor_Init(void)
{
     HAL_GPIO_WritePin(MotorFRPin_GPIO_Port,MotorFRPin_Pin,GPIO_PIN_SET); 
     HAL_GPIO_WritePin(MotorENPin_GPIO_Port,MotorENPin_Pin,GPIO_PIN_SET);
     HAL_GPIO_WritePin(MotorBRKPin_GPIO_Port,MotorBRKPin_Pin,GPIO_PIN_SET);
     gMotorState = 0;
    if(GPIO_PIN_SET == HAL_GPIO_ReadPin(HorizontalLimitInttrupt_GPIO_Port,HorizontalLimitInttrupt_Pin))
    {
      gHorizontalLimitFlag = 1;         //闸机处于关闭状态
    }
    else
    {
      gHorizontalLimitFlag = 0;         
    }
    
    if(GPIO_PIN_SET == HAL_GPIO_ReadPin(VerticalLimitInttrupt_GPIO_Port,VerticalLimitInttrupt_Pin))
    {
      gVerticalLimitFlag = 1;        //闸机处于开启状态
    }
    else
    {
      gVerticalLimitFlag = 0;
    }
}

void BSP_Motor_Running(RunDir dir)
{ 
  if(UPDir == dir) //如果dir == 1，表示电机UP转
  {
    HAL_GPIO_WritePin(MotorFRPin_GPIO_Port,MotorFRPin_Pin,GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(MotorENPin_GPIO_Port,MotorENPin_Pin,GPIO_PIN_RESET);
    
  }
  else   //电机down转
  {
    HAL_GPIO_WritePin(MotorFRPin_GPIO_Port,MotorFRPin_Pin,GPIO_PIN_SET); 
    HAL_GPIO_WritePin(MotorENPin_GPIO_Port,MotorENPin_Pin,GPIO_PIN_RESET);
  }
  gMotorDir = dir;      //表示电机转动的方向
  gMotorState = 1;      //表示电机处在转动转态
}

void BSP_Motor_Stop(void)
{
    HAL_GPIO_WritePin(MotorFRPin_GPIO_Port,MotorFRPin_Pin,GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(MotorENPin_GPIO_Port,MotorENPin_Pin,GPIO_PIN_SET);
    gMotorState = 0;
}

void BSP_Motor_Start(void)
{

}

void BSP_AtmosphereLights(void);

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
