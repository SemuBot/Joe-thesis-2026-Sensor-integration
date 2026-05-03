/*
 * tim.c
 *
 *  Created on: Mar 25, 2026
 *      Author: aleks
 */

/* USER CODE BEGIN Header */

#include <tim.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

void MX_TIM1_Init(void)
{
    TIM_IC_InitTypeDef cfg = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 71;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 0xFFFF;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_IC_Init(&htim1);

    cfg.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;
    cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
    cfg.ICPrescaler = TIM_ICPSC_DIV1;
    cfg.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&htim1, &cfg, TIM_CHANNEL_1);

    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void MX_TIM2_Init(void)
{
    TIM_IC_InitTypeDef cfg = {0};

    htim2.Instance           = TIM2;
    htim2.Init.Prescaler     = 71;
    htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim2.Init.Period        = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&htim2);

    cfg.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;
    cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
    cfg.ICPrescaler = TIM_ICPSC_DIV1;
    cfg.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&htim2, &cfg, TIM_CHANNEL_4);

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void MX_TIM3_Init(void)
{
    TIM_IC_InitTypeDef cfg = {0};

    htim3.Instance           = TIM3;
    htim3.Init.Prescaler     = 71;
    htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3.Init.Period        = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&htim3);

    cfg.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;
    cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
    cfg.ICPrescaler = TIM_ICPSC_DIV1;
    cfg.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&htim3, &cfg, TIM_CHANNEL_2);

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // PA8 = TIM1_CH1
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // PA10 = TIM2_CH4
        GPIO_InitStruct.Pin       = GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF10_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        // PC7 = TIM3_CH2
        GPIO_InitStruct.Pin       = GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

