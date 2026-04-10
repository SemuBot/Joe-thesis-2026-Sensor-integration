/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

#include <spi.h>
#include <tim.h>

#include "obstacle_detection.h"
#include <GPIO.h>
#include <uart.h>
#include "system_clock.h"
#include "sensor.h"

#define SCAN_SAMPLES 200

void reset_interrupt(void);

static uint16_t amp_data[SCAN_SAMPLES];
extern ch_dev_t chirp_devices[NUMBER_OF_SENSORS];









int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_USART2_UART_Init();
    //MX_TIM8_Init();
    //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

    char buf[512];

    sensor_group_init();
    for (int sensor = 0; sensor < NUMBER_OF_SENSORS; sensor++){
    	sensor_init(&chirp_devices[sensor], sensor);
    }

    reset_interrupt();



    while (1) {
        for (uint8_t sensor = 0; sensor < NUMBER_OF_SENSORS; sensor++) {
            for (uint8_t meas_num = 0; meas_num < 2; meas_num++) {
                ch_trigger(&chirp_devices[sensor]);

                uint8_t sensor_bit = (1 << sensor);
                data_ready &= ~sensor_bit;  // clear only this sensor's bit, not all
                uint32_t start = HAL_GetTick();
                while (!(data_ready & sensor_bit) && ((HAL_GetTick() - start) < 1000)) __WFI();

                if (data_ready & sensor_bit) {
                    data_ready &= ~sensor_bit;
                    uint8_t last_meas = ch_meas_get_last_num(&chirp_devices[sensor]);
                    uint8_t n = icu_gpt_algo_get_num_targets(&chirp_devices[sensor]);
                    sprintf(buf, "S%u meas=%u targets=%u\r\n", sensor, last_meas, n);
                    uart_print(buf);
                    for (uint8_t i = 0; i < n; i++) {
                        float r = icu_gpt_algo_get_target_range(&chirp_devices[sensor], i, CH_RANGE_ECHO_ONE_WAY) / 32.0f;
                        if (r > 5000.0f) continue;
                        uint16_t a = icu_gpt_algo_get_target_amplitude(&chirp_devices[sensor], i);
                        sprintf(buf, "  t%u mm=%.0f amp=%u\r\n", i, r, a);
                        uart_print(buf);
                    }
                } else {
                    sprintf(buf, "S%u meas=%u timeout\r\n", sensor, meas_num);
                    uart_print(buf);
                    reset_interrupt();
                }
            }
            HAL_Delay(20);
        }
        HAL_Delay(180);
    }
}

void reset_interrupt(void){
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
    HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

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
