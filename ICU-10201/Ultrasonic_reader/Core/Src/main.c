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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "sensor.h"
#include "obstacle_detection.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>
#include "rosidl_runtime_c/string_functions.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK) { Error_Handler(); }}

extern UART_HandleTypeDef huart2;
extern rcl_publisher_t pub_debug;
extern std_msgs__msg__String debug_msg;

// Create publishers
rcl_publisher_t pub_left, pub_right, pub_middle, pub_debug;
sensor_msgs__msg__Range msg_left, msg_right, msg_middle;
std_msgs__msg__String debug_msg;

extern bool cubemx_transport_open(struct uxrCustomTransport * transport);
extern bool cubemx_transport_close(struct uxrCustomTransport * transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err);
extern size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

extern void * microros_allocate(size_t size, void * state);
extern void microros_deallocate(void * pointer, void * state);
extern void * microros_reallocate(void * pointer, size_t size, void * state);
extern void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();

    // Set micro-ROS transport
    rmw_uros_set_custom_transport(
        true,
        (void *) &huart2,
        cubemx_transport_open,
        cubemx_transport_close,
        (write_custom_func) cubemx_transport_write,
        cubemx_transport_read);

    // Set allocators
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Wait for agent
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);
    }

    // Init micro-ROS
    rclc_support_t support;
    rcl_node_t node;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "ultrasonic_node", "", &support));



    RCCHECK(rclc_publisher_init_default(
        &pub_left, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "ultrasonic/left"));

    RCCHECK(rclc_publisher_init_default(
        &pub_right, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "ultrasonic/right"));

    RCCHECK(rclc_publisher_init_default(
        &pub_middle, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "ultrasonic/middle"));

    RCCHECK(rclc_publisher_init_default(
        &pub_debug, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "ultrasonic/debug"));

    // Configure messages
    msg_left.radiation_type  = sensor_msgs__msg__Range__ULTRASOUND;
    msg_left.field_of_view   = 0.26f;
    msg_left.min_range       = 0.05f;
    msg_left.max_range       = 2.00f;
    rosidl_runtime_c__String__assign(&msg_left.header.frame_id, "sensor_left");

    msg_right.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    msg_right.field_of_view  = 0.26f;
    msg_right.min_range      = 0.05f;
    msg_right.max_range      = 2.00f;
    rosidl_runtime_c__String__assign(&msg_right.header.frame_id, "sensor_right");

    msg_middle.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    msg_middle.field_of_view  = 0.26f;
    msg_middle.min_range      = 0.05f;
    msg_middle.max_range      = 2.00f;
    rosidl_runtime_c__String__assign(&msg_middle.header.frame_id, "sensor_middle");

    // Init sensors
    sensor_group_init();
    for (int s = 0; s < NUMBER_OF_SENSORS; s++) {
        sensor_init(&chirp_devices[s], s);
    }
    reset_interrupt();

    char buf[256];
    obstacle_result_t result = {0};
    static uint8_t near_targets[NUMBER_OF_SENSORS] = {0};
    static uint8_t far_targets[NUMBER_OF_SENSORS] = {0};
    while (1) {
        for (uint8_t sensor = 0; sensor < NUMBER_OF_SENSORS; sensor++) {

            for (uint8_t meas_num = 0; meas_num < 2; meas_num++) {
                ch_trigger(&chirp_devices[sensor]);

                uint8_t sensor_bit = (1 << sensor);
                data_ready &= ~sensor_bit;
                uint32_t start = HAL_GetTick();
                while (!(data_ready & sensor_bit) &&
                       ((HAL_GetTick() - start) < 1000)) {
                    __WFI();
                }

                if (data_ready & sensor_bit) {
                    data_ready &= ~sensor_bit;

                    result = obstacle_check(&chirp_devices[sensor]);

                    if (meas_num == 0) {
                        near_targets[sensor] = result.num_targets;
						obstacle_avg_update(sensor, 0, result);

                    } else {
                        obstacle_avg_update(sensor, 1, result);
                        far_targets[sensor] = result.num_targets;


                        uint8_t use_meas;
                        if (near_targets[sensor] > 1)
                            use_meas = 0;  // Publish near measurement
                        else
                            use_meas = 1;  // Publish far measurement

                        float avg_range; uint16_t avg_amp;
                        obstacle_avg_get(sensor, use_meas, &avg_range, &avg_amp);

                        if(sensor == 1){
							sprintf(buf, "Sensor: %s || meas: %s || range: %u || amp: %u\r\n",
									sensor == 1 ? "left" : "right",
									use_meas == 0 ? "near" : "far",
									(uint16_t)avg_range,
									avg_amp);
							uart_print(&pub_debug, &debug_msg, buf);
                        }

                        uint32_t now_ms = HAL_GetTick();

                        if (sensor == 1) {
                            msg_left.header.stamp.sec     = now_ms / 1000;
                            msg_left.header.stamp.nanosec = (now_ms % 1000) * 1000000;
                            msg_left.range = (avg_range > 0) ? (avg_range / 1000.0f) : msg_left.max_range;
                            rosidl_runtime_c__String__assign(&msg_left.header.frame_id,
                                use_meas == 0 ? "sensor_left_near" : "sensor_left_far");
                            rcl_publish(&pub_left, &msg_left, NULL);
                        } else if (sensor == 0){
                            msg_right.header.stamp.sec     = now_ms / 1000;
                            msg_right.header.stamp.nanosec = (now_ms % 1000) * 1000000;
                            msg_right.range = (avg_range > 0) ? (avg_range / 1000.0f) : msg_right.max_range;
                            rosidl_runtime_c__String__assign(&msg_right.header.frame_id,
                                use_meas == 0 ? "sensor_right_near" : "sensor_right_far");
                            rcl_publish(&pub_right, &msg_right, NULL);
                        } else if (sensor == 2){
                        	msg_middle.header.stamp.sec     = now_ms / 1000;
                        	msg_middle.header.stamp.nanosec = (now_ms % 1000) * 1000000;
                        	msg_middle.range = (avg_range > 0) ? (avg_range / 1000.0f) : msg_middle.max_range;
                            rosidl_runtime_c__String__assign(&msg_middle.header.frame_id,
                                use_meas == 0 ? "sensor_middle_near" : "sensor_middle_far");
                            rcl_publish(&pub_middle, &msg_middle, NULL);
                        }
                    }
                } else {
                    reset_interrupt();
                }
            }
            HAL_Delay(20);
        }
        HAL_Delay(180);
    }



}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
