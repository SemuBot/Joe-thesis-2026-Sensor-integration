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
#include "hcsr04.h"
#include "tim.h"
#include "system_clock.h"
#include "measurement.h"

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
static agent_state_t agent_state = AGENT_WAITING;

extern bool cubemx_transport_open(struct uxrCustomTransport * transport);
extern bool cubemx_transport_close(struct uxrCustomTransport * transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err);
extern size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

extern void * microros_allocate(size_t size, void * state);
extern void microros_deallocate(void * pointer, void * state);
extern void * microros_reallocate(void * pointer, size_t size, void * state);
extern void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();

    char buf[64];

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
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t pub_left, pub_right, pub_middle, pub_debug;

    sensor_msgs__msg__Range msg_left, msg_right, msg_middle;
    std_msgs__msg__String debug_msg;

	// Configure messages
	msg_left.radiation_type  = sensor_msgs__msg__Range__ULTRASOUND;
	msg_left.field_of_view   = 0.26f;
	msg_left.min_range       = 0.04f;
	msg_left.max_range       = 4.00f;

	msg_right.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
	msg_right.field_of_view  = 0.26f;
	msg_right.min_range      = 0.04f;
	msg_right.max_range      = 4.00f;

	msg_middle.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
	msg_middle.field_of_view  = 0.26f;
	msg_middle.min_range      = 0.04f;
	msg_middle.max_range      = 4.00f;

	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	hcsr04_init();

	uint32_t last_ping = 0;
	float avg_range_mm = 0;
	while (1) {

		switch (agent_state){

			case AGENT_WAITING:
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				HAL_Delay(500);

				if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
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

					rosidl_runtime_c__String__init(&debug_msg.data);
					rosidl_runtime_c__String__init(&msg_left.header.frame_id);
					rosidl_runtime_c__String__init(&msg_right.header.frame_id);
					rosidl_runtime_c__String__init(&msg_middle.header.frame_id);

					rosidl_runtime_c__String__assign(&msg_left.header.frame_id, "sensor_left");
					rosidl_runtime_c__String__assign(&msg_right.header.frame_id, "sensor_right");
					rosidl_runtime_c__String__assign(&msg_middle.header.frame_id, "sensor_middle");

					sprintf(buf, "Agent connected\r\n");
					uart_print(&pub_debug, &debug_msg, buf);

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					agent_state = AGENT_CONNECTED;
				}
				break;

			case AGENT_CONNECTED:

				// Check Micro-ROS agent connectivity. If disconnected then stop all topics and try to reconnect. //
				if (HAL_GetTick() - last_ping > 3000) {
					sprintf(buf, "Pinging Agent\r\n");
					uart_print(&pub_debug, &debug_msg, buf);
					last_ping = HAL_GetTick();
					if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
						rcl_publisher_fini(&pub_left,   &node);
						rcl_publisher_fini(&pub_right,  &node);
						rcl_publisher_fini(&pub_middle, &node);
						rcl_publisher_fini(&pub_debug,  &node);
						rcl_node_fini(&node);
						rclc_support_fini(&support);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
						agent_state = AGENT_LOST;
						break;
					}
				}

				for (uint8_t s = 0; s < HCSR04_NUM_SENSORS; s++) {
					hcsr04_sensors[s].captured = 0;
					hcsr04_sensors[s].measuring = 0;

					hcsr04_trigger(s);

					/*
					sprintf(buf, "S%u triggered\r\n", s);
					uart_print(&pub_debug, &debug_msg, buf);
					*/
					uint32_t start = HAL_GetTick();
					while (!hcsr04_sensors[s].captured &&
						   (HAL_GetTick() - start) < 50) {
						HAL_Delay(1);
					}

					/*
					sprintf(buf, "S%u captured=%u status=%u range=%u mm\r\n",
						  s,
						  hcsr04_sensors[s].captured,
						  hcsr04_sensors[s].status,
						  (uint16_t)hcsr04_sensors[s].range_mm);
					uart_print(&pub_debug, &debug_msg, buf);
					*/

					if (hcsr04_sensors[s].status == HCSR04_OK) {
						measurement_update(s, &hcsr04_sensors[s], &avg_range_mm);
						uint32_t now_ms = HAL_GetTick();

						if (s == 0) {
							msg_right.range = avg_range_mm / 1000.0f;
							msg_right.header.stamp.sec = now_ms / 1000;
							msg_right.header.stamp.nanosec = (now_ms % 1000) * 1000000;
							rcl_publish(&pub_right, &msg_right, NULL);
						} else if (s == 1) {
							msg_middle.range = avg_range_mm / 1000.0f;
							msg_middle.header.stamp.sec = now_ms / 1000;
							msg_middle.header.stamp.nanosec = (now_ms % 1000) * 1000000;
							rcl_publish(&pub_middle, &msg_middle, NULL);
						} else {
							msg_left.range = avg_range_mm / 1000.0f;
							msg_left.header.stamp.sec = now_ms / 1000;
							msg_left.header.stamp.nanosec = (now_ms % 1000) * 1000000;
							rcl_publish(&pub_left, &msg_left, NULL);
						}
					}
					HAL_Delay(30);
				}
				break;

			case AGENT_LOST:
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				HAL_Delay(200);  // fast blink to indicate reconnecting
				if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
					agent_state = AGENT_WAITING;  // go through full init again
				}
				break;

		}
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
        hcsr04_tim1_ic_callback(htim);
    if (htim->Instance == TIM2)
        hcsr04_tim2_ic_callback(htim);
    if (htim->Instance == TIM3)
        hcsr04_tim3_ic_callback(htim);
}




/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {  // was TIM1
        HAL_IncTick();
    }
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
