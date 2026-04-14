/*
 * sensor.h
 *
 *  Created on: Apr 5, 2026
 *      Author: aleks
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <invn/soniclib/soniclib.h>
#include <invn/soniclib/sensor_fw/icu_gpt/icu_gpt.h>
#include "measurement_config.h"
#include "obstacle_detection.h"

#define NUMBER_OF_SENSORS 2

extern volatile ch_interrupt_type_t last_int_type;
extern volatile uint32_t irq_count;
extern volatile uint32_t data_irq_count;
extern volatile uint8_t data_ready;

extern ch_group_t chirp_group;
extern ch_dev_t chirp_devices[NUMBER_OF_SENSORS];
extern ch_dev_t *dev_ptr;

void sensor_group_init(void);
void sensor_init(ch_dev_t *dev, uint8_t io_index);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void io_int_callback(ch_group_t *grp_ptr, uint8_t io_index, ch_interrupt_type_t int_type);

#endif
