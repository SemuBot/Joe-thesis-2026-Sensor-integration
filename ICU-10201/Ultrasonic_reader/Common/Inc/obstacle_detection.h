/*
 * background.h
 *
 *  Created on: Mar 25, 2026
 *      Author: aleks
 */

#ifndef INC_OBSTACLE_DETECTION_H_
#define INC_OBSTACLE_DETECTION_H_

#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <stdint.h>
#include <invn/soniclib/soniclib.h>
#include <invn/soniclib/sensor_fw/icu_gpt/icu_gpt.h>
#include <invn/soniclib/chirp_bsp.h>
#include "stm32f3xx_hal.h"

#define MAX_VALID_AMP        	10000
#define MAX_VALID_MM			1300.0f

typedef enum {
    OBSTACLE_CLEAR    = 0,
    OBSTACLE_DETECTED = 1,
} obstacle_status_t;

typedef struct {
    obstacle_status_t status;
    float             range_mm;
    uint16_t          amplitude;
    uint8_t           num_targets;
} obstacle_result_t;

obstacle_result_t obstacle_check(ch_dev_t *dev_ptr);
void obstacle_avg_update(uint8_t sensor, uint8_t meas, obstacle_result_t result);
void obstacle_avg_get(uint8_t sensor, uint8_t meas, float *out_mm, uint16_t *out_amp);

#endif

#endif /* INC_OBSTACLE_DETECTION_H_ */
