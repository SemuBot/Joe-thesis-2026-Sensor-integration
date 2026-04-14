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

#define MAX_BASELINE_TARGETS 8
#define BASELINE_FRAMES      30
#define BASELINE_TOL_MM      40.0f
#define MAX_VALID_AMP        10000

typedef enum {
    OBSTACLE_CLEAR    = 0,
    OBSTACLE_DETECTED = 1,
} obstacle_status_t;

void obstacle_calibrate(ch_dev_t *dev_ptr, volatile uint8_t *data_ready_flag);
obstacle_status_t obstacle_check(ch_dev_t *dev_ptr, float *closest_mm_out, uint16_t *closest_amp_out);
void obstacle_smooth_update(float range_mm, uint16_t amp);
void obstacle_smooth_get(float *avg_range, uint16_t *avg_amp);
void obstacle_smooth_reset(void);
uint8_t obstacle_smooth_get_count(void);

#endif

#endif /* INC_OBSTACLE_DETECTION_H_ */
