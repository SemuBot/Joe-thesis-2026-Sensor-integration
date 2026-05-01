/*
 * obstacle_detection.c
 *
 *  Created on: Mar 25, 2026
 *      Author: aleks
 */
#include "obstacle_detection.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "sensor.h"
#include "measurement_config.h"

#define SMOOTH_WINDOW	5

static float    	avg_range[NUMBER_OF_SENSORS][NUM_MEAS][SMOOTH_WINDOW];
static uint16_t 	avg_amp[NUMBER_OF_SENSORS][NUM_MEAS][SMOOTH_WINDOW];
static uint8_t  	avg_idx[NUMBER_OF_SENSORS][NUM_MEAS];
static uint8_t  	avg_count[NUMBER_OF_SENSORS][NUM_MEAS];


obstacle_result_t obstacle_check(ch_dev_t *dev_ptr)
{
	obstacle_result_t result = {0};

	result.num_targets = 	icu_gpt_algo_get_num_targets(dev_ptr);
	result.status = 		OBSTACLE_CLEAR;
	result.range_mm = 	0.0;
	result.amplitude = 	0;

    float furthest_mm = 	0;
    uint16_t furthest_amp = 0;
    for (uint8_t i = 0; i < result.num_targets; i++) {
        float range = icu_gpt_algo_get_target_range(dev_ptr, i, CH_RANGE_ECHO_ONE_WAY) / 32.0f;
        uint16_t amp = icu_gpt_algo_get_target_amplitude(dev_ptr, i);

        if (amp > MAX_VALID_AMP || range > MAX_VALID_MM || range <= 0) {
        	result.num_targets -= 1;
			continue;
    	}

		if (range > furthest_mm) {
			furthest_mm = 	range;
			furthest_amp = 	amp;
		}
		result.status = OBSTACLE_DETECTED;

    }

    result.range_mm = 	furthest_mm;
    result.amplitude = 	furthest_amp;

    return result;
}

void obstacle_avg_update(uint8_t sensor, uint8_t meas, obstacle_result_t result)
{
    uint8_t idx = avg_idx[sensor][meas];
    avg_range[sensor][meas][idx] = 	result.range_mm;
    avg_amp[sensor][meas][idx]   = 	result.amplitude;
    avg_idx[sensor][meas] = 		(idx + 1) % SMOOTH_WINDOW;
    if (avg_count[sensor][meas] < SMOOTH_WINDOW) avg_count[sensor][meas]++;
}

void obstacle_avg_get(uint8_t sensor, uint8_t meas, float *out_mm, uint16_t *out_amp)
{
    uint8_t count = avg_count[sensor][meas];
    if (count == 0) {
        *out_mm  = 0.0f;
        *out_amp = 0;
        return;
    }
    float sum_r = 0;
    uint32_t sum_amp = 0;
    for (int i = 0; i < count; i++) {
        sum_r += avg_range[sensor][meas][i];
        sum_amp += avg_amp[sensor][meas][i];
    }
    *out_mm  = sum_r / count;
    *out_amp = (uint16_t)(sum_amp / count);
}


