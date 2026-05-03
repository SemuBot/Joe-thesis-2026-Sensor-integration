/**
 * @file    measurement.h
 * @brief   Moving average filter for HC-SR04 distance measurements
 * @details Implements a circular buffer based moving average per sensor
 *          to smooth noisy ultrasonic distance readings. Each sensor
 *          maintains an independent averaging window.
 *
 * @author  Aleks Jõe
 * @date    May 2026
 * @version 1.0
 */

#ifndef INC_MEASUREMENT_H_
#define INC_MEASUREMENT_H_

#include "hcsr04.h"

/**
 * @brief Number of samples in the moving average window.
 * @note  Higher values produce smoother output but increase latency.
 *        At 11Hz update rate, SMOOTH_WINDOW=5 introduces ~450ms of lag.
 */
#define SMOOTH_WINDOW	5

/**
 * @brief  Update moving average and retrieve current averaged distance
 * @details Clamps the measured distance to [HCSR04_MIN_RANGE_MM, HCSR04_MAX_RANGE_MM]
 *          before adding to the averaging buffer. This prevents out-of-range
 *          readings from corrupting the average.
 *
 * @param  sensor_index  Index of the sensor (0 to HCSR04_NUM_SENSORS-1)
 * @param  sensor        Pointer to the sensor struct containing the latest measurement
 * @param  avg_out       Output pointer — receives the current moving average in mm
 */
void measurement_update(uint8_t sensor_index, hcsr04_sensor_t *sensor, float *avg_out);



#endif /* INC_MEASUREMENT_H_ */
