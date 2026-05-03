
#include "hcsr04.h"
#include "measurement.h"



static float 	avg_range[HCSR04_NUM_SENSORS][SMOOTH_WINDOW];
static uint8_t  avg_idx[HCSR04_NUM_SENSORS];
static uint8_t  avg_count[HCSR04_NUM_SENSORS];

/**
 * @brief  Add a new sample to the moving average buffer for a sensor
 * @details Uses a circular buffer — oldest sample is overwritten when
 *          the buffer is full. Buffer size is defined by SMOOTH_WINDOW.
 *
 * @param  sensor_index  Index of the sensor (0 to HCSR04_NUM_SENSORS-1)
 * @param  range_mm      New distance sample in millimeters to add to buffer
 */
static void avg_update(uint8_t sensor_index, float range_mm)
{
	uint8_t index = 				avg_idx[sensor_index];
	avg_range[sensor_index][index] = 	range_mm;
	avg_idx[sensor_index] = 		(index + 1) % SMOOTH_WINDOW;
	if (avg_count[sensor_index] < SMOOTH_WINDOW) avg_count[sensor_index]++;
}



/**
 * @brief  Calculate and retrieve the current moving average for a sensor
 * @details Computes the arithmetic mean of all valid samples currently
 *          in the buffer. Returns 0 if no samples have been collected yet.
 *
 * @param  sensor_index  Index of the sensor (0 to HCSR04_NUM_SENSORS-1)
 * @param  out           Output pointer — receives the average distance in mm
 */
static void avg_get(uint8_t sensor_index, float *out)
{
	uint8_t count = avg_count[sensor_index];
	if (count == 0) {
		*out = 0.0f;
		return;
	}

	float sum_range = 0;
	for(uint8_t i = 0; i < count; i++){
		sum_range += avg_range[sensor_index][i];
	}
	*out = sum_range / count;
}



void measurement_update(uint8_t sensor_index, hcsr04_sensor_t *sensor, float *avg_out)
{
    float mm;
    if (sensor->range_mm <= HCSR04_MIN_RANGE_MM)
        mm = HCSR04_MIN_RANGE_MM;
    else if (sensor->range_mm >= HCSR04_MAX_RANGE_MM)
        mm = HCSR04_MAX_RANGE_MM;
    else
        mm = sensor->range_mm;

    avg_update(sensor_index, mm);
    avg_get(sensor_index, avg_out);
}



