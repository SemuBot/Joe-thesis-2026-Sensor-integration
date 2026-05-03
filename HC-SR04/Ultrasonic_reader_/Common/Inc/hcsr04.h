/**
 * @file    hcsr04.h
 * @brief   HC-SR04 ultrasonic sensor driver using timer input capture
 * @details Supports 3 sensors using TIM1 (CH1), TIM2 (CH4) and TIM3 (CH2)
 *          for accurate pulse width measurement via hardware input capture.
 *          Distance is calculated from echo pulse duration using speed of sound.
 *
 * @author  Aleks Jõe
 * @date    May 2026
 * @version 1.0
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#include "stm32f3xx_hal.h"
#include <stdint.h>

#define HCSR04_NUM_SENSORS	3
#define HCSR04_TIMEOUT_US	30000
#define HCSR04_MAX_RANGE_MM	4000
#define HCSR04_MIN_RANGE_MM	20



/**
 * @brief Sensor measurement result configuration for HC-SR04
 */
typedef enum {
    HCSR04_OK      = 0,
    HCSR04_TIMEOUT = 1,
    HCSR04_ERROR   = 2,
} hcsr04_status_t;

/**
 * @brief Per-sensor state and configuration for HC-SR04
 */
typedef struct {
    GPIO_TypeDef 		*trig_port;
    uint16_t      		trig_pin;
    GPIO_TypeDef 		*echo_port;
    uint16_t      		echo_pin;
    TIM_HandleTypeDef 	*htim;
    uint32_t           	channel;
    volatile uint32_t  	rise_time;
    volatile uint32_t  	fall_time;
    volatile uint8_t   	captured;
    volatile uint8_t   	measuring;
    float              	range_mm;
    hcsr04_status_t    	status;
} hcsr04_sensor_t;


extern hcsr04_sensor_t hcsr04_sensors[HCSR04_NUM_SENSORS];


void hcsr04_init(void);

/**
 * @brief   Trigger a single HC-SR04 sensor measurement
 * @details Sends a 10us pulse on the TRIG pin to initiate an ultrasonic burst.
 *          The sensor will respond with a HIGH pulse on ECHO whose duration
 *          is proportional to the distance of the nearest obstacle.
 *
 * @param   sensor  Index of the sensor to trigger (0 to HCSR04_NUM_SENSORS-1)
 * @note    Minimum 60ms must elapse between triggers on the same sensor
 *          to avoid echo interference from previous measurements.
 */
void hcsr04_trigger(uint8_t sensor);

/**
 * @brief Trigger all sensors simultaneously
 */
void hcsr04_trigger_all(void);

/**
 * @brief   Get the last measured range for a sensor
 * @param   sensor  Sensor index (0 to HCSR04_NUM_SENSORS-1)
 * @return  Distance in millimeters, or 0 if no valid measurement
 */
float hcsr04_get_range_mm(uint8_t sensor);

/**
 * @brief   Get the status of the last measurement
 * @param   sensor  Sensor index (0 to HCSR04_NUM_SENSORS-1)
 * @return  HCSR04_OK, HCSR04_TIMEOUT or HCSR04_ERROR
 */
hcsr04_status_t hcsr04_get_status(uint8_t sensor);

/** @defgroup HCSR04_Callbacks Timer Input Capture Callbacks
 *  @brief Internal callbacks invoked by HAL timer interrupt handlers.
 *         These should not be called directly by application code.
 *  @{
 */
void hcsr04_tim1_ic_callback(TIM_HandleTypeDef *htim);
void hcsr04_tim2_ic_callback(TIM_HandleTypeDef *htim);
void hcsr04_tim3_ic_callback(TIM_HandleTypeDef *htim);

/** @} */


#endif /* INC_HCSR04_H_ */
