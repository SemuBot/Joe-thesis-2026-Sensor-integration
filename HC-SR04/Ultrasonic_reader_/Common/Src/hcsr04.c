#include "hcsr04.h"
#include "tim.h"
#include "gpio.h"



extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;



hcsr04_sensor_t hcsr04_sensors[HCSR04_NUM_SENSORS] = {
    // Sensor 0: TRIG=PC0, ECHO=PA10, TIM2_CH4
    {
        .trig_port = GPIOC, .trig_pin = GPIO_PIN_0,
        .echo_port = GPIOA, .echo_pin = GPIO_PIN_10,
        .htim = &htim2, .channel = TIM_CHANNEL_4,
    },
    // Sensor 1: TRIG=PC2, ECHO=PA8, TIM1_CH1
    {
        .trig_port = GPIOC, .trig_pin = GPIO_PIN_2,
        .echo_port = GPIOA, .echo_pin = GPIO_PIN_8,
        .htim = &htim1, .channel = TIM_CHANNEL_1,
    },
    // Sensor 2: TRIG=PC3, ECHO=PC7, TIM3_CH2
    {
        .trig_port = GPIOC, .trig_pin = GPIO_PIN_3,
        .echo_port = GPIOC, .echo_pin = GPIO_PIN_7,
        .htim = &htim3, .channel = TIM_CHANNEL_2,
    },
};



void hcsr04_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Start input capture interrupts for all channels
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
}



void hcsr04_trigger(uint8_t sensor)
{
    if (sensor >= HCSR04_NUM_SENSORS) return;
    hcsr04_sensor_t *s = &hcsr04_sensors[sensor];

    s->captured  = 0;
    s->measuring = 1;
    s->status    = HCSR04_OK;

    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) * 10;
    while ((DWT->CYCCNT - start) < ticks);
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
}



void hcsr04_trigger_all(void)
{
    for (uint8_t i = 0; i < HCSR04_NUM_SENSORS; i++)
        hcsr04_trigger(i);
}



float hcsr04_get_range_mm(uint8_t sensor)
{
    if (sensor >= HCSR04_NUM_SENSORS) return 0;
    return hcsr04_sensors[sensor].range_mm;
}



hcsr04_status_t hcsr04_get_status(uint8_t sensor)
{
    if (sensor >= HCSR04_NUM_SENSORS) return HCSR04_ERROR;
    return hcsr04_sensors[sensor].status;
}



static void process_capture(hcsr04_sensor_t *s, uint32_t capture_val)
{
    if (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) {
        s->rise_time = capture_val;
        s->measuring = 1;
    } else {
        if (s->measuring == 0) return;

        uint32_t pulse_us;
        if (capture_val >= s->rise_time)
            pulse_us = capture_val - s->rise_time;
        else
            pulse_us = (0xFFFFFFFF - s->rise_time) + capture_val;

        float mm = (pulse_us * 343.0f) / 2000.0f;

        if (mm >= HCSR04_MIN_RANGE_MM && mm <= HCSR04_MAX_RANGE_MM) {
            s->range_mm = mm;
            s->status   = HCSR04_OK;
        } else {
            s->status = HCSR04_ERROR;
        }

        s->captured  = 1;
        s->measuring = 0;
    }
}



void hcsr04_tim1_ic_callback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        process_capture(&hcsr04_sensors[1],
            HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
}



void hcsr04_tim2_ic_callback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        process_capture(&hcsr04_sensors[0],
            HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4));
}



void hcsr04_tim3_ic_callback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        process_capture(&hcsr04_sensors[2],
            HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2));
}


