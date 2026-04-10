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

static float   baseline_mm[MAX_BASELINE_TARGETS];
static uint8_t baseline_count = 0;

#define SMOOTH_WINDOW 20

static float    smooth_range[SMOOTH_WINDOW] = {0};
static uint16_t smooth_amp[SMOOTH_WINDOW]   = {0};
static uint8_t  smooth_idx = 0;
static uint8_t  smooth_count = 0;
static uint32_t tof = 0;

static void uart_print(const char *msg)
{
    extern UART_HandleTypeDef huart2;
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}

void obstacle_calibrate(ch_dev_t *dev_ptr, volatile uint8_t *data_ready_flag)
{
    char buf[128];
    float accum_mm[MAX_BASELINE_TARGETS] = {0};
    uint8_t accum_n[MAX_BASELINE_TARGETS] = {0};
    baseline_count = 0;

    uart_print("Calibrating... keep area clear\r\n");

    for (int frame = 0; frame < BASELINE_FRAMES; frame++) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
        HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

        *data_ready_flag = 0;
        uint32_t start = HAL_GetTick();
        while (!(*data_ready_flag) && ((HAL_GetTick() - start) < 1000)) __WFI();

        if (!(*data_ready_flag)) {
            sprintf(buf, "cal frame %d timeout\r\n", frame);
            uart_print(buf);
            continue;
        }

        uint8_t n = icu_gpt_algo_get_num_targets(dev_ptr);
        sprintf(buf, "cal frame %d targets=%u\r\n", frame, n);
        uart_print(buf);

        for (uint8_t i = 0; i < n && i < MAX_BASELINE_TARGETS; i++) {
            float r = icu_gpt_algo_get_target_range(dev_ptr, i, CH_RANGE_ECHO_ONE_WAY) / 32.0f;
            uint16_t a = icu_gpt_algo_get_target_amplitude(dev_ptr, i);
            sprintf(buf, "  cal t%u mm=%.0f amp=%u\r\n", i, r, a);
            uart_print(buf);
            accum_mm[i] += r;
            accum_n[i]++;
        }
    }

    for (int i = 0; i < MAX_BASELINE_TARGETS; i++) {
        if (accum_n[i] >= BASELINE_FRAMES / 2) {
            baseline_mm[baseline_count] = accum_mm[i] / accum_n[i];
            sprintf(buf, "Baseline[%d] mm=%.0f\r\n", baseline_count, baseline_mm[baseline_count]);
            uart_print(buf);
            baseline_count++;
        }
    }

    uart_print("Calibration done\r\n");
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

obstacle_status_t obstacle_check(ch_dev_t *dev_ptr, float *closest_mm_out, uint16_t *closest_amp_out)
{
    uint8_t n = icu_gpt_algo_get_num_targets(dev_ptr);
    obstacle_status_t status = OBSTACLE_CLEAR;
    float furthest = 0.0f;
    uint16_t amp = 0;

    for (uint8_t i = 0; i < n; i++) {
        float r = icu_gpt_algo_get_target_range(dev_ptr, i, CH_RANGE_ECHO_ONE_WAY) / 32.0f;
        uint16_t a = icu_gpt_algo_get_target_amplitude(dev_ptr, i);

        if (a > MAX_VALID_AMP) continue;

        uint8_t is_background = 0;
        for (int b = 0; b < baseline_count; b++) {
            float diff = r - baseline_mm[b];
            if (diff < 0) diff = -diff;
            if (diff < BASELINE_TOL_MM) {
                is_background = 1;
                break;
            }
        }

        if (!is_background) {
            if (r > furthest) {
                furthest = r;
                amp = a;
            }
            status = OBSTACLE_DETECTED;
        }
    }

    if (closest_mm_out != NULL) *closest_mm_out = furthest;
    if (closest_amp_out != NULL) *closest_amp_out = amp;

    return status;
}

void obstacle_smooth_update(float range_mm, uint16_t amp)
{
    smooth_range[smooth_idx] = range_mm;
    smooth_amp[smooth_idx]   = amp;
    smooth_idx = (smooth_idx + 1) % SMOOTH_WINDOW;
    if (smooth_count < SMOOTH_WINDOW) smooth_count++;
}

void obstacle_smooth_get(float *avg_range, uint16_t *avg_amp)
{
    float sum_r = 0;
    uint32_t sum_a = 0;
    for (int i = 0; i < smooth_count; i++) {
        sum_r += smooth_range[i];
        sum_a += smooth_amp[i];
    }
    *avg_range = sum_r / smooth_count;
    *avg_amp   = (uint16_t)(sum_a / smooth_count);
}

void obstacle_smooth_reset(void)
{
    smooth_idx   = 0;
    smooth_count = 0;
}

uint8_t obstacle_smooth_get_count(void)
{
    return smooth_count;
}


