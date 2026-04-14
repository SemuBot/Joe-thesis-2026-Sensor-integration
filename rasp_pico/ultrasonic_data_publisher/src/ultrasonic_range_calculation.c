#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "ultrasonic_range_calculation.h"
#include "pico/stdlib.h"

/*
Return pulse length in microseconds
*/
int32_t ultrasonic_pulse_us(uint8_t echo_pin, uint32_t timeout_us)
{
    uint32_t t0 = time_us_32();

    // wait for rising edge
    while (!gpio_get(echo_pin)) {
        if ((time_us_32() - t0) > timeout_us) return -1;
    }
    uint32_t start = time_us_32();

    // wait for falling edge
    while (gpio_get(echo_pin)) {
        if ((time_us_32() - start) > timeout_us) return -1;
    }
    return (int32_t)(time_us_32() - start);
}

int32_t read_ultrasonic_cm(uint8_t trig_pin, uint8_t echo_pin){
    gpio_put(trig_pin, 0);
    sleep_us(2);
    gpio_put(trig_pin, 1);
    sleep_us(10);
    gpio_put(trig_pin, 0);

    int32_t pulse_us = ultrasonic_pulse_us(echo_pin, 30000);
    if (pulse_us < 0) return -1;

    // distance (cm) = (echo_us * 0.0343) / 2  (divide by 2 for round trip)
    float distance_cm = (pulse_us * SPEED_OF_SOUND_CM_PER_US) * 0.5f;
    if (distance_cm < 0) return -1;

    return (int32_t)(distance_cm + 0.5f);
    
}