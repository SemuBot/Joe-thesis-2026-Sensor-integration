#ifndef ULTRASONIC_DATA_H
#define ULTRASONIC_DATA_H

#include <stdio.h>
#include <stdint.h>
#define SPEED_OF_SOUND_CM_PER_US 0.0343f

// Ultrasonic sensors connected pins definitions

#define TRIG_PIN_1 2
#define ECHO_PIN_1 3
#define TRIG_PIN_2 6
#define ECHO_PIN_2 7
#define TRIG_PIN_3 10
#define ECHO_PIN_3 11

// Ultrasonic sensors distance calculation functions 

int32_t ultrasonic_pulse_us(uint8_t echo_pin, uint32_t timeout_us);
int32_t read_ultrasonic_cm(uint8_t trig_pin, uint8_t echo_pin);

#endif // ULTRASONIC_DATA_H