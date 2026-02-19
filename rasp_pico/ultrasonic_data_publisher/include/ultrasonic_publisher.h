#ifndef ULTRASONIC_PUBLISHER_H
#define ULTRASONIC_PUBLISHER_H

#include <stdio.h>
#include <stdint.h>

#define NUMBER_OF_SENSORS 3
#define WINDOW_SIZE 5
extern rcl_publisher_t range_publishers[NUMBER_OF_SENSORS];
extern sensor_msgs__msg__Range range_msgs[NUMBER_OF_SENSORS];

void callback_function(rcl_timer_t *timer, int64_t last_call_time);
void create_range_publishers(rcl_node_t *node);

#endif