#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>

#include "pico/stdlib.h"
#include "ultrasonic_range_calculation.h"
#include "ultrasonic_publisher.h"

rcl_publisher_t range_publishers[NUMBER_OF_SENSORS];
sensor_msgs__msg__Range range_msgs[NUMBER_OF_SENSORS];

static float distance_buffers[NUMBER_OF_SENSORS][WINDOW_SIZE];
static float distance_sums[NUMBER_OF_SENSORS];
static uint8_t buffer_index = 0;
static uint8_t buffer_count = 0;

float distance_m;
int32_t distance_cm;
float moving_average;

static uint8_t trig_pins[NUMBER_OF_SENSORS] = {
    TRIG_PIN_1,
    TRIG_PIN_2,
    TRIG_PIN_3
};

uint8_t echo_pins[NUMBER_OF_SENSORS] = {
    ECHO_PIN_1,
    ECHO_PIN_2,
    ECHO_PIN_3
};

void callback_function(rcl_timer_t *timer, int64_t last_call_time){

    int distances_cm[NUMBER_OF_SENSORS][5] = {
        read_ultrasonic_cm(TRIG_PIN_1, ECHO_PIN_1),
        read_ultrasonic_cm(TRIG_PIN_2, ECHO_PIN_2),
        read_ultrasonic_cm(TRIG_PIN_3, ECHO_PIN_3)
    };

    for (int i = 0; i < NUMBER_OF_SENSORS; i++){
        distance_cm = read_ultrasonic_cm(trig_pins[i], echo_pins[i]);

        if(distance_cm > 0){
            distance_m = distance_cm / 100.0f;
        } else {
            distance_m = range_msgs[i].max_range;
        }

        distance_sums[i] -= distance_buffers[i][buffer_index];
        distance_buffers[i][buffer_index] = distance_m;
        distance_sums[i] += distance_m;

        if (buffer_count < WINDOW_SIZE){
            moving_average = distance_sums[i] / (buffer_count + 1);
        } else {
            moving_average = distance_sums[i] / WINDOW_SIZE;
        }

        range_msgs[i].range = moving_average;
        range_msgs[i].header.stamp.nanosec = rmw_uros_epoch_nanos();

        rcl_publish(&range_publishers[i], &range_msgs[i], NULL);
    }
    buffer_index = (buffer_index + 1) % WINDOW_SIZE;

    if (buffer_count < WINDOW_SIZE){
        buffer_count++;
    }
}

void create_range_publishers(rcl_node_t *node){
    const char * range_topics[NUMBER_OF_SENSORS] = {
        "ultrasonic/front_left",
        "ultrasonic/front_center",
        "ultrasonic/front_right"
    };

    for(int i = 0; i < NUMBER_OF_SENSORS; i++) {
        rclc_publisher_init_default(
            &range_publishers[i],
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
            range_topics[i]
        );
    }

    const char * frame_ids[NUMBER_OF_SENSORS] = {
        "ultrasonic_front_left",
        "ultrasonic_front_center",
        "ultrasonic_front_right"
    };

    for (int i = 0; i < NUMBER_OF_SENSORS; i++){
        sensor_msgs__msg__Range__init(&range_msgs[i]);

        range_msgs[i].radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
        range_msgs[i].field_of_view = 0.26f;     // 15 degrees
        range_msgs[i].min_range = 0.02f;         // 2 cm
        range_msgs[i].max_range = 4.00f;         // 400 cm

        range_msgs[i].header.frame_id.data = (char *)frame_ids[i];
        range_msgs[i].header.frame_id.size = strlen(frame_ids[i]);
        range_msgs[i].header.frame_id.capacity = strlen(frame_ids[i]) + 1;

        range_msgs[i].header.stamp.sec = 0;
        range_msgs[i].header.stamp.nanosec = rmw_uros_epoch_nanos();

        for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
            for (int j = 0; j < WINDOW_SIZE; j++){
                distance_buffers[i][j] = 0.0f;
            }
            distance_sums[i] = 0.0f;
        }
    }

}


