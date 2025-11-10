#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#define SPEED_OF_SOUND_CM_PER_US 0.0343f

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
const uint trig_pin = 4;
const uint echo_pin = 5;
const uint LED_PIN = 25;

/*
Return pulse length in microseconds
*/
static int32_t ultrasonic_pulse_us(uint echo_pin, uint32_t timeout_us)
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

static int32_t read_ultrasonic_cm(void){
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

void callback_function(rcl_timer_t *timer, int64_t last_call_time){
    
    int32_t distance_cm = read_ultrasonic_cm();
    msg.data = distance_cm;

    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);

    // Toggle led pin
    gpio_xor_mask(1 << LED_PIN);
    
}



int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    
    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);

    rcl_node_t node;
    rclc_support_t support;
    rclc_executor_t executor;

    rcl_timer_t timer;
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rcl_allocator_t allocator;
    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node,"pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), 
        "ultrasonic_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        callback_function);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    msg.data = 0;

}
