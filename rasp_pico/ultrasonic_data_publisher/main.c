#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <stdint.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "ultrasonic_range_calculation.h"
#include "ultrasonic_publisher.h"


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
    
    uint8_t trig_pins[] = {TRIG_PIN_1, TRIG_PIN_2, TRIG_PIN_3};
    uint8_t echo_pins[] = {ECHO_PIN_1, ECHO_PIN_2, ECHO_PIN_3};

    for(int i = 0; i < NUMBER_OF_SENSORS; i++){
        gpio_init(trig_pins[i]);
        gpio_set_dir(trig_pins[i], GPIO_OUT);

        gpio_init(echo_pins[i]);
        gpio_set_dir(echo_pins[i], GPIO_IN);
    }

    rcl_node_t node;
    rclc_support_t support;
    rclc_executor_t executor;

    rcl_timer_t timer;
    const int timeout_ms = 5000;
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

    rclc_node_init_default(&node,"ultrasonic_range_publisher", "", &support);

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(200),
        callback_function);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    
    create_range_publishers(&node);
    
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    
    return 0;

}