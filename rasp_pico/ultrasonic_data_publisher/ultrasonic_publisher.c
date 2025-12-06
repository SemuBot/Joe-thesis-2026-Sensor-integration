#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
//#include "pico/cyw43_arch.h"
#include "ultrasonic_data.h"

#define NUMBER_OF_SENSORS 3

rcl_publisher_t publisher;


std_msgs__msg__Int32MultiArray msg;

std_msgs__msg__Int32__Sequence msg_sequence;
const uint LED_PIN = 0;


void callback_function(rcl_timer_t *timer, int64_t last_call_time){

    msg.data.data[0] = read_ultrasonic_cm(TRIG_PIN_1, ECHO_PIN_1);
    msg.data.data[1] = read_ultrasonic_cm(TRIG_PIN_2, ECHO_PIN_2);
    msg.data.data[2] = read_ultrasonic_cm(TRIG_PIN_3, ECHO_PIN_3);

    rcl_publish(&publisher, &msg, NULL);
    /*
    if(distance_cm < 10){
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    } else {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    }*/
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
    
    //cyw43_arch_init();
    uint8_t trig_pins[] = {TRIG_PIN_1, TRIG_PIN_2, TRIG_PIN_3};
    uint8_t echo_pins[] = {ECHO_PIN_1, ECHO_PIN_2, ECHO_PIN_3};

    for(int i = 0; i < sizeof(trig_pins); i++){
        gpio_init(trig_pins[i]);
        gpio_set_dir(trig_pins[i], GPIO_OUT);

        gpio_init(echo_pins[i]);
        gpio_set_dir(echo_pins[i], GPIO_IN);
    }

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

    rclc_node_init_default(&node,"ultrasonic_range_publisher", "", &support);
    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), 
        "cmd_range");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        callback_function);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    
    std_msgs__msg__Int32MultiArray__init(&msg);
    msg.data.data = malloc(sizeof(int32_t) * NUMBER_OF_SENSORS);
    msg.data.size = NUMBER_OF_SENSORS;
    msg.data.capacity = NUMBER_OF_SENSORS;

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    std_msgs__msg__Int32MultiArray__fini(&msg);
    return 0;

}
