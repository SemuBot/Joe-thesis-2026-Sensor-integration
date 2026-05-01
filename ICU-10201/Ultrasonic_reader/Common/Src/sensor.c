/*
 * sensor.c
 *
 *  Created on: Apr 5, 2026
 *      Author: aleks
 */

#include "sensor.h"
#include "main.h"
#include "usart.h"


ch_group_t chirp_group;
ch_dev_t chirp_devices[NUMBER_OF_SENSORS];

volatile uint8_t data_ready = 0;
volatile uint32_t exti2_count = 0;
extern rcl_publisher_t pub_debug;
extern std_msgs__msg__String debug_msg;

ch_thresholds_t my_thresholds_0;
ch_thresholds_t my_thresholds_1;

char buf[256];

void sensor_group_init(void)
{

	sprintf(buf, "\r\nSonicLib start\r\n");
	uart_print(&pub_debug, &debug_msg, buf);
    uint8_t ret;

    dwt_delay_init();



    ret = ch_group_init(&chirp_group, NUMBER_OF_SENSORS, 1, CHIRP_RTC_CAL_PULSE_MS);
    if (ret != 0) {
        sprintf(buf, "ch_group_init failed: %u\r\n", ret);
        uart_print(&pub_debug, &debug_msg, buf);

    }

	my_thresholds_0.threshold[0].start_sample = 0;
	my_thresholds_0.threshold[0].level        = 40000;
	for (int i = 0; i < 8; i++) {
		my_thresholds_0.threshold[i].start_sample = measurement_config_cfg.meas_cfg[0].thresholds.stop_index[i];
		my_thresholds_0.threshold[i].level        = measurement_config_cfg.meas_cfg[0].thresholds.threshold[i];
	}

	my_thresholds_1.threshold[0].start_sample = 0;
	my_thresholds_1.threshold[0].level        = 40000;
	for (int i = 0; i < 8; i++) {
		my_thresholds_1.threshold[i].start_sample = measurement_config_cfg.meas_cfg[1].thresholds.stop_index[i];
		my_thresholds_1.threshold[i].level        = measurement_config_cfg.meas_cfg[1].thresholds.threshold[i];
	}

	for (uint8_t sensor = 0; sensor < NUMBER_OF_SENSORS; sensor++){
		ch_dev_t *dev_ptr = &chirp_devices[sensor];
		ret = ch_init(dev_ptr, &chirp_group, sensor, icu_gpt_init);
		sprintf(buf, "Sensor %u ch_init ret=%u\r\n", sensor, ret); uart_print(&pub_debug, &debug_msg, buf);
		HAL_Delay(50);
	}

    ch_io_int_callback_set(&chirp_group, io_int_callback);

    sprintf(buf, "Starting group...\r\n");
    uart_print(&pub_debug, &debug_msg, buf);

    HAL_Delay(100);

    exti2_count = 0;
    ret = ch_group_start(&chirp_group);
    sprintf(buf, "ch_group_start ret=%u\r\n", ret); uart_print(&pub_debug, &debug_msg, buf);

    sprintf(buf, "exti2_count=%lu\r\n", exti2_count);
    uart_print(&pub_debug, &debug_msg, buf);

	sprintf(buf, "Group init finished...\r\n");
	uart_print(&pub_debug, &debug_msg, buf);
}

void sensor_init(ch_dev_t *dev, uint8_t io_index)
{
    uint8_t ret;

	uint32_t pmut_freq = ch_get_frequency(dev);
	sprintf(buf, "PMUT frequency=%lu Hz\r\n", pmut_freq);
	uart_print(&pub_debug, &debug_msg, buf);

	uint32_t rtc_cal = ch_get_rtc_cal_result(dev);
	sprintf(buf, "RTC cal result=%lu\r\n", rtc_cal);
	uart_print(&pub_debug, &debug_msg, buf);

	if (ch_sensor_is_connected(dev)) {
		sprintf(buf, "Sensor connected! Part#=%u fw=%s\r\n",
				ch_get_part_number(dev),
				ch_get_fw_version_string(dev));
	} else {
		sprintf(buf, "ERROR: sensor NOT connected\r\n");
	}
	uart_print(&pub_debug, &debug_msg, buf);

	ret = icu_gpt_algo_init(dev, &measurement_config_cfg);
	sprintf(buf, "icu_gpt_algo_init ret=%u\r\n", ret);
	uart_print(&pub_debug, &debug_msg, buf);
	if (ret != 0) Error_Handler();

	ret = ch_meas_import(dev, &measurement_config_queue, &measurement_config_cfg);
	sprintf(buf, "ch_meas_import ret=%u\r\n", ret);
	uart_print(&pub_debug, &debug_msg, buf);
	if (ret != 0) Error_Handler();

	ret = ch_init_algo(dev);
	sprintf(buf, "ch_init_algo ret=%u\r\n", ret);
	uart_print(&pub_debug, &debug_msg, buf);
	if (ret != 0) Error_Handler();

	sprintf(buf, "meas0 num_samples=%u\r\n", ch_meas_get_num_samples(dev, 0));
	uart_print(&pub_debug, &debug_msg, buf);
	sprintf(buf, "meas1 num_samples=%u\r\n", ch_meas_get_num_samples(dev, 1));
	uart_print(&pub_debug, &debug_msg, buf);
	sprintf(buf, "meas0 odr=%u\r\n", ch_meas_get_odr(dev, 0));
	uart_print(&pub_debug, &debug_msg, buf);
	sprintf(buf, "meas1 odr=%u\r\n", ch_meas_get_odr(dev, 1));
	uart_print(&pub_debug, &debug_msg, buf);

	ret = icu_gpt_set_thresholds(dev, 0, &my_thresholds_0);
	sprintf(buf, "set thresholds meas0 ret=%u\r\n", ret);
	uart_print(&pub_debug, &debug_msg, buf);

	ret = icu_gpt_set_thresholds(dev, 1, &my_thresholds_1);
	sprintf(buf, "set thresholds meas1 ret=%u\r\n", ret);
	uart_print(&pub_debug, &debug_msg, buf);

	sprintf(buf, "set ringdown cancel & static filter R0 - %u, R1 - %u, F0 - %u, F1 - %u\r\n",
			icu_gpt_set_ringdown_cancel(dev, 0, 0),
			icu_gpt_set_ringdown_cancel(dev, 1, 0),
			icu_gpt_set_static_filter(dev, 0, 0),
			icu_gpt_set_static_filter(dev, 1, 0));
	uart_print(&pub_debug, &debug_msg, buf);
	/*
	uint16_t holdoff_samples = ch_mm_to_samples(dev, 100);
	sprintf(buf, "mm=100, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);

	holdoff_samples = ch_mm_to_samples(dev, 200);
	sprintf(buf, "mm=200, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);

	holdoff_samples = ch_mm_to_samples(dev, 300);
	sprintf(buf, "mm=300, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);

	holdoff_samples = ch_mm_to_samples(dev, 400);
	sprintf(buf, "mm=400, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);

	holdoff_samples = ch_mm_to_samples(dev, 500);
	sprintf(buf, "mm=500, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);

	holdoff_samples = ch_mm_to_samples(dev, 600);
	sprintf(buf, "mm=600, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);

	holdoff_samples = ch_mm_to_samples(dev, 700);
	sprintf(buf, "mm=700, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);

	holdoff_samples = ch_mm_to_samples(dev, 800);
	sprintf(buf, "mm=800, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);

	holdoff_samples = ch_mm_to_samples(dev, 900);
	sprintf(buf, "mm=900, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);

	holdoff_samples = ch_mm_to_samples(dev, 1000);
	sprintf(buf, "mm=1000, holdoff=%u samples\r\n", holdoff_samples);
	uart_print(&pub_debug, &debug_msg, buf);
	*/

	ch_thresholds_t readback;
	icu_gpt_get_thresholds(dev, 0, &readback);
	for (int i = 0; i < 8; i++) {
		sprintf(buf, "0: thresh[%d] start=%u level=%u\r\n", i,
				readback.threshold[i].start_sample,
				readback.threshold[i].level);
		uart_print(&pub_debug, &debug_msg, buf);
	}

	icu_gpt_get_thresholds(dev, 1, &readback);
	for (int i = 0; i < 8; i++) {
		sprintf(buf, "1: thresh[%d] start=%u level=%u\r\n", i,
				readback.threshold[i].start_sample,
				readback.threshold[i].level);
		uart_print(&pub_debug, &debug_msg, buf);
	}

	uint16_t holdoff = ch_get_rx_holdoff(dev);
	sprintf(buf, "confirmed holdoff=%u samples\r\n", holdoff);
	uart_print(&pub_debug, &debug_msg, buf);

	ret = ch_set_mode(dev, CH_MODE_TRIGGERED_TX_RX);
	sprintf(buf, "ch_set_mode triggered ret=%u\r\n", ret);
	uart_print(&pub_debug, &debug_msg, buf);
	if (ret != 0) Error_Handler();
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_11) {  // sensor 0
        ch_interrupt(&chirp_group, 0);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
        HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
    if (GPIO_Pin == GPIO_PIN_6) {   // sensor 1
        ch_interrupt(&chirp_group, 1);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
        HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
    if (GPIO_Pin == GPIO_PIN_2) {   // sensor 2
    	exti2_count++;
        ch_interrupt(&chirp_group, 2);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
        HAL_NVIC_ClearPendingIRQ(EXTI2_TSC_IRQn);
        HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
    }
}

void io_int_callback(ch_group_t *grp_ptr, uint8_t io_index, ch_interrupt_type_t int_type)
{
    (void)grp_ptr;
    if (int_type == CH_INTERRUPT_TYPE_DATA_RDY) {
        data_ready |= (1 << io_index);  // bit 0 = sensor 0, bit 1 = sensor 1, bit 2 = sensor 2
    }
}

void reset_interrupt(void){
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
    HAL_NVIC_ClearPendingIRQ(EXTI2_TSC_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
}

