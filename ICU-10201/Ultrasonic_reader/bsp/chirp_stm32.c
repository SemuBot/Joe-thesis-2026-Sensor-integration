#include <invn/soniclib/chirp_bsp.h>
#include <spi.h>
#include "stm32f3xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

static volatile uint32_t chirp_event_flags = 0;

static void int1_dir_in(ch_dev_t *dev_ptr) {
    GPIO_InitTypeDef g = {0};
    g.Pull = GPIO_PULLUP;
    g.Mode = GPIO_MODE_IT_FALLING;
    if (dev_ptr->io_index == 0) {
        g.Pin = GPIO_PIN_11;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (dev_ptr->io_index == 1) {
        g.Pin = GPIO_PIN_6;
        HAL_GPIO_Init(GPIOA, &g);
    } else {
        g.Pin = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOB, &g);
    }
}

static void int1_dir_out(ch_dev_t *dev_ptr) {
    GPIO_InitTypeDef g = {0};
    g.Pull  = GPIO_PULLUP;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    if (dev_ptr->io_index == 0) {
        g.Pin = GPIO_PIN_11;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (dev_ptr->io_index == 1) {
        g.Pin = GPIO_PIN_6;
        HAL_GPIO_Init(GPIOA, &g);
    } else {
        g.Pin = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOB, &g);
    }
}

void chbsp_set_int1_dir_in(ch_dev_t *dev_ptr)      { int1_dir_in(dev_ptr); }
void chbsp_set_int1_dir_out(ch_dev_t *dev_ptr)     { int1_dir_out(dev_ptr); }

void chbsp_group_set_int1_dir_in(ch_group_t *grp) {
    for (uint8_t i = 0; i < ch_get_num_ports(grp); i++)
        int1_dir_in(ch_get_dev_ptr(grp, i));
}

void chbsp_group_set_int1_dir_out(ch_group_t *grp) {
    for (uint8_t i = 0; i < ch_get_num_ports(grp); i++)
        int1_dir_out(ch_get_dev_ptr(grp, i));
}

void chbsp_int1_clear(ch_dev_t *dev_ptr) {
    if (dev_ptr->io_index == 0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    else if (dev_ptr->io_index == 1)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

void chbsp_group_int1_clear(ch_group_t *grp) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_RESET);
}

void chbsp_int1_set(ch_dev_t *dev_ptr) {
    if (dev_ptr->io_index == 0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    else if (dev_ptr->io_index == 1)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void chbsp_group_int1_set(ch_group_t *grp) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_SET);
}

void chbsp_int1_interrupt_enable(ch_dev_t *dev_ptr) {
    if (dev_ptr->io_index == 0)
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    else if (dev_ptr->io_index == 1)
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    else
        HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
}

void chbsp_group_int1_interrupt_enable(ch_group_t *grp) {
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
}

void chbsp_int1_interrupt_disable(ch_dev_t *dev_ptr) {
    if (dev_ptr->io_index == 0)
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    else if (dev_ptr->io_index == 1)
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    else
        HAL_NVIC_DisableIRQ(EXTI2_TSC_IRQn);
}

void chbsp_group_int1_interrupt_disable(ch_group_t *grp) {
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_DisableIRQ(EXTI2_TSC_IRQn);
}

void chbsp_delay_ms(uint32_t ms) { HAL_Delay(ms); }
uint32_t chbsp_timestamp_ms(void) { return HAL_GetTick(); }

void chbsp_event_notify(uint32_t event_mask) {
    chirp_event_flags |= event_mask;
}

uint8_t chbsp_event_wait(uint16_t time_out_ms, uint32_t event_mask) {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < time_out_ms) {
        if (chirp_event_flags & event_mask) {
            chirp_event_flags &= ~event_mask;
            return 0;
        }
        __WFI();
    }
    return 1;
}

void chbsp_event_wait_setup(uint32_t event_mask)
{
    chirp_event_flags &= ~event_mask;
}

void chbsp_spi_cs_on(ch_dev_t *dev_ptr)
{
	if (dev_ptr->io_index == 0)
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);	// CS0 = PA12
	else if (dev_ptr->io_index == 1)
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);	// CS1 = PA7
	else
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);	// CS2 = PB1
}



void chbsp_spi_cs_off(ch_dev_t *dev_ptr)
{
    if (dev_ptr->io_index == 0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); 	// CS 0 - HIGH
    else if (dev_ptr->io_index == 1)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);		// CS 1 - HIGH
    else
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		// CS 2 - HIGH
}

int chbsp_spi_write(ch_dev_t *dev_ptr, const uint8_t *data, uint16_t num_bytes) {
    return (HAL_SPI_Transmit(&hspi2, (uint8_t *)data, num_bytes, 1000) == HAL_OK) ? 0 : 1;
}

int chbsp_spi_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
    return (HAL_SPI_Receive(&hspi2, data, num_bytes, 1000) == HAL_OK) ? 0 : 1;
}

/* simple stubs for unused functions */
void chbsp_debug_toggle(uint8_t dbg_pin_num) {}
void chbsp_debug_on(uint8_t dbg_pin_num) {}
void chbsp_program_disable(ch_dev_t *dev_ptr) {}

void chbsp_group_set_int2_dir_out(ch_group_t *grp_ptr) {}
void chbsp_set_int2_dir_out(ch_dev_t *dev_ptr) {}
void chbsp_group_set_int2_dir_in(ch_group_t *grp_ptr) {}
void chbsp_set_int2_dir_in(ch_dev_t *dev_ptr) {}
void chbsp_group_int2_clear(ch_group_t *grp_ptr) {}
void chbsp_int2_clear(ch_dev_t *dev_ptr) {}
void chbsp_group_int2_set(ch_group_t *grp_ptr) {}
void chbsp_int2_set(ch_dev_t *dev_ptr) {}
void chbsp_group_int2_interrupt_enable(ch_group_t *grp_ptr) {}
void chbsp_int2_interrupt_enable(ch_dev_t *dev_ptr) {}
void chbsp_group_int2_interrupt_disable(ch_group_t *grp_ptr) {}
void chbsp_int2_interrupt_disable(ch_dev_t *dev_ptr) {}

void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void chbsp_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;

    while ((DWT->CYCCNT - start) < ticks) {
    }
}

void chbsp_pulse_len_hint_us(uint32_t us)
{
    chbsp_delay_us(us);
}

int chbsp_i2c_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) { return 1; }
void chbsp_i2c_reset(ch_dev_t *dev_ptr) {}
int chbsp_spi_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) { return 1; }
void chbsp_print_str(const char *str) {}


