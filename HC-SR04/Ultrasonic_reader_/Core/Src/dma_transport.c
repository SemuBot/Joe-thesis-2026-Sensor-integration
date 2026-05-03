/*
 * dma_transport.c
 *
 *  Created on: Apr 14, 2026
 *      Author: aleks
 */
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include "main.h"
#include "stm32f3xx_hal.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0;

bool cubemx_transport_open(struct uxrCustomTransport * transport)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_StatusTypeDef ret = HAL_UART_Transmit(uart, (uint8_t*)buf, len, 1000);
    return (ret == HAL_OK) ? len : 0;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    uint32_t start = HAL_GetTick();
    size_t dma_tail;

    do {
        __disable_irq();
        dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
        __enable_irq();

        if (dma_head != dma_tail) break;
        HAL_Delay(1);
    } while ((int)(HAL_GetTick() - start) < timeout);

    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)) {
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }

    return wrote;
}

#endif // RMW_UXRCE_TRANSPORT_CUSTOM



