/*
 * microros_allocators.c
 *
 *  Created on: Apr 14, 2026
 *      Author: aleks
 */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

int absoluteUsedMemory = 0;
int usedMemory = 0;

void *pvPortMallocMicroROS(size_t xWantedSize)
{
    return malloc(xWantedSize);
}

void vPortFreeMicroROS(void *pv)
{
    free(pv);
}

void *pvPortReallocMicroROS(void *pv, size_t xWantedSize)
{
    return realloc(pv, xWantedSize);
}

void *pvPortCallocMicroROS(size_t num, size_t xWantedSize)
{
    return calloc(num, xWantedSize);
}

size_t getBlockSize(void *pv)
{
    // Not available with standard malloc — return 0
    return 0;
}

void * microros_allocate(size_t size, void * state)
{
    (void) state;
    absoluteUsedMemory += size;
    usedMemory += size;
    return pvPortMallocMicroROS(size);
}

void microros_deallocate(void * pointer, void * state)
{
    (void) state;
    if (NULL != pointer) {
        vPortFreeMicroROS(pointer);
    }
}

void * microros_reallocate(void * pointer, size_t size, void * state)
{
    (void) state;
    absoluteUsedMemory += size;
    usedMemory += size;
    if (NULL == pointer) {
        return pvPortMallocMicroROS(size);
    } else {
        return pvPortReallocMicroROS(pointer, size);
    }
}

void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
{
    (void) state;
    absoluteUsedMemory += number_of_elements * size_of_element;
    usedMemory += number_of_elements * size_of_element;
    return pvPortCallocMicroROS(number_of_elements, size_of_element);
}



