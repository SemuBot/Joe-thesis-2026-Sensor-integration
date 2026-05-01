/*
 * custom_memory_manager.c
 *
 *  Created on: Apr 14, 2026
 *      Author: aleks
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Simple bare metal heap using standard malloc/free
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

size_t xPortGetFreeHeapSizeMicroROS(void) { return 0; }
size_t xPortGetMinimumEverFreeHeapSizeMicroROS(void) { return 0; }
void vPortInitialiseBlocksMicroROS(void) {}



