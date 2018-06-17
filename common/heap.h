#pragma once
//{{{

#ifdef __cplusplus
 extern "C" {
#endif
//}}}
#include <stdint.h>
#include <stdlib.h>

typedef struct HeapRegion {
	uint8_t* pucStartAddress;
	size_t xSizeInBytes;
	} HeapRegion_t;

void* pvPortMalloc (size_t xWantedSize);
void vPortFree (void* pv);

size_t xPortGetFreeHeapSize();
size_t xPortGetMinimumEverFreeHeapSize();

void heapInit (const HeapRegion_t* const pxHeapRegions);

//{{{
#ifdef __cplusplus
}
#endif
//}}}
