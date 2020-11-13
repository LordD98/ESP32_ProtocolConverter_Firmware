
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freeRTOS/semphr.h"

//SemaphoreHandle_t printSemaphore;

#define APP_CORE 0			// 0
#define PROTOCOL_CORE 0		// 1


extern TaskHandle_t PRO_TASK;
extern TaskHandle_t APP_TASK;
extern TaskHandle_t APP_MT_TASK;