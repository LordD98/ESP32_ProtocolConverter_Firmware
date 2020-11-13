
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Task_APP.h"
#include "Task_PRO.h"

TaskHandle_t PRO_TASK;
TaskHandle_t APP_TASK;

void app_main()
{
	//printSemaphore = xSemaphoreCreateMutex();
	
	xTaskCreatePinnedToCore(vProTask, "CORE1_PRO_TASK", 8192, NULL, 16, &PRO_TASK, PROTOCOL_CORE);
	xTaskCreatePinnedToCore(vAppTask, "CORE0_APP_TASK", 8192, NULL, 12, &APP_TASK, APP_CORE);
	
	vTaskDelay(100 / portTICK_PERIOD_MS);
	vTaskDelete(NULL);
}