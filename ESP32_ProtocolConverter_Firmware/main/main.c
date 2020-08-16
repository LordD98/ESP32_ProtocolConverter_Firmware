
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Task_APP.h"
#include "Task_PRO.h"

TaskHandle_t APP_TASK;
TaskHandle_t PRO_TASK;

void app_main()
{
	xTaskCreatePinnedToCore(vAppTask, "CORE0_APP_TASK", 2048, NULL, 12, &APP_TASK, 0);
	xTaskCreatePinnedToCore(vProTask, "CORE1_PRO_TASK", 2048, NULL, 12, &PRO_TASK, 1);
	
	vTaskDelete(NULL);
}