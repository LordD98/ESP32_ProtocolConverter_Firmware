
#include "Task_PRO.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "UART_Interface.h"
#include "Deserialize.h"
#include "string.h"

static const char *TAG = "PRO_TASK";

uint32_t lastUpdate = 0;	// in ms since thread start
uint8_t progBuf[2048];		// Buffer that stores program data
uint32_t dataLen;			// length of data in progBuf

void vProTask(void *pvParameters)
{
	esp_log_level_set(TAG, ESP_LOG_INFO);

	Config_UART_Interface();
	
	while (1)
	{
		TaskMessage taskMsg;
		if(xTaskNotifyWait(0, 0, &taskMsg, portMAX_DELAY) != pdPASS)
			continue;
		
		uint32_t timestamp = esp_log_timestamp(); // in ms
		
		switch (taskMsg)
		{
		default:
		case None:
			lastUpdate = esp_log_timestamp();
			break;
		case UART_ReceptionComplete:
			HandleUARTData();
			break;
		}
		
		lastUpdate = timestamp;
	}
}

static void HandleUARTData()
{
	ESP_LOGI(TAG, "Handle UART data");
	vTaskDelay(10 / portTICK_PERIOD_MS);
	
	for (int i = 0; i < dataLen; i++)
	{
		printf("%d (%c)\n", progBuf[i], progBuf[i]);
	}
	
	Deserialize(progBuf, dataLen);
	
}

// CRC-16/IBM-3740 : width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1 residue=0x0000 name="CRC-16/IBM-3740" (ASCII "123456789" => check)
uint16_t CRC16_CCITT_FALSE(uint8_t *buf, uint32_t len)
{
	const ushort poly = 0x1021;
	ushort table[256];
	ushort initialValue = 0xffff;
	ushort temp, a;
	ushort crc = initialValue;
	for (int i = 0; i < 256; ++i)
	{
		temp = 0;
		a = (ushort)(i << 8);
		for (int j = 0; j < 8; ++j)
		{
			if (((temp ^ a) & 0x8000) != 0)
				temp = (ushort)((temp << 1) ^ poly);
			else
				temp <<= 1;
			a <<= 1;
		}
		table[i] = temp;
	}
	for (int i = 0; i < len; ++i)
	{
		crc = (ushort)((crc << 8) ^ table[((crc >> 8) ^ (0xff & buf[i]))]);
	}
	return crc;
}