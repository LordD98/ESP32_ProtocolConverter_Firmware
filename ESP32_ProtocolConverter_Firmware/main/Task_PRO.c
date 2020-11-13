#include "main.h"
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
uint8_t allocReqBuf[512];	// Buffer that stores program data
uint32_t dataLen;			// length of data in progBuf
uint32_t allocReqLen;		// length of data in progBuf

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
		
		//ESP_LOGI(TAG, "COMMAND");
		switch (taskMsg)
		{
		default:
		case None:
			lastUpdate = esp_log_timestamp();
			break;
		case UART_StatusRequest:
			HandleUARTStatusRequest();
			break;
		case UART_Query:
			HandleUARTQuery();
			break;
		case UART_AllocRequest:
			HandleUARTAllocRequest();
			break;
		case UART_Data:
			HandleUARTData();
			break;
		}
		
		lastUpdate = timestamp;
	}
}

#include "Task_APP.h"

static void HandleUARTStatusRequest()
{
	uint8_t responseBuffer[13];
	responseBuffer[0] = '+';
	responseBuffer[1] = '+'; 
	responseBuffer[2] = '+';
	responseBuffer[3] = 0; 	// Length = 1
	responseBuffer[4] = 0;  //
	responseBuffer[5] = 0; 	//
	responseBuffer[6] = 1; 	//
	responseBuffer[7] = (uint8_t)connectionState;
	uint32_t crc = CRC16_CCITT_FALSE(responseBuffer + 3, 5);
	responseBuffer[8] = crc >> 8;
	responseBuffer[9] = crc;
	responseBuffer[10] = '+';
	responseBuffer[11] = '+'; 
	responseBuffer[12] = '+';
		
	//vTaskDelay(100 / portTICK_PERIOD_MS);
	
	uart_write_bytes(UART_NUM_0, (const char*)responseBuffer, 13);
	uart_write_bytes(UART_NUM_0, "\r\n", 1);

	if (connectionState == Connected)
		ESP_LOGI(TAG, "Status: Connected");
	else
		ESP_LOGI(TAG, "Status: Disconnected");
}

static void HandleUARTQuery()
{
	//portDISABLE_INTERRUPTS();
	
	ESP_LOGI(TAG, "Handle UART query");
	vTaskDelay(10 / portTICK_PERIOD_MS);
	// response length is 10 + 6*N; N is number of returned pointers
	const uint16_t N = 1;
	uint8_t responseBuffer[16 + 6*N]; // 6 chars extra for '+++'
	void* pointers[] = { ets_delay_us };
	
	responseBuffer[0] = '+';
	responseBuffer[1] = '+'; 
	responseBuffer[2] = '+';

	uint32_t length = 4 + 6*N;
	responseBuffer[3] = length >> 24;
	responseBuffer[4] = length >> 16; 
	responseBuffer[5] = length >> 8;
	responseBuffer[6] = length;

	responseBuffer[7] = 0;											// Version	// TODO: Move to global header: global_defs.h (main.h)
	responseBuffer[8] = 2;											// 0.2

	responseBuffer[9] = N >> 8;										// N Pointers
	responseBuffer[10] = N;											//

	for(int i = 0; i < N; i++)
	{
		responseBuffer[6*i + 11] = i>>8;  							// ID i
		responseBuffer[6*i + 12] = i;  								//
		responseBuffer[6*i + 13] = (uint32_t)pointers[i] >> 24; 	// Address
		responseBuffer[6*i + 14] = (uint32_t)pointers[i] >> 16;  	//
		responseBuffer[6*i + 15] = (uint32_t)pointers[i] >> 8;  	//
		responseBuffer[6*i + 16] = (uint32_t)pointers[i];  			//
	}
	
	uint32_t crc = CRC16_CCITT_FALSE(responseBuffer+3, 6*N + 8);
	responseBuffer[6*N + 11] = crc>>8;
	responseBuffer[6*N + 12] = crc;

	responseBuffer[6*N + 13] = '+';
	responseBuffer[6*N + 14] = '+'; 
	responseBuffer[6*N + 15] = '+';
		
	//vTaskDelay(100 / portTICK_PERIOD_MS);
	
	//xSemaphoreTake(printSemaphore, portMAX_DELAY);
	uart_write_bytes(UART_NUM_0, (const char*)responseBuffer, 16 + 6*N);
	uart_write_bytes(UART_NUM_0, "\r\n", 1);
	//xSemaphoreGive(printSemaphore);
	
	//vTaskDelay(200 / portTICK_PERIOD_MS);
	//xSemaphoreTake(printSemaphore, portMAX_DELAY);
	//for (int i = 0; i < 70; i++)
	//{
	//	printf("%u: 0x%x\n", i, responseBuffer[i]);	
	//}
	//xSemaphoreGive(printSemaphore);

	//portENABLE_INTERRUPTS();
}

static void HandleUARTAllocRequest()
{
	ESP_LOGI(TAG, "Handle UART allocation request");
	vTaskDelay(10 / portTICK_PERIOD_MS);
	//ets_delay_us(10000);
	
	//xSemaphoreTake(printSemaphore, portMAX_DELAY);
	//for (int i = 0; i < allocReqLen; i++)
	//{
	//	printf("%d (0x%x) (%c)\n", allocReqBuf[i], allocReqBuf[i], allocReqBuf[i]);
	//}
	//xSemaphoreGive(printSemaphore);
	
	// Check CRC
	if(CRC16_CCITT_FALSE(allocReqBuf, allocReqLen - 2) != (allocReqBuf[allocReqLen - 2] << 8 | allocReqBuf[allocReqLen - 1]))
	{
		//printf("0x%x, 0x%x, 0x%x\n", CRC16_CCITT_FALSE(allocReqBuf, allocReqLen - 2), allocReqBuf[allocReqLen - 2], allocReqBuf[allocReqLen - 1]);
		ESP_LOGE(TAG, "CRC of received allocation request is incorrect");
		return;
	}
	
	free(functionResourceReferenceAssociations);
	free(userFunctions);
	heap_caps_free(userData);
	
	uint32_t dataLength = allocReqBuf[0] << 24 | allocReqBuf[1] << 16 | allocReqBuf[2] << 8 | allocReqBuf[3];
	
	uint16_t numFunctions = allocReqBuf[4] << 8 | allocReqBuf[5];
	functionCount = numFunctions;
	functionResourceReferenceAssociations = (uint16_t*)malloc(numFunctions * sizeof(uint16_t));
	userFunctions = (void**)malloc(sizeof(void*)*numFunctions);


	// response length is 8 + 4*N; (2 length + 4 data Pointer + 2 CRC); N is number of returned function pointers
	uint8_t responseBuffer[14 + 4*functionCount];   // 6 chars extra for '+++'
	
	// Old data allocation:
	//uint32_t dataAddress = (uint32_t)heap_caps_malloc(((dataLength + 1) / 4)*sizeof(uint32_t), MALLOC_CAP_32BIT | MALLOC_CAP_EXEC);
	//
	//for (uint32_t i = 0; i < numFunctions; i++)
	//{
	//	uint32_t functionLength = allocReqBuf[4*i + 6] << 24 | allocReqBuf[4*i + 7] << 16 | allocReqBuf[4*i + 8] << 8 | allocReqBuf[4*i + 9];
	//	uint32_t address = (uint32_t)heap_caps_malloc(((functionLength+1)/4)*sizeof(uint32_t), MALLOC_CAP_32BIT | MALLOC_CAP_EXEC);
	//	userFunctions[i] = (void*)address;
	//	responseBuffer[4*i + 9] = address >> 24;
	//	responseBuffer[4*i + 10] = address >> 16;
	//	responseBuffer[4*i + 11] = address >> 8;
	//	responseBuffer[4*i + 12] = address;
	//}
	
	// New data allocation:
	uint32_t totalLength = dataLength;
	totalLength = ((uint32_t)((totalLength+3)/4))*4; // ceil to multiple of 4
	uint32_t currentFunctionAddress = totalLength;
	for (uint32_t i = 0; i < numFunctions; i++)
	{
		uint32_t functionLength = allocReqBuf[4*i + 6] << 24 | allocReqBuf[4*i + 7] << 16 | allocReqBuf[4*i + 8] << 8 | allocReqBuf[4*i + 9];
		functionLength = ((uint32_t)((functionLength+3)/4))*4; // ceil to multiple of 4
		totalLength += functionLength;
	}

	uint32_t dataAddress = (uint32_t)heap_caps_malloc(totalLength, MALLOC_CAP_32BIT | MALLOC_CAP_EXEC);
	userData = (uint32_t*)dataAddress;
	currentFunctionAddress += dataAddress;
	for (uint32_t i = 0; i < numFunctions; i++)
	{
		userFunctions[i] = (void*)currentFunctionAddress;
		responseBuffer[4*i + 9] = currentFunctionAddress >> 24;
		responseBuffer[4*i + 10] = currentFunctionAddress >> 16;
		responseBuffer[4*i + 11] = currentFunctionAddress >> 8;
		responseBuffer[4*i + 12] = currentFunctionAddress;

		uint32_t functionLength = allocReqBuf[4*i + 6] << 24 | allocReqBuf[4*i + 7] << 16 | allocReqBuf[4*i + 8] << 8 | allocReqBuf[4*i + 9];
		functionLength = ((uint32_t)((functionLength+3)/4))*4; // ceil to multiple of 4
		currentFunctionAddress += functionLength;
	}
	

	responseBuffer[0] = '+';
	responseBuffer[1] = '+'; 
	responseBuffer[2] = '+';
	
	responseBuffer[3] = dataAddress >> 24;
	responseBuffer[4] = dataAddress >> 16;
	responseBuffer[5] = dataAddress >> 8;
	responseBuffer[6] = dataAddress;

	responseBuffer[7] = numFunctions>>8;
	responseBuffer[8] = numFunctions;

	
	uint32_t crc = CRC16_CCITT_FALSE(responseBuffer + 3, 4*numFunctions + 6);
	responseBuffer[4*numFunctions + 9] = crc >> 8;
	responseBuffer[4*numFunctions + 10] = crc;

	responseBuffer[4*numFunctions + 11] = '+';
	responseBuffer[4*numFunctions + 12] = '+';
	responseBuffer[4*numFunctions + 13] = '+';
	
	//xSemaphoreTake(printSemaphore, portMAX_DELAY);
	uart_write_bytes(UART_NUM_0, (const char*)responseBuffer, 4*numFunctions + 14);
	uart_write_bytes(UART_NUM_0, "\r\n", 1);
	//xSemaphoreGive(printSemaphore);

	//vTaskDelay(200 / portTICK_PERIOD_MS);
	//xSemaphoreTake(printSemaphore, portMAX_DELAY);
	//for (int i = 0; i < 70; i++)
	//{
	//	printf("%u: 0x%x\n", i, responseBuffer[i]);	
	//}
	//xSemaphoreGive(printSemaphore);
}

static void HandleUARTData()
{
	ESP_LOGI(TAG, "Handle UART data");
	//vTaskDelay(10 / portTICK_PERIOD_MS);
	
	//xSemaphoreTake(printSemaphore, portMAX_DELAY);
	//for (int i = 0; i < dataLen; i++)
	//{
	//	printf("%d (%c)\n", progBuf[i], progBuf[i]);
	//}
	//xSemaphoreGive(printSemaphore);
	
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