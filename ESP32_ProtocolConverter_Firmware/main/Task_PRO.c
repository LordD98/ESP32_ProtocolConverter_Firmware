#include "main.h"
#include "Task_PRO.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "UART_Interface.h"
#include "Deserialize.h"
#include "string.h"
#include "setup.h"

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
	const uint16_t N = 3;
	uint8_t responseBuffer[16 + 6*N]; // 6 chars extra for '+++'
	void* pointers[] = { ets_delay_us, &I2C0_Data, &I2C1_Data };
	
	responseBuffer[0] = '+';
	responseBuffer[1] = '+'; 
	responseBuffer[2] = '+';

	uint32_t length = 4 + 6*N;
	responseBuffer[3] = length >> 24;
	responseBuffer[4] = length >> 16; 
	responseBuffer[5] = length >> 8;
	responseBuffer[6] = length;

	responseBuffer[7] = 0;											// Version	// TODO: Move to global header: global_defs.h (main.h)
	responseBuffer[8] = 3;											// 0.3

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
	
	//free(constBufferResourceReferenceAssociations);
	free(bufferResourceReferenceAssociations);
	free(functionResourceReferenceAssociations);

	for (int i = 0; i < constBufferCount; i++)
	{
		free(constBuffers[i]);
	}
	for (int i = 0; i < bufferCount; i++)
	{
		free(buffers[i]);
	}
	free(userFunctions);
	heap_caps_free(userData);
	

	uint32_t dataLength = allocReqBuf[0] << 24 | allocReqBuf[1] << 16 | allocReqBuf[2] << 8 | allocReqBuf[3];
	
	uint16_t numConstBuffers = allocReqBuf[4] << 8 | allocReqBuf[5];
	uint16_t numBuffers = allocReqBuf[6 + 4*numConstBuffers] << 8 | allocReqBuf[7 + 4*numConstBuffers];
	uint16_t numConstants = allocReqBuf[8 + 4*(numConstBuffers + numBuffers)] << 8 | allocReqBuf[9 + 4*(numConstBuffers + numBuffers)];
	uint16_t numFunctions = allocReqBuf[10 + 4*(numConstBuffers + numBuffers)] << 8 | allocReqBuf[11 + 4*(numConstBuffers + numBuffers)];
	
	if (dataLength != 8 + 4*(numConstBuffers + numBuffers + numFunctions))
	{
		ESP_LOGE(TAG, "Received allocation request is invalid");
		return;
	}
	
	constBufferCount = numConstBuffers;
	bufferCount = numBuffers;
	functionCount = numFunctions;
	
	//constBufferResourceReferenceAssociations = (uint16_t*)malloc(numConstBuffers * sizeof(uint16_t));
	constBuffers = (RingBuffer**)malloc(sizeof(void*)*numConstBuffers);
	bufferResourceReferenceAssociations = (uint16_t*)malloc(numBuffers * sizeof(uint16_t));
	buffers = (RingBuffer**)malloc(sizeof(void*)*numBuffers);
	functionResourceReferenceAssociations = (uint16_t*)malloc(numFunctions * sizeof(uint16_t));
	userFunctions = (void**)malloc(sizeof(void*)*numFunctions);


	// response length is 12 + 4*N;		N is number of returned function pointers
	uint8_t responseBuffer[18 + 4*(functionCount + constBufferCount + bufferCount)];     // 6 chars extra for '+++'
	
	
	for(uint32_t i = 0 ; i < constBufferCount ; i++)
	{
		uint32_t constBufferLength = allocReqBuf[4*i + 6] << 24 | allocReqBuf[4*i + 7] << 16 | allocReqBuf[4*i + 8] << 8 | allocReqBuf[4*i + 9];
		constBuffers[i] = malloc(sizeof(RingBuffer) + sizeof(uint8_t)*(constBufferLength+1));
		constBuffers[i]->length = constBufferLength + 1;
		constBuffers[i]->readCursor = 0;
		constBuffers[i]->writeCursor = 0;

		responseBuffer[4*i + 5] = (uint32_t)constBuffers[i] >> 24;
		responseBuffer[4*i + 6] = (uint32_t)constBuffers[i] >> 16;
		responseBuffer[4*i + 7] = (uint32_t)constBuffers[i] >> 8;
		responseBuffer[4*i + 8] = (uint32_t)constBuffers[i];
	}

	for (uint32_t i = 0; i < bufferCount; i++)
	{
		uint32_t bufferLength = allocReqBuf[4*(i + constBufferCount) + 8] << 24 | allocReqBuf[4*(i + constBufferCount) + 9] << 16 | allocReqBuf[4*(i + constBufferCount) + 10] << 8 | allocReqBuf[4*(i + constBufferCount) + 11];
		buffers[i] = malloc(sizeof(RingBuffer) + sizeof(uint8_t)*(bufferLength+1));
		buffers[i]->length = bufferLength + 1;
		buffers[i]->readCursor = 0;
		buffers[i]->writeCursor = 0;

		responseBuffer[4*(i + constBufferCount) + 7] = (uint32_t)buffers[i] >> 24;
		responseBuffer[4*(i + constBufferCount) + 8] = (uint32_t)buffers[i] >> 16;
		responseBuffer[4*(i + constBufferCount) + 9] = (uint32_t)buffers[i] >> 8;
		responseBuffer[4*(i + constBufferCount) + 10] = (uint32_t)buffers[i];
	}



	// Data&Function space calculation:
	uint32_t totalLength = 4*numConstants;
	uint32_t currentFunctionAddress = totalLength;
	for (uint32_t i = 0; i < numFunctions; i++)
	{
		uint32_t functionLength = allocReqBuf[4*(i + constBufferCount + bufferCount) + 12] << 24 | allocReqBuf[4*(i + constBufferCount + bufferCount) + 13] << 16 | allocReqBuf[4*(i + constBufferCount + bufferCount) + 14] << 8 | allocReqBuf[4*(i + constBufferCount + bufferCount) + 15];
		functionLength = ((uint32_t)((functionLength+3)/4))*4; // ceil to multiple of 4
		totalLength += functionLength;
	}

	// Data&Function space allocation:
	uint32_t dataAddress = (uint32_t)heap_caps_malloc(totalLength, MALLOC_CAP_32BIT | MALLOC_CAP_EXEC);
	userData = (uint32_t*)dataAddress;
	currentFunctionAddress += dataAddress;
	for (uint32_t i = 0; i < numFunctions; i++)
	{
		userFunctions[i] = (void*)currentFunctionAddress;
		responseBuffer[4*(i + constBufferCount + bufferCount) + 13] = currentFunctionAddress >> 24;
		responseBuffer[4*(i + constBufferCount + bufferCount) + 14] = currentFunctionAddress >> 16;
		responseBuffer[4*(i + constBufferCount + bufferCount) + 15] = currentFunctionAddress >> 8;
		responseBuffer[4*(i + constBufferCount + bufferCount) + 16] = currentFunctionAddress;

		uint32_t functionLength = allocReqBuf[4*(i + constBufferCount + bufferCount) + 12] << 24 | allocReqBuf[4*(i + constBufferCount + bufferCount) + 13] << 16 | allocReqBuf[4*(i + constBufferCount + bufferCount) + 14] << 8 | allocReqBuf[4*(i + constBufferCount + bufferCount) + 15];
		functionLength = ((uint32_t)((functionLength+3)/4))*4; // ceil to multiple of 4
		currentFunctionAddress += functionLength;
	}
	

	responseBuffer[0] = '+';
	responseBuffer[1] = '+'; 
	responseBuffer[2] = '+';
	
	responseBuffer[3] = constBufferCount >> 8;
	responseBuffer[4] = constBufferCount;

	responseBuffer[5 + 4*constBufferCount] = bufferCount >> 8;
	responseBuffer[6 + 4*constBufferCount] = bufferCount;

	responseBuffer[7 + 4*(constBufferCount + bufferCount)] = dataAddress >> 24;
	responseBuffer[8 + 4*(constBufferCount + bufferCount)] = dataAddress >> 16 ;
	responseBuffer[9 + 4*(constBufferCount + bufferCount)] = dataAddress >> 8;
	responseBuffer[10 + 4*(constBufferCount + bufferCount)] = dataAddress;

	responseBuffer[11 + 4*(constBufferCount + bufferCount)] = functionCount >> 8;
	responseBuffer[12 + 4*(constBufferCount + bufferCount)] = functionCount;

	
	uint32_t crc = CRC16_CCITT_FALSE(responseBuffer + 3, 4*(constBufferCount + bufferCount + functionCount) + 10);
	responseBuffer[4*(constBufferCount + bufferCount + functionCount) + 13] = crc >> 8;
	responseBuffer[4*(constBufferCount + bufferCount + functionCount) + 14] = crc;

	responseBuffer[4*(constBufferCount + bufferCount + functionCount) + 15] = '+';
	responseBuffer[4*(constBufferCount + bufferCount + functionCount) + 16] = '+';
	responseBuffer[4*(constBufferCount + bufferCount + functionCount) + 17] = '+';
	
	//xSemaphoreTake(printSemaphore, portMAX_DELAY);
	uart_write_bytes(UART_NUM_0, (const char*)responseBuffer, 4 * (constBufferCount + bufferCount + functionCount) + 18);
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