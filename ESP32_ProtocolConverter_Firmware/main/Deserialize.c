
#include "Deserialize.h"
#include "esp_log.h"
#include "Task_PRO.h"
#include "Task_APP.h"


static const char *TAG = "Deserializer";
//ModuleBlockCode mbc;

void Deserialize(uint8_t buf[], uint16_t len)
{
	// Check CRC
	if(CRC16_CCITT_FALSE(buf, len - 2) != (buf[len-2] << 8 | buf[len-1])) // maybe check CRC of entire packet (including CRC) and comparing to 0
	{
		ESP_LOGE(TAG, "CRC of received data is incorrect");
		return;
	}

	uint8_t *buffer = buf;
	uint8_t **bufCursor = &buffer;
	uint8_t *endPtr = buffer + len;  // points to the byte after the last data byte
	uint32_t length = readUint32(bufCursor);
	if (length != len - 6) // -6; not including CRC and length
	{
		ESP_LOGE(TAG, "Length of received data does not match given length");
		return;
	}
	ESP_LOGI(TAG, "Beginning deserialization");
	
	initFunction = -1;
	readBufferManager(bufCursor);
	readFunctionList(bufCursor);
	readData(bufCursor);

	if(initFunction >= 0)
		xTaskNotify(APP_TASK, initFunction, eSetValueWithOverwrite);
}

uint8_t readUint8(uint8_t **buf)
{
	uint8_t result = **buf;
	*buf += 1;
	return result;
}

uint16_t readUint16(uint8_t **buf)
{
	//uint16_t result = (**buf<<8) | (**(buf+1));
	//*buf += 2;
	
	uint16_t result = (**buf << 8);
	(*buf) ++;
	result |= **buf;
	(*buf) ++;
	
	//result |= ++**buf++;
	return result;
}

uint32_t readUint32(uint8_t **buf)
{
	//uint32_t result = (**buf << 24) | (**(buf + 1)<<16) | (**(buf+2) << 8) | (**(buf + 3));
	//*buf += 4;
	
	uint32_t result = (**buf << 24);
	(*buf)++;
	result |= **buf << 16;
	(*buf)++;
	result |= **buf << 8;
	(*buf)++;
	result |= **buf;
	(*buf)++;
	
	return result;
}

#include <inttypes.h>
void readFunctionList(uint8_t **buf)
{
	/*
	Count of functions (MSB first, 2 bytes)
	{ // List of functions (machinecode)
		// Trigger IL
			ViewportTriggerCode: Trigger (2 byte)
				// ie. GPIOTrigger
				// ie. ManualTrigger
				// ie. UARTReceiveTrigger,
			Length of following trigger data (MSB first, 4 bytes)
			... Additional trigger data
		Length of code n (MSB first, 4 bytes)
		Code (n bytes, as specified above)
	}
	*/
	
	uint16_t functionCount = readUint16(buf);

	for (int i = 0; i < functionCount; i++)
	{
		uint16_t triggerCode = readUint16(buf);
		uint32_t triggerSize = readUint32(buf);

		switch (triggerCode)
		{
		case StartupTrigger:
		{
			if (initFunction < 0)
				initFunction = i;
			else
				ESP_LOGE(TAG, "Only one initialization function allowed");
			break;
		}
		case ManualTrigger:
		{
			uint16_t manualTriggerResourceRef = readUint16(buf);
			functionResourceReferenceAssociations[i] = manualTriggerResourceRef;
			break;
		}
		case GPIOTrigger:
			*buf += triggerSize;
			break;
		case UARTReceiveTrigger:
			*buf += triggerSize;
			break;
		}
		
		uint32_t codeSize = readUint32(buf);
		uint32_t nextWord;
		for (int j = 0; j < codeSize; j+=4)
		{
			if (codeSize > j + 3) 
			{
				//nextWord = readUint32(buf); // only if there are at least 4 bytes to read
				
				uint8_t byte1 = readUint8(buf);
				uint8_t byte2 = readUint8(buf);
				uint8_t byte3 = readUint8(buf);
				uint8_t byte4 = readUint8(buf);
				nextWord = byte4 << 24 | byte3 << 16 | byte2 << 8 | byte1;
			}
			else
			{
				uint8_t bytesLeft = codeSize - j;
				switch (bytesLeft)
				{
				case 1:
					{
						uint8_t byte1 = readUint8(buf);
						nextWord = byte1;
						break;
					}
				case 2:
					{
						uint8_t byte1 = readUint8(buf);
						uint8_t byte2 = readUint8(buf);
						nextWord = byte2<<8 | byte1;
						break;
					}
				default:
				case 3:
					{
						uint8_t byte1 = readUint8(buf);
						uint8_t byte2 = readUint8(buf);
						uint8_t byte3 = readUint8(buf);
						nextWord = byte3 << 16 | byte2 << 8 | byte1;
						break;
					}
				}
			}

			//*(volatile uint32_t*)((uint32_t)userFunctions[i] + j) = nextWord;
			ALIGNED_WRITE(((uint32_t)userFunctions[i]) + j, nextWord);
		}
	}

	//ESP_LOGE(TAG, "Length of received buffers does not match given length"); // Error detection?
}

void readData(uint8_t **buf)
{
	/*
	// Global Constants
		Length of data region n (MSB first, 4 bytes)
		Constants (n bytes, as specified above)
	*/

	uint32_t dataLength = readUint32(buf);

	uint32_t nextWord;
	for (int i = 0; i < dataLength; i += 4)
	{
		nextWord = readUint32(buf);  // only if there are at least 4 bytes to read
		*(volatile uint32_t*)((uint32_t)userData + i) = nextWord;
	}

	//ESP_LOGE(TAG, "Length of received buffers does not match given length"); // Error detection?
}

void readBufferManager(uint8_t **buf)
{
	uint32_t size = readUint32(buf);
	if (size < 2)
	{
		ESP_LOGE(TAG, "Invalid buffer size");
		return;	
	}
	uint16_t bufferCount = readUint16(buf);
	size -= 2;
		
	while (bufferCount != 0)
	{
		uint32_t bufSize = readUint32(buf);
		size -= 4;
		BufferType type = readUint8(buf);
		size -= 1;
		bufferCount--;
	}
	if (size != 0)
		ESP_LOGE(TAG, "Length of received buffers does not match given length");
}

void readTriggerChain(uint8_t **buf)
{
	uint16_t chainLength = readUint16(buf);
	while (chainLength != UINT16_MAX)
	{
		readModule(buf);
		chainLength--;
	}
}

void readModule(uint8_t **buf)
{
	ModuleBlockCode code = readUint16(buf);
	switch (code)
	{
	default:
		ESP_LOGE(TAG, "Invalid module code");
		//for testing, so that if module info has odd # of bytes, it won't go out of sync
		//(*buf)--;
		return;										// This cannot deadlock because readUint16 will always take 2 bytes from the data-queue
	case BufferRetrieve:
		ESP_LOGI(TAG, "Detected BufferRetrieve");
		return;
	case BufferStore:
		ESP_LOGI(TAG, "Detected BufferStore");
		return;
	case BufferStoreConstant:
		ESP_LOGI(TAG, "Detected BufferStoreConstant");
		return;
	case BufferTransfer:
		ESP_LOGI(TAG, "Detected BufferTransfer");
		return;
	case BufferAdvancedTransfer:
		ESP_LOGI(TAG, "Detected BufferAdvancedTransfer");
		return;
	case BufferDiscard:
		ESP_LOGI(TAG, "Detected BufferDiscard");
		return;
	case GPIOOutBlock:
		ESP_LOGI(TAG, "Detected GPIOOutBlock");
		return;
	case GPIOInBlock:
		ESP_LOGI(TAG, "Detected GPIOInBlock");
		return;
	case GPIOModeBlock:
		ESP_LOGI(TAG, "Detected GPIOModeBlock");
		return;
	case GPIOTrigger:
		ESP_LOGI(TAG, "Detected GPIOTrigger");
		return;
	case ManualTrigger:
		ESP_LOGI(TAG, "Detected ManualTrigger");
		uint16_t mtRef = readUint16(buf);
		return;
	case UARTReceiveTrigger:
		ESP_LOGI(TAG, "Detected UARTReceiveTrigger");
		return;
	case UARTSendBlock:
		ESP_LOGI(TAG, "Detected UARTSendBlock");
		return;
	case UARTModeBlock:
		ESP_LOGI(TAG, "Detected UARTModeBlock");
		return;
	case SPISendBlock:
		ESP_LOGI(TAG, "Detected SPISendBlock");
		return;
	case SPIMasterSettingBlock:
		ESP_LOGI(TAG, "Detected SPIMasterSettingBlock");
		return;
	}
}
