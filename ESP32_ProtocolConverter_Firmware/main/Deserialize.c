
#include "Deserialize.h"
#include "esp_log.h"
#include "Task_PRO.h"


static const char *TAG = "Deserializer";
//ModuleBlockCode mbc;

void Deserialize(uint8_t *buf, uint16_t len)
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
	
	readBufferManager(bufCursor);
	while (*bufCursor < endPtr)
	{
		uint8_t elementCode = readUint8(bufCursor);
		if (elementCode == (uint8_t)TriggerChain)
			readTriggerChain(bufCursor);
		else if (elementCode == (uint8_t)BufferOp)
			readModule(bufCursor);
	}
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
	
	uint16_t result = (**buf << 24);
	(*buf)++;
	result |= **buf << 16;
	(*buf)++;
	result |= **buf << 8;
	(*buf)++;
	result |= **buf;
	(*buf)++;
	
	return result;
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
