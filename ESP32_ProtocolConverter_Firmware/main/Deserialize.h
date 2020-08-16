#pragma once

#include "main.h"


typedef enum
{
	ModuleBlock = 0,  // abstract, should never be used
	BufferRetrieve,
	BufferStore,
	BufferStoreConstant,
	BufferTransfer,
	BufferAdvancedTransfer,
	BufferDiscard,
	GPIOOutBlock,
	GPIOInBlock,
	GPIOModeBlock,
	GPIOTrigger,
	ManualTrigger,
	UARTReceiveTrigger,
	UARTSendBlock,
	UARTModeBlock,
	SPISendBlock,
	SPIMasterSettingBlock,

	TestControlCode = 0xFFFF
} ModuleBlockCode;

typedef enum
{
	TriggerChain = 0,
	BufferOp
} ViewportElementCode;

typedef enum
{
	LIFO = 0,
	FIFO
} BufferType;

void Deserialize(uint8_t *buf, uint16_t len);
uint8_t readUint8(uint8_t **buf);
uint16_t readUint16(uint8_t **buf);
uint32_t readUint32(uint8_t **buf);
void readBufferManager(uint8_t **buf);
void readTriggerChain(uint8_t **buf);
void readModule(uint8_t **buf);