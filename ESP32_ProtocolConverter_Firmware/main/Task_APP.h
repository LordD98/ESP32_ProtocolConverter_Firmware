#pragma once

#include <stdint.h>
#include "setup.h"

//#define ALIGNED_READ(PTR) __atomic_load_n(PTR, __ATOMIC_RELAXED)
//#define ALIGNED_WRITE(PTR, VAL) __atomic_store_n(PTR, (uint32_t)VAL, __ATOMIC_RELAXED)
#define ALIGNED_READ(PTR, DEST) asm("l32i %0, %1, 0;" : "=r"(DEST) : "rm"(PTR) :)
#define ALIGNED_WRITE(PTR, VAL) asm("s32i %0, %1, 0;" : : "r"((uint32_t)VAL), "rm"(PTR) :)

// on APP CPU
void vAppTask(void *pvParameters);
void vManualTriggerTask(void *pvParameters);
uint32_t call8FromAddresses(uint32_t origin, uint32_t target);
void printInc();

void vCall8OffsetTest(); // just for debugging

void vUserCode0();
void vUserCode1();
void vUserCode2();
void vUserCode3();
void vUserCode4();
void vUserCode5();
void vUserCode6();
void vUserCode7();

//extern uint16_t *constBufferResourceReferenceAssociations;
extern uint16_t *bufferResourceReferenceAssociations;
extern uint16_t *functionResourceReferenceAssociations;

extern int16_t initFunction;

extern uint16_t constBufferCount;
extern uint16_t bufferCount;
extern uint16_t functionCount;

extern RingBuffer **constBuffers;
extern RingBuffer **buffers;
extern void **userFunctions;
extern uint32_t *userData;

//extern volatile uint32_t* GPIO_BASE_REG;
//extern volatile uint32_t BIT_MASKS_SET[];
//extern volatile uint32_t BIT_MASKS_CLEAR[];
//void DisableGPIO_Lower(uint32_t gpioNum); // 0-31
//void DisableGPIO_Upper(uint32_t gpioNum); // 32-39
//void EnableGPIO_OutDriveHigh_Lower(uint32_t gpioNum); // 0-31
//void EnableGPIO_OutDriveHigh_Upper(uint32_t gpioNum); // 32-39
//void EnableGPIO_Input_Lower(uint32_t gpioNum); // 0-31
//void EnableGPIO_Input_Upper(uint32_t gpioNum); // 32-39