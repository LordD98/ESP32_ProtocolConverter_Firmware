#pragma once

#include <stdint.h>

//#define ALIGNED_READ(PTR) __atomic_load_n(PTR, __ATOMIC_RELAXED)
//#define ALIGNED_WRITE(PTR, VAL) __atomic_store_n(PTR, (uint32_t)VAL, __ATOMIC_RELAXED)
#define ALIGNED_READ(PTR, DEST) asm("l32i %0, %1, 0;" : "=r"(DEST) : "rm"(PTR) :)
#define ALIGNED_WRITE(PTR, VAL) asm("s32i %0, %1, 0;" : : "r"((uint32_t)VAL), "rm"(PTR) :)

// on APP CPU
void vAppTask(void *pvParameters);
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