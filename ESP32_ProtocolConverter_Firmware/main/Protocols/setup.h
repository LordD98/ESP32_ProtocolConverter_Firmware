#pragma once

#define MIN(X,Y) ((X)<(Y)?(X):(Y))
#define MAX(X,Y) ((X)>(Y)?(X):(Y))

#define RBP_READ(RB) ({uint8_t _RBP_READ_RET_VALUE_PRIVATE = (RB)->data[(RB)->readCursor]; ((RB)->readCursor = ((RB)->readCursor+1) % (RB)->length); _RBP_READ_RET_VALUE_PRIVATE;})
#define RBP_WRITE(RB,V) do { (RB)->data[(RB)->writeCursor]=(V); (RB)->writeCursor = ((RB)->writeCursor+1) % (RB)->length; } while(0)
#define RB_READ(RB) ({uint8_t _RBP_READ_RET_VALUE_PRIVATE = (RB).data[(RB).readCursor]; ((RB).readCursor = ((RB).readCursor+1) % (RB).length); _RBP_READ_RET_VALUE_PRIVATE;})
#define RB_WRITE(RB,V) do { (RB).data[(RB).writeCursor]=(V); (RB).writeCursor = ((RB).writeCursor+1) % (RB).length; } while(0)

#define RB_EMPTY(RB) ((RB).readCursor == (RB).writeCursor)
#define RB_REM(RB) ((RB).writeCursor - (RB).readCursor)
#define RBP_EMPTY(RB) ((RB)->readCursor == (RB)->writeCursor)
#define RBP_REM(RB) ((RB)->writeCursor - (RB)->readCursor)

typedef struct
{
	uint32_t readCursor;
	uint32_t writeCursor;
	uint32_t length;
	uint8_t data[];
} RingBuffer;					// Rule: Buffer should only have lengths that are powers of two, so that there is no overhead when reading or writing across the boundary (readCursor%=length is a (NOP with) truncation)

void SetupProtocols();
void AllocInt_Custom(int source, void(*handler)());

void i2c0_master_isr(void *param);
void i2c1_master_isr(void *param);

extern RingBuffer *I2C0_Data;
extern RingBuffer *I2C1_Data;