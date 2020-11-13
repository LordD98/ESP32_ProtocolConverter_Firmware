
#include <stdint.h>

typedef enum 
{
	None,
	UART_StatusRequest,
	UART_Query,
	UART_AllocRequest,
	UART_Data,
} TaskMessage;

extern uint8_t progBuf[2048];		// Buffer that stores program data
extern uint8_t allocReqBuf[512];	// Buffer that stores alloc requests
extern uint32_t dataLen;
extern uint32_t allocReqLen;

extern uint32_t lastUpdate; // in ms since thread start

// on PRO CPU
void vProTask(void *pvParameters);
static void HandleUARTStatusRequest();
static void HandleUARTQuery();
static void HandleUARTAllocRequest();
static void HandleUARTData();
uint16_t CRC16_CCITT_FALSE(uint8_t *buf, uint32_t len);