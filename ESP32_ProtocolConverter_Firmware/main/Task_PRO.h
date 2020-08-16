
#include <stdint.h>

typedef enum 
{
	None,
	UART_ReceptionComplete
} TaskMessage;

extern uint8_t progBuf[2048]; // Buffer that stores program data
extern uint32_t dataLen;

extern uint32_t lastUpdate; // in ms since thread start

// on PRO CPU
void vProTask(void *pvParameters);
static void HandleUARTData();
uint16_t CRC16_CCITT_FALSE(uint8_t *buf, uint32_t len);