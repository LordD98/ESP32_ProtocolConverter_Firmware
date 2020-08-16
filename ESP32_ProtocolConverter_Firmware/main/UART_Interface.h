#pragma once

typedef enum
{
	Disconnected,
	Connected,
} ConnectionState;


typedef enum
{
	NOP,			// NOP
	STATUS_GET,		// Queries status&connection information
	PING,			// Returns "PONG", a quick check if an interface adapter is listening on the COM port
	//"SPEEDTEST",	// Test the connection speed
	CONNECT,		// Puts the adapter in connected state
	DISCONNECT,		// Puts the adapter in disconnected state
	REFRESH,		// Refreshes existing connection
	DATA,			// Updates the program
	TRIGGER, 		// Ativates a manual trigger
} UartCommands;

	
extern ConnectionState state;
extern uint8_t rxbuf[2048];
extern uint16_t urxlen; 
extern uint16_t dataStart; 

void Config_UART_Interface();
void ResetRXBuffer();
void UART_RX_ISR();
void HandleUartCommandTask();