
#include "main.h"
#include "Task_PRO.h"
#include "UART_Interface.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"

#define PATTERN_CHR_NUM    (3)         // Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define RX_RINGBUF_SIZE (1024)

static const char *TAG = "UART_Interface";

static const char* COMMANDS[8] = 
{
	"",				// NOP
	"STATUS",		// Queries status&connection information
	"PING",			// Returns "PONG", a quick check if an interface adapter is listening on the COM port
	//"SPEEDTEST",	// Test the connection speed
	"CONNECT",		// Puts the adapter in connected state
	"DISCONNECT",	// Puts the adapter in disconnected state
	"REFRESH",		// Refreshes existing connection
	"DATA",			// Updates the program
	"TRIGGER",		// Ativates a manual trigger
};

uint8_t rxbuf[2048];
uint16_t dataStart, urxlen;
ConnectionState connectionState = Disconnected;
TaskHandle_t UART_COMMAND_TASK;
bool awaitingCommand = false;	// Commands have to be transmitted in the form of "+++COMMAND+++"
								// any "+++" in data transmission has to be faster than 125µs between each '+', otherwise transmission is aborted

void HandleUartCommandTask()
{
	while (1)
	{
		if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(10000)) == pdFAIL)
		{
			// Timeout (no notification)
			if(esp_log_timestamp() - lastUpdate > 9990 && awaitingCommand)
				ResetRXBuffer();
			continue;
		}
		uint32_t timestamp = esp_log_timestamp();    // in ms
		
		
		//Check command
		int8_t detectedCommand = -1;
		for(int i = 1 ; i < 8; i++)
		{
			if (memcmp(COMMANDS[i], rxbuf, strlen(COMMANDS[i])) == 0)
			{
				detectedCommand = i;
				dataStart = strlen(COMMANDS[i]);
				break;
			}
		}
		if (detectedCommand == -1)
			if (urxlen == 3)			// NOP
				detectedCommand = 0;	//
		if (detectedCommand == -1)
		{
			ESP_LOGE(TAG, "No command detected!");
			continue;
		}
		//else
		//{
		//	ESP_LOGI(TAG, "%s command detected!", COMMANDS[detectedCommand]);
		//}
		
		// Print command including appended data
		//for(int i = 0 ; i < urxlen-3; i++)
		//{
		//	uart_write_bytes(UART_NUM_0, (const char*)&rxbuf[i], 1);
		//}
		//uart_write_bytes(UART_NUM_0, (const char*)"\n", 1);
		
		
		
		switch(connectionState)
		{	
		case Disconnected:
			switch (detectedCommand)
			{
			case NOP:
				break;
			case STATUS_GET:
				ESP_LOGI(TAG, "Status: Disconnected");
				break;
			case PING:
				ESP_LOGI(TAG, "PONG");
				break;
			case CONNECT:
				{
					connectionState = Connected;
					ESP_LOGI(TAG, "Connected");
					break;
				}
			case DISCONNECT:
				break;
			case REFRESH:
				break;
			case DATA:
				break;
			case TRIGGER:
				break;
			}
			break;
		case Connected:
			switch (detectedCommand)
			{
			case NOP:
				break;
			case STATUS_GET:
				ESP_LOGI(TAG, "Status: Connected");
				break;
			case PING:
				ESP_LOGI(TAG, "PONG");
				break;
			case CONNECT:
				break;
			case DISCONNECT:
				{
					connectionState = Disconnected;
					ESP_LOGI(TAG, "Disconnected");
					break;
				}
			case REFRESH:
				{
					connectionState = Connected;
					ESP_LOGI(TAG, "Connection refreshed");
					break;
				}
			case DATA:
				dataLen = urxlen - 3 - dataStart;
				ESP_LOGI(TAG, "Data received, length=%u", dataLen);
				memcpy(&progBuf[0], &rxbuf[dataStart], dataLen);
				
				xTaskNotifyFromISR(PRO_TASK, UART_ReceptionComplete, eSetValueWithOverwrite, NULL);
				break;
			case TRIGGER:
				ESP_LOGI(TAG, "Trigger %u activated", *(uint16_t*)&rxbuf[dataStart]);
				break;
			}
			break;
		}
		
		vTaskDelay(1 / portTICK_RATE_MS);
		ResetRXBuffer();
		lastUpdate = timestamp;
	}
}

void ResetRXBuffer()
{
	urxlen = 0;
	dataStart = 0;
	awaitingCommand = false;
}

void IRAM_ATTR UART_RX_ISR(void *arg)
{
	ets_printf(TAG, "ISR\n");
	
	if (UART0.int_st.rxfifo_ovf)
	{
		uart_flush_input(UART_NUM_0);
		uart_clear_intr_status(UART_NUM_0, UART_RXFIFO_OVF_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
		ets_printf(TAG, "UART receive error!\n");
		ResetRXBuffer();
		UART0.int_ena.brk_det = 0;
		return;
	}
	if (UART0.int_st.at_cmd_char_det)
	{
		uart_clear_intr_status(UART_NUM_0, UART_AT_CMD_CHAR_DET_INT_CLR);
			
		if (awaitingCommand)
		{
			while (UART0.status.rxfifo_cnt)
			{
				rxbuf[urxlen] = READ_PERI_REG(UART_FIFO_REG(UART_NUM_0));
				urxlen++;
			}
			
			xTaskNotifyFromISR(UART_COMMAND_TASK, 0, eSetValueWithOverwrite, NULL);
		}
		uart_flush_input(UART_NUM_0);
		awaitingCommand = !awaitingCommand;
	}
	if (UART0.int_st.rxfifo_full || UART0.int_st.brk_det)
	{
		if (awaitingCommand)
		{
			while (UART0.status.rxfifo_cnt)
			{
				rxbuf[urxlen] = READ_PERI_REG(UART_FIFO_REG(UART_NUM_0));
				urxlen++;
			}
			//uart_flush_input(UART_NUM_0);
		}
		uart_flush_input(UART_NUM_0);
		uart_clear_intr_status(UART_NUM_0, UART_RXFIFO_FULL_INT_CLR | UART_BRK_DET_INT_CLR);
	}
	
	ets_printf(TAG, "ISR Done\n");
}

void Config_UART_Interface()
{
	xTaskCreatePinnedToCore(HandleUartCommandTask, "UART_COMMAND_TASK", 2048, NULL, 12, &UART_COMMAND_TASK, 1);
	
	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config = 
	{
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	uart_param_config(UART_NUM_0, &uart_config);
	
	uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);    	//Set UART pins (using UART0 default pins ie no changes.)
	//uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);  							//Install UART driver, and get the queue.
	uart_driver_install(UART_NUM_0, RX_RINGBUF_SIZE, 0, 0, NULL, 0);
  
	uart_isr_free(UART_NUM_0);
	uart_isr_register(UART_NUM_0, UART_RX_ISR, NULL, ESP_INTR_FLAG_IRAM, NULL);
	//uart_enable_rx_intr(UART_NUM_0);
	//uart_disable_rx_intr(UART_NUM_0);
	//uart_disable_rx_ovf_intr(UART_NUM_0);
	//UART_INT_ENA_REG
	
	UART0.int_ena.val =  UART_AT_CMD_CHAR_DET_INT_ENA | UART_RXFIFO_FULL_INT_ENA | UART_BRK_DET_INT_ENA;    // disable all unnecessary uart0 interrupts
	
	//uint32_t FullTreshold = 2;
	////*(volatile uint32_t*)UART_CONF1_REG(0) = ((*(volatile uint32_t*)UART_CONF1_REG(0)) & 0xFFFFFF80) | (FullTreshold & 0x7F);
	////*(volatile uint32_t*)UART_MEM_CONF_REG(0) = ((*(volatile uint32_t*)UART_MEM_CONF_REG(0)) & (~(1 << 25 | 1 << 26 | 1 << 27))) | ((FullTreshold & 0x310) << (25-7));
	//UART0.conf1.rxfifo_full_thrhd = FullTreshold;
	//UART0.mem_conf.rx_flow_thrhd_h3 = FullTreshold >> 7;
	
	uint32_t ToutTicks = 1000;
	UART0.conf1.rx_tout_thrhd = ToutTicks;
	UART0.mem_conf.rx_tout_thrhd_h3 = ToutTicks >> 7;
	
	uart_enable_pattern_det_intr(UART_NUM_0, '+', 3, 80000, 10, 10);   // min time between at_cmd-chars = 80e3 =^= 1ms
	
	//uart_enable_intr_mask(UART_NUM_0, UART_AT_CMD_CHAR_DET_INT_ENA | UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_TOUT_INT_ENA);  // enable pattern, overflow & timeout  interrupts

	ESP_LOGI(TAG, "Func address: %x", (uint32_t)(void*)Config_UART_Interface);
	vTaskDelay(10/portTICK_PERIOD_MS);
	uart_write_bytes(UART_NUM_0, (const char*)"Config done\n", 12);
	
	//uint32_t time0 = esp_log_timestamp();
	//uint32_t time1 = esp_log_timestamp();
	//ESP_LOGI(TAG, "Config done\n");
	//uint32_t time2 = esp_log_timestamp();
	//uint32_t timeDiff = time2 - time1 - (time1-time0);
	//ESP_LOGI(TAG, "That message took %d milliseconds to send\n", timeDiff);
	
	//esp_err_t uart_intr_config(uart_port_t uart_num, const uart_intr_config_t *intr_conf); // esp32_technical_reference_manual_en.pdf S.356
}

/*
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
	for (;;) 
	{
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) 
	        {
                //Event of UART receving data
                //We'd better handler data event fast, there would be much more data events than
                //other types of events. If we take too much time on data event, the queue might
                //be full.
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) 
                    {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    }
					else 
                    {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}
*/