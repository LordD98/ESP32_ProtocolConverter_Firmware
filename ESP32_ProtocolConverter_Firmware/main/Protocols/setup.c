
#include "main.h"
#include "setup.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"

#define REG_SET(R,V) *((volatile uint32_t*)(R))=(V)
#define REG_OR(R,V) *((volatile uint32_t*)(R))|=(V)
#define REG_AND(R,V) *((volatile uint32_t*)(R))&=(V)
#define REG_READ(R) *((volatile uint32_t*)(R))

#define I2C0_CLI(INT) REG_SET(I2C0_INT_CLR_REG, (INT))


#define I2C0_BASE_REG 0x3FF53000
#define I2C1_BASE_REG 0x3FF57000
#define I2C0_APB_BASE_REG 0x60013000
#define I2C1_APB_BASE_REG 0x60017000

#define I2C0_INT_RAW_REG 0x3FF53020
#define I2C0_INT_ENA_REG 0x3FF53028
#define I2C0_INT_CLR_REG 0x3FF53024
#define I2C_INT_STATUS_REG 0x3FF5302C // This is the one to read

//#define I2C0_RAM_ADDR (I2C0_APB_BASE_REG + 0x100)
#define I2C0_FIFO_ADDR (I2C0_APB_BASE_REG + 0x1C)

#define I2C_TX_SEND_EMPTY_INT		(1<<12)
#define I2C_RX_REC_FULL_INT			(1<<11)
#define I2C_ACK_ERR_INT				(1<<10)
#define I2C_TRANS_START_INT			(1<<9)
#define I2C_TIME_OUT_INT			(1<<8)
#define I2C_TRANS_COMPLETE_INT		(1<<7)
#define I2C_MASTER_TRAN_COMP_INT	(1<<6)
#define I2C_ARBITRATION_LOST_INT	(1<<5)
#define I2C_SLAVE_TRAN_COMP_INT		(1<<4)
#define I2C_END_DETECT_INT			(1<<3)
#define I2C_RXFIFO_OVF_INT			(1<<2)
#define I2C_TXFIFO_EMPTY_INT		(1<<1)
#define I2C_RXFIFO_FULL_INT			(1<<0)


RingBuffer *I2C0_Data;
RingBuffer *I2C1_Data;

void SetupProtocols()
{
	AllocInt_Custom(ETS_I2C_EXT0_INTR_SOURCE, i2c0_master_isr);
}

void AllocInt_Custom(int source, void(*handler)())
{
	esp_intr_alloc_intrstatus(source, /*flags*/0, 0, 0, handler, /*arg*/NULL, NULL);
}

IRAM_ATTR void i2c0_master_isr(void *param)
{
	uint32_t status = REG_READ(I2C_INT_STATUS_REG);
	//ets_printf("INT: %u\n", status);
	
	while(status != 0)
	{
		//uart_write_bytes(UART_NUM_0, "i", 1);
		status = REG_READ(I2C_INT_STATUS_REG);
        
		if (status & I2C_TX_SEND_EMPTY_INT)
		{
			//uart_write_bytes(UART_NUM_0, "A", 1);
			I2C0_CLI(I2C_TX_SEND_EMPTY_INT);
		}
		else if (status & I2C_RX_REC_FULL_INT)
		{
			//uart_write_bytes(UART_NUM_0, "B", 1);
			I2C0_CLI(I2C_RX_REC_FULL_INT);
		}
		else if (status & I2C_ACK_ERR_INT)
		{
			//uart_write_bytes(UART_NUM_0, "C", 1);
			I2C0_CLI(I2C_ACK_ERR_INT);
			//if (p_i2c->mode == I2C_MODE_MASTER)
			//{
			//	p_i2c_obj[i2c_num]->status = I2C_STATUS_ACK_ERROR;
			//	I2C[i2c_num]->int_clr.ack_err = 1;
			//	//get error ack value from slave device, stop the commands
			//	i2c_master_cmd_begin_static(i2c_num);
			//}
		}
		else if (status & I2C_TRANS_START_INT)
		{
			//uart_write_bytes(UART_NUM_0, "D", 1);
			//ets_printf("INT: I2C_TRANS_START_INT\n");
			I2C0_CLI(I2C_TRANS_START_INT);
		}
		else if (status & I2C_TIME_OUT_INT)
		{
			//uart_write_bytes(UART_NUM_0, "E", 1);
			I2C0_CLI(I2C_TIME_OUT_INT);
		}
		else if (status & I2C_TRANS_COMPLETE_INT)
		{
			//uart_write_bytes(UART_NUM_0, "F", 1);
			//ets_printf("INT: I2C_TRANS_COMPLETE_INT\n");
			I2C0_CLI(I2C_TRANS_COMPLETE_INT);
			//if (p_i2c->mode == I2C_MODE_SLAVE)
			//{
			//	int rx_fifo_cnt = I2C[i2c_num]->status_reg.rx_fifo_cnt;
			//	for (idx = 0; idx < rx_fifo_cnt; idx++)
			//	{
			//		p_i2c->data_buf[idx] = I2C[i2c_num]->fifo_data.data;
			//	}
			//	xRingbufferSendFromISR(p_i2c->rx_ring_buf, p_i2c->data_buf, rx_fifo_cnt, &HPTaskAwoken);
			//	I2C[i2c_num]->int_clr.rx_fifo_full = 1;
			//}
			//else
			//{
			//	// add check for unexcepted situations caused by noise.
			//	if(p_i2c->status != I2C_STATUS_ACK_ERROR && p_i2c->status != I2C_STATUS_IDLE)
			//	{
			//		i2c_master_cmd_begin_static(i2c_num);
			//	}
			//}
		}
		else if (status & I2C_MASTER_TRAN_COMP_INT)
		{
			//uart_write_bytes(UART_NUM_0, "G", 1);
			I2C0_CLI(I2C_MASTER_TRAN_COMP_INT);
		}
		else if (status & I2C_ARBITRATION_LOST_INT)
		{
			//uart_write_bytes(UART_NUM_0, "H", 1);
			I2C0_CLI(I2C_ARBITRATION_LOST_INT);
		}
		else if (status & I2C_SLAVE_TRAN_COMP_INT)
		{
			//uart_write_bytes(UART_NUM_0, "I", 1);
			I2C0_CLI(I2C_SLAVE_TRAN_COMP_INT);
		}
		else if (status & I2C_END_DETECT_INT)
		{
			//uart_write_bytes(UART_NUM_0, "J", 1);
			//ets_printf("INT: I2C_END_DETECT_INT\n");
			I2C0_CLI(I2C_END_DETECT_INT);
		}
		else if (status & I2C_RXFIFO_OVF_INT)
		{
			//uart_write_bytes(UART_NUM_0, "K", 1);
			I2C0_CLI(I2C_RXFIFO_OVF_INT);
		}
		else if (status & I2C_TXFIFO_EMPTY_INT)
		{
			//uart_write_bytes(UART_NUM_0, "L", 1);
			int tx_fifo_space_rem = 32 - ((REG_READ(0x3FF53008) >> 18) & 0x3F);
			//ets_printf("TXFIFO Empty!\n");
			//ets_printf("RB: %p, Len: %u, S: %u, E: %u, Rem: %u\n", I2C0_Data, I2C0_Data->length, I2C0_Data->readCursor, I2C0_Data->writeCursor, tx_fifo_space_rem);
			//REG_OR(I2C0_FIFO_CONF_REG, (1 << 12) | (1 << 13));  // reset rx/tx fifo
			//REG_AND(I2C0_FIFO_CONF_REG, ~((1 << 12) | (1 << 13)));
			for(int i = 0 ; i < MIN(tx_fifo_space_rem, RBP_REM(I2C0_Data)) ; i++)
			{
				REG_SET(I2C0_FIFO_ADDR, RBP_READ(I2C0_Data));
			}
			
			if (RBP_EMPTY(I2C0_Data))
			{
				I2C0_Data = NULL;
				REG_AND(I2C0_INT_ENA_REG, ~I2C_TXFIFO_EMPTY_INT);  // Disable int.
				//ets_printf("NULLed\n");
				//uart_write_bytes(UART_NUM_0, "n", 1);
			}
			
			I2C0_CLI(I2C_TXFIFO_EMPTY_INT);
			
			//int tx_fifo_rem = I2C_FIFO_LEN - I2C[i2c_num]->status_reg.tx_fifo_cnt;
			//size_t size = 0;
			//uint8_t *data = (uint8_t*) xRingbufferReceiveUpToFromISR(p_i2c->tx_ring_buf, &size, tx_fifo_rem);
			//if (data)
			//{
			//	ets_printf("INT: I2C_TXFIFO_EMPTY_INT; data:\n");
			//	for (idx = 0; idx < size; idx++)
			//	{
			//		//ets_printf("data[%u]: 0x%x\n", idx, data[idx]);
			//		WRITE_PERI_REG(I2C_DATA_APB_REG(i2c_num), data[idx]);
			//	}
			//	vRingbufferReturnItemFromISR(p_i2c->tx_ring_buf, data, &HPTaskAwoken);
			//	I2C[i2c_num]->int_ena.tx_fifo_empty = 1;
			//}
			//else
			//{
			//	ets_printf("INT: I2C_TXFIFO_EMPTY_INT; no data\n");
			//	I2C[i2c_num]->int_ena.tx_fifo_empty = 0;
			//}
		}
		else if (status & I2C_RXFIFO_FULL_INT)
		{
			//uart_write_bytes(UART_NUM_0, "M", 1);
			//int rx_fifo_cnt = I2C[i2c_num]->status_reg.rx_fifo_cnt;
			//for (idx = 0; idx < rx_fifo_cnt; idx++)
			//{
			//	p_i2c->data_buf[idx] = I2C[i2c_num]->fifo_data.data;
			//}
			//xRingbufferSendFromISR(p_i2c->rx_ring_buf, p_i2c->data_buf, rx_fifo_cnt, &HPTaskAwoken);
			I2C0_CLI(I2C_RXFIFO_FULL_INT);
		}
		else
		{
			//uart_write_bytes(UART_NUM_0, "X", 1);
			I2C0_CLI(status);  // clear all
		}

		//uart_putc(UART_NUM_0, 'i');
		//uart_write_bytes(UART_NUM_0, "i", 1);
		//putchar('i');
		//uart_tx_all(UART_NUM_0, "i", 0, 0, 0);
	}
}