#define REG_SET(R,V) *((volatile uint32_t*)(R))=(V)
#define REG_OR(R,V) *((volatile uint32_t*)(R))|=(V)
#define REG_AND(R,V) *((volatile uint32_t*)(R))&=(V)
#define cREG_READ(R) *((volatile uint32_t*)(R))
#define I2C0_CMD0_REG 0x3FF53058
#define I2C0_CMD1_REG 0x3FF5305C
#define I2C0_CMD2_REG 0x3FF53060
#define I2C0_CMD3_REG 0x3FF53064
#define I2C0_CMD4_REG 0x3FF53068
#define I2C0_CMD5_REG 0x3FF5306C
#define I2C0_CMD6_REG 0x3FF53070
#define I2C0_CMD7_REG 0x3FF53074
#define I2C0_CMD8_REG 0x3FF53078
#define I2C0_CMD9_REG 0x3FF5307C
#define I2C0_CMD10_REG 0x3FF53080
#define I2C0_CMD11_REG 0x3FF53084
#define I2C0_CMD12_REG 0x3FF53088
#define I2C0_CMD13_REG 0x3FF5308C
#define I2C0_CMD14_REG 0x3FF53090
#define I2C0_CMD15_REG 0x3FF53094

//#define I2C0_BASE_REG 0x3FF40000 + 13000
#define I2C0_BASE_REG 0x60000000 + 13000
#define I2C0_RAM_ADDR I2C0_BASE_REG + 0x100
#define I2C0_FIFO_ADDR I2C0_BASE_REG + 0x1C
#define I2C0_FIFO_CONF_REG I2C0_BASE_REG + 0x18



/* i2c - Example
   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples
   See README.md file to get detailed usage of this example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO 26               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 27               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(1) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO 22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(0) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 1e6        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ESP_SLAVE_ADDR 0x53		/*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */











#include "i2c_reg.h"





#include "soc/i2c_struct.h"

#define I2C_ENTER_CRITICAL_ISR(mux)    portENTER_CRITICAL_ISR(mux)
#define I2C_EXIT_CRITICAL_ISR(mux)     portEXIT_CRITICAL_ISR(mux)
#define I2C_ENTER_CRITICAL(mux)        portENTER_CRITICAL(mux)
#define I2C_EXIT_CRITICAL(mux)         portEXIT_CRITICAL(mux)

#define I2C_DRIVER_ERR_STR             "i2c driver install error"
#define I2C_DRIVER_MALLOC_ERR_STR      "i2c driver malloc error"
#define I2C_NUM_ERROR_STR              "i2c number error"
#define I2C_TIMEING_VAL_ERR_STR        "i2c timing value error"
#define I2C_ADDR_ERROR_STR             "i2c null address error"
#define I2C_DRIVER_NOT_INSTALL_ERR_STR "i2c driver not installed"
#define I2C_SLAVE_BUFFER_LEN_ERR_STR   "i2c buffer size too small for slave mode"
#define I2C_EVT_QUEUE_ERR_STR          "i2c evt queue error"
#define I2C_SEM_ERR_STR                "i2c semaphore error"
#define I2C_BUF_ERR_STR                "i2c ringbuffer error"
#define I2C_MASTER_MODE_ERR_STR        "Only allowed in master mode"
#define I2C_MODE_SLAVE_ERR_STR         "Only allowed in slave mode"
#define I2C_CMD_MALLOC_ERR_STR         "i2c command link malloc error"
#define I2C_TRANS_MODE_ERR_STR         "i2c trans mode error"
#define I2C_MODE_ERR_STR               "i2c mode error"
#define I2C_SDA_IO_ERR_STR             "sda gpio number error"
#define I2C_SCL_IO_ERR_STR             "scl gpio number error"
#define I2C_CMD_LINK_INIT_ERR_STR      "i2c command link error"
#define I2C_GPIO_PULLUP_ERR_STR        "this i2c pin does not support internal pull-up"
#define I2C_ACK_TYPE_ERR_STR           "i2c ack type error"
#define I2C_DATA_LEN_ERR_STR           "i2c data read length error"
#define I2C_PSRAM_BUFFER_WARN_STR      "Using buffer allocated from psram"
#define I2C_FIFO_FULL_THRESH_VAL       (28)
#define I2C_FIFO_EMPTY_THRESH_VAL      (5)
#define I2C_IO_INIT_LEVEL              (1)
#define I2C_CMD_ALIVE_INTERVAL_TICK    (1000 / portTICK_PERIOD_MS)
#define I2C_CMD_EVT_ALIVE              (0)
#define I2C_CMD_EVT_DONE               (1)
#define I2C_EVT_QUEUE_LEN              (1)
#define I2C_SLAVE_TIMEOUT_DEFAULT      (32000)     /* I2C slave timeout value, APB clock cycle number */
#define I2C_SLAVE_SDA_SAMPLE_DEFAULT   (10)        /* I2C slave sample time after scl positive edge default value */
#define I2C_SLAVE_SDA_HOLD_DEFAULT     (10)        /* I2C slave hold time after scl negative edge default value */
#define I2C_MASTER_TOUT_CNUM_DEFAULT   (8)         /* I2C master timeout cycle number of I2C clock, after which the timeout interrupt will be triggered */
#define I2C_ACKERR_CNT_MAX             (10)
#define I2C_FILTER_CYC_NUM_DEF         (7)         /* The number of apb cycles filtered by default*/

typedef struct
{
	uint8_t byte_num; /*!< cmd byte number */
	uint8_t ack_en; /*!< ack check enable */
	uint8_t ack_exp; /*!< expected ack level to get */
	uint8_t ack_val; /*!< ack value to send */
	uint8_t* data; /*!< data address */
	uint8_t byte_cmd; /*!< to save cmd for one byte command mode */
	i2c_opmode_t op_code; /*!< haredware cmd type */
}i2c_cmd_t;

typedef struct i2c_cmd_link
{
	i2c_cmd_t cmd; /*!< command in current cmd link */
	struct i2c_cmd_link *next;  /*!< next cmd link */
} i2c_cmd_link_t;

typedef struct
{
	i2c_cmd_link_t* head; /*!< head of the command link */
	i2c_cmd_link_t* cur; /*!< last node of the command link */
	i2c_cmd_link_t* free; /*!< the first node to free of the command link */
} i2c_cmd_desc_t;

typedef enum
{
	I2C_STATUS_READ, /*!< read status for current master command */
	I2C_STATUS_WRITE, /*!< write status for current master command */
	I2C_STATUS_IDLE, /*!< idle status for current master command */
	I2C_STATUS_ACK_ERROR,
	/*!< ack error status for current master command */
	I2C_STATUS_DONE, /*!< I2C command done */
	I2C_STATUS_TIMEOUT,
	/*!< I2C bus status error, and operation timeout */
} i2c_status_t;

typedef struct
{
	int type;
} i2c_cmd_evt_t;

typedef struct
{
	int i2c_num; /*!< I2C port number */
	int mode; /*!< I2C mode, master or slave */
	intr_handle_t intr_handle; /*!< I2C interrupt handle*/
	int cmd_idx; /*!< record current command index, for master mode */
	int status; /*!< record current command status, for master mode */
	int rx_cnt; /*!< record current read index, for master mode */
	uint8_t data_buf[I2C_FIFO_LEN]; /*!< a buffer to store i2c fifo data */

	i2c_cmd_desc_t cmd_link; /*!< I2C command link */
	QueueHandle_t cmd_evt_queue; /*!< I2C command event queue */
#if CONFIG_SPIRAM_USE_MALLOC
	uint8_t* evt_queue_storage; /*!< The buffer that will hold the items in the queue */
	int intr_alloc_flags; /*!< Used to allocate the interrupt */
	StaticQueue_t evt_queue_buffer; /*!< The buffer that will hold the queue structure*/
#endif
	xSemaphoreHandle cmd_mux; /*!< semaphore to lock command process */
	size_t tx_fifo_remain; /*!< tx fifo remain length, for master mode */
	size_t rx_fifo_remain; /*!< rx fifo remain length, for master mode */

	xSemaphoreHandle slv_rx_mux; /*!< slave rx buffer mux */
	xSemaphoreHandle slv_tx_mux; /*!< slave tx buffer mux */
	size_t rx_buf_length; /*!< rx buffer length */
	RingbufHandle_t rx_ring_buf; /*!< rx ringbuffer handler of slave mode */
	size_t tx_buf_length; /*!< tx buffer length */
	RingbufHandle_t tx_ring_buf; /*!< tx ringbuffer handler of slave mode */
} i2c_obj_t;


extern i2c_obj_t *p_i2c_obj[I2C_NUM_MAX];
extern void i2c_isr_handler_default(void* arg);
extern void IRAM_ATTR i2c_master_cmd_begin_static(i2c_port_t i2c_num);
extern esp_err_t IRAM_ATTR i2c_hw_fsm_reset(i2c_port_t i2c_num);


extern portMUX_TYPE i2c_spinlock[I2C_NUM_MAX];
/* DRAM_ATTR is required to avoid I2C array placed in flash, due to accessed from ISR */
extern i2c_dev_t* const I2C[I2C_NUM_MAX];


extern esp_err_t i2c_master_clear_bus(i2c_port_t i2c_num);


















/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
	//i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
	//i2c_master_start(cmd_handle);
	//i2c_master_write_byte(cmd_handle, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_DIS);
	//i2c_master_write(cmd_handle, data_wr, 2, ACK_CHECK_DIS);
	//i2c_master_stop(cmd_handle);
	//esp_err_t ret;
	//ret = i2c_master_cmd_begin(i2c_num, cmd_handle, 1000 / portTICK_RATE_MS);
	//return ret;


	// Sometimes when the FSM get stuck, the ACK_ERR interrupt will occur endlessly until we reset the FSM and clear bus.
	//static uint8_t clear_bus_cnt = 0;
	//ret = ESP_FAIL;
	//i2c_obj_t* p_i2c = p_i2c_obj[i2c_num];
	//portTickType ticks_start = xTaskGetTickCount();
	//portBASE_TYPE res = xSemaphoreTake(p_i2c->cmd_mux, 1000 / portTICK_RATE_MS);
	//if (res == pdFALSE)
	//{
	//	return ESP_ERR_TIMEOUT;
	//}
	//xQueueReset(p_i2c->cmd_evt_queue);
	//if (p_i2c->status == I2C_STATUS_TIMEOUT || I2C[i2c_num]->status_reg.bus_busy == 1)
	//{
	//	i2c_hw_fsm_reset(i2c_num);
	//	clear_bus_cnt = 0;
	//}
	//i2c_reset_tx_fifo(i2c_num);
	I2C[i2c_num]->fifo_conf.tx_fifo_rst = 1;
	I2C[i2c_num]->fifo_conf.tx_fifo_rst = 0;
	
	//i2c_reset_rx_fifo(i2c_num);
	I2C[i2c_num]->fifo_conf.rx_fifo_rst = 1;
	I2C[i2c_num]->fifo_conf.rx_fifo_rst = 0;
	
	//i2c_cmd_desc_t* cmd = (i2c_cmd_desc_t*) cmd_handle;
	//p_i2c->cmd_link.free = cmd->free;
	//p_i2c->cmd_link.cur = cmd->cur;
	//p_i2c->cmd_link.head = cmd->head;
	//p_i2c->status = I2C_STATUS_IDLE;
	//p_i2c->cmd_idx = 0;
	//p_i2c->rx_cnt = 0;
	//p_i2c->tx_fifo_remain = I2C_FIFO_LEN;
	//p_i2c->rx_fifo_remain = I2C_FIFO_LEN;
	//i2c_reset_tx_fifo(i2c_num);
	//i2c_reset_rx_fifo(i2c_num);
	// These two interrupts some times can not be cleared when the FSM gets stuck.
	// so we disable them when these two interrupt occurs and re-enable them here.
	I2C[i2c_num]->int_ena.ack_err = 1;
	I2C[i2c_num]->int_ena.time_out = 1;


	//start send commands, at most 32 bytes one time, isr handler will process the remaining commands.
	//i2c_master_cmd_begin_static(i2c_num);








	

//	while (p_i2c->cmd_link.head)
//	{
//		i2c_cmd_t *cmd = &p_i2c->cmd_link.head->cmd;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].val = 0;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].ack_en = cmd->ack_en;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].ack_exp = cmd->ack_exp;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].ack_val = cmd->ack_val;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].byte_num = cmd->byte_num;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].op_code = cmd->op_code;
//		if (cmd->op_code == I2C_CMD_WRITE)
//		{
//			uint32_t wr_filled = 0;
//			//TODO: to reduce interrupt number
//			if(cmd->data)
//			{
//				while (p_i2c->tx_fifo_remain > 0 && cmd->byte_num > 0)
//				{
//					REG_SET(I2C_DATA_APB_REG(i2c_num), *cmd->data++);
//					p_i2c->tx_fifo_remain--;
//					cmd->byte_num--;
//					wr_filled++;
//				}
//			} else
//			{
//				REG_SET(I2C_DATA_APB_REG(i2c_num), cmd->byte_cmd);
//				p_i2c->tx_fifo_remain--;
//				cmd->byte_num--;
//				wr_filled++;
//			}
//			//Workaround for register field operation.
//			I2C[i2c_num]->command[p_i2c->cmd_idx].byte_num = wr_filled;
//			I2C[i2c_num]->command[p_i2c->cmd_idx + 1].val = 0;
//			I2C[i2c_num]->command[p_i2c->cmd_idx + 1].op_code = I2C_CMD_END;
//			p_i2c->tx_fifo_remain = I2C_FIFO_LEN;
//			p_i2c->cmd_idx = 0;
//			if (cmd->byte_num > 0)
//			{
//			}
//			else
//			{
//				p_i2c->cmd_link.head = p_i2c->cmd_link.head->next;
//			}
//			p_i2c->status = I2C_STATUS_WRITE;
//			break;
//		}
//		else if (cmd->op_code == I2C_CMD_READ)
//		{
//			//TODO: to reduce interrupt number
//			p_i2c->rx_cnt = cmd->byte_num > p_i2c->rx_fifo_remain ? p_i2c->rx_fifo_remain : cmd->byte_num;
//			cmd->byte_num -= p_i2c->rx_cnt;
//			I2C[i2c_num]->command[p_i2c->cmd_idx].byte_num = p_i2c->rx_cnt;
//			I2C[i2c_num]->command[p_i2c->cmd_idx].ack_val = cmd->ack_val;
//			I2C[i2c_num]->command[p_i2c->cmd_idx + 1].val = 0;
//			I2C[i2c_num]->command[p_i2c->cmd_idx + 1].op_code = I2C_CMD_END;
//			p_i2c->status = I2C_STATUS_READ;
//			break;
//		}
//		else
//		{
//		}
//		p_i2c->cmd_idx++;
//		p_i2c->cmd_link.head = p_i2c->cmd_link.head->next;
//		if (p_i2c->cmd_link.head == NULL || p_i2c->cmd_idx >= 15)
//		{
//			p_i2c->tx_fifo_remain = I2C_FIFO_LEN;
//			p_i2c->cmd_idx = 0;
//			break;
//		}
//	}
	I2C[i2c_num]->int_clr.end_detect = 1;
	I2C[i2c_num]->int_ena.end_detect = 1;
	I2C[i2c_num]->ctr.trans_start = 0;





	//ESP_LOGI("TRANS", "CMD 0: 0x%x", cREG_READ(I2C0_CMD0_REG));
	//ESP_LOGI("TRANS", "CMD 1: 0x%x", cREG_READ(I2C0_CMD1_REG));
	//ESP_LOGI("TRANS", "CMD 2: 0x%x", cREG_READ(I2C0_CMD2_REG));
	//ESP_LOGI("TRANS", "CMD 3: 0x%x", cREG_READ(I2C0_CMD3_REG));
	//ESP_LOGI("TRANS", "CMD 4: 0x%x", cREG_READ(I2C0_CMD4_REG));
	//ESP_LOGI("TRANS", "CMD 5: 0x%x", cREG_READ(I2C0_CMD5_REG));
	//ESP_LOGI("TRANS", "CMD 6: 0x%x", cREG_READ(I2C0_CMD6_REG));
	//ESP_LOGI("TRANS", "CMD 7: 0x%x", cREG_READ(I2C0_CMD7_REG));
	//ESP_LOGI("TRANS", "CMD 8: 0x%x", cREG_READ(I2C0_CMD8_REG));
	//ESP_LOGI("TRANS", "CMD 9: 0x%x", cREG_READ(I2C0_CMD9_REG));
	//ESP_LOGI("TRANS", "CMD 10: 0x%x", cREG_READ(I2C0_CMD10_REG));
	//ESP_LOGI("TRANS", "CMD 11: 0x%x", cREG_READ(I2C0_CMD11_REG));
	//ESP_LOGI("TRANS", "CMD 12: 0x%x", cREG_READ(I2C0_CMD12_REG));
	//ESP_LOGI("TRANS", "CMD 13: 0x%x", cREG_READ(I2C0_CMD13_REG));
	//ESP_LOGI("TRANS", "CMD 14: 0x%x", cREG_READ(I2C0_CMD14_REG));
	//ESP_LOGI("TRANS", "CMD 15: 0x%x", cREG_READ(I2C0_CMD15_REG));
	
	//for (uint32_t i = 0; i < 32; i++) // 262143
	//{
	//
	//	//ESP_LOGI("TRANS", "RAM %u READ: 0x%x", i, cREG_READ(I2C0_RAM_ADDR + 4*i));
	//	//ESP_LOGI("TRANS", "RAM %u SET : 0x%x", i, REG_SET(I2C0_RAM_ADDR + 4*i, 0x7F7F7F7F));
	//	//printf("%x,", cREG_READ(0x60000000 + 4*i));
	//	//if(i % 1024 == 0)
	//	//	printf("\n");
	//}

	//i2c_reset_tx_fifo(0);

	
	REG_OR(I2C0_BASE_REG + 0x0018, 0x1<<10); // enable apb non-fifo access
	
	ESP_LOGI("TRANS", "TX FIFO count: %u", 0x3F & (cREG_READ(I2C0_BASE_REG+ 0x08)>>18));
	ESP_LOGI("TRANS", "RX FIFO count: %u", 0x3F & (cREG_READ(I2C0_BASE_REG+ 0x08)>>8));
	ESP_LOGI("TRANS", "FIFO ?? count: %u", 0xF & (cREG_READ(I2C0_BASE_REG + 0x08) >> 14));
	for(uint32_t i = 0 ; i < 32 ; i++)
	{
		ESP_LOGI("TRANS", "FIFO %u READ: 0x%x", i, cREG_READ(I2C0_FIFO_ADDR));
	}

	REG_AND(I2C0_FIFO_CONF_REG, ~(0x1<<10));  // disable apb non-fifo access
	REG_OR(I2C0_FIFO_CONF_REG, 1 << 12);
	REG_OR(I2C0_FIFO_CONF_REG, 1 << 13);
	REG_AND(I2C0_FIFO_CONF_REG, ~(1 << 12));
	REG_AND(I2C0_FIFO_CONF_REG, ~(1 << 13));

	//for (uint32_t i = 0; i < 32; i++)
	//{
	//	ESP_LOGI("TRANS", "FIFO %u WRITE: 0x%x", i, REG_WRITE(I2C0_FIFO_ADDR, i+10));
	//}
	//for (uint32_t i = 0; i < 32; i++)
	//{
	//	ESP_LOGI("TRANS", "FIFO %u READ: 0x%x", i, cREG_READ(I2C0_FIFO_ADDR));
	//}
	for (uint32_t i = 0; i < 32; i++)
	{
		ESP_LOGI("tr", "nonFIFO %u READ: 0x%x", i, cREG_READ(I2C0_BASE_REG + 0xF8));
		ESP_LOGI("tr", "nonFIFO %u READ: 0x%x", i, cREG_READ(I2C0_BASE_REG + 0xFC));
		ESP_LOGI("tr", "nonFIFO %u READ: 0x%x", i, cREG_READ(I2C0_BASE_REG + 0x100 + 4U*i));
	}

	//REG_SET(I2C_DATA_APB_REG(i2c_num), 0x45<<1);
	//REG_SET(I2C_DATA_APB_REG(i2c_num), 0x1A);
	//REG_SET(I2C_DATA_APB_REG(i2c_num), 0x1B);
	//REG_SET(I2C_DATA_APB_REG(i2c_num), 0x1C);
	
	ESP_LOGI("TRANS", "TX FIFO count: %u", 0x3F & (cREG_READ(I2C0_BASE_REG + 0x08) >> 18));
	ESP_LOGI("TRANS", "RX FIFO count: %u", 0x3F & (cREG_READ(I2C0_BASE_REG + 0x08) >> 8));
	ESP_LOGI("TRANS", "FIFO ?? count: %u", 0xF & (cREG_READ(I2C0_BASE_REG + 0x08) >> 14));
	for (uint32_t i = 0; i < 32; i++)
	{
		ESP_LOGI("TRANS", "FIFO %u READ: 0x%x", i, cREG_READ(I2C0_FIFO_ADDR));
	}
	for (uint32_t i = 0; i < 32; i++)
	{
		ESP_LOGI("tr", "nonFIFO %u READ: 0x%x", i, cREG_READ(I2C0_BASE_REG + 0xF8));
		ESP_LOGI("tr", "nonFIFO %u READ: 0x%x", i, cREG_READ(I2C0_BASE_REG + 0xFC));
		ESP_LOGI("tr", "nonFIFO %u READ: 0x%x", i, cREG_READ(I2C0_BASE_REG + 0x100 + 4U*i));
	}



	
#define I2C_CMD_RSTART 0	// RSTART command to control the transmission of a START or RESTART I2C condition.
#define I2C_CMD_WRITE 1		// WRITE command for the I2C Master to transmit data.
#define I2C_CMD_READ 2		// READ command for the I2C Master to receive data.
#define I2C_CMD_STOP 3		// STOP command to control the transmission of a STOP I2C condition.
#define I2C_CMD_END 4		// END command for continuous data transmission. When the END command is
	REG_SET(I2C0_CMD0_REG, 0x3FFF & ((I2C_CMD_RSTART << 11) | (1 << 10) | (0 << 8)));			// Prepare commands
	REG_SET(I2C0_CMD1_REG, 0x3FFF & ((I2C_CMD_WRITE << 11) | (1 << 10) | (0 << 8) | (1+2)));	// 1 address + 2 data
	REG_SET(I2C0_CMD2_REG, 0x3FFF & ((I2C_CMD_STOP << 11) | (1 << 10) | (0 << 8)));				//	
	//REG_SET(I2C0_CMD3_REG, 0x3FFF & ((I2C_CMD_END << 11) | (1 << 10) | (0 << 8))); 				//
					WRITE_PERI_REG(I2C_DATA_APB_REG(i2c_num), 0x75);
					WRITE_PERI_REG(I2C_DATA_APB_REG(i2c_num), 0xA7);
					WRITE_PERI_REG(I2C_DATA_APB_REG(i2c_num), 0xB7);

	I2C[i2c_num]->ctr.trans_start = 1;
	ets_delay_us(100);
	I2C[i2c_num]->ctr.trans_start = 0;
	


	
//	int clear_bus_cnt = 0;
//	TickType_t ticks_start = xTaskGetTickCount();
//	// Wait event bits
//	i2c_cmd_evt_t evt;
//	while (1)
//	{
//		TickType_t wait_time = xTaskGetTickCount();
//		if (wait_time - ticks_start > (1000 / portTICK_RATE_MS))
//		{ // out of time
//		    wait_time = I2C_CMD_ALIVE_INTERVAL_TICK;
//		}
//		else
//		{
//			wait_time = (1000 / portTICK_RATE_MS) - (wait_time - ticks_start);
//			if (wait_time < I2C_CMD_ALIVE_INTERVAL_TICK)
//			{
//				wait_time = I2C_CMD_ALIVE_INTERVAL_TICK;
//			}
//		}
//		 In master mode, since we don't have an interrupt to detective bus error or FSM state, what we do here is to make
//		 sure the interrupt mechanism for master mode is still working.
//		 If the command sending is not finished and there is no interrupt any more, the bus is probably dead caused by external noise.
//		portBASE_TYPE evt_res = xQueueReceive(p_i2c->cmd_evt_queue, &evt, wait_time);
//		if (evt_res == pdTRUE)
//		{
//			if (evt.type == I2C_CMD_EVT_DONE)
//			{
//				if (p_i2c->status == I2C_STATUS_TIMEOUT)
//				{
//					// If the I2C slave are powered off or the SDA/SCL are connected to ground, for example,
//					// I2C hw FSM would get stuck in wrong state, we have to reset the I2C module in this case.
//					i2c_hw_fsm_reset(i2c_num);
//					clear_bus_cnt = 0;
//					ret = ESP_ERR_TIMEOUT;
//				}
//				else if (p_i2c->status == I2C_STATUS_ACK_ERROR)
//				{
//					clear_bus_cnt++;
//					if (clear_bus_cnt >= I2C_ACKERR_CNT_MAX)
//					{
//						i2c_master_clear_bus(i2c_num);
//						clear_bus_cnt = 0;   
//					}
//					ret = ESP_FAIL;
//				}
//				else
//				{
//					ret = ESP_OK;
//				}
//				break;
//			}
//			if (evt.type == I2C_CMD_EVT_ALIVE)
//			{
//			}
//		}
//		else
//		{
//			ret = ESP_ERR_TIMEOUT;
//			// If the I2C slave are powered off or the SDA/SCL are connected to ground, for example,
//			// I2C hw FSM would get stuck in wrong state, we have to reset the I2C module in this case.
//			i2c_hw_fsm_reset(i2c_num);
//			clear_bus_cnt = 0;
//			break;
//		}
//	}
//	p_i2c->status = I2C_STATUS_DONE;
//	xSemaphoreGive(p_i2c->cmd_mux);
	
	ets_delay_us(20);
	//i2c_cmd_link_delete(cmd);


    //i2c_master_cmd_begin_static(i2c_num);

////	while (p_i2c->cmd_link.head)
//	{
//		i2c_cmd_t *cmd = &p_i2c->cmd_link.head->cmd;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].val = 0;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].ack_en = cmd->ack_en;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].ack_exp = cmd->ack_exp;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].ack_val = cmd->ack_val;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].byte_num = cmd->byte_num;
//		I2C[i2c_num]->command[p_i2c->cmd_idx].op_code = cmd->op_code;
////		if (cmd->op_code == I2C_CMD_WRITE)
////		{
//			uint32_t wr_filled = 0;
////			//TODO: to reduce interrupt number
////			if(cmd->data)
////			{
//				while (p_i2c->tx_fifo_remain > 0 && cmd->byte_num > 0)
//				{
//					WRITE_PERI_REG(I2C_DATA_APB_REG(i2c_num), *cmd->data++);
//					p_i2c->tx_fifo_remain--;
//					cmd->byte_num--;
//					wr_filled++;
//				}
////			}
////			//Workaround for register field operation.
//			I2C[i2c_num]->command[p_i2c->cmd_idx].byte_num = wr_filled;
//			I2C[i2c_num]->command[p_i2c->cmd_idx + 1].val = 0;
//			I2C[i2c_num]->command[p_i2c->cmd_idx + 1].op_code = I2C_CMD_END;
////			p_i2c->tx_fifo_remain = I2C_FIFO_LEN;
//			p_i2c->cmd_idx = 0;
////			
//			p_i2c->cmd_link.head = p_i2c->cmd_link.head->next;
////			p_i2c->status = I2C_STATUS_WRITE;
////			break;
////		}
////	
////		p_i2c->cmd_idx++;
////		p_i2c->cmd_link.head = p_i2c->cmd_link.head->next;
////		
//	}
//	I2C[i2c_num]->int_clr.end_detect = 1;
//	I2C[i2c_num]->int_ena.end_detect = 1;
//	I2C[i2c_num]->ctr.trans_start = 0;
//	I2C[i2c_num]->ctr.trans_start = 1;





























	//ets_delay_us(60);
    //i2c_master_cmd_begin_static(i2c_num);
	
	//I2C[i2c_num]->int_clr.trans_complete = 1;
	//// add check for unexcepted situations caused by noise.
	//if (p_i2c->status != I2C_STATUS_ACK_ERROR && p_i2c->status != I2C_STATUS_IDLE)
	//{
	//	i2c_master_cmd_begin_static(i2c_num);
	//}
	//ets_delay_us(20);


	//i2c_cmd_desc_t* cmd2 = (i2c_cmd_desc_t*) cmd;
	//while (cmd2->free)
	//{
	//	i2c_cmd_link_t* ptmp = cmd2->free;
	//	cmd2->free = cmd2->free->next;
	//	free(ptmp);
	//}
	//cmd2->cur = NULL;
	//cmd2->free = NULL;
	//cmd2->head = NULL;
	//free(cmd);
	

	return ESP_OK;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
	int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	i2c_param_config(i2c_master_port, &conf);
	esp_err_t ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	//REG_OR(I2C0_BASE_REG + 0x0018, 0x1<<10); // enable apb non-fifo access
	return ret;
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
	int i;
	for (i = 0; i < len; i++)
	{
		printf("%02x ", buf[i]);
		if ((i + 1) % 16 == 0)
		{
			printf("\n");
		}
	}
	printf("\n");
}

static void i2c_test_task(void *arg)
{
	int i = 0;
	int ret;
	uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);
	int cnt = 0;
	int size;
	for (i = 0; i < DATA_LENGTH; i++)
	{
		data_wr[i] = i + 10;
	}
	//we need to fill the slave buffer so that master can read later
	ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, RW_TEST_LENGTH);
	vTaskDelete(NULL);
}

void app_main_2(void)
{
	ESP_ERROR_CHECK(i2c_master_init());
	xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}