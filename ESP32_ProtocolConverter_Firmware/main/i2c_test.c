
#include "esp_log.h"
#include "string.h"
#include "driver/include/driver/i2c.h"

#define I2C0_BASE_REG 0x3FF53000
#define I2C1_BASE_REG 0x3FF57000
#define I2C0_APB_BASE_REG 0x60013000
#define I2C1_APB_BASE_REG 0x60017000

#define I2C0_CTR_REG 0x3FF53004
#define I2C0_SR_REG 0x3FF53008
#define I2C0_TO_REG 0x3FF5300C

#define I2C0_SLAVE_ADDR_REG 0x3FF53010
#define I2C0_RXFIFO_ST_REG 0x3FF53014
#define I2C0_FIFO_CONF_REG 0x3FF53018

#define I2C0_SDA_HOLD_REG 0x3FF53030
#define I2C0_SDA_SAMPLE_REG 0x3FF53034
#define I2C0_SCL_LOW_PERIOD_REG 0x3FF53000 // first in memory
#define I2C0_SCL_HIGH_PERIOD_REG 0x3FF53038
#define I2C0_SCL_START_HOLD_REG 0x3FF53040
#define I2C0_SCL_RSTART_SETUP_REG 0x3FF53044
#define I2C0_SCL_STOP_HOLD_REG 0x3FF53048
#define I2C0_SCL_STOP_SETUP_REG 0x3FF5304C

#define I2C0_SCL_FILTER_CFG_REG 0x3FF53050
#define I2C0_SDA_FILTER_CFG_REG 0x3FF53054

#define I2C0_INT_RAW_REG 0x3FF53020
#define I2C0_INT_ENA_REG 0x3FF53028
#define I2C0_INT_CLR_REG 0x3FF53024
#define I2C_INT_STATUS_REG 0x3FF5302C // This is the one to read

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

#define I2C0_CLI(INT) REG_SET(I2C0_INT_CLR_REG, (INT))


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

#define I2C0_RAM_ADDR (I2C0_APB_BASE_REG + 0x100)
#define I2C0_FIFO_ADDR (I2C0_APB_BASE_REG + 0x1C)

#define I2C_CMD_RSTART 0	// RSTART command to control the transmission of a START or RESTART I2C condition.
#define I2C_CMD_WRITE 1		// WRITE command for the I2C Master to transmit data.
#define I2C_CMD_READ 2		// READ command for the I2C Master to receive data.
#define I2C_CMD_STOP 3		// STOP command to control the transmission of a STOP I2C condition.
#define I2C_CMD_END 4		// END command for continuous data transmission. When the END command is

#define REG_SET(R,V) *((volatile uint32_t*)(R))=(V)
#define REG_OR(R,V) *((volatile uint32_t*)(R))|=(V)
#define REG_AND(R,V) *((volatile uint32_t*)(R))&=(V)
#define REG_READ(R) *((volatile uint32_t*)(R))

#define DEBUG_PIN 2

const int i2c_num = 0;
const uint16_t sda_io_num = 21;
const uint16_t scl_io_num = 22;

#define RBP_READ(RB) RB->data[RB->readCursor=((RB->readCursor+1) % RB->length)]
#define RBP_WRITE(RB,V) RB->data[RB->writeCursor=((RB->writeCursor+1) % RB->length)]=(V)
#define RB_READ(RB) (RB).data[(RB).readCursor=(((RB).readCursor+1) % (RB).length)]
#define RB_WRITE(RB,V) (RB).data[(RB).writeCursor=(((RB).writeCursor+1) % (RB).length)]=(V)
typedef struct
{
	uint16_t length;
	uint16_t writeCursor;
	uint16_t readCursor;
	uint8_t data[];
} RingBuffer;					// Rule: Buffer can only have lengths that are powers of two, so that there is no overhead when reading or writing across the boundary (readCursor%=length is a (NOP with) truncation)

RingBuffer *I2C_Data;

void i2c_exec();
void i2c_init();

void i2c_exec()
{
	//gpio_config_t io_conf;
	//io_conf.intr_type = GPIO_INTR_DISABLE;
	//io_conf.mode = GPIO_MODE_OUTPUT;
	//io_conf.pin_bit_mask = 1 << DEBUG_PIN;
	//io_conf.pull_down_en = 0;
	//io_conf.pull_up_en = 1;
	//gpio_config(&io_conf);
	//
	//
	////gpio_set_level(DEBUG_PIN, 0);
	////gpio_set_pull_mode(DEBUG_PIN, GPIO_PULL_MODE_DOWN);
	////gpio_set_direction(DEBUG_PIN, GPIO_MODE_OUTPUT);
	//gpio_set_level(DEBUG_PIN, 0);
	//
	////ESP_LOGI("I2C", "EXEC START");
	//i2c_init();
	//
	//ets_delay_us(2000);
	//gpio_set_level(DEBUG_PIN, 1);

	uint16_t arraySize = 10;
	I2C_Data = (RingBuffer*)malloc(sizeof(RingBuffer) + sizeof(uint8_t) * (arraySize - 1));
	I2C_Data->length = arraySize;
	I2C_Data->writeCursor = arraySize-1;
	I2C_Data->readCursor = arraySize-1;

	I2C_Data->data[0] = (0x00<<1) | 0x0;
	for (int i = 1; i < I2C_Data->length; i++)
	{
		I2C_Data->data[i] = (i%10) + (0x10*((i%100)/10)) + (0x100*(i/100));
	}
	//I2C_Data->writeCursor = I2C_Data->length-1; //(NOP)


	REG_OR(I2C0_FIFO_CONF_REG, (1 << 12) | (1 << 13)); // reset rx/tx fifo
	REG_AND(I2C0_FIFO_CONF_REG, ~((1 << 12)|(1 << 13)));
	
	REG_AND(I2C0_CTR_REG, ~(1<<5));			// not strictly necessary
	
	//REG_OR(I2C0_BASE_REG + 0x0018, 0x1 << 10);  // enable apb non-fifo access
	//REG_AND(I2C0_FIFO_CONF_REG, ~(0x1 << 10));   // disable apb non-fifo access
	
	
	REG_SET(I2C0_CMD0_REG, 0x3FFF & ((I2C_CMD_RSTART << 11) | (1 << 10) | (0 << 8))); 							// Prepare commands
	int s, commandCount = 1;																					//
	for(s = arraySize ; s > 255 ; s-=255)																		//
	{																											//
		REG_SET(I2C0_CMD0_REG+4U*commandCount, 0x3FFF & ((I2C_CMD_WRITE << 11) | (1 << 10) | (0 << 8) | 255));	//
		commandCount++;																							//
	}																											//
	REG_SET(I2C0_CMD0_REG + 4U*commandCount,  0x3FFF & ((I2C_CMD_WRITE << 11) | (1 << 10) | (0 << 8) | s));		//
	REG_SET(I2C0_CMD0_REG + 4U*(commandCount+1), 0x3FFF & ((I2C_CMD_STOP << 11) | (1 << 10) | (0 << 8))); 		//
	

	//REG_SET(I2C0_FIFO_ADDR, (0x75<<1) | 0x1);
	for (int i = 0; i < 10; i++)
	{
		REG_SET(I2C0_FIFO_ADDR, RBP_READ(I2C_Data));
	}
	
	
	uint32_t ctr = REG_READ(I2C0_CTR_REG);
	REG_SET(I2C0_CTR_REG, ctr | (1<<5)); // Transmission start
	//REG_OR(I2C0_INT_ENA_REG, I2C_TXFIFO_EMPTY_INT);
	ets_delay_us(1000);
	REG_SET(I2C0_CTR_REG, ctr & (~(1<<5))); // Transmission start end

	//gpio_set_level(DEBUG_PIN, 0);
	//ESP_LOGI("I2C", "EXEC END");
}

void i2c_init()
{

//#define DPORT_PERIP_CLK_EN_REG (DR_REG_DPORT_BASE + 0x0C0)
//#define DPORT_PERIP_RST_EN_REG (DR_REG_DPORT_BASE + 0x0C4)
//#define DPORT_I2C_EXT0_CLK_EN (1<<7)
//#define DPORT_I2C_EXT0_RST (1<<7)
//	//i2c_hw_disable(i2c_num);
//	REG_AND(DPORT_PERIP_CLK_EN_REG, ~DPORT_I2C_EXT0_CLK_EN);
//	REG_OR(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT0_RST);
//	//i2c_hw_enable(i2c_num);
//	REG_OR(DPORT_PERIP_CLK_EN_REG, DPORT_I2C_EXT0_CLK_EN);
//	REG_AND(DPORT_PERIP_RST_EN_REG, ~DPORT_I2C_EXT0_RST);






	//ESP_LOGI("I2C", "INIT START");
	REG_SET(I2C0_CTR_REG, 1<<4 | 1<<1 | 1<<0); //  1<<1 | 1<<0 is open drain (0<<1 | 0<<0 doesn't work, even if the docs imply this would be a symmetrically driven output)

	REG_SET(I2C0_SLAVE_ADDR_REG, 0);
	REG_SET(I2C0_FIFO_CONF_REG, 0x3F << 20 | 0x3F << 14); // max data in bytes to be sent: 14*255-1 // (but i2c ram is only 32 bytes long, so this requires ISR trickery)
	REG_SET(I2C0_INT_CLR_REG, 0xFFFFFFFF);

	REG_SET(I2C0_INT_ENA_REG, 0); // For now
	

	const uint8_t i2c_freq_mhz = 1;
	
	// Timing (in ABP Clock cycles)
	#define XTL_CLK_MHZ 160
	#define ABP_CLK_MHZ XTL_CLK_MHZ // period = 6.25 ns

	#define SDA_START_HOLD_TIME_NS 260
	#define SDA_HOLD_TIME_NS 450
	#define SDA_SAMPLE_TIME_NS 130
	//#define SCL_HIGH_PERIOD_NS 260ns	// minimum
	//#define SCL_LOW_PERIOD_NS 500ns	// choose both as 1/2f_scl
	#define SCL_RSTART_SETUP_TIME_NS 260
	#define SCL_START_HOLD_TIME_NS 260
	#define STOP_SETUP_TIME_NS 260
	#define STOP_HOLD_TIME_NS 450

#define NS_TO_ABP(X) (((X)*ABP_CLK_MHZ)/1000)

	REG_SET(I2C0_TO_REG, 0xFFF & NS_TO_ABP(8000 / i2c_freq_mhz));
	REG_SET(I2C0_SDA_HOLD_REG, 0x3FF & NS_TO_ABP(SDA_START_HOLD_TIME_NS));
	REG_SET(I2C0_SDA_SAMPLE_REG, 0x3FF & NS_TO_ABP(SDA_SAMPLE_TIME_NS));
	REG_SET(I2C0_SCL_LOW_PERIOD_REG, 0x3FFF & NS_TO_ABP(500 / i2c_freq_mhz));
	REG_SET(I2C0_SCL_HIGH_PERIOD_REG, 0x3FFF & NS_TO_ABP(500 / i2c_freq_mhz));
	REG_SET(I2C0_SCL_START_HOLD_REG, 0x3FF & NS_TO_ABP(SCL_START_HOLD_TIME_NS));
	REG_SET(I2C0_SCL_RSTART_SETUP_REG, 0x3FF & NS_TO_ABP(SCL_RSTART_SETUP_TIME_NS));
	REG_SET(I2C0_SCL_STOP_HOLD_REG, 0x3FFF & NS_TO_ABP(STOP_HOLD_TIME_NS));
	REG_SET(I2C0_SCL_STOP_SETUP_REG, 0x3FF & NS_TO_ABP(STOP_SETUP_TIME_NS));
	 
	// Input filter
	//REG_SET(I2C0_SCL_FILTER_CFG_REG, );
	//REG_SET(I2C0_SDA_FILTER_CFG_REG, );
	
	///i2c_filter_enable(i2c_num, 7);
    //I2C_ENTER_CRITICAL(&i2c_spinlock[i2c_num]);
	//I2C[i2c_num]->scl_filter_cfg.thres = cyc_num;
	//I2C[i2c_num]->sda_filter_cfg.thres = cyc_num;
	//I2C[i2c_num]->scl_filter_cfg.en = 1;
	//I2C[i2c_num]->sda_filter_cfg.en = 1;

	// GPIO Setup
	
	// I2C only available via gpio matrix, not in IO_MUX

	int sda_in_sig, sda_out_sig, scl_in_sig, scl_out_sig;
	switch (i2c_num)
	{
	case I2C_NUM_1:
		sda_out_sig = I2CEXT1_SDA_OUT_IDX;
		sda_in_sig = I2CEXT1_SDA_IN_IDX;
		scl_out_sig = I2CEXT1_SCL_OUT_IDX;
		scl_in_sig = I2CEXT1_SCL_IN_IDX;
		break;
	case I2C_NUM_0:
	default:
		sda_out_sig = I2CEXT0_SDA_OUT_IDX;
		sda_in_sig = I2CEXT0_SDA_IN_IDX;
		scl_out_sig = I2CEXT0_SCL_OUT_IDX;
		scl_in_sig = I2CEXT0_SCL_IN_IDX;
		break;
	}

	// SDA
	gpio_set_level(sda_io_num, 1);
	if (sda_io_num < 32)
	{
		GPIO.out_w1ts = (0x1 << sda_io_num);
	}
	else
	{
		GPIO.out1_w1ts.val = (0x1 << (sda_io_num - 32));
	}

	//PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[sda_io_num], PIN_FUNC_GPIO);
	////REG_SET_FIELD(REG, 0x7, 2) _r _f _v
	////REG_WRITE(REG, ((REG_READ(REG) & ~((_f ## _V) << (_f ## _S))) | (((_v) & (_f ## _V)) << (_f ## _S))));     
	////REG_WRITE(REG, (REG_READ(REG) & ~(0x7 << 12)) | (((VAL) & 0x7) << 12));
	//REG_WRITE(GPIO_PIN_MUX_REG[sda_io_num], (REG_READ(GPIO_PIN_MUX_REG[sda_io_num]) & ~(0x7 << 12)) | (((PIN_FUNC_GPIO) & 0x7) << 12));
	REG_WRITE(GPIO_PIN_MUX_REG[sda_io_num], 0x00002B00); 



	// gpio_set_direction(sda_io_num, GPIO_MODE_INPUT_OUTPUT_OD); 	// for now

	//REG_OR(GPIO_PIN_MUX_REG[sda_io_num], (1U << 9)); //PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[sda_io_num]);

	if (sda_io_num < 32)
	{
		GPIO.enable_w1ts = (0x1 << sda_io_num);
	}
	else
	{
		GPIO.enable1_w1ts.val = (0x1 << (sda_io_num - 32));
	}

	if (true) // true == OD, false drive high
	{
		GPIO.pin[sda_io_num].pad_driver = 1; // 1 == OD, 0 == drive high
	}
	else
	{
		GPIO.pin[sda_io_num].pad_driver = 0;
	}

	//gpio_set_direction(sda_io_num, GPIO_MODE_INPUT_OUTPUT_OD);	// for now

	//////gpio_set_direction(sda_io_num, GPIO_MODE_OUTPUT);  // this causes a problem somehow
	//////gpio_set_direction(sda_io_num, GPIO_MODE_INPUT_OUTPUT_OD); // for now
	//gpio_set_pull_mode(sda_io_num, GPIO_PULLUP_ONLY);
	//REG_AND(GPIO_PIN_MUX_REG[sda_io_num], ~FUN_PD);
	//REG_OR(GPIO_PIN_MUX_REG[sda_io_num], FUN_PU);
	//gpio_set_pull_mode(sda_io_num, GPIO_FLOATING); // no pullup

	//gpio_matrix_out(sda_io_num, sda_out_sig, 0, 0);
	////gpio_matrix_out(sda_io_num, SIG_GPIO_OUT_IDX, false, false); // WRONG; ("SIG_GPIO_OUT_IDX" resets part of gpio matrix) actual reason: Even though I2C uses normal GPIO functionality (GPIO matrix) to output the signals, the GPIO matrix uses different signals for "true" GPIO and I2C
	REG_SET(GPIO_FUNC0_OUT_SEL_CFG_REG + 4*sda_io_num, sda_out_sig);

	//gpio_matrix_in(sda_io_num, sda_in_sig, 0);
	REG_SET(GPIO_FUNC0_IN_SEL_CFG_REG + 4*sda_in_sig, sda_io_num);
	



	// SCL
	//gpio_set_level(scl_io_num, 1);
	if (scl_io_num < 32)
	{
		GPIO.out_w1ts = (0x1 << scl_io_num);
	}
	else
	{
		GPIO.out1_w1ts.val = (0x1 << (scl_io_num - 32));
	}
	
	//PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[scl_io_num], PIN_FUNC_GPIO);

	//if (GPIO_MODE_INPUT_OUTPUT_OD & GPIO_MODE_DEF_INPUT)
	//{
	//	PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[scl_io_num]); // true
	//}
	//else
	//{
	//	PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[scl_io_num]);
	//}

	if (GPIO_MODE_INPUT_OUTPUT_OD & GPIO_MODE_DEF_OUTPUT)
	{
		if (scl_io_num < 32)
		{
			GPIO.enable_w1ts = (0x1 << scl_io_num); // true
		}
		else
		{
			GPIO.enable1_w1ts.data = (0x1 << (scl_io_num - 32));
		}
	}
	else
	{ 
		if (scl_io_num < 32)
		{
			GPIO.enable_w1tc = (0x1 << scl_io_num);
		}
		else
		{
			GPIO.enable1_w1tc.data = (0x1 << (scl_io_num - 32));
		}

		// Ensure no other output signal is routed via GPIO matrix to this pin
		REG_SET(GPIO_FUNC0_OUT_SEL_CFG_REG + (scl_io_num * 4), SIG_GPIO_OUT_IDX);
	}

	if (GPIO_MODE_INPUT_OUTPUT_OD & GPIO_MODE_DEF_OD)
	{
		GPIO.pin[scl_io_num].pad_driver = 1; // 1 == OD, 0 == drive high
	}
	else
	{
		GPIO.pin[scl_io_num].pad_driver = 0;
	}
	//gpio_set_direction(scl_io_num, GPIO_MODE_OUTPUT); // for now // this causes a problem
	//gpio_set_direction(scl_io_num, GPIO_MODE_INPUT_OUTPUT_OD); // master
	//gpio_set_direction(scl_io_num, GPIO_MODE_INPUT); // slave
	//gpio_set_pull_mode(scl_io_num, GPIO_PULLUP_ONLY);
	REG_WRITE(GPIO_PIN_MUX_REG[scl_io_num], 0x00002B00); 
	
	gpio_matrix_out(scl_io_num, scl_out_sig, 0, 0);
	gpio_matrix_in(scl_io_num, scl_in_sig, 0);
	



	// ISR register
//#define PRO_INTR_STATUS_REG_1 
//#define I2C_INT_REG PRO_INTR_STATUS_REG_1
//	#define I2C0_INT_NO 49
//	#define I2C1_INT_NO 50
//#define I2C0_INT_BIT 17
//#define I2C1_INT_BIT 18
REG_SET(I2C0_INT_ENA_REG, 0
 | I2C_TX_SEND_EMPTY_INT 
 | I2C_RX_REC_FULL_INT 
 | I2C_ACK_ERR_INT 
 | I2C_TRANS_START_INT			//////////////////////////////////////	
 | I2C_TIME_OUT_INT 
 | I2C_TRANS_COMPLETE_INT		/////////////////////////////////////
 | I2C_MASTER_TRAN_COMP_INT		/////////////////////////////////////
 | I2C_ARBITRATION_LOST_INT
 | I2C_SLAVE_TRAN_COMP_INT	
 | I2C_END_DETECT_INT		
 | I2C_RXFIFO_OVF_INT		
// | I2C_TXFIFO_EMPTY_INT
// | I2C_RXFIFO_FULL_INT
);

REG_SET(I2C0_INT_CLR_REG, 0xFFFFFFFF); 		// should be part of isr?
	

void i2c0_master_isr(void *param);
void *fn = (void*)i2c0_master_isr;
//void(*fn)(void*)
	//i2c_driver_install(); 
	//i2c_isr_register(i2c_num, i2c_isr_handler_default, p_i2c_obj[i2c_num], intr_alloc_flags, &p_i2c_obj[i2c_num]->intr_handle);
	esp_intr_alloc(ETS_I2C_EXT0_INTR_SOURCE, 0, i2c0_master_isr, NULL, NULL);
	esp_intr_alloc_intrstatus(ETS_I2C_EXT0_INTR_SOURCE, /*flags*/0, 0, 0, i2c0_master_isr, /*arg*/NULL, NULL);
	AllocInt_Custom(ETS_I2C_EXT0_INTR_SOURCE, i2c0_master_isr);

	return;
	//ESP_LOGI("I2C", "INIT END");
	//*/
}

IRAM_ATTR void i2c0_master_isr(void *param)
{
	uint32_t status = REG_READ(I2C_INT_STATUS_REG);
	//ets_printf("INT: status: %u\n", status);
	REG_SET(0x3FF4400C, 1 << DEBUG_PIN);
	//gpio_set_level(DEBUG_PIN, 0);
	
	while (status != 0)
	{
		status = REG_READ(I2C_INT_STATUS_REG);
        
		if(status & I2C_TX_SEND_EMPTY_INT)
		{
			I2C0_CLI(I2C_TX_SEND_EMPTY_INT);
		}
		else if(status & I2C_RX_REC_FULL_INT)
		{
			I2C0_CLI(I2C_RX_REC_FULL_INT);
		}
		else if(status & I2C_ACK_ERR_INT)
		{
			//I2C[i2c_num]->int_ena.ack_err = 0;
			I2C0_CLI(I2C_ACK_ERR_INT);
			//if (p_i2c->mode == I2C_MODE_MASTER)
			//{
			//	p_i2c_obj[i2c_num]->status = I2C_STATUS_ACK_ERROR;
			//	I2C[i2c_num]->int_clr.ack_err = 1;
			//	//get error ack value from slave device, stop the commands
			//	i2c_master_cmd_begin_static(i2c_num);
			//}
		}
		else if(status & I2C_TRANS_START_INT)
		{
			//ets_printf("INT: I2C_TRANS_START_INT\n");
			I2C0_CLI(I2C_TRANS_START_INT);
		}
		else if(status & I2C_TIME_OUT_INT)
		{
			//I2C[i2c_num]->int_ena.time_out = 0;
			I2C0_CLI(I2C_TIME_OUT_INT);
			//p_i2c_obj[i2c_num]->status = I2C_STATUS_TIMEOUT;
			//i2c_master_cmd_begin_static(i2c_num);
		}
		else if(status & I2C_TRANS_COMPLETE_INT)
		{
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
		else if(status & I2C_MASTER_TRAN_COMP_INT)
		{
			I2C0_CLI(I2C_MASTER_TRAN_COMP_INT);
		}
		else if(status & I2C_ARBITRATION_LOST_INT)
		{
			I2C0_CLI(I2C_ARBITRATION_LOST_INT);
			//p_i2c_obj[i2c_num]->status = I2C_STATUS_TIMEOUT;
			//i2c_master_cmd_begin_static(i2c_num);
		}
		else if(status & I2C_SLAVE_TRAN_COMP_INT)
		{
			I2C0_CLI(I2C_SLAVE_TRAN_COMP_INT);
		}
		else if(status & I2C_END_DETECT_INT)
		{
			//ets_printf("INT: I2C_END_DETECT_INT\n");
			//I2C[i2c_num]->int_ena.end_detect = 0;
			I2C0_CLI(I2C_END_DETECT_INT);
			//i2c_master_cmd_begin_static(i2c_num);
		}
		else if(status & I2C_RXFIFO_OVF_INT)
		{
			I2C0_CLI(I2C_RXFIFO_OVF_INT);
		}
		else if(status & I2C_TXFIFO_EMPTY_INT)
		{
			int tx_fifo_space_rem = 32 - ((REG_READ(0x3FF53008)>>18) & 0x3F);
			//ets_printf("Rem: %u\n", tx_fifo_rem);
			REG_SET(0x3FF44008, 1 << DEBUG_PIN);
			//REG_OR(I2C0_FIFO_CONF_REG, (1 << 12) | (1 << 13));  // reset rx/tx fifo
			//REG_AND(I2C0_FIFO_CONF_REG, ~((1 << 12) | (1 << 13)));
			for(int i = 1 ; i < tx_fifo_space_rem ; i++)
			{
				REG_SET(I2C0_FIFO_ADDR, RBP_READ(I2C_Data));
			}
			
			if(I2C_Data->readCursor == I2C_Data->writeCursor)
				REG_AND(I2C0_INT_ENA_REG, ~I2C_TXFIFO_EMPTY_INT); // Disable int.
			
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
		else if(status & I2C_RXFIFO_FULL_INT)
		{
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
			I2C0_CLI(status); // clear all
		}
	}
	//if (p_i2c->mode == I2C_MODE_MASTER)
	//{
	//	//i2c_cmd_evt_t evt;
	//	//evt.type = I2C_CMD_EVT_ALIVE;
	//	//xQueueSendFromISR(p_i2c->cmd_evt_queue, &evt, &HPTaskAwoken);
	//}
}
