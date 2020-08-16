
#include "Task_APP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_alloc_caps.h"

#define TAG "APP_TASK"

void vAppTask(void *pvParameters)
{
	//p = heap_caps_malloc(sizeof(...), MALLOC_CAP_32BIT);
	//heap_caps_free(p);
	//*entryUserCode0 = 0; // 11110000 => 00100000 => 00000000
	ESP_LOGI(TAG, "0");
	vUserCode0();
	ESP_LOGI(TAG, "1");
	vUserCode2();
	ESP_LOGI(TAG, "2");
	printf("%p\n", vAppTask);
	printf("%p\n", vUserCode1);
	printf("%p\n", vUserCode1+12);
	for (int i = -1; i < 98 / 4; i++)
	{
		uint32_t data = *((uint32_t*)(vUserCode1 + 4*i));
		//printf("%d: %u\n", i, *(uint8_t*)(vUserCode1 + i));
		printf("%d: 0x%x\n", 4*i + 0, (data & 0xFF));		// little endian, so the LSByte is stored at the smallest address
		printf("%d: 0x%x\n", 4*i + 1, (data >> 8) & 0xFF);	//
		printf("%d: 0x%x\n", 4*i + 2, (data >> 16) & 0xFF);	//
		printf("%d: 0x%x\n", 4*i + 3, (data >> 24) & 0xFF);	//
	}
	ESP_LOGI(TAG, "2_2");	
	//uint32_t l1_val;
	(vUserCode1 + 12)();
	//asm volatile("movi %%a10, %0" : "=r" (l1_val));
	//asm volatile("BREAK 2, 10");
	//register uint32_t l1_val asm("a10");
	//ESP_LOGI(TAG, "a10p was %x", l1_val);
	//ESP_LOGI(TAG, "a10v was %x", *(uint32_t*)((((uint32_t)l1_val) / 4) * 4));
	//ESP_LOGI(TAG, "3");
	//for (int i = -1; i < 98 / 4; i++)
	//{
	//	uint32_t data = *(uint32_t*)(vUserCode3 + 4*i);
	//	printf("%d: 0x%x\n", 4*i + 0, (data & 0xFF)); 		// little endian, so the LSByte is stored at the smallest address
	//	printf("%d: 0x%x\n", 4*i + 1, (data >> 8) & 0xFF); 	//
	//	printf("%d: 0x%x\n", 4*i + 2, (data >> 16) & 0xFF); //
	//	printf("%d: 0x%x\n", 4*i + 3, (data >> 24) & 0xFF); //
	//}
	//ESP_LOGI(TAG, "2_3");	
	//uint32_t data2 = *((uint32_t*)(vUserCode1 + 4*4));
	ESP_LOGI(TAG, "3");
	//volatile uint32_t* volatile ptr = (uint32_t*)(vUserCode1 + 16);
	uint32_t* ptr = (uint32_t*)(vUserCode1 + 16);
	ESP_LOGI(TAG, "4: %p", vUserCode1 + 16);
	//uint32_t val = *ptr;
	//uint32_t val = ALIGNED_READ(ptr); // fügt unnötige memw Instruktionen hinzu (2x), trotzdem besser als volatile
	uint32_t val;
	ALIGNED_READ(ptr, val);
	//*(uint32_t*)(vUserCode1 + 16) = 0x00A032F0;    // 0x00 prevents jump
	//*ptr = (uint32_t)0x00A032F0;     // 0x00 prevents jump
	ALIGNED_WRITE(ptr, 0x00A032F0);
	ESP_LOGI(TAG, "5, %u", val);
	for (int i = -1; i < 98 / 4; i++)
	{
		uint32_t data = *(uint32_t*)(vUserCode1 + 4*i);
		printf("%d: 0x%x\n", 4*i + 0, (data & 0xFF)); 		// little endian, so the LSByte is stored at the smallest address
		printf("%d: 0x%x\n", 4*i + 1, (data >> 8) & 0xFF); 	//
		printf("%d: 0x%x\n", 4*i + 2, (data >> 16) & 0xFF); //
		printf("%d: 0x%x\n", 4*i + 3, (data >> 24) & 0xFF); //
	}
	//ESP_LOGI(TAG, "6");
	//*(uint8_t*)(vUserCode1 + 4) = 'o';
	ESP_LOGI(TAG, "6");
	(vUserCode1 + 12)();
	ESP_LOGI(TAG, "7");
	
	ESP_LOGI(TAG, "printInc: %p", printInc);
	ESP_LOGI(TAG, "vCall8OffsetTest: %p", vCall8OffsetTest);
	ESP_LOGI(TAG, "l1: %p", vCall8OffsetTest + 12);
	ESP_LOGI(TAG, "l2: %p", vCall8OffsetTest + 22);
	for (int i = -1; i < 36 / 4; i++)
	{
		uint32_t data = *(uint32_t*)(vCall8OffsetTest + 4*i);
		printf("%d: 0x%x\n", 4*i + 0, (data & 0xFF));  		// little endian, so the LSByte is stored at the smallest address
		printf("%d: 0x%x\n", 4*i + 1, (data >> 8) & 0xFF);  //
		printf("%d: 0x%x\n", 4*i + 2, (data >> 16) & 0xFF);  //
		printf("%d: 0x%x\n", 4*i + 3, (data >> 24) & 0xFF);  //
	}
	vCall8OffsetTest();


	//ESP_LOGI(TAG, "8");
	//pvPortMallocCaps(4*sizeof(uint32_t), MALLOC_CAP_);
	uint32_t *varCode = heap_caps_malloc(4*sizeof(uint32_t), MALLOC_CAP_32BIT | MALLOC_CAP_EXEC);
	ESP_LOGI(TAG, "varCode: %p", varCode);
	//varCode[0] = 0x3d006136;
	//varCode[1] = 0x00f01df0;
	//
	//for (int i = 0; i < 4; i++)
	//{
	//	uint32_t data = 0;
	//	ALIGNED_READ(&varCode[i], data);
	//	printf("%d: 0x%x\n", 4*i + 0, (data & 0xFF));  		// little endian, so the LSByte is stored at the smallest address
	//	printf("%d: 0x%x\n", 4*i + 1, (data >> 8) & 0xFF);  //
	//	printf("%d: 0x%x\n", 4*i + 2, (data >> 16) & 0xFF);  //
	//	printf("%d: 0x%x\n", 4*i + 3, (data >> 24) & 0xFF);  //
	//}
	//
	//void (*f)() = varCode;
	//f();

	ESP_LOGI(TAG, "8");
	uint32_t lastCall = call8FromAddresses((uint32_t)varCode + 11, (uint32_t)printInc);
	varCode[0] = 0x3d006136;
	varCode[1] = call8FromAddresses((uint32_t)varCode + 5, (uint32_t)printInc) << 8 | 0xf0;
	varCode[2] = (lastCall & 0xFF) << 24 | call8FromAddresses((uint32_t)varCode + 8, (uint32_t)printInc);
	varCode[3] = 0xf01d0000 | (lastCall >> 8);
	for (int i = 0; i < 4; i++)
	{
		uint32_t data = 0;
		ALIGNED_READ(&varCode[i], data);
		printf("%d: 0x%x\n", 4*i + 0, (data & 0xFF));  		// little endian, so the LSByte is stored at the smallest address
		printf("%d: 0x%x\n", 4*i + 1, (data >> 8) & 0xFF);  //
		printf("%d: 0x%x\n", 4*i + 2, (data >> 16) & 0xFF);  //
		printf("%d: 0x%x\n", 4*i + 3, (data >> 24) & 0xFF);  //
	}
	ESP_LOGI(TAG, "9");
	void (*g)() = varCode;
	g();

	//	0 : 0x36    // entry a1,0x30; 0x36 0x61 0x0 => 00110110 01100001 00000000 // vUserCode1 + 12
	//	1 : 0x61    //
	//	2 : 0x0     //
	//	3 : 0x3d    // NOP.N => 11110000 00111101 => F0 3D
	//
	//	4 : 0xf0    //
	//	5 : 0xe5    // call8 printInc;	=> offset(18bit) 10 0101" => 0x([2,6,A,E])5
	//	6 : 0x3     //
	//	7 : 0x0     //
	//	
	//	8 : 0xa5    // call8 printInc;	=> offset(18bit) 10 0101" => 0x([2,6,A,E])5
	//	9 : 0x3     //
	//	10 : 0x0     //
	//	11 : 0xa5    // call8 printInc;	=> offset(18bit) 10 0101" => 0x([2,6,A,E])5
	//	
	//	12 : 0x3     //
	//	13 : 0x0     //
	//	14 : 0x1d    // 0x1D 0xF0 => retw.n
	//	15 : 0xf0    //
	
	//ESP_LOGI(TAG, "9");
	//uint32_t printIncAddress = (uint32_t)printInc;
	//uint32_t printIncCall = ;
	//uint32_t loadAddress = ;
	//varCode[0] = 0x3d006136;
	//varCode[1] = call8FromAddresses(varCode+5,(uint32_t)printInc) << 8 | 0xf0;
	//varCode[2] = (lastCall&0xFF)<<24 | call8FromAddresses(varCode + 8, (uint32_t)printInc);
	//varCode[3] = 0xf01d0000 | (lastCall >> 8);
	//
	//void (*h)() = varCode;
	//h();

	ESP_LOGI(TAG, "10");
	//free(varCode);
	//ESP_LOGI(TAG, "11");

	vTaskDelete(NULL);
	//while (1) ;
}

uint32_t call8FromAddresses(uint32_t origin, uint32_t target) // target has to be 32 bit aligned (entry), origin does not
{
	uint32_t roundedPC = origin & 0xFFFFFFFC;
	int32_t difference = target - roundedPC - 4;
	int32_t offset = difference / 4; // just shifting >>2 would probably leave the MSBits as zero for negative numbers, which would be wrong
	//offset &= 0x0003FFFF;
	//return ((offset << 6) | 0x25) & 0x00FFFFFF; // maybe this uses the wrong endianess of offset?
	return (((((uint32_t)offset) << 6)&(~0x3F)) | 0x25) & 0x00FFFFFF;
	//if (offset > 0)
	//{
	//	offset &= 0x3FFFF;
	//	return ((offset << 6) | 0x25) & 0x00FFFFFF;
	//}
	//else
	//{
	//	offset = -offset;
	//	offset &= 0x3FFFF;
	//	offset = -offset;
	//	return (((offset << 6) | 0x25) & 0x00FFFFFF);
	//}

	/*
		(data & 0xFF) = i0			// first instruction byte as seen in disassembly
		(data >> 8) & 0xFF) = i1
		(data >> 16) & 0xFF) = i2

		=> data = i2<<16 | i1<<8 | i0;

		ie. entry =  0x36, 0x61, 0x0	// entry a1,0x30; 0x36 0x61 0x0 => 00110110 01100001 00000000 // vUserCode1 + 12

	target: ENTRY (at 32bit boundary; ?524284 to 524288 bytes from PC)
	target = (PC & 0xF..FC0) + SEXT(offset)<<2 + 4 // we can ignore sign extension, everything is handled by CPU since sign extension only matters if converted from 18 bit -> 32 bit. (We: 32 bit -> 18 bit)
	
	=> offset = ((target - (PC & 0xF..FC) - 4)>>2) & 0x3FFFF
	*/
}

//uint32_t callX8FromAddresses(uint32_t origin, uint32_t target)
//{
//	uint32_t val1 = origin & 0xFFFFFFFC;
//	int32_t val2 = target - 4 - val1;
//	int32_t offset = val2 / 4;
//	//offset &= 0x3FFFF;
//	//return ((offset << 6) | 0x25) & 0x00FFFFFF;
//	if(offset > 0)
//	{
//		offset &= 0x3FFFF;
//		return ((offset << 6) | 0x25) & 0x00FFFFFF;
//	}
//	else
//	{
//		offset = -offset;
//		offset &= 0x3FFFF;
//		offset = -offset;
//		return (((offset << 6) | 0x25) & 0x00FFFFFF);
//	}
//	0 0 0 0 0 0 0 0 0 0 0 0 s 1 1 1 0 0 0 0 0
//}

IRAM_ATTR void printInc()
{
	static uint32_t count = 0;
	printf("ASM: %u\n", count);
	count++;
}
/* Not necessary & not valid
saving a register to the stack: (32 bits)
an => *sp
inc sp
...
dec sp
*sp => an
*/

// Alternativ speicher mit malloc alloziieren, Werte: "asm = entry; ...; retw.n;"
// entry a1, 0x30 // IMMER mit a1 (Konvention)
// entry: 0x36 0x|6|1 0x00 => 006136 => 000000000110(0x30 >> 3 = 0x6) 0001(a1) 00110110(opcode ENTRY)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-label"
IRAM_ATTR void vUserCode0() // instruction at this address exactly is "entry" = 3 bytes
{
	//const char* ts1 = "test";
	//goto entryUserCode1;
entryUserCode0 :   //  first "nop" is at 3 bytes offset from function address
	asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
	//asm volatile("l32r a10,%0" : : "r"(ts1) :);
	//asm volatile("nop;nop;call8 printf;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;"); // 32 instr. = 64 bytes of space (each nop (converted to nop.n) is "F0 3D" = 2 bytes)
	// return is "retw.n" = "1D F0" = 2 bytes
}

IRAM_ATTR void vCall8OffsetTest()
{
	asm volatile(
		"_nop;															\n"
		"call8 C8OT_l1;													\n"
		"j C8OT_l2;														\n"
	"C8OT_l1:															\n"
		"entry a1,0x30;													\n"	// entry instruction needs to be aligned // here at offset 12
		"call8 printInc;												\n"
		"retw.n;														\n" // prev nothing here
	"C8OT_l2:															\n"
		"call8 C8OT_l1;													\n"
		"call8 printInc;												\n"
	);
}

IRAM_ATTR void vUserCode1()
{
	//	asm(".section .data      \n"
	//	    "1: .asciz \"Hello\" \n"
	//	    ".section .text      \n"
	//	    "movi a10, $1b       \n"
	//	    "call8 printf        \n"
	//	);
	asm volatile(
		//".section .data													\n"
		//"l1: .asciz \"Print from inside custom assembly function!\n\"	\n"
		//".section .text													\n"
		//"nop;															\n"
		//"nop;															\n"
		".byte 0x00;													\n"
		"l1: .string \"text\";											\n" // should be 116 101 120 116 0 in decimal // offset +4
		".byte 0x00;													\n"
		".byte 0x00;													\n"
		".byte 0x00;													\n"
		"entry a1,0x30;													\n"	// entry instruction needs to be aligned // .byte 0x00; // here at offset 12
		"nop;															\n"	// 15 => F0 3D
		"_movi a3,0xFF;													\n" // 00110010 10100000 0xFF => 0xFF at offset 17		//"movi a10,0x40081d08;													\n"				 // loads absolute address of l1 (string) into a10
		"bnez a3, l2;														\n"
		"call8 printInc;												\n"
		"l2:															\n"
		"call8 printInc;												\n"
		"retw.n;														\n" // prev nothing here
		"nop;															\n" // offset 22
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"retw.n;														\n" // offset 33 // assembler automatically inserts filler 0-bytes after this for alignment, but number of NOPs is conserved
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
		"nop;															\n"
	);
}

IRAM_ATTR void vUserCode2()
{
	asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;retw.n;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
}

IRAM_ATTR void vUserCode3()
{
	asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
}

IRAM_ATTR void vUserCode4()
{
	asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
}

IRAM_ATTR void vUserCode5()
{
	asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
}

IRAM_ATTR void vUserCode6()
{
	asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
}

IRAM_ATTR void vUserCode7()
{
	asm volatile("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;");
}
#pragma GCC diagnostic pop