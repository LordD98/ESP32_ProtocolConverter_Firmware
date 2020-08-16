.section .data

;l1: 
;	.asciz \"Print from inside custom assembly function!\n\"

.section .text
	nop;
	nop;
	.byte 0x00;
l1: .string \"text\";										should be 116 101 120 116 0 in decimal // offset +4
	.byte 0x00;
	.byte 0x00;
	.byte 0x00;
	entry a1,0x30;											entry instruction needs to be aligned // .byte 0x00; // here at offset 12
	nop;													15 => F0 3D
	_movi a3,0xFF;											00110010 10100000 0xFF => 0xFF at offset 17		//	movi a10,0x40081d08;						 loads absolute address of l1 (string) into a10
	bnez a3, l2;
	call8 printInc;
l2:
	call8 printInc;
	retw.n;													prev nothing here
	nop;													offset 22
	nop;
	nop;
	nop;
	retw.n;													offset 33 // assembler automatically inserts filler 0-bytes after this for alignment, but number of NOPs is conserved
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;
	nop;