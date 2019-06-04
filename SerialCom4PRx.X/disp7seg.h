/*
* https://everlighteurope.com/index.php?controller=attachment?id_attachment=4657
* 
*	10	.	.	.	6					
*		  --a--							
*		|		|		1	e		b	6
*		f		b		2	d		a	7
*		|		|		3	  COM +		8
*		  --g--			4	c		f	9
*		|		|		5	dp		g	10
*		e		c						
*		|		|						
*		  --d--		dp				
*	1	.	.	.	5		
*
*   pin:    5   10  9   1   2   4   6   7
*	bar:    dp  g   f   e   d   c   b   a
* 	bit:    7   6   5   4   3   2   1   0	
*/										

					
static const unsigned char display7s[] = {										
	0xC0,	0xF9,	0xA4,	0xB0,		//	0	1	2	3
	0x99,	0x92,	0x82,	0xF8,		//	4	5	6	7
	0x80,	0x98,	0x88,	0x83,		//	8	9	A	b
	0xC6,	0xA1,	0x86,	0x8E,		//	C	d	E	F
	0x40,	0x79,	0x24,	0x30,		//	0.	1.	2.	3.
	0x19,	0x12,	0x02,	0x78,		//	4.	5.	6.	7.
	0x00,	0x18,	0x08,	0x03,		//	8.	9.	A.	b.
	0x46,	0x21,	0x06,	0x0E,		//	C.	d.	E.	F.
    0xFF                                // all off   
};	
// prototype of method to call to light up the display
void showOnDisplay(unsigned char);