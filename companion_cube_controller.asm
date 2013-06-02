/*
 * companion_cube_controller.asm
 * 
 *  Created: 23.03.2013 20:07:38
 *   Author: Michael Happl
 *
 *	Contoller: ATMega88p
 *
 *	Description: This program reads an analog voltage, uses the upper 8 bit of it as pwm value. this pwm is used to control leds with
 *  a TLC59025 16-channel led driver. Next, the program is extended by adding a pulsed brightness of the leds. This is done by writing 
 *  a new compare value each time the timer1 has an overflow. Switching between the analog controlled brightness is done via a switch
 *  which is connected to the external interrut 0.
 */ 

//------------------------------------------------------------
// Just some space for custom definitions
.set	SDI1 = PORTD3	//PD3 is wired to the serial data input pin of the first led driver
.set	SDI2 = PORTD4	//PD4 is wired to the serial data input pin of the second led driver
.set	SDI3 = PORTD5	//PD5 is wired to the serial data input pin of the third led driver
.set	D_CLK = PORTD6	//PD6 is used as clock source for the serial data input of all 3 led drivers
.set	LE = PORTD7		//PD7 is wired to the latch enable of all 3 led drivers

.def	t0_cnt = r17	//Register17 is used to count the data bits being written to the led drivers
.def	up_down = r18	//register18 is used to save in which direction  the data from the table is read
.def	read_cnt = r19	//in register19 the number of the value in the table is stored
//------------------------------------------------------------

.cseg				//program is stored in the flash memory

reset:				
.org	0			//compiling starts at adress 00
	rjmp	setup	//

.org	0x001
	rjmp	ext_int0		//Interrupt routine for the external interrupt 0

.org	0x00B
	rjmp	t1_cmpa_int		//interrupt routine for the compare match a interrupt of the timer1

.org	0x00D
	rjmp	t1_of_int		//interrupt routine for the timer1 overflow

.org	0x010
	rjmp	t0_overflow		//Interrupt routine for the timer0 overflow

.org	0x015
	rjmp	adc_int			//jump to the interrupt routine for the ADC

.org	0x01A				//Start after the interrupt vectors

setup:
	ldi		r16,high(RAMEND)	//initializing the stack pointers
	out		SPH,r16				
	ldi		r16,low(RAMEND)		 
	out		SPL,r16			
	
	//First, we need to activate all interrupts:
	
	sei						//global interrupt enable

	//then we need to configure the ADC module:

	ldi		r16, (1<<REFS0) | (1<<ADLAR)
	sts		ADMUX,r16		//AVCC as reference voltage; result is left orientated; channel0 as input
	ldi		r16, (1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)
	sts		ADCSRA,r16		//ADC is enabled; first conversion started; auto trigger and interrupt is enabled; 128 prescaler => 125kHz ADC clock
	ldi		r16, (1<<ADC0D)
	sts		DIDR0,r16		//disabling the digital input buffer on PC0
							//we don't need to modify ADCSRB because it's default set to free running mode for the ADC

	//we also need to set the pwm pin OC1A (PB1) as an output

	ldi		r16, (1<<DDB1)	//only PB1 needs to be an output
	out		DDRB,r16
	ldi		r16, (1<<PORTB1)
	out		PORTB,r16		//Starting with outputs disabled (OE = 1)

	//the control pins for the LED drivers also need to be configured

	ldi		r16, (1<<DDD3) | (1<<DDD4) | (1<<DDD5) | (1<<DDD6) | (1<<DDD7)
	out		DDRD,r16		//All SDI pins, the clock and the latch enable need to be outputs
	ldi		r16, (1<<PORTD2) | (1<<D_CLK)	//PD2 is the interrupt input for the switch and is configured with an internal pull up
	out		PORTD,r16		//The data inputs start-value doesn't matter, they will be set later; the latch enable must be 0; 

	//the timer0 provides the clock source for the data input

	ldi		r16,128			//50% duty cycle
	out		OCR0A,r16
	ldi		r16, (1<<COM0A1) | (1<<COM0A0) | (1<<WGM01) | (1<<WGM00)	//output pin (OC0A) is set at compare match and cleared at top
	out		TCCR0A,r16		//Fast pwm mode with 0xFF as top (results in 62,5kHz clock rate for the data input)
	ldi		r16, (1<<CS00)	
	out		TCCR0B,r16		//clock prescaler is set to 1
	ldi		r16, (1<<TOIE0)	//timer0 overflow interrupt is enabed
	sts		TIMSK0,r16
	clr		t0_cnt			//The counter register is cleared at the start to ensure correct counting

	//External interrupt 0 is configured:

	ldi		r16, (1<<ISC01)
	sts		EICRA,r16		//Interrupt activates on falling edge on pin PD2
	ldi		r16, (1<<INT0)
	out		EIMSK,r16		//Interrrupt is activated

	//writing the adress of the data table to the X register and set the up/down register to upward reading of the values

	ldi		XL,LOW(data_table)
	ldi		XH,HIGH(data_table)
	clr		up_down			//if the up/down register is cleared, the data is read from bottom to top => starting with upward reading
	clr		read_cnt		//therefore the read cycle counter must start at zero

loop:				//At this moment the main program is just a loop of doing nothing
	nop
	rjmp	loop

	//the ADC interrupt routine

adc_int:			
	in		r15,SREG		//saving the status register
	push	r15				//and pushing it on the stack so it will be save if another interrupt happens while this one is running
	push	r16				//and also pushing r16, in case the interrupt happend while some other value was in there
	lds		r16,ADCH		//reading the upper 8 bits of the conversion result
	sts		OCR1AL,r16		//and set them as compare value for the pwm
	rjmp	ret_int
	
	//In the overflow interrupt routine the data for the shift register inputs (SDI1 to SDI3) is written to the pins

t0_overflow:		
	in		r15,SREG		//saving the status register
	push	r15				//and pushing it on the stack so it will be save if another interrupt happens while this one is running
	push	r16				//and also pushing r16, in case the interrupt happend while some other value was in there
	inc		t0_cnt			//counting the activations of this routine
	ldi		r16, (1<<SDI3) | (1<<SDI2) | (1<<SDI1)
	out		PORTD,r16		//This section can be modified later, but at this moment all leds will be turned on at the same time
	ldi		r16,17
	cpse	t0_cnt,r16		//after 17 activations the start value of SDI1 - SDI3 isn't stored in the led drivers anymore => skip next instuction
	rjmp	ret_int			//othervise we just return and wait for the next activation
	sbi		PORTD,LE		//now the data latches are activated		
	clr		r16				//then the timer0 is turned off
	out		TCCR0B,r16		
	cbi		PORTD,LE		//the pulse doesn't need to be very long, so LE is turned off again	

	//Next we're going to set the timer1 module to fast pwm mode:
	
	rcall	t1_conf_adc

ret_int:					//because the status register and the r16 have to be restored we need one interrupt return routine
	pop		r16				//restoring r16
	pop		r15
	out		SREG,r15		//and the status register
	reti					//now everything is done

	//timer0 is set to fast pwm mode with 255 as top for the adc controlled operation

t1_conf_adc:
	ldi		r16, (1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)
	sts		ADCSRA,r16		//ADC is enabled; and settings are restored
	clr		r16				//the high byte is set to 0, just to be sure
	sts		OCR1AH,r16
	ldi		r16,128			//Initial 50% duty cycle
	sts		OCR1AL,r16
	ldi		r16, (1<<COM1A1) | (1<<COM1A0 ) | (1<<WGM11)
	sts		TCCR1A,r16		//inverting mode, WGM11 = 1 and WGM10 = 0 (fast pwm mode with ICR1 as TOP value)
	ldi		r16, (1<<WGM13) | (1<<WGM12) | (1<<CS10)
	sts		TCCR1B,r16		//no input, so INCNC1 and ICES1 are 0, WGM13 and 12 are both one for fast pwm and clock prescaler = 1
	clr		r16
	sts		ICR1H,r16		//the high byte is set to 0, just to be sure
	ser		r16				
	sts		ICR1L,r16		//set top value for pwm to 255
	ret

	//timer0 is set to fast pwm mode 255 as top for the adc controlled operation

t1_conf_pulse:
	clr		r16
	sts		ADCSRA,r16		//ADC is disabled
	clr		r16				//because the table is read bottom up, we must start with a zero as compare value
	sts		OCR1AH,r16
	sts		OCR1AL,r16
	ldi		r16, (1<<COM1A1) | (1<<COM1A0 ) | (1<<WGM11)
	sts		TCCR1A,r16		//inverting mode, WGM11 = 1 and WGM10 = 0 (fast pwm mode with ICR1 as TOP value)
	ldi		r16, (1<<WGM13) | (1<<WGM12) | (1<<CS10)
	sts		TCCR1B,r16		//no input, so INCNC1 and ICES1 are 0, WGM13 and 12 are both one for fast pwm and clock prescaler = 1
	ldi		r16,0x3F	
	sts		ICR1H,r16		//set top value for pwm to 0x3FFF (16383)
	ser		r16	
	sts		ICR1L,r16
	ldi		r16, (1<<OCIE1A)
	sts		TIMSK1,r16		//and starting with the compare interrupt enabled
	ret

	//if the button is pressed the program needs to switch from analog control to pulsing brightness

ext_int0:	
	in		r15,SREG		//saving the status register
	push	r15				//and pushing it on the stack so it will be save if another interrupt happens while this one is running
	push	r16				//and also pushing r16, in case the interrupt happend while some other value was in there
	lds		r16,TIMSK1		//looking if overflow or compare interrupt is set
	sbrs	r16,OCIE1A		//if the output compare interrupt is set, we need to switch to analog control
	rjmp	of_check		//if not, we need to check the overflow interrupt too
	rjmp	analog_ctrl

of_check:
	sbrc	r16,TOIE1		//if the overflow interrupt is enabled
	rjmp	analog_ctrl		//we switch to analog control
	rcall	t1_conf_pulse	//if both interrupts are disabled the timer can be set to pulsing control
	rjmp	ret_int			//then we just return

analog_ctrl:
	rcall	t1_conf_adc		//for analog controll we just call the right configuration subroutine
	rjmp	ret_int			//return routine is the same as for all other interrupts because the status register and r16 need to be restored

	//here the new compare value is written into the OCRA1-registers and if its too low, the compare interrupt is enabled

t1_of_int:		
	in		r15,SREG		//saving the status register
	push	r15				//and pushing it on the stack so it will be save if another interrupt happens while this one is running
	push	r16				//and also pushing r16, in case the interrupt happend while some other value was in there
	sbrc	up_down,0		//check if top-down or bottom up reading the values
	rjmp	t1_of_down		//if the bit is set => reading downwards
	cpi		read_cnt,128	//reached top?
	breq	up_to_down		//if yes => reverse the reading direction
	rcall	read_up			//else the program continues reading the values upward
	rjmp	ret_int			//return routine is the same as for all other interrupts because the status register and r16 need to be restored

t1_of_down:
	cpi		read_cnt,64		//when counting downwards in the overflow interrupt mode check if reached the middle 
	breq	of_to_cmp		//if yes => changing to compare interrupt mode
	rcall	read_down		//else => just read downwards another time
	rjmp	ret_int

up_to_down:			//switching the counting direction
	sbr		up_down,0		//first the direction bit is set to downward reading
	rcall	read_down		//and then the downward reading subroutine is called
	rjmp	ret_int			//thats already everything

of_to_cmp:			//deactivating the overflow interrupt and activating the compare interrupt
	ldi		r16, (1<<OCIE1A)
	sts		TIMSK1,r16		//disabling the overflow and enabling the compare interrupt
	rcall	read_down		//and continue reading downwards
	rjmp	ret_int

	//here the new compare value is written into the OCRA1-registers and if its too high, the overflow interrupt is enabled

t1_cmpa_int:		//here the new compare value is written into the OCRA1-registers and if its too high, the overflow interrupt is enabled
	in		r15,SREG		//saving the status register
	push	r15				//and pushing it on the stack so it will be save if another interrupt happens while this one is running
	push	r16				//and also pushing r16, in case the interrupt happend while some other value was in there
	sbrc	up_down,0		//check if top-down or bottom up reading the values
	rjmp	t1_cmp_down		//if the bit is set => counting down
	cpi		read_cnt,64		//else => reading upwards; reached the middle of the table?
	breq	cmp_to_of		//if yes => program switches from compare to overflow interrup
	rcall	read_up			//else => continue reading upwards
	rjmp	ret_int			//return routine is the same as for all other interrupts because the status register and r16 need to be restored

t1_cmp_down:
	cpi		read_cnt,0		//already reached the bottom of the table?
	breq	down_to_up		//if yes => reverse reading direction
	rcall	read_down		//othervise just continue reading downwards
	rjmp	ret_int

down_to_up:
	cbr		up_down,0		//clear the direction bit in the up/down register => upwards reading
	rcall	read_up			//and now just read upwards
	rjmp	ret_int			//and that's it already

cmp_to_of:			//deactivating the compare interrupt and activating the overflow interrupt
	ldi		r16, (1<<TOIE1)
	sts		TIMSK1,r16		//disabling the compare and enabling the overflow interrupt
	rjmp	ret_int			//no data is loaded into the compare registers, this is done in the next interrupt

	//a small subroutine for the upward reading of the values so it doesn't need to be witten twice:

read_up:
	cli						//all interrups are disabled, so the 16 bit value can be written without being disturbed
	ld		r16,X+			//we're starting by reading the upper byte of the compare value
	sts		OCR1AH,r16		//and writing it into the compare register of timer1
	ld		r16,X+			//because the pointer has been incremented in the last instruction, it now reads the low byte of the compare value
	sts		OCR1AL,r16		//and storing it in the compare register of timer1
	inc		read_cnt		//the numer of the value is stored by incrementing the count-register
	sei						//at the end all interrupts are enabled again
	ret

	//for downward reading of the values there is also a small subroutine

read_down:
	cli						//all interrups are disabled, so the 16 bit value can be written without being disturbed
	ld		r16,-X			//since the last reading cycle the pointer hasn't been modified we need decrement it first
	sts		OCR1AL,r16		//then we can read the lower byte of the compare value and store it in the right register
	ld		r16,-X			//the pointer now is still pointing at the lower byte, so wee again need to pre decrement it
	sts		OCR1AH,r16		//so we can read the value and store it in the compare register of timer1
	dec		read_cnt		//the number of the value is updated by decrementing the count register
	sei						//at the end all interrupts are enabled again
	ret

	//the following table contains the compare values for the pwm:

data_table:			//the backslash means that the table continues in the next line
.dw 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0001, 0x0002, 0x0003, 0x0005, 0x0007, 0x0009, \
0x000C, 0x0010, 0x0015, 0x001A, 0x0021, 0x0029, 0x0032, 0x003D, 0x0049, 0x0057, 0x0067, 0x0079, 0x008D, 0x00A3, 0x00BC, 0x00D8, 0x00F7, \
0x0118, 0x013E, 0x0166, 0x0192, 0x01C2, 0x01F6, 0x022D, 0x026A, 0x02AA, 0x02EF, 0x0339, 0x0388, 0x03DC, 0x0435, 0x0494, 0x04F7, 0x0561, \
0x05D0, 0x0644, 0x06BF, 0x073F, 0x07C6, 0x0852, 0x08E4, 0x097D, 0x0A1B, 0x0ABF, 0x0B6A, 0x0C1A, 0x0CD1, 0x0D8D, 0x0E4F, 0x0F17, 0x0FE4, \
0x10B7, 0x118F, 0x126C, 0x134F, 0x1436, 0x1522, 0x1612, 0x1707, 0x1800, 0x18FC, 0x19FC, 0x1AFF, 0x1C06, 0x1D0E, 0x1E19, 0x1F27, 0x2035, \
0x2146, 0x2257, 0x2369, 0x247B, 0x258D, 0x269F, 0x27AF, 0x28BF, 0x29CD, 0x2AD9, 0x2BE3, 0x2CEA, 0x2DEE, 0x2EEE, 0x2FEB, 0x30E3, 0x31D6, \
0x32C4, 0x33AD, 0x3490, 0x356D, 0x3643, 0x3713, 0x37DB, 0x389C, 0x3955, 0x3A05, 0x3AAE, 0x3B4D, 0x3BE4, 0x3C71, 0x3CF5, 0x3D6F, 0x3DDF, \
0x3E45, 0x3EA1, 0x3EF2, 0x3F39, 0x3F75, 0x3FA6, 0x3FCD, 0x3FE8, 0x3FF9, 0x3FFF
