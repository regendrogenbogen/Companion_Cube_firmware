/*
 * companion_cube_controller.asm
 * 
 *  Created: 23.03.2013 20:07:38
 *   Author: Michael Happl
 *
 *	Contoller: ATMega88p
 *
 *	Description: First prototype of the controller firmware. This program reads in the analog value from the potentiometer
 *		and puts it out as a PWM value to vary the brightness of an LED.
 */ 

//------------------------------------------------------------
// Just some space for custom definitions
.set	SDI1 = PORTD3	//PD3 is wired to the serial data input pin of the first led driver
.set	SDI2 = PORTD4	//PD4 is wired to the serial data input pin of the second led driver
.set	SDI3 = PORTD5	//PD5 is wired to the serial data input pin of the third led driver
.set	LE = PORTD7		//PD7 is wired to the latch enable of all 3 led drivers
//------------------------------------------------------------

.cseg				//program is stored in the flash memory

reset:				
.org	0			//compiling starts at adress 00
	rjmp	setup	//

.org	0x00D
	rjmp	t1_overflow		//Interrupt routine for the timer1 overflow

.org	0x015
	rjmp	adc_int			//jump to the interrupt routine for the ADC

.org	0x01A				//Start after the interrupt vectors

setup:
	ldi		r16,high(RAMEND)	//initializing the stack pointers
	out		SPH,r16				
	ldi		r16,low(RAMEND)		 
	out		SPL,r16			
	
	//First, we need to activate interrupts for the ADC:
	
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
	out		PORTB,r16		//Starting with outputs disabled

	//the control pins for the LED drivers also need to be configured

	ldi		r16, (1<<DDB1)	//only PB1 needs to be an output
	out		DDRB,r16
	ldi		r16, (1<<PORTB1)
	out		PORTB,r16		//Starting with outputs disabled

	//the timer0 provides the 

	//Next we're going to set the timer1 module to fast pwm mode:
	
	ldi		r16,0b10000000	//Initial 50% duty cycle
	sts		OCR1AL,r16
	ldi		r16, (1<<COM1A1) | (1<<COM1A0 ) | (1<<WGM11)
	sts		TCCR1A,r16		//inverting mode, WGM11 = 1 and WGM10 = 0 (fast pwm mode with ICR1 as TOP value)
	ldi		r16, (1<<WGM13) | (1<<WGM12) | (1<<CS10)
	sts		TCCR1B,r16		//no input, so INCNC1 and ICES1 are 0, WGM13 and 12 are both one for fast pwm and clock prescaler = 1
	ser		r16				
	sts		ICR1L,r16		//set top value for pwm to 255

	
loop:				//At this moment the main program is just a loop of doing nothing
	rjmp	loop

adc_int:			//the ADC interrupt routine
	lds		r16,ADCH		//reading the upper 8 bits of the conversion result
	sts		OCR1AL,r16		//and set them as compare value for the pwm
	reti
	
t1_overflow:
	reti			//just return without doing anything