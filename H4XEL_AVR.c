/*
 * H4XEL_AVR.c
 *
 * Created: 2016-12-09 15:38:50
 *  Author: tmk16al
 */ 


#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define FOSC 1000000 //Clock speed, 1MHz
#define BAUD 2400 //Baud rate, 2400 bits/sec
#define MYUBRR (FOSC/(16UL*BAUD))-1

#include <H4XEL_AVR.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> 
#include <avr/iomx8.h>



#define A 0x01							//00000001 = A for fast usage and understanding
#define B 0x02							//00000010 = B for fast usage and understanding

volatile uint8_t oldAB;
volatile uint8_t newAB;
volatile uint8_t step = 10;
volatile uint8_t speed = 10;




/*Setup for the interrupts connected to the encoder, and sets original stage*/
void setup() {
	DDRC &= ~((1<<PC0)|(1<<PC1));		//Sets PC0 & PC1 as input
	PORTC = (1<<PC0)|(1<<PC1);			//PC0 & PC1 is mow pull-up enabled
	PCICR = (1<<PCIE1);					//Enables PCINt8 & PCINT9, in control register
	PCMSK1 = (1<<PCINT8)|(1<<PCINT9);	//Enables PCINT8 & PCINT9, in
	oldAB = PINC & (A|B);				//Initialize the first state.
	sei();								//Enables interrupts (SREG)
	DDRC |= (1<<PC5)|(1<<PC4);			//---- For testing ----
}

void setupPWM() {
	DDRD = (1<<PD6);					//Sets PD6/OC0A to output, for PWM
	TCCR0A = (1<<WGM01)|(1<<WGM00)|(1<<COM0A1)|(1<<(COM0A0));	//Fast PWM, set OC0A on Compare Match
	TCCR0B = (1<<CS00);					//Clock select = no prescaling
	OCR0A = 0x00;						//Clear reference value
}

void setupUSART() {

	uint8_t sreg;
	/* Save global interrupt flag */
	sreg = SREG;
	/* Disable interrupts */
	cli();

	/* Set baud rate */
	UBRR0H = 0; //(unsigned char) (ubrr >> 8);
	UBRR0L = 25; //(unsigned char) ubrr;
	
	/* Enable reciever RXD and transmitter TXD */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1 << RXCIE0);
	
	
	/* Set frame format: 8 bit data & 1 stop bit */
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);

	/* Restore global interrupt flag */
	SREG = sreg;

	/* Enable interrupts */
	sei();
	
}

void increase(){
	if (OCR0A <= (255-step)){
		OCR0A += step;
	}
}

void decrease(){
	if (OCR0A >= step){
		OCR0A -= step;
	}
}

void USART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */ 
	while (!(UCSR0A & (1<<UDRE0)))
		;
	
	/* Put data into buffer, sends the data */ 
	UDR0 = data;
}


/*Interrupt Service Routine - Handles an event when the pins changes.*/ 
ISR(PCINT1_vect){
	/*Local variable for only used inside ISR, Looks at pin input register
	  and sets it as the present stage.*/
	newAB = PINC & (A|B);
	
	/* Ren� Sommer algorithm for increasing and decreasing the speed.*/
	if (((((oldAB&(1<<A))>>1) | ((oldAB&(1<<B))<<1)) ^ (newAB)) == 0x01) {
		decrease();
	} else if (((((oldAB&(1<<A))>>1) | ((oldAB&(1<<B))<<1)) ^ (newAB)) == 0x02) {
		increase();
	}
	oldAB = newAB;
}

ISR(USART_RX_vect){
	int recievedByte;
	recievedByte = UDR0;

	if (recievedByte == 250) {
		PORTC ^= (1<<PC5)|(1<<PC4);
	} else {

	}
	/* Echo the recieved byte */
	USART_Transmit(recievedByte);
}



/*The main program begins with a setup for interruptions on PCINT8 & PCINT9
  then a infinite while-loop starts, awaiting for an interrupt to occur.
  The interrupt then handles the events, 
  like increasing or decreasing the speed of the motor.  
*/
int main(void){
	setup();
	setupPWM();
	setupUSART();
	
	while (1){

	}
}