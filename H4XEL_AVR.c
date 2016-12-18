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
#define PRESCALER 256
#define RPMCONST (60*FOSC/24/PRESCALER)
#define BUFFERSIZE 8

#include <H4XEL_AVR.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> 
#include <avr/iomx8.h>

volatile uint8_t rpm;
volatile uint16_t timeBuffer[BUFFERSIZE];
volatile uint8_t bufferIndex;
volatile uint32_t totalTime;
volatile uint16_t averageTime;
volatile uint8_t counter;

volatile double Kp = 1.0;
volatile double Ki = 0.5;
volatile double	Kd = 4.0;
volatile double Kb = 1.0;

volatile uint8_t reference = 60;
volatile short error;
volatile short prevError;
volatile double ipart;
volatile double dpart;
volatile short u;
volatile short usat;


void setupLEDS() {
	PORTC = (1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5);
	DDRC = (1<<PC0);
}

/*Setup for the interrupts connected to the encoder, and sets original stage*/
void setupInterrupt() {
	PORTD = (1<<PD2)|(1<<PD3);				//Enable pullup
	EICRA = (1<<ISC10)|(1<<ISC11);			//Trigger on INT1 rising edge
	EIMSK = (1<<INT1);						//Enable INT1
	sei();									//Enables interrupts (SREG)
}

void setupPWM() {
	DDRD = (1<<PD6);					//Sets PD6/OC0A to output, for PWM
	TCCR0A = (1<<WGM01)|(1<<WGM00)|(1<<COM0A1)|(1<<(COM0A0));	//Fast PWM, set OC0A on Compare Match
	TCCR0B = (1<<CS00);					//Clock select = no prescaling
	OCR0A = 0x00;						//Clear reference value
}

void setupUSART() {
	/* Set baud rate */
	UBRR0H = 0; //(unsigned char) (ubrr >> 8);
	UBRR0L = 25; //(unsigned char) ubrr;
	/* Enable reciever RXD and transmitter TXD */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	/* Set frame format: 8 bit data & 1 stop bit */
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
}

void setupClock() {
	TCCR1B = (1<<CS12); 				//Prescaler 256 bit
	TCNT1 = 0;
}

void control() {
	error = (reference - rpm);
	ipart += error;
	dpart = error - prevError;
	u = (Kp * error) + (Ki * ipart) + (Kd * dpart);
	prevError = error;

	if (u < 0) {
		usat = 0;
	} else if (u > 255) {
		usat = 255;
	} else {
		usat = u;
	}

	ipart -= Kb * (u - usat);					//Anti-Windup
	OCR0A = usat;
}

void switchLEDS() {
	if (DDRC == (1<<PC5)) {
		DDRC = (1<<PC0);
	} else {
		DDRC = (DDRC<<1);		
	}
}

void USART_Transmit(unsigned char data) {
	/* Wait for empty transmit buffer */ 
	while (!(UCSR0A & (1<<UDRE0)));
	
	/* Put data into buffer, sends the data */ 
	UDR0 = data;
}

/*Interrupt Service Routine - Handles an event when the pins changes.*/ 
ISR(INT1_vect) {
	uint16_t time = TCNT1;
	uint8_t pinFilter = PIND & (1<<PD2);

	if (time > 55){						//Ridiculous time filter
		if (pinFilter != 0x04) {		//INT0 low filter

			/* Adds the time into a ringbuffer - 8 values. timeBuffer is unint16_t */
			timeBuffer[bufferIndex] = time;
			bufferIndex = (bufferIndex + 1) % BUFFERSIZE;

			/* Calculates the average time. totalTime is uint32_t and averageTime is uint16_t */
			totalTime = timeBuffer[0] + timeBuffer[1] + timeBuffer[2] + timeBuffer[3]
					+ timeBuffer[4] + timeBuffer[5] + timeBuffer[6] + timeBuffer[7];
			averageTime = (totalTime>>3);
			TCNT1 = 0;

			if (counter == 3) {
				switchLEDS();
				counter = 0;
			} else {
				counter += 1;
			}
		}
	}
}

ISR(USART_RX_vect) {
	uint8_t recievedByte;
	recievedByte = UDR0;

	if (recievedByte == 250) {
		USART_Transmit(rpm);
	} else {
		reference = recievedByte;
	}
}

/* Main loop */
int main(void){
	setupLEDS();
	setupInterrupt();
	setupPWM();
	setupUSART();
	setupClock();

	while (1) {
		rpm = RPMCONST/averageTime;

		if (TCNT0 == 255 || TCNT0 == 127) {				//Update controller when TCNT0 reaches MAX
			control();
		}
	}
}

	// /*Local variable for only used inside ISR, Looks at pin input register
	//   and sets it as the present stage.*/
	// newAB = PINC & (A|B);
	
	// /* René Sommer algorithm for increasing and decreasing the speed.*/
	// if (((((oldAB&(1<<A))>>1) | ((oldAB&(1<<B))<<1)) ^ (newAB)) == 0x01) {
	// } else if (((((oldAB&(1<<A))>>1) | ((oldAB&(1<<B))<<1)) ^ (newAB)) == 0x02) {
	// }
	// oldAB = newAB;