/*
 * H4XEL_AVR.c
 *
 * Created: 2016-12-09 15:38:50
 *  Author: tmk16al
 */ 

#ifndef H4XEL_AVR_H
#define H4XEL_AVR_H


/*Setup for the interrupts connected to the encoder, and sets original stage*/
void setupLEDS(void);
void setupInterrupt(void);
void setupPWM(void);
void setupUSART(void);
void setupClock(void);
void control(void);
void switchLEDS(void);
void USART_Transmit(unsigned char data);

#endif