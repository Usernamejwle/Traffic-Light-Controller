// TrafficLight.c
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.
// Your Name: Gavin Fielder
// last modified by Gavin Fielder 2/27/2018
// 

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2 Program 3.1, Example 3.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include "stdint.h"

// Ports
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	
#define GPIO_PORTA_DATA_R       (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
	
#define GPIO_PORTC_DATA_R       (*((volatile unsigned long *)0x400063FC))
#define GPIO_PORTC_DIR_R        (*((volatile unsigned long *)0x40006400))
#define GPIO_PORTC_AFSEL_R      (*((volatile unsigned long *)0x40006420))
#define GPIO_PORTC_DEN_R        (*((volatile unsigned long *)0x4000651C))
#define GPIO_PORTC_AMSEL_R      (*((volatile unsigned long *)0x40006528))
#define GPIO_PORTC_PCTL_R       (*((volatile unsigned long *)0x4000652C))
	
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))

#define SYSTICK_STCTRL					(*((volatile unsigned long *)0xE000E010))
#define SYSTICK_STRELOAD				(*((volatile unsigned long *)0xE000E014))			
#define SYSTICK_STCURRENT				(*((volatile unsigned long *)0xE000E018))
	
//Pin usage masks
#define PORTA_USING 0x1C //PA2-4 output
#define PORTC_USING 0xF0 //PC4-7 input
#define PORTE_USING 0x3E //PE1-5 output
/*
* Inputs:
* 	PC4: First street walk button
* 	PC5: Warner street walk button
* 	PC6: First street traffic sensor
* 	PC7: Warner street traffic sensor
* Outputs:
* 	PA2: 1st street green light
* 	PA3: 1st street yellow light
* 	PA4: 1st street red light
* 	PE1: Warner street green light
* 	PE2: Warner street yellow light
* 	PE3: Warner street red light
* 	PE4: First walk light
* 	PE5: Warner walk light
*/ 
 
	
//Clock frequency
const uint8_t fbus = 16; //MHz

//Function prototypes
void Systick_Delay_1sec(void);
void Systick_Delay(uint16_t s);
void Systick_Init(void);
void PortA_Init(void);
void PortC_Init(void);
void PortE_Init(void);
void PortA_Output(uint8_t out);
void PortE_Output(uint8_t out);

//State structure
typedef struct state_struct {
	uint16_t delay; //seconds 2 bytes
	uint8_t outA; //output 1 byte
	uint8_t outE; //output 1 byte
	const struct state_struct* next[16]; //4x16=64 bytes
} state_t; //68 bytes total

//States
const state_t s[7] = {
	// Condition...  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
	{7,0x10,0x02,{&s[0],&s[5],&s[1],&s[1],&s[1],&s[5],&s[1],&s[1],&s[0],&s[5],&s[1],&s[1],&s[1],&s[5],&s[1],&s[1]}}, //WarnerGo
	{2,0x10,0x04,{&s[2],&s[2],&s[4],&s[4],&s[2],&s[2],&s[4],&s[4],&s[2],&s[2],&s[4],&s[4],&s[2],&s[2],&s[2],&s[4]}}, //WarnerWait
	{7,0x04,0x08,{&s[3],&s[3],&s[3],&s[3],&s[2],&s[3],&s[3],&s[3],&s[3],&s[3],&s[3],&s[3],&s[3],&s[3],&s[3],&s[3]}}, //FirstGo
	{2,0x08,0x08,{&s[0],&s[5],&s[4],&s[4],&s[0],&s[5],&s[4],&s[4],&s[0],&s[5],&s[4],&s[0],&s[0],&s[5],&s[4],&s[5]}}, //FirstWait
	{4,0x10,0x38,{&s[0],&s[0],&s[0],&s[0],&s[2],&s[2],&s[2],&s[2],&s[0],&s[0],&s[0],&s[0],&s[0],&s[2],&s[0],&s[2]}}, //WarnerWalk
	{4,0x10,0x12,{&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6],&s[6]}}, //FirstWalk
	{3,0x10,0x02,{&s[0],&s[1],&s[1],&s[1],&s[1],&s[1],&s[1],&s[1],&s[0],&s[0],&s[1],&s[1],&s[1],&s[1],&s[1],&s[1]}}  //WarnerGoShort
};	//68x7=476 bytes total

//State names
#define WarnerGo      s[0]
#define WarnerWait    s[1]
#define FirstGo       s[2]
#define FirstWait     s[3]
#define WarnerWalk    s[4]
#define FirstWalk     s[5]
#define WarnerGoShort s[6]

int main(void){ 
	//Declare variables
	state_t currentState;
	uint8_t condition;
	
	//Initialize peripherals
  Systick_Init();
	PortA_Init(); //PA2-4 output
	PortC_Init(); //PC4-7 input
	PortE_Init(); //PE1-5 output
	
	//Initialize state machine
	currentState = WarnerGo;
	
	//loop precondition: new state
  while(1){
		//Output on new state
		PortA_Output(currentState.outA);
		PortE_Output(currentState.outE);
		//Delay on new state
		Systick_Delay(currentState.delay);
		//Read inputs
		condition = ((GPIO_PORTC_DATA_R & 0xF0) >> 4);
		//Fetch new state
		currentState = *(currentState.next[condition]);
  }
}


// Initializes Port A
// Inputs: None
// Outputs: None
// Notes: Sets all used pins as output
void PortA_Init(void){ 
    int delay;
    SYSCTL_RCGC2_R |= 0x1; //send clock signal
    delay = 0; //wait
    GPIO_PORTA_DIR_R |= PORTA_USING; //set bits (output)
    GPIO_PORTA_AFSEL_R &= ~PORTA_USING; //clear bits
    GPIO_PORTA_AMSEL_R &= ~PORTA_USING; //clear bits
    GPIO_PORTA_DEN_R |= PORTA_USING; //set bits
}

// Initializes Port C
// Inputs: None
// Outputs: None
// Notes: Sets all used pins as inputs
void PortC_Init(void){ 
    int delay;
    SYSCTL_RCGC2_R |= 0x4; //send clock signal
    delay = 0; //wait
    GPIO_PORTC_DIR_R &= ~PORTC_USING; //clear bits (input)
    GPIO_PORTC_AFSEL_R &= ~PORTC_USING; //clear bits 
    GPIO_PORTC_AMSEL_R &= ~PORTC_USING; //clear bits 
    GPIO_PORTC_DEN_R |= PORTC_USING; //set bits 
}

// Initializes Port E
// Inputs: None
// Outputs: None
// Notes: Sets all used pins as outputs
void PortE_Init(void){ 
    int delay;
    SYSCTL_RCGC2_R |= PORTE_USING; //send clock signal
    delay = 0; //wait
    GPIO_PORTE_DIR_R |= PORTE_USING; //set bits (output)
    GPIO_PORTE_AFSEL_R &= ~PORTE_USING; //clear bits 
    GPIO_PORTE_AMSEL_R &= ~PORTE_USING; //clear bits 
    GPIO_PORTE_DEN_R |= PORTE_USING; //set bits 
}

// Initializes Systick
// Inputs: None
// Outputs: None
// Notes: Enables systick
void Systick_Init(void) {
		SYSTICK_STCTRL = 0x00; //disable systick during setup
		SYSTICK_STRELOAD = 0xFFFFFF; 
		SYSTICK_STCURRENT = 0; //any write to current clears
		SYSTICK_STCTRL = 0x05; //enable without interrupt
}

// Delays for a specified time in seconds
// Inputs: s  the time of delay in seconds
// Outputs: None
void Systick_Delay(uint16_t s) {
	int i = 0;
	for (i = 0; i < s; i++)
		Systick_Delay_1sec();
}

// Delays for 1 second
// Inputs: None
// Outputs: None
void Systick_Delay_1sec(void) {
		uint32_t microseconds = 1000000; //1 sec
		SYSTICK_STRELOAD = microseconds*fbus;
		SYSTICK_STCURRENT = 0;
		//wait for systick count bit 1
		while (!(SYSTICK_STCTRL & 0x00010000));
}

// Outputs on Port A
// Inputs: the output values for used pins, unshifted
// Outputs: None
// Notes: Uses friendly code for used pins
void PortA_Output(uint8_t out) {
	GPIO_PORTA_DATA_R |= out; //sets the 1 bits
	GPIO_PORTA_DATA_R &= (out | ~PORTA_USING); //clears the 0 bits
}
	
// Outputs on Port E
// Inputs: the output values for used pins, unshifted
// Outputs: None
// Notes: Uses friendly code for used pins
void PortE_Output(uint8_t out) {
	GPIO_PORTE_DATA_R |= out; //sets the 1 bits
	GPIO_PORTE_DATA_R &= (out | ~PORTE_USING); //clears the 0 bits
}
