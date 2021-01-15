/******************************************************************************/
/* BLINKY.C: LED Flasher                                                      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/
                  
#include <stdio.h>
#include <stdbool.h>
#include <LPC23xx.H>                    /* LPC23xx definitions                */
#include "LCD.h"                        /* Graphic LCD function prototypes    */


#include "type.h"
#include "demo.h"



/* Import external IRQ handlers from IRQ.c file                               */
extern __irq void T0_IRQHandler  (void);
extern __irq void ADC_IRQHandler (void);


/* Import external variables from IRQ.c file                                  */
extern short AD_last;
extern unsigned char clock_1s;

void delay ()
{
    unsigned int temp, ct;
    for(ct=0; ct<300; ct++) {
        for(temp=0; temp < 65000; temp++);
    }
}

void delay_1 ()
{
    unsigned int temp, ct;
    for(ct=0; ct<100; ct++) {
        for(temp=0; temp < 65000; temp++);
    }
}

void delay_2 ()
{
    unsigned int temp, ct;
    for(ct=0; ct<50; ct++) {
        for(temp=0; temp < 65000; temp++);
    }
}
/*************************************************************************************************************************/

/* Initialising the PWM block to control the servo motors */

/* Generates the pulse 
 *
 * 	_________		  		 _________		   		_________
 *	|				|     		| 				|		  		|				|
 *	|				|	 	  		|					|     		|				|
 *	| 			|		  		|					|		  		|				|
 *	|				|_________| 				|_________|				|_________
 *  <--2ms-->
 *  <-------20ms------>
 *  <---------------------------5s---------------------------->
 *
 * This pulse is required to rotate the servo motor from 0 to 180 degrees
 * The pulse is applied for a duration of 5 seconds
 * During these 5 seconds, the servo motor will hold its position and resist from moving out of its position
 * After 5 seconds, the servo is free to rotate when an external torque is applied
 *
 */


bool locked = true;

/* Initialises the PWM block (But does not enable it)
 * Must be called in main() before while(1)
 */
void pwm_init(){

	PINSEL3 |= (1<<15) | (0<<14); 														/* Pin 1.23 = PWM (Alt Fn 2) 																	*/
	PCONP |= 1<<6;																		/* Turn ON Pulse Width Modulator              								*/
	PCLKSEL0 |= (0<<13) | (0<<12); 														/* Clock to PWM = 12MHz																				*/
	
	PWM1CTCR = 0;																		/* PWM Timer in Timer mode of operation												*/
	PWM1PCR = 0x00;																		/* Single edge PWM mode																				*/
		
	PWM1PR = 12000 - 1;																	/* Setting resolution to 1ms																	*/		
	PWM1MR0 = 20;																		/* Setting time period to 20ms																*/	
    PWM1MR4 = 2;																		/* Setting Ton = 2ms																					*/
	
	PWM1MCR = 1<<1;																		/* PWMTC is reset on match with PWMMR0												*/
	PWM1LER = (1<<4) | (1<<0);															/* Update match registers PWMMR0 and PWMMR4										*/
	
	PWM1PCR = 1<<12; 																	/* PWM1 output enabled																				*/
	PWM1TCR = 1<<1;																		/* Reset PWMTC and PWMPR																			*/
}

/* Initialises Timer0 (But does not enable it)
 * Must be called in main() before while(1)
 */
void timer0_init(){
	
	PCONP |= 1<<0;																		/* Turn ON Timer0																							*/
	PCLKSEL0 |= (0<<3) | (0<<2);														/* Clock to Timer0 = 12MHz                    								*/
	T0CTCR = 0;																			/* Timer0 in Timer mode of operation          								*/		
	T0PR = 12000000 - 1;																/* Setting Timer0 resolution to 1s            								*/	
	T0MR0 = 5;																			/* Setting Match register 0 to 5s             								*/
	T0MCR = 0x05;																		/* Stop Timer0 and raise interrupt on match   								*/
	VICVectAddr4 = (unsigned long)T0_IRQHandler;										/* Address of Timer0 ISR																			*/
	VICVectCntl4  = 14;                          										/* use it for Timer0 Interrupt 																*/
	VICIntEnable  = (1 << 4);                   										/* Enable Timer0 Interrupt     																*/
}

/* Sets Ton = 2ms to achieve the duty cycle needed to rotate the motor 180 degrees
 * Must be called in while(1) after start_timer()
 */
void unlock(){

	locked = false;
	PWM1MR4 = 2;																		/* Setting Ton = 2ms to rotate servo by 180 degrees 					*/
	PWM1LER = (1<<4) | (1<<0);															/* Update match registers PWMMR0 and PWMMR4										*/
}

/* Sets Ton = 1ms to achieve the duty cycle needed to rotate the motor 0 degrees
 * Must be called in while(1) after start_timer()
 */
void lock(){
	
	locked = true;
	PWM1MR4 = 1;																		/* Setting Ton = 2ms to rotate servo by 0 degrees 						*/
	PWM1LER = (1<<4) | (1<<0);															/* Update match registers PWMMR0 and PWMMR4										*/
}

/* Enables Timer0 and the PWM block
 * Must be called in while(1) before lock() and unlock() so that PWM is enabled and
 * the signal runs for a duration of 5s
 */
void start_timer(){

	PWM1TCR = (1<<0) | (1<<3); 															/* Enable PWM Timer Counters and PWM mode											*/
	T0TCR = 1<<1;
	T0TCR = 1<<0;
} 



/**************************************************************************************************************************/

/* Mapping the GPIO pins to the keypad */

#define c1 (IOPIN1&1<<20)
#define c2 (IOPIN1&1<<21)
#define c3 (IOPIN1&1<<22)
 
unsigned char r_loc,c_loc;
unsigned char key[4][3]={"123","456","789","*0#"};
unsigned char keypad(void);

unsigned char keypad()
{		
	IODIR1 |= 0xf00fc;
    IOPIN1 &= ~(0xff<<16);
    IOPIN1 |= 0xf0<<16;
 
    while(c1 && c2 && c3);
    while(!c1 || !c2 || !c3) {
        if(!c1 && c2 && c3)     c_loc=0;
        else if(c1 && !c2 && c3)    c_loc=1;
        else if(c1 && c2 && !c3)    c_loc=2;
 
        IOCLR1 = 1<<16;
        IOSET1 = 0x0e<<16;
        if(!c1 || !c2 || !c3) {
            r_loc=0;
            break;
        }
 
        IOCLR1 = 1<<17;
        IOSET1 = 0x0d<<16;
        if(!c1 || !c2 || !c3) {
            r_loc=1;
            break;
        }
        
        IOCLR1 = 1<<18;
        IOSET1 = 0x0b<<16;
        if(!c1 || !c2 || !c3) {
            r_loc=2;
            break;
        }
 
        IOCLR1 = 1<<19;
        IOSET1 = 0x07<<16;
        if(!c1 || !c2 || !c3) {
            r_loc=3;
            break;
        }
    }
    while(!c1 || !c2 || !c3);
    return (key[r_loc][c_loc]);
}

/*****************************************************************************************************************************/

/* Logic to check for correct/incorrect password by reading the keys pressed and controlling the servo-motor accordingly */ 

int main (void) {
	long int sounds[40] = {
	0x52,0x49, 0x46 ,0x46, 0x68,0x39 ,0x85, 0x00, 0x57 ,0x41, 0x56 ,0x45, 0x66, 0x6D ,0x74 ,0x20 ,0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x44, 0xAC ,0x00, 0x00, 0x10 ,0xB1, 0x02, 0x00, 0x04, 0x00, 0x10, 0x00, 0x64, 0x61, 0x74,0x61};
	int try = 0;
	int i,k;
	unsigned char pass[4] = "1234";
	int count;
	char str1pr[10];
	unsigned char rx_arr[4];
	
	lcd_init();
	lcd_clear();
	PINSEL1 = 0x200000;
	pwm_init();
	timer0_init();
	
	while(1) {
		
		start:
		lcd_clear();
		if (try == 1){
			
			lcd_clear();
			lcd_print("no entry");
			delay();
			for (k=0;k<10;k++){
				for (i=0;i<40000;i++){
					DACR = sounds[i%40] << 6;
				}
				delay_1();
			}	
			goto start1;
		}	
		lcd_clear();
		lcd_print("Password: ");
		
		for(count=0; count <4; count++) {
	
			delay_2();
			set_cursor (0, 1);		
			rx_arr[count] = keypad();
			lcd_init();
			
			if (count ==0 ){
				lcd_clear();
			}
			sprintf(str1pr, "%c" , rx_arr[count]);
			set_cursor (count,0);
			//lcd_print(str1pr);
			lcd_print("*");
			if (count == 3)
				delay_1();
		 }
				 
		if ( (pass[0] == rx_arr[0]) && (pass[1] == rx_arr[1]) && (pass[2] == rx_arr[2]) &&  (pass[3] == rx_arr[3]) ){
			
			lcd_clear();
			lcd_print("Correct");	
			if(locked){
				
				start_timer();
				unlock();
			}
			else{
				start_timer();
				lock();
			}
			rx_arr[0]=0;rx_arr[1]=0;rx_arr[2]=0;rx_arr[3]=0;
			delay();
		}
		else{

				try = try+1;
				lcd_clear();
				lcd_print("incorrect password");
				rx_arr[0]=0;rx_arr[1]=0;rx_arr[2]=0;rx_arr[3]=0;
				delay();
				goto start;
			}
		}
		start1:
}

/* END */

/*************************************************************************************************************************/
