/*
 * Team Id: 1079
 * Author List: Sourabh Khemka
 * Filename: Crow-Robot.c
 * Theme: Thirsty Crow (TC)
 * Functions: magnet_pin_config(), motor_pin_config() ,magnet_on(), magnet_off(), forward(), backward(), left(), right(), soft_left(), soft_right(), stop(), main(), ISR(INT4_vect)
 *			  uart0_init(), uart_tx(), ISR(USART0_RX_vect), line_sensor_values(), PWM_init(), line_follow(), reset(), main()
 * Global Variables: count, data, val_1, val_2, val_3, recv_inst, des_val_1, des_val_2, des_val_3, e_val_1, e_val_2, e_val_3, out
 */
#define RX  (1<<4)
#define TX  (1<<3)
#define TE  (1<<5)
#define RE  (1<<7)
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// count : Stores zero on declaration and is incremented on occurrence of interrupt at pin PE4
int count = 0;
volatile unsigned char data;
uint8_t val_1, val_2, val_3;
char recv_inst;
/*uint8_t des_val_1=100, des_val_2=245, des_val_3=140;*/
uint8_t e_val_1, e_val_2, e_val_3;
int out;
int rot_flag = 0, magnet_flag = 0;




/*
 * Function Name: magnet_pin_config()
 * Input: None
 * Output: None
 * Logic: Function sets the Data Direction bit corresponding to the physical pin to be used as OUTPUT as 1
 *
 *        If the DDRxn is set PORTxn outputs 5V when set as 1
 *        If the DDRxn is not set then PINxn will sense the HIGH or LOW signal on the physical pin
 *
 *		  The PORTxn when SET switches on the electromagnet and when CLEARED switches off the electromagnet
 *        Initially the function clears the PORTxn i-e magnet is off when the configuration function is called
 * Example Call: magnet_pin_config();
 */
void magnet_pin_config()
{
	DDRH |= (1<<DDH0);
	PORTH &= ~(1<<PH0);
}




/*
 * Function Name: motor_pin_config()
 * Input: None
 * Output: None
 * Logic: Function sets the Data Direction bit corresponding to the physical pin to be used as OUTPUT as 1
 *
 *        If the DDRxn is set PORTxn outputs 5V when set as 1
 *        If the DDRxn is not set then PINxn will sense the HIGH or LOW signal on the physical pin
 *
 *		  On call the function configures the physical pins 0, 1, 2 & 3 as PORTs and initially clears the bits corresponding to these ports
 *        i-e initially the motors are powered off
 * Example Call: motor_pin_config();
 */
void motor_pin_config()
{
	// 0X0F sets the value of DDRA register as 0000 1111
	// this implies that physical pins PAn where n = 0,1,2,3 will act as PORTs
	DDRA = 0X0F;

	// The following operation clears the value of the bit if it is set and remains cleared if is initially clear
	PORTA &= ~(1<<PA0);
	PORTA &= ~(1<<PA1);
	PORTA &= ~(1<<PA2);
	PORTA &= ~(1<<PA3);
}





/*
 * Function Name: magnet_on()
 * Input: None
 * Output: None
 * Logic: Function sets the bit corresponding to the PORT connected to GATE of MOSFET i-e makes the gate voltage HIGH
 * Example Call: magnet_on();
 */
void magnet_on()
{
	// The following operation sets the bit if its cleared and the value remains as 1 if it is initially 1
	PORTH |= (1<<PH0);
}





/*
 * Function Name: magnet_off()
 * Input: None
 * Output: None
 * Logic: Function CLEARS the bit corresponding to the PORT connected to GATE of MOSFET i-e makes the gate voltage LOW
 * Example Call: magnet_off();
 */
void magnet_off()
{
	PORTH &= ~(1<<PH0);
}



/*
 * Function Name: lock()
 * Input: None
 * Output: None
 * Logic: Locks the wheel by making all direction pins HIGH
 * Example Call: lock();
 */
void lock()
{
	PORTA |= (1<<PA0);
	PORTA |= (1<<PA1);
	PORTA |= (1<<PA2);
	PORTA |= (1<<PA3);
	
	stop();	
}



/*
 * Function Name: forward()
 * Input: None
 * Output: None
 * Logic: Function SETS one and CLEARS the other bit corresponding to the PORTs connected to direction pin of both the motors
 *        This makes the motors to rotate in one direction resulting in forward motion of the crow-bot
 * Example Call: forward();
 */
void forward()
{	
    EIMSK &= ~(1 << INT4);
    rot_flag = 1;
    
    cli();
    PORTE &= ~(1<<PE3) & ~(1<<PE5);
    
    PWM_init();
    OCR3A = OCR3C = 255;
    
    PORTA &= ~(1<<PA0);
    PORTA |= (1<<PA1);
    PORTA &= ~(1<<PA2);
    PORTA |= (1<<PA3);
    
    while(rot_flag)
    {
	    line_sensor_values();
	    if( val_1 < 200 && val_2 < 200 && val_3 < 200)
	    {
		    lock();
		    rot_flag = 0;
	    }
    }
    
    rot_flag = 1;
    PORTA |= (1<<PA0);		PORTA |= (1<<PA2);
    
    while (rot_flag)
    {
	    line_sensor_values();
	    if(val_3 > 200)
	    {
		    lock();
		    rot_flag = 0;
	    }
    }
    
    sei();

}





/*
 * Function Name: backward()
 * Input: None
 * Output: None
 * Logic: Function SETS one and CLEARS the other bit corresponding to the PORTs connected to direction pin of both the motors
 *        The PORT which is SET in case of forward() function is CLEARED here and the PORT which is CLEARED in case of forward() if SET here
 *        This makes the motors to rotate in the direction opposite to that in case of forward(), resulting in backward motion of the crow-bot
 * Example Call: backward();
 */
void backward()
{
    EIMSK &= ~(1 << INT4);
    rot_flag = 1;
    
    cli();
    PORTE &= ~(1<<PE3) & ~(1<<PE5);
    
    PWM_init();
    OCR3A = OCR3C = 255;
    
    rot_flag = 1;
    PORTA |= (1<<PA0);		PORTA |= (1<<PA2);
    
    while (rot_flag)
    {
	    line_sensor_values();
	    if(val_3 > 200)
	    {
		    lock();
		    rot_flag = 0;
	    }
    }
    
    sei();		

}



/*
 * Function Name: pebble()
 * Input: dir
 * Output: None
 * Logic: 
 *
 * Example Call: pebble()
 */
void pebble()
{	
	
	if(recv_inst == 78)
	{
		//FORWARD TO THE PICK-UP POSITION
		count = 0;

		EIMSK |= (1 << INT4);

		while(count < 250)
		{
			PORTA &= ~(1<<PA0);
			PORTA |= (1<<PA1);
			PORTA &= ~(1<<PA2);
			PORTA |= (1<<PA3);
		}

		lock();

		EIMSK &= ~(1 << INT4);	
	
		if(magnet_flag == 0)
			{
				magnet_on();
				magnet_flag = 1;
			}
		else if(magnet_flag == 1)
			{
				magnet_off();
				magnet_flag = 0;
			}			
	
		_delay_ms(1000);
		
		backward();
		forward();
		
		recv_inst = 0;
		uart_tx('1');
	}	


	else if(recv_inst == 75)
	{
		count = 0;
	
		PORTE |= (1<<PE3) | (1<<PE5);

		EIMSK |= (1 << INT4);

		while(count < 420)
		{
			PORTA |= (1<<PA0);
			PORTA &= ~(1<<PA1);
			PORTA |= (1<<PA2);
			PORTA &= ~(1<<PA3);
		}

		lock();

		EIMSK &= ~(1 << INT4);				
	
		count = 0;
    
		PORTE |= (1<<PE3) | (1<<PE5);

		EIMSK |= (1 << INT4);

		while(count < 220)
		{
		PORTA &= ~(1<<PA0);
		PORTA |= (1<<PA1);
		PORTA |= (1<<PA2);
		PORTA &= ~(1<<PA3);
		}

		lock();

		EIMSK &= ~(1 << INT4);

		count = 0;
    
		PORTE |= (1<<PE3) | (1<<PE5);

		EIMSK |= (1 << INT4);

		while(count < 30)
		{
			PORTA &= ~(1<<PA0);
			PORTA |= (1<<PA1);
			PORTA &= ~(1<<PA2);
			PORTA |= (1<<PA3);
		}

		lock();
		
		_delay_ms(200);
		
		if(magnet_flag == 0)
		{
			magnet_on();
			magnet_flag = 1;
		}
		else if(magnet_flag == 1)
		{
			magnet_off();
			magnet_flag = 0 ;
		}
		
		_delay_ms(1000);

		EIMSK &= ~(1 << INT4);
	
		backward();
		forward();
		
		recv_inst = 0;
		uart_tx('1');
	}
	
	else if (recv_inst == 77)
	{
		count = 0;
	
		PORTE |= (1<<PE3) | (1<<PE5);

		EIMSK |= (1 << INT4);

		while(count < 420)
		{
			PORTA |= (1<<PA0);
			PORTA &= ~(1<<PA1);
			PORTA |= (1<<PA2);
			PORTA &= ~(1<<PA3);
		}

		lock();

		EIMSK &= ~(1 << INT4);		
	
		count = 0;
    
		PORTE |= (1<<PE3) | (1<<PE5);

		EIMSK |= (1 << INT4);

		while(count < 220)
		{
			PORTA |= (1<<PA0);
			PORTA &= ~(1<<PA1);
			PORTA &= ~(1<<PA2);
			PORTA |= (1<<PA3);
		}

		lock();

		EIMSK &= ~(1 << INT4);

		count = 0;
    
		PORTE |= (1<<PE3) | (1<<PE5);

		EIMSK |= (1 << INT4);

		while(count < 30)
		{
			PORTA &= ~(1<<PA0);
			PORTA |= (1<<PA1);
			PORTA &= ~(1<<PA2);
			PORTA |= (1<<PA3);
		}

		lock();
		
		_delay_ms(200);
		
		if(magnet_flag == 0)
		{
			magnet_on();
			magnet_flag = 1;
		}
		else if(magnet_flag == 1)
		{
			magnet_off();
			magnet_flag = 0 ;
		}
		
		_delay_ms(1000);
		
		EIMSK &= ~(1 << INT4);
	
		backward();
		forward();
		
		recv_inst = 0;
		uart_tx('1');
	}

}




/*
 * Function Name: soft_left()
 * Input: None
 * Output: None
 * Logic: Function is to turn the bot left by 45 degrees.
 *		  Function enables the interrupt function at PE4 by writing INT4 in EIMSK register as 1
 *		  and the bits corresponding to direction pins of DC motors are set to 1 to make the bot turn in left direction.
 *
 *		  The count variable in function ISR keeps incrementing and once count increments by 190,
 *		  interrupt at PE4 is disabled by clearing INT4 bit in EIMSK register and stop() function is called
 *        to stop the motors.
 *
 *		  This makes the bot to turn left by 45 degrees
 * Example Call: soft_left();
 */
void soft_left()
{   
	cli();
	PWM_init();
	
	OCR3A = OCR3C = 170;
	rot_flag = 1;
	PORTA |= (1<<PA0);
	PORTA &= ~(1<<PA1);
	PORTA &= ~(1<<PA2);
	PORTA |= (1<<PA3);
	while(rot_flag)
	{
		line_sensor_values();
		if(val_1 < 150)
		{
			lock();
			rot_flag = 0;
		}
	}
	
	OCR3A = OCR3C = 255;
	rot_flag = 1;
	PORTA &= ~(1<<PA0);
	PORTA |= (1<<PA1);
	PORTA &= ~(1<<PA2);
	PORTA |= (1<<PA3);
	while(rot_flag)
	{
		line_sensor_values();
		if(val_1 > 150)
		{
			lock();		rot_flag = 0;
		}
	}
	
	
	
	rot_flag = 1;
	PORTA &= ~(1<<PA0);
	PORTA |= (1<<PA1);
	PORTA &= ~(1<<PA2);
	PORTA |= (1<<PA3);
	while(rot_flag)
	{
		line_sensor_values();
		if (val_1 < 100)
		{
			sei();
			
		    count = 0;

		    EIMSK |= (1 << INT4);

		    while(count < 100)
		    {
				PORTA &= ~(1<<PA0);
				PORTA |= (1<<PA1);
				PORTA &= ~(1<<PA2);
				PORTA |= (1<<PA3);
		    }

		    lock(); rot_flag = 0;

		    EIMSK &= ~(1 << INT4);
		}
	}
	
	
	
	
	
	rot_flag = 1;
	
	OCR3A = OCR3C = 180;
	
	PORTA |= (1<<PA0);
	PORTA &= ~(1<<PA1);
	PORTA &= ~(1<<PA2);
	PORTA |= (1<<PA3);
	
	
	while(rot_flag)
	{
		line_sensor_values();
		if(val_3 > 150)	
			{
				lock();
				rot_flag = 0;
			}			
	}
	
	sei();
	
	forward();
}



/*
 * Function Name: soft_right()
 * Input: None
 * Output: None
 * Logic: Function is to turn the bot right by 45 degrees.
 *		  Function enables the interrupt function at PE4 by writing INT4 in EIMSK register as 1
 *		  and the bits corresponding to direction pins of DC motors are set to 1 to make the bot turn in right direction.
 *
 *		  The count variable in function ISR keeps incrementing and once count increments by 190,
 *		  interrupt at PE4 is disabled by clearing INT4 bit in EIMSK register and stop() fucnciton is called
 *		  to stop the motors.
 *
 *		  This makes the bot to turn right by 45 degrees
 * Example Call: soft_right();
 */
void soft_right()
{	
	cli();
	PWM_init();
	
	OCR3A = OCR3C = 170;
	rot_flag = 1;
	PORTA &= ~(1<<PA0);
	PORTA |= (1<<PA1);
	PORTA |= (1<<PA2);
	PORTA &= ~(1<<PA3);
	while(rot_flag)
	{
		line_sensor_values();
		if(val_2 < 150)
		{
			lock();
			rot_flag = 0;
		}
	}
	
	OCR3A = OCR3C = 255;
	rot_flag = 1;
	PORTA &= ~(1<<PA0);
	PORTA |= (1<<PA1);
	PORTA &= ~(1<<PA2);
	PORTA |= (1<<PA3);
	while(rot_flag)
	{
		line_sensor_values();
		if(val_2 > 150)
		{
			lock();		rot_flag = 0;
		}
	}
	
	
	
	rot_flag = 1;
	PORTA &= ~(1<<PA0);
	PORTA |= (1<<PA1);
	PORTA &= ~(1<<PA2);
	PORTA |= (1<<PA3);
	while(rot_flag)
	{
		line_sensor_values();
		if (val_2 < 100)
		{
			sei();
			
			count = 0;

			EIMSK |= (1 << INT4);

			while(count < 100)
			{
				PORTA &= ~(1<<PA0);
				PORTA |= (1<<PA1);
				PORTA &= ~(1<<PA2);
				PORTA |= (1<<PA3);
			}

			lock(); rot_flag = 0;

			EIMSK &= ~(1 << INT4);
		}
	}
	
	
	
	
	
	rot_flag = 1;
	
	OCR3A = OCR3C = 180;
	
	PORTA &= ~(1<<PA0);
	PORTA |= (1<<PA1);
	PORTA |= (1<<PA2);
	PORTA &= ~(1<<PA3);
	
	
	while(rot_flag)
	{
		line_sensor_values();
		if(val_3 > 150)
		{
			lock();
			rot_flag = 0;
		}
	}
	
	sei();
	
	forward();
}



/*
 * Function Name: rotate_360()
 * Input: None
 * Output: None
 * Logic: Function is to rotate the bot by 360 degrees.
 *
 * Example Call: rotate_360();
 */
void rotate_360()
{	
	count = 0;
	sei();
	EIMSK |= (1 << INT4);

	while(count < 400)
	{
		PORTA &= ~(1<<PA0);
		PORTA |= (1<<PA1);
		PORTA |= (1<<PA2);
		PORTA &= ~(1<<PA3);
	}

	lock();

	EIMSK &= ~(1 << INT4);
	
	cli();
	
	rot_flag = 1;
	
	OCR3A = OCR3C = 150;
	
	PORTA &= ~(1<<PA0);
	PORTA |= (1<<PA1);
	PORTA |= (1<<PA2);
	PORTA &= ~(1<<PA3);
	
	
	while(rot_flag)
	{
		line_sensor_values();
		if(val_3 > 220)
		{
			lock();
			rot_flag = 0;
		}
	}
	
	sei();
	
	forward();
}


/*
 * Function Name: stop()
 * Input: None
 * Output: None
 * Logic: Function CLEARS the bits corresponding to all the PORTS connected to the direction pins of the motor driver
 *        This stops the rotation of both the motors i-e stops the crow-bot's movement
 * Example Call: stop();
 */
void stop()
{
	PORTA &= ~(1<<PA0);
	PORTA &= ~(1<<PA1);
	PORTA &= ~(1<<PA2);
	PORTA &= ~(1<<PA3);
}



/*
 * Function Name: ISR()
 * Input: INT4_vect
 * Output: None
 * Logic: The function is INTERRUPT SERVICE ROUTINE for INT4 i-e the function is called on interrupt at PE4
 *		  The input INT4_vect specifies the interrupt pin with which the function is associated.
 *		  Function increments the global count variable by one i-e at each interrupt at PE4 the global variable
 *		  count is incremented by one.
 * Example Call: ISR(INT4_vect);
 */
ISR(INT4_vect)
{
    count ++;
}



/*
 * Function Name: uart0_init()
 * Input: None
 * Output: None
 * Logic: Function initializes required registers for UART communication 
* Example Call:  uart0_init()
*/
void uart0_init()
{
	UCSR0B = 0x00;							//disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; 							//9600BPS at 14745600Hz
	UBRR0H = 0x00;
	UCSR0B = 0x98;
	//UCSR0C = 3<<1;							//setting 8-bit character and 1 stop bit
	//UCSR0B = RX | TX;
}



/*
 * Function Name: uart_tx(char data)
 * Input: USART0_RX_vect
 * Output: None
 * Logic: The function stores the 8 bit input value in the UDR register when the transmit flag is HIGH 
*			to transmit the data over UART
* Example Call:  uart_tx(char data)
*/
void uart_tx(char data)
{
	while(!(UCSR0A & TE));						//waiting to transmit
	UDR0 = data;
}



/*
 * Function Name: ISR(USART0_RX_vect)
 * Input: USART0_RX_vect
 * Output: None
 * Logic: It is function for Interrupt Service Routine and is called when an internal interrupt occurs 
*			corresponding to USART0_RX_vect interrupt
* Example Call:  ISR(USART0_RX_vect)
*/
ISR(USART0_RX_vect)
{
	recv_inst = UDR0;
}




/*
 * Function Name: line_sensor_values()
 * Input: None
 * Output: None
 * Logic: Function first initializes Analog to Digital Conversion at the ADC pins of micro controller
*			then converts the incoming analog values to digital values ranging from 0-255 and refreshes the
*			values in corresponding global variables
* Example Call: line_sensor_values();
*/
void line_sensor_values()
{
	
	// INTITIALISING ADC
	//ADMUX |= (1<<ADLAR)|(1<<REFS0);
	//ADCSRA |= (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);

	// CONNECTING ANALOG PIN PF0 TO ADC	
	
	ADMUX &= ~(1<<MUX0) & ~(1<<MUX1) & ~(1<<MUX2) & ~(1<<MUX3);
	ADCSRA |= (1<<ADSC);
	while((ADCSRA&0x10)==0);	// WAIT FOR CONVERSION TO COMPLETE (using ADC interrupt flag to do so) 
	val_2 = ADCH;				// STORING VALUES OF THE 8 BITS OF ADCH IN uint_8 VAR 
	ADCSRA = ADCSRA|0x10;		// clear the ADC interrupt flag (ADIF) by setting it to 1

	// CONNECTING ANALOG PIN PF2(ADC2) TO ADC		
	ADMUX |= (1<<MUX1);
	ADCSRA |= (1<<ADSC);
	while((ADCSRA&0x10)==0);		
	val_1 = ADCH;
	ADCSRA = ADCSRA|0x10;
	
	// CONNECTING ANALOG PIN PF3(ADC3) TO ADC
	ADMUX |= (1<<MUX0);
	ADCSRA |= (1<<ADSC);
	while((ADCSRA&0x10)==0);	
	val_3 = ADCH;
	ADCSRA = ADCSRA|0x10;
}



/*
 * Function Name: PWM_init()
 * Input: None
 * Output: None
 * Logic: Function sets the required registers and flip flops for generation of PWM pulses in the OCR output pins
 * Example Call: PWM_init();
 */
void PWM_init()
{	
	ICR3 = 0XFF;
	TCCR3A = (1<<WGM31) | (1<<COM3A1) | (1<<COM3C1);
	TCCR3B = (1<<WGM33) | (1<<CS30);
}



/*
 * Function Name: reset()
 * Input: None
 * Output: None
 * Logic: Function is to perform software reset with help of Watch Dog
 * Example Call: reset();
 */
void reset()
{			
	WDTCSR = 0X10; //SETS WDCE BIT (WDCE MUST BE SET TO CLEAR WDE(watch dog enable) BIT OR CHANGE TIMING SEQUENCE)
	WDTCSR &= ~(1<<WDP3) & ~(1<<WDP2) & ~(1<<WDP1) & ~(1<<WDP0);  //SOFTWARE RESET ON 2048 CYCLES (16 ms)
	WDTCSR |= (1<<WDE);
	
	while(1);
	 
	// WDRF IS SET IN CASE WATCH DOG RESET OCCURS
	// WDRF ALSO OVERWRITES WDE
	//CLEARING WDRF CLRS WDE AND HELPS MULTIPLE RESETS 
}



/*
 * Function Name: line_follow()
 * Input: None
 * Output: None
 * Logic: Function alligns the bot on the node and is called after every rotation
 * Example Call: line_follow();
 */
void line_follow()
{
	line_sensor_values();
	
	PORTE |= (1<<PE3) | (1<<PE5);
	
	if (1)//abs(val_1 - val_2) > 115
	{
		if (val_1 < val_2)
		{
			sei();
			
			count = 0;

			EIMSK |= (1 << INT4);

			while(count < 30)
			{
				PORTA |= (1<<PA0);
				PORTA &= ~(1<<PA1);
				PORTA |= (1<<PA2);
				PORTA &= ~(1<<PA3);
			}

			lock(); rot_flag = 0;

			EIMSK &= ~(1 << INT4);
			
			sei();
			
			count = 0;

			EIMSK |= (1 << INT4);

			while(count < 150)
			{
				PORTA &= ~(1<<PA0);
				PORTA |= (1<<PA1);
				PORTA |= (1<<PA2);
				PORTA &= ~(1<<PA3);
			}

			lock(); rot_flag = 0;

			EIMSK &= ~(1 << INT4);
			
			
			cli();
			PWM_init();
			rot_flag = 1;
			
			OCR3A = OCR3C = 165;
			
			PORTA |= (1<<PA0);
			PORTA &= ~(1<<PA1);
			PORTA &= ~(1<<PA2);
			PORTA |= (1<<PA3);
			
			
			while(rot_flag)
			{
				line_sensor_values();
				if(val_3 > 150)
				{
					lock();
					rot_flag = 0;
				}
			}
			
			sei();
			
			_delay_ms(500);
			
			forward();
		}
		else if (val_1 > val_2)
		{
			sei();
			
			count = 0;

			EIMSK |= (1 << INT4);

			while(count < 30)
			{
				PORTA |= (1<<PA0);
				PORTA &= ~(1<<PA1);
				PORTA |= (1<<PA2);
				PORTA &= ~(1<<PA3);
			}

			lock(); rot_flag = 0;

			EIMSK &= ~(1 << INT4);
			
			sei();
			
			count = 0;

			EIMSK |= (1 << INT4);

			while(count < 100)
			{
				PORTA |= (1<<PA0);
				PORTA &= ~(1<<PA1);
				PORTA &= ~(1<<PA2);
				PORTA |= (1<<PA3);
			}

			lock(); rot_flag = 0;

			EIMSK &= ~(1 << INT4);
			
			
			cli();
			PWM_init();
			rot_flag = 1;
			
			OCR3A = OCR3C = 165;
			
			PORTA &= ~(1<<PA0);
			PORTA |= (1<<PA1);
			PORTA |= (1<<PA2);
			PORTA &= ~(1<<PA3);
			
			
			while(rot_flag)
			{
				line_sensor_values();
				if(val_3 > 150)
				{
					lock();
					rot_flag = 0;
				}
			}
			
			sei();
			
			_delay_ms(500);
			
			forward();
		}
	}
	

}



/*
 * Function Name: main()
 * Input: None
 * Output: None
 * Logic: main function is executed when the C file is run. main calls all required functions in given order for
 *			proper implementation of theme's hardware part
 * Example Call: Called automatically by the Operating System
 */
int main(void)
{
	motor_pin_config();
	magnet_pin_config();
	uart0_init();
	
	// INITIALIZING ADC
	DDRF = 0X00;			//ALL pins of PORTF as input
	PORTF = 0X00;			//set pins as floating
	ADCSRA = 0x00;
	ADMUX = 0x20;
	ACSR = 0x80;
	ADCSRA = 0x86;
	
	// INITIALIZING EXTERNAL INTERRUPTS
	EICRB &= ~(1 << ISC00);
	EICRB |= (1 << ISC01);
	EIMSK |= (1 << INT4);
	DDRH |= 0X01;
	DDRE = 0X28;
	
	OCR3A = OCR3C = 255;
	sei();
		
	forward();
		
	while(1)
	{	
		
		if(recv_inst == 76)
		{
			soft_left();
			_delay_ms(1000);
			forward();	
			_delay_ms(100);
			line_follow();
			recv_inst = 0;
			uart_tx('1');
		}
		else if(recv_inst == 82)
		{
			soft_right();
			_delay_ms(1000);
			forward();	
			_delay_ms(100);
			line_follow();
			recv_inst = 0;
			uart_tx('1');
		}
		else if(recv_inst == 79)
		{
			rotate_360();
			_delay_ms(1000);
			forward();	
			_delay_ms(100);
			line_follow();
			recv_inst = 0;
			uart_tx('1');
		}		
		else if(recv_inst == 83)
			{
				lock();
				DDRB = 0X01;
				PORTB |= (1<<PB0);
				_delay_ms(5000);
				PORTB &= ~(1<<PB0);
				recv_inst = 0;
				uart_tx('1');
				_delay_ms(1000);
			}
			
		pebble();
		_delay_ms(1000);								
					
	}
}