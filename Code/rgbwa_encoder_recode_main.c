/*
 * rgbwa_encoder_recode.c
 *
 * Created: 18/05/2018 23:59:35
 * Author : Moritz Plattner
 * Purpose: A simple RGB, warm- and cold-white LED stripe controller
 *          Up to 6 channels, soft-PWM, use via rotary encoder with a button
 *
 * The code for the rotary encoder was originally written 
 * by Peter Danegger: https://www.mikrocontroller.net/articles/Drehgeber
 * I used his example with little to no modification.
 * My code is written for a Atmega32u4 with a 16MHz clock. Adaptions might be
 * needed for other MCUs or clocks.
 */

// Include header files
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>

// Define in/outputs for better readability
#define enc1		PB4									// Encoder pin 1
#define enc2		PB5									// Encoder pin 2
#define button		PB6									// Button input
#define LED_B		PD0									// Blue
#define LED_W		PD1									// White
#define LED_A		PD2									// Amber
#define LED_X		PD3									// Not used
#define LED_G		PD4									// Green
#define LED_R		PD7									// Red

#define XTAL        16e6								// 16MHz
#define PHASE_A     (PINB & 1<<enc1)
#define PHASE_B     (PINB & 1<<enc2)

volatile int8_t enc_delta;
volatile int8_t taster_delta = 0;

// Table of allowed encoder states used to eliminate bouncing/errors while reading the encoder
const int8_t table[16] PROGMEM = {0,0,-1,0,0,0,0,1,1,0,0,0,0,-1,0,0};
	
// Variables for the individual LED brightnesses
uint8_t R = 0;
uint8_t G = 0;
uint8_t B = 0;
uint8_t W = 0;
uint8_t A = 0;
uint8_t X = 127;									// The unused color is set to ~50% duty cycle to get a 
													// trigger signal if checking the outputs with a scope

uint8_t brightness = 127;							// initial value of the brightness (0 = min, 127= max)
uint16_t color = 0;									// initial value of the RGB color
uint8_t speed = 30;									// initial value of how fast the colors change (higher -> slower)
uint16_t white = 255;								// Initial value of the white color (0 = W, 255 = both max, 511 = A)

uint8_t key_counter;								// Variable used to count how long the button has been pressed
uint8_t key_short;
uint8_t key_long;

ISR(TIMER0_COMPA_vect)								// On timer event A check current encoder state
{
	static int8_t enc_last=0;						// Store previous value

	enc_last = (enc_last << 2)  & 0x0F;
	if (PHASE_A) enc_last |=2;
	if (PHASE_B) enc_last |=1;
	enc_delta = pgm_read_byte(&table[enc_last]);	// +1 if rotated CW, -1 if rotated CCW
	
}

ISR(TIMER0_COMPB_vect)
{
	uint8_t input = PINB & (1 << button);			// Read the button
	
	if (key_counter > 200)							// Check for a long key press
	{
		key_long = 1;								// If so, set the variable and reset the counter
		key_counter = 0;
	}
	
	if (!input)										// If the button is pressed
	{
		key_counter++;								// Raise the counter by one
		if (key_counter > 250)						// And keep it from overflowing
		{
			key_counter = 249;
		}
	}
	else if (key_counter > 3)						// If no button is pressed anymore, but the counter is >3
	{
		key_short = 1;								// Register a short keypress (registered on release)
		key_counter = 0;
	}
	else 
	{
		key_counter = 0;
	}
}

void reg_init(void)									// Initialize registers
{
	DDRD |= ( (1 << LED_R) | (1 << LED_G) | (1 << LED_B) | (1 << LED_W) | (1 << LED_A) | (1 << LED_X));		// Set LED pins as outputs
	PORTB |= ( (1 << enc1) | (1 << enc2) | (1 << button));													// Set Pullups for all inputs
	TCCR0A |= (1<<WGM01);								// CTC
	TCCR0B |= ((1<<CS01) | (1<<CS00));					// XTAL /64
	OCR0A = (uint8_t)(XTAL / 64.0 * 1e-3 - 0.5);
	OCR0B = (uint8_t)(XTAL / 64.0 * 1e-4 - 0.5);
	TIMSK0 |= ((1<<OCIE0A) | (1<<OCIE0B));
	sei();												// allow interrupts
}

int8_t encode_read(void)							// Read current state change of the rotary encoder
{
	int8_t val;
	cli();											// Disable interrupts
	val = enc_delta;
	enc_delta = 0;
	sei();
	return val;										// Allow Interrupts again
}

int8_t get_key_short(void)							// Check for a short key press
{
	uint8_t result;
	cli();
	result = key_short;
	key_short = 0;
	sei();
	return result;
}

int8_t get_key_long(void)							// Check for a long key press
{
	uint8_t result;
	cli();
	result = key_long;
	key_long = 0;
	sei();
	return result;
}

void int_to_rgb(void)								// It is pretty simple and a lot of work to explain, so no comments here
{
	if (color < 256)
	{
		R = 255;
		G = 0 + color;
		B = 0;
	}
	else if (color < 512)
	{
		R = 255 - (color - 256);
		G = 255;
		B = 0;
	}
	else if (color < 768)
	{
		R = 0;
		G = 255;
		B = 0 + (color - 512);
	}
	else if (color < 1024)
	{
		R = 0;
		G = 255 - (color - 768);
		B = 255;
	}
	else if (color <  1280)
	{
		R = 0 + (color - 1024);
		G = 0;
		B = 255;
	}
	else if (color <  1536)
	{
		R = 255;
		G = 0;
		B = 255 - (color - 1280);
	}
	else if (color < 2000)
	{
		color = 0;									// If the value reaches 1536 the variable will be reset to 0.
	}
	else
	{
		color = 1535;								// If the value reaches >2000 (underflow from 0) it is set to 1535
	}												// This way you can color cycle both directions and are reset to the correct point after a over/underflow
}

void int_to_wa(void)								// Same as int_to_rgb(), just with one less LED
{
	if (white < 256)
	{
		W = 255;
		A = 0 + white;
	}
	else if (white < 512)
	{
		W = 255 - (white - 256);
		A = 255;
	}
	else if (white < 768)
	{
		W = 0 + (white - 512);
		A = 255;
	}
	else if (white < 1024)
	{
		W = 255;
		A = 255 - (white - 768);
	}
	else if (white < 2000)
	{
		white = 0;
	}
	else
	{
		white = 1023;
	}
}

void softPWM(void)									// Software PWM for the LEDs
{	
	uint8_t R_out = R*brightness/127;				// Apply multiplicative brightness to all LEDs
	uint8_t G_out = G*brightness/127;
	uint8_t B_out = B*brightness/127;
	uint8_t W_out = W*brightness/127;
	uint8_t A_out = A*brightness/127;
	uint8_t X_out = X*brightness/127;
	if (R_out != 0) PORTD |= ( 1 << LED_R);			// Enable the LEDs only if their brightness values are > 0, so that they stay black if their values are 0
	if (G_out != 0) PORTD |= ( 1 << LED_G);
	if (B_out != 0) PORTD |= ( 1 << LED_B);
	if (W_out != 0) PORTD |= ( 1 << LED_W);
	if (A_out != 0) PORTD |= ( 1 << LED_A);
	if (X_out != 0) PORTD |= ( 1 << LED_X);
	for(uint8_t i=0;i<255;++i)						// Soft-PWM-loop
	{
		if(i>=R_out)	PORTD &= ~(1 << LED_R);		// Disable red
		if(i>=G_out)	PORTD &= ~(1 << LED_G);		// Disable green
		if(i>=B_out)	PORTD &= ~(1 << LED_B);		// ...
		if(i>=W_out)	PORTD &= ~(1 << LED_W);
		if(i>=A_out)	PORTD &= ~(1 << LED_A);
		if(i>=X_out)	PORTD &= ~(1 << LED_X);
	}
}

void change_brightness(void)						// Change the brightness of all LEDs
{
	while (!get_key_short())
	{
		brightness += 2*encode_read();
		if (brightness > (200))						// If a underflow happened, reset it to 0
		{
			brightness = 0;
		}
		else if (brightness > 127)					// If a overflow happened, reset it to 127
		{
			brightness = 127;
		}
		softPWM();									// In order to "preview" the brightness, the PWM loop is run here as well
	}
}

void mode_white(void)								// Change between warm and cold white manually
{
	R = 0;											// Set all RGB LEDs to 0
	B = 0;
	G = 0;
	
	while(!get_key_short())	
	{
		white +=6*encode_read();
		int_to_wa();
		softPWM();
		if (get_key_long())							// After a long key press, switch to brightness regulation
		{
			change_brightness();
		}
	}
}

void mode_rgb_man(void)								// Manually select the desired RGB color
{
	W = 0;											// Set all white LEDs to 0
	A = 0;
	while(!get_key_short())
	{
		color +=6*encode_read();
		int_to_rgb();
		softPWM();
		if (get_key_long())							// After a long key press, switch to brightness regulation
		{
			change_brightness();
		}
	}
}

void mode_rgb_sweep()								// Color "wheel" - cycles through the RGB spectrum
{
	W = 0;											// Set all white LEDs to 0
	A = 0;
	while(!get_key_short())
	{
		color += 1;
		
		for (uint8_t i=0;i<speed;++i)
		{
			speed -= 3*encode_read();				// change the velocity how fast the colors cycle
			
			if (speed < 10)							// The for loop will run with the same color between 10 and 100 times
			{
				speed = 10;
			}
			if (speed > 100)
			{
				speed = 100;
			}
			
			int_to_rgb();
			softPWM();
			if (get_key_long())						// After a long key press, switch to brightness regulation
			{
				change_brightness();
			}
		}
	}
}

int main(void)
{
	reg_init();
	while (1)
	{
		mode_white();
		mode_rgb_man();
		mode_rgb_sweep();
	}
}