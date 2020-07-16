// Jacinta Roberts (n9954619)
// Assignment 2: Race to Zombie Mountain! (TeensyPewPew!)
//
//--- INCLUDE FILES ---//
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include <cpu_speed.h>
#include <avr/interrupt.h>
#include <graphics.h>
#include <lcd.h>
#include <macros.h>
#include <sprite.h>
#include <usb_serial.h>

#include "lcd_model.h"
#include "lcd.h"
#include "cpu_speed.h"
#include "sprite.h"
#include "graphics.h"
#include "macros.h"
#include "cab202_adc.h"

//--- PINS & PORTS ---//
// Up Joystick - D1
// Down Joystick - B7
// Left Joystick - B1
// Right Joystick - D0
// Center Joystick - B0
// L Button - F6
// R Button - F5
// L LED - B2
// R LED - B3

//--- CONFIGURATION ---//
#define FREQ (8000000.0)
#define PRESCALE (256)
#define DELAY (30)
#define TOP_BORDER (8)
#define CAR_WIDTH (8)
#define CAR_HEIGHT (7)
#define SCENERY_WIDTH (7)
#define SCENERY_HEIGHT (7)
#define OBSTACLE_WIDTH (7)
#define OBSTACLE_HEIGHT (4)
#define FUEL_WIDTH (8)
#define FUEL_HEIGHT (8)
#define MAX_FUEL (100)
#define FUEL_CONSUMPTION (0.06) // 0.04
#define OVERFLOW_TOP (1023)
#define ADC_MAX (1023)
#define BIT(x) (1 << (x))

//--- GLOBAL VARIABLES ---//
volatile uint32_t overflow_count = 0; /* Timer overflow counts */
volatile uint32_t speed_overflow = 0;
volatile uint32_t fuel_overflow = 0;
volatile uint8_t counter[7]; /* Debouncing switches */
volatile uint8_t pressed[7];

bool game_over = false;
bool fuel_on_screen = false;
double speed = 0; /* Dashboard variables */
double paused_speed = 0;
double distance = 0;
double fuel = 100;
double condition = 100;
double cur_time = 0; /* Time display */
double finish_time = 0;
double finish_y = TOP_BORDER + 1;
double vary_step = 0; /* Curved road position */
int road_left_x[84];
int road_right_x[84];
int sprite_x_pos = 0; /* Sprite X and Y position */
int sprite_y_pos = 0; 
int road_width; /* Difficulty */
int offset;
int finish_distance;
int finish_left = 0; /* Finish line position */
int finish_right;

Sprite car;
Sprite house;
Sprite hill;
Sprite cactus1;
Sprite cactus2;
Sprite oasis;
Sprite rock;
Sprite fuel_depot;
Sprite zombie;

// Sprite bitmaps
uint8_t car_bitmap[] = {	0b00011000, 
							0b00011000, 
							0b11111111,	
							0b00111100,	
							0b00111100,  
							0b11111111,	
							0b00011000, };					

uint8_t house_bitmap[] = {	0b00010000, 
							0b00111000, 
							0b11111110,	
							0b10010010,
							0b10010010,							
							0b10010010,  
							0b11111110, };	
							
uint8_t cactus_bitmap[] = {	0b00111000,
							0b10111010, 
							0b10111010, 
							0b01111100,	
							0b00111000,	
							0b00111000,  
							0b01111100, };	

uint8_t hill_bitmap[] = {	0b11000110,
							0b10101010, 
							0b10010010, 
							0b10010010,	
							0b10000010,	
							0b10000010,  
							0b10000010, };								
							
uint8_t oasis_bitmap[] = {	0b00111000,
							0b01000100, 
							0b10000010, 
							0b10000010,	
							0b01000010,	
							0b01000100,  
							0b00111000, };								
							
uint8_t rock_bitmap[] = {	0b00110000, 
							0b00111000,	
							0b01111100,	
							0b11111110, };
							
uint8_t zombie_bitmap[] = {	0b1111110, 
							0b0011000,	
							0b0100000,	
							0b1111110, };
			
uint8_t fuel_depot_bitmap[] = {	0b11111111,
								0b10000000,
								0b10000000, 
								0b11111111, 
								0b10000000,	
								0b10000000,	
								0b10000000,  
								0b10000000, };

// Direct screen write bitmaps and arrays								
uint8_t heart_original[] = {  0b00000000,
							  0b01101100,
							  0b11111110,
							  0b11111110,
							  0b01111100,
							  0b00111000,
							  0b00010000,
							  0b00000000, };

uint8_t heart_direct[8];																

uint8_t arrow_original[] = {  0b00000000,
							  0b00010000,
							  0b00111000,
							  0b01111100,
							  0b00010000,
							  0b00010000,
							  0b00010000,
							  0b00000000, };

uint8_t arrow_direct[8];

uint8_t fuel_original[] = {  0b10000000,
							 0b10111000,
							 0b01000110,
							 0b10000101,
							 0b10000101,
							 0b10000110,
							 0b11111000,
							 0b00000000, };

uint8_t fuel_direct[8];

//--- BASIC CONTROLS AND LAYOUT FUNCTIONS ---//

// Transmits a string using usb_serial
void usb_serial_send( char * message ) {
	// Type cast of uint8_t is necessary to avoid:
	// "error: pointer targets in passing argument 1 of 'usb_serial_write' differ in signedness."
	usb_serial_write((uint8_t *) message, strlen(message));
}

// Initialise adc
void adc_init() {
	// ADC Enable and pre-scaler of 128: ref table 24-5 in datasheet
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Read adc
uint16_t adc_read(uint8_t channel) {
	// Select AVcc voltage reference and pin combination.
	// Low 5 bits of channel spec go in ADMUX(MUX4:0)
	// 5th bit of channel spec goes in ADCSRB(MUX5).
	ADMUX = (channel & ((1 << 5) - 1)) | (1 << REFS0);
	ADCSRB = (channel & (1 << 5));

	// Start single conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1 << ADSC);

	// Wait for ADSC bit to clear, signalling conversion complete.
	while ( ADCSRA & (1 << ADSC) ) {}

	// Result now available.
	return ADC;
}

// Set Duty Cycle for PWM
void set_duty_cycle(int duty_cycle) {
	// Set bits 8 and 9 of Output Compare Register 4A.
	TC4H = duty_cycle >> 8;

	// Set bits 0..7 of Output Compare Register 4A.
	OCR4A = duty_cycle & 0xff;
}

// Turn on LEDs
void turn_leds_on( void ) {
	SET_BIT(PORTB, 2);
	SET_BIT(PORTB, 3);
}

// Turn off LEDs
void turn_leds_off( void ) {
	CLEAR_BIT(PORTB, 2);
	CLEAR_BIT(PORTB, 3);
}

// Function to draw double to 0 decimal places
void draw_double(uint8_t x, uint8_t y, double value, colour_t colour) {
	char buffer[20];
	snprintf(buffer, sizeof(buffer), "%.0f", value);
	draw_string(x, y, buffer, colour);
}

// Function to draw double to 0 decimal places
void draw_double_2(uint8_t x, uint8_t y, double value, colour_t colour) {
	char buffer[20];
	snprintf(buffer, sizeof(buffer), "%.2f", value);
	draw_string(x, y, buffer, colour);
}

// Function to draw integer
void draw_int(uint8_t x, uint8_t y, int value, colour_t colour) {
	char buffer[20];
	snprintf(buffer, sizeof(buffer), "%d", value);
	draw_string(x, y, buffer, colour);
}

//--- USB BIDIRECTION SERIAL COMMUNICATION FUNCTIONS ---//

// Send current game state snapshot to the server
void save_game( void ){
	// Display 'save game' screen on TEENSY
	clear_screen();
	draw_string(0, 0, "Saving game...", FG_COLOUR);
	show_screen();
	_delay_ms(500);
	clear_screen( );
	
	// Start save input stream with an 'S' flag
	usb_serial_putchar('S');
	
	// Send speed
	char speedString[10];
	sprintf(speedString, "%.0f,", paused_speed);
	for (int i = 0; i < strlen(speedString); i++){
		usb_serial_putchar(speedString[i]);
	}
	
	// Send condition
	char conditionInfo[10];
	sprintf(conditionInfo, "%.0f,", condition);
	for (int i = 0; i < strlen(conditionInfo); i++){
		usb_serial_putchar(conditionInfo[i]);
	}

	// Send fuel
	char fuelInfo[10];
	sprintf(fuelInfo, "%.0f,", fuel);
	for (int i = 0; i < strlen(fuelInfo); i++){
		usb_serial_putchar(fuelInfo[i]);
	}

	// Send distance
	char distanceInfo[10];
	sprintf(distanceInfo, "%.0f,", distance);
	for (int i = 0; i < strlen(distanceInfo); i++){
		usb_serial_putchar(distanceInfo[i]);
	}

	// Send current elapsed time (overflow_count)
	char currentTimeInfo[10];
	sprintf(currentTimeInfo, "%u,", (unsigned int)overflow_count);
	for (int i = 0; i < strlen(currentTimeInfo); i++){
		usb_serial_putchar(currentTimeInfo[i]);
	}

	// Send Car position
	char carXInfo[10];
	char carYInfo[10];
	sprintf(carXInfo, "%.0f,", round(car.x));
	sprintf(carYInfo, "%.0f,", round(car.y));
	for (int i = 0; i < strlen(carXInfo); i++){
		usb_serial_putchar(carXInfo[i]);
	}
	for (int i = 0; i < strlen(carYInfo); i++){
		usb_serial_putchar(carYInfo[i]);
	}
	
	// Send House position
	char houseXInfo[10];
	char houseYInfo[10];
	sprintf(houseXInfo, "%.0f,", round(house.x));
	sprintf(houseYInfo, "%.0f,", round(house.y));
	for (int i = 0; i < strlen(houseXInfo); i++){
		usb_serial_putchar(houseXInfo[i]);
	}
	for (int i = 0; i < strlen(houseYInfo); i++){
		usb_serial_putchar(houseYInfo[i]);
	}
	
	// Send Hill position
	char hillXInfo[10];
	char hillYInfo[10];
	sprintf(hillXInfo, "%.0f,", round(hill.x));
	sprintf(hillYInfo, "%.0f,", round(hill.y));
	for (int i = 0; i < strlen(hillXInfo); i++){
		usb_serial_putchar(hillXInfo[i]);
	}
	for (int i = 0; i < strlen(hillYInfo); i++){
		usb_serial_putchar(hillYInfo[i]);
	}
	
	// Send Cactus1 position
	char cactus1XInfo[10];
	char cactus1YInfo[10];
	sprintf(cactus1XInfo, "%.0f,", round(cactus1.x));
	sprintf(cactus1YInfo, "%.0f,", round(cactus1.y));
	for (int i = 0; i < strlen(cactus1XInfo); i++){
		usb_serial_putchar(cactus1XInfo[i]);
	}
	for (int i = 0; i < strlen(cactus1YInfo); i++){
		usb_serial_putchar(cactus1YInfo[i]);
	}	
			
	// Send Cactus2 position
	char cactus2XInfo[10];
	char cactus2YInfo[10];
	sprintf(cactus2XInfo, "%.0f,", round(cactus2.x));
	sprintf(cactus2YInfo, "%.0f,", round(cactus2.y));
	for (int i = 0; i < strlen(cactus2XInfo); i++){
		usb_serial_putchar(cactus2XInfo[i]);
	}
	for (int i = 0; i < strlen(cactus2YInfo); i++){
		usb_serial_putchar(cactus2YInfo[i]);
	}
			
	// Send Oasis position
	char oasisXInfo[10];
	char oasisYInfo[10];
	sprintf(oasisXInfo, "%.0f,", round(oasis.x));
	sprintf(oasisYInfo, "%.0f,", round(oasis.y));
	for (int i = 0; i < strlen(oasisXInfo); i++){
		usb_serial_putchar(oasisXInfo[i]);
	}
	for (int i = 0; i < strlen(oasisYInfo); i++){
		usb_serial_putchar(oasisYInfo[i]);
	}

	// Send Rock position
	char rockXInfo[10];
	char rockYInfo[10];
	sprintf(rockXInfo, "%.0f,", round(rock.x));
	sprintf(rockYInfo, "%.0f,", round(rock.y));
	for (int i = 0; i < strlen(rockXInfo); i++){
		usb_serial_putchar(rockXInfo[i]);
	}
	for (int i = 0; i < strlen(rockYInfo); i++){
		usb_serial_putchar(rockYInfo[i]);
	}
	
	// Send Zombie position
	char zombieXInfo[10];
	char zombieYInfo[10];
	sprintf(zombieXInfo, "%.0f,", round(zombie.x));
	sprintf(zombieYInfo, "%.0f,", round(zombie.y));
	for (int i = 0; i < strlen(zombieXInfo); i++){
		usb_serial_putchar(zombieXInfo[i]);
	}
	for (int i = 0; i < strlen(zombieYInfo); i++){
		usb_serial_putchar(zombieYInfo[i]);
	}

	// Send Fuel position
	char fuelXInfo[10];
	char fuelYInfo[10];
	sprintf(fuelXInfo, "%.0f,", round(fuel_depot.x));
	sprintf(fuelYInfo, "%.0f,", round(fuel_depot.y));
	for (int i = 0; i < strlen(fuelXInfo); i++){
		usb_serial_putchar(fuelXInfo[i]);
	}
	for (int i = 0; i < strlen(fuelYInfo); i++){
		usb_serial_putchar(fuelYInfo[i]);
	}
	
	// Send vary_step (Road position)
	char varyStepInfo[10];
	sprintf(varyStepInfo, "%.0f,", vary_step);
	for (int i = 0; i < strlen(varyStepInfo); i++){
		usb_serial_putchar(varyStepInfo[i]);
	}
	
	// Send difficulty info
	char finishDistanceInfo[10];
	char roadWidthInfo[10];
	char offsetInfo[10];
	sprintf(finishDistanceInfo, "%d,", finish_distance );
	sprintf(roadWidthInfo, "%d,", road_width );
	sprintf(offsetInfo, "%d,", offset );
	for ( int i = 0; i < strlen(finishDistanceInfo); i++){
		usb_serial_putchar(finishDistanceInfo[i]);
	}
	for ( int i = 0; i < strlen(roadWidthInfo); i++){
		usb_serial_putchar(roadWidthInfo[i]);
	}
	for ( int i = 0; i < strlen(offsetInfo); i++){
		usb_serial_putchar(offsetInfo[i]);
	}
	
	usb_serial_putchar(',');
	usb_serial_putchar('T');
}

// Receive last saved game state snapshot from server
void load_game(){
	// Start Load flag
	usb_serial_putchar('L');
	char loadedString[200];
	unsigned char received = 'E';
	int i = 0;
	bool receiveChars = true;
	int position = 0;

	usb_serial_flush_input();
	
	while (receiveChars){

		// Print screen on Teensy
		while (usb_serial_available( ) == 0){
			clear_screen();
			draw_string(0, 0, "Loading...", 1);
			show_screen();
		}
	
		// Separate string
		while (usb_serial_available( ) ){
			received = usb_serial_getchar( );
			if (received == 'T'){ /* If reached end-flag, stop */
				receiveChars = false;
			}
			loadedString[i] = received;
			i++;
		}

		loadedString[i] = '\0';

		if ( position == 0 ) {
			speed = atof(loadedString); /* Get speed */
			paused_speed = speed;
		}
		else if ( position == 1 ) {
			condition = atof(loadedString); /* Get condition */
		}
		else if ( position == 2 ) {
			fuel = atof(loadedString); /* Get fuel */
		}
		else if ( position == 3 ) {
			distance = atof(loadedString); /* Get distance */
		}
		else if ( position == 4 ) {
			overflow_count = (unsigned int)atoi(loadedString); /* Get overflow_count */
		}
		else if ( position == 5 ) {
			car.x = atoi(loadedString); /* Get Car x-location */
		}
		else if ( position == 6 ) {
			car.y = atoi(loadedString); /* Get Car y-location */
		}
		else if ( position == 7 ) {
			house.x = atoi(loadedString); /* Get House x-location */
		}
		else if ( position == 8 ) {
			house.y = atoi(loadedString); /* Get House y-location */			
		}
		else if ( position == 9 ) {
			hill.x = atoi(loadedString); /* Get Hill x-location */
		}
		else if ( position == 10 ) {
			hill.y = atoi(loadedString); /* Get Hill y-location */			
		}
		else if ( position == 11 ) {
			cactus1.x = atoi(loadedString); /* Get Cactus1 x-location */
		}
		else if ( position == 12 ) {
			cactus1.y = atoi(loadedString); /* Get Cactus1 y-location */			
		}
		else if ( position == 13 ) {
			cactus2.x = atoi(loadedString); /* Get Cactus2 x-location */
		}
		else if ( position == 14 ) {
			cactus2.y = atoi(loadedString); /* Get Cactus2 y-location */			
		}
		else if ( position == 15 ) {
			oasis.x = atoi(loadedString); /* Get Oasis x-location */
		}
		else if ( position == 16 ) {
			oasis.y = atoi(loadedString); /* Get Oasis y-location */			
		}
		else if ( position == 17 ) {
			rock.x = atoi(loadedString); /* Get Rock x-location */
		}
		else if ( position == 18 ) {
			rock.y = atoi(loadedString); /* Get Rock y-location */			
		}
		else if ( position == 19 ) {
			zombie.x = atoi(loadedString); /* Get Zombie x-location */
		}
		else if ( position == 20 ) {
			zombie.y = atoi(loadedString); /* Get Zombie y-location */			
		}
		else if ( position == 21 ) {
			fuel_depot.x = atoi(loadedString); /* Get Fuel Depot x-location */
		}
		else if ( position == 22 ) {
			fuel_depot.y = atoi(loadedString); /* Get Fuel Depot y-location */			
		}
		else if ( position == 23 ) {
			vary_step = atof(loadedString); /* Get vary_step (Curved Road) */
		}
		else if ( position == 24 ) {
			finish_distance = atoi(loadedString); /* Get finish distance (Difficulty) */
		}
		else if ( position == 25 ) {
			road_width = atoi(loadedString); /* Get road width (Difficulty) */
		}
		else if ( position == 26 ) {
			offset = atoi(loadedString); /* Get road offset (Difficulty) */
		}
		
		// Increment the position and flush the input
		position++;
		usb_serial_flush_input( );
		
		// Put Load end flag
		usb_serial_putchar('M');
		
		// Reset index
		i = 0;

	}
}

//--- SET UP FUNCTIONS ---//
	
// LCD - Direct screen write setup (heart, arrow and fuel dashboard)
void setup_direct( void ){
	// Convert to direct - Columns
	for( int i = 0; i < 8; i++){
		// Rows
		for( int j = 0; j < 8; j++){
			// Heart - Condition
			uint8_t bit = BIT_VALUE(heart_original[j], (7-i));
			WRITE_BIT(heart_direct[i], j, bit);
			// Arrow - Speed
			bit = BIT_VALUE(arrow_original[j], (7-i));
			WRITE_BIT(arrow_direct[i], j, bit);
			// Fuel tank - Fuel
			bit = BIT_VALUE(fuel_original[j], (7-i));
			WRITE_BIT(fuel_direct[i], j, bit);
		}
	}
}

// LCD - Direct screen write draw (heart, arrow and fuel tank)
void draw_direct( void ) {
	// Draw heart
	LCD_CMD(lcd_set_function, lcd_instr_basic | lcd_addr_horizontal);
	LCD_CMD(lcd_set_x_addr, 1);
	LCD_CMD(lcd_set_y_addr, 1/8);
	for( int i = 0; i < 8; i++ ){
		LCD_DATA(heart_direct[i]);
	}
	
	// Draw arrow
	LCD_CMD(lcd_set_x_addr, 36); /* Same y-location as above */
	for( int i = 0; i < 8; i++ ){
		LCD_DATA(arrow_direct[i]);
	}
	
	// Draw fuel tank
	LCD_CMD(lcd_set_x_addr, 60); /* Same y-location as above */
	for( int i = 0; i < 8; i++ ){
		LCD_DATA(fuel_direct[i]);
	}
}

// LCD - Direct screen erase (heart, arrow and fuel tank)
void erase_direct(void) {
	// Erase heart
    LCD_CMD(lcd_set_function, lcd_instr_basic | lcd_addr_horizontal);
    LCD_CMD(lcd_set_x_addr, 1);
    LCD_CMD(lcd_set_y_addr, 0);

    for (int i = 0; i < 8; i++) {
        LCD_DATA(0);
    }
	
	// Erase arrow
	LCD_CMD(lcd_set_function, lcd_instr_basic | lcd_addr_horizontal);
    LCD_CMD(lcd_set_x_addr, 36);
    LCD_CMD(lcd_set_y_addr, 0);

    for (int i = 0; i < 8; i++) {
        LCD_DATA(0);
    }
	
	// Erase fuel
	LCD_CMD(lcd_set_function, lcd_instr_basic | lcd_addr_horizontal);
    LCD_CMD(lcd_set_x_addr, 60);
    LCD_CMD(lcd_set_y_addr, 0);

    for (int i = 0; i < 8; i++) {
        LCD_DATA(0);
    }
}

// Debounce switches
void debounce( void ) {
	// Initialise mask to 4 ones and current_bit to an array of 7 integers
	uint8_t mask = 0b00001111;
	
	// Set up an array to store whether each switch is currently pressed
	int control_pressed[7] =
	{
	BIT_VALUE(PIND, 1), // 0: Up - D1
	BIT_VALUE(PINB, 7), // 1: Down - B7
	BIT_VALUE(PINB, 1), // 2: Left - B1
	BIT_VALUE(PIND, 0), // 3: Right - D0
	BIT_VALUE(PINB, 0), // 4: Center - B0
	BIT_VALUE(PINF, 6), // 5: Left Button - F6
	BIT_VALUE(PINF, 5), // 6: Right Button - F5
	};		
	
	// For loop to test each switch
	for ( int i = 0; i < 7; i++ ) {
		// Left shift the counter and bitwise AND it with the mask, then bitwise OR to add history 
		counter[i] = ( (counter[i] << 1) & mask ) | control_pressed[i];

		// Determine the state of each switch, if it has been closed for 7 iterations of the timer, it is officially 'closed'
		if ( counter[i] == mask ) {
			pressed[i] = 1; /* Return true (closed) */
		}
		else if ( counter[i] == 0 ) {
			pressed[i] = 0; /* Return false (open) */
		}
	}
}

// Function to generate an appropriate set of coordinates (1 = scenery, 2 = obstacle, 3 = fuel)
void generate_sprite_coords ( int option ) {
	// Left and Right X-Coordinates of the road
	int safe_left_road = offset + 6; /* hard 36, medium 34, easy 32 = offset + 6 */
	int leftest_road = offset - 6; /* hard 24, medium 22, easy 20 = offset - 6 */
	int safe_right_road = road_width / 2 + 35; /* hard 46, medium 48, easy 50 = road_width/2 + 35 */
	int rightest_road = road_width/2 + 48; /* hard 59, medium 61, easy 63 = road_width/2 + 48 */
	
	// If scenery, get position not on road
	if ( option == 1 ) {
		sprite_y_pos = rand( ) % ( LCD_Y - TOP_BORDER - SCENERY_HEIGHT) + TOP_BORDER;
		do {
			sprite_x_pos = rand( ) % ( LCD_X - 2 - SCENERY_WIDTH) + 1;
		} while ( (sprite_x_pos > leftest_road - SCENERY_WIDTH ) && ( sprite_x_pos < rightest_road ) );
	}
	
	// If obstacle, get a position on the road
	else if ( option == 2 ) {
		sprite_y_pos = rand( ) % ( LCD_Y - TOP_BORDER - OBSTACLE_HEIGHT ) + TOP_BORDER;
		do {
			sprite_x_pos = rand( ) % ( LCD_X - 2 - TOP_BORDER - OBSTACLE_WIDTH) + 1;
		} while ( !( (sprite_x_pos > safe_left_road ) && ( sprite_x_pos < safe_right_road - OBSTACLE_WIDTH ) ) );
	}
	
	// Fuel position, get position on side of road
	else if ( option == 3 ) {
		int choice = rand( ) % (2); /* 50% chance to spawn left or right */
		if ( choice == 1 ) { /* Spawn left side */
			sprite_x_pos = road_left_x[1] - FUEL_WIDTH - 1;
		}
		else if ( choice == 0 ) { /* Spawn right side */
			sprite_x_pos = road_right_x[1] + 2;
		}
	}
}

// Create sprites (car, scenery, obstacles and fuel depot)
void create_sprites( void ) {
	sprite_x_pos = ( LCD_X - CAR_WIDTH ) / 2 + 1;
	sprite_y_pos = LCD_Y - CAR_HEIGHT - 1;
	sprite_init( &car, sprite_x_pos, sprite_y_pos, CAR_WIDTH, CAR_HEIGHT, car_bitmap );
	
	// Create scenery (cactus, house, oasis and hill)
	sprite_init( &house, sprite_x_pos, sprite_y_pos, SCENERY_WIDTH, SCENERY_HEIGHT, house_bitmap );
	sprite_init( &hill, sprite_x_pos, sprite_y_pos, SCENERY_WIDTH, SCENERY_HEIGHT, hill_bitmap );
	sprite_init( &cactus1, sprite_x_pos, sprite_y_pos, SCENERY_WIDTH, SCENERY_HEIGHT, cactus_bitmap );
	sprite_init( &cactus2, sprite_x_pos, sprite_y_pos, SCENERY_WIDTH, SCENERY_HEIGHT, cactus_bitmap );
	sprite_init( &oasis, sprite_x_pos, sprite_y_pos, SCENERY_WIDTH, SCENERY_HEIGHT, oasis_bitmap );
	
	// Create obstacles (rock and zombie)
	sprite_init( &rock, sprite_x_pos, sprite_y_pos, OBSTACLE_WIDTH, OBSTACLE_HEIGHT, rock_bitmap );
	sprite_init( &zombie, sprite_x_pos, sprite_y_pos, OBSTACLE_WIDTH, OBSTACLE_HEIGHT, zombie_bitmap );
	
	// Create fuel depot
	sprite_y_pos = -1 * rand( ) % (51);
	sprite_init( &fuel_depot, 0, sprite_y_pos, FUEL_WIDTH, FUEL_HEIGHT, fuel_depot_bitmap );
}

// Function to test for bounding-box sprite collision
bool collided ( Sprite sprite_1, Sprite sprite_2 ) {
	// Sprite 1 Coordinates
	int s1_top = sprite_1.y;
	int s1_bottom = s1_top + sprite_1.height;
	int s1_left = sprite_1.x;
	int s1_right = s1_left + sprite_1.width;

	// Sprite 2 Coordinates
	int s2_top = sprite_2.y;
	int s2_bottom = s2_top + sprite_2.height;
	int s2_left = sprite_2.x;
	int s2_right = s2_left + sprite_2.width;

 // Return false is any of the conditions below are satisfied  
	if ( s1_bottom < s2_top ) return false;
	if ( s1_top > s2_bottom ) return false;
	if ( s1_right < s2_left ) return false;
	if ( s1_left > s2_right ) return false;
    return true;
}

// Function to test for pixel-level sprite collision
bool pixel_collided ( Sprite sprite_1, Sprite sprite_2 ) {
	int dx = abs(sprite_1.x + sprite_1.width - sprite_2.x);
	int dy;
	// If sprite 1 above
	if (sprite_1.y <= sprite_2.y ) {
		dy = abs(sprite_1.y + sprite_1.height) - sprite_2.y;
	}
	// If sprite 2 above
	else {
		dy = abs(sprite_2.y + sprite_2.height) - sprite_1.y;
	}
		
	uint8_t overlap;

	for(uint16_t i = 0; i < abs(dy); i++){
		// If sprite1 in range of collision box
		if( (sprite_1.y + sprite_1.height >= sprite_2.y) && (sprite_1.y <= sprite_2.y + sprite_2.height ) ) {
			// If sprite1 to the left
			if( (sprite_1.x <= sprite_2.x + sprite_2.width / 2) && (sprite_1.x + sprite_1.width >= sprite_2.x) ) {
				overlap = sprite_1.bitmap[sprite_1.height - i - 1] & sprite_2.bitmap[i] >> (sprite_2.width - dx);
			}
			// If sprite1 to the right
			else if( (sprite_1.x >= sprite_2.x) && (sprite_1.x <= sprite_2.x + sprite_2.width) ) {
				overlap = sprite_1.bitmap[sprite_1.height - i - 1] >> ( abs(sprite_2.width - dx + 1) ) & sprite_2.bitmap[i];
			}
		}
		
		// If there's a 1 in the shifted bitmaps, pixel-level collision has occurred
		if( overlap != 0 ) {
			return true;
		}
	}

	// Otherwise return false for no collision
	return false;
}

// Function to reset car to safe location on road
void reset_car ( void ) {	
	// Get a new x-position on the road
	int safe_x = road_left_x[LCD_Y - CAR_HEIGHT] + 2;
	car.x = safe_x;
	
	// While colliding with an obstacle or zombie, move to the right (safe location)
	while ( collided( car, rock ) || collided( car, zombie ) ){
		safe_x += 1;
		car.x = safe_x;
	}
	
	// Reset to speed = 0 and fuel = 100
	speed = 0;
	speed_overflow = 0;
	fuel = 100;
	fuel_overflow = 0;
}

// Initialise positions of sprites (other than car)
void initialise_location( void ) {
	generate_sprite_coords( 1 );
	house.x = sprite_x_pos;
	house.y = sprite_y_pos;

	// Ensure scenery do not spawn on top of each other
	do {
	generate_sprite_coords( 1 );
	hill.x = sprite_x_pos;
	hill.y = sprite_y_pos;
	} while ( collided( hill, house ) );
	
	do {
	generate_sprite_coords( 1 );
	cactus1.x = sprite_x_pos;
	cactus1.y = sprite_y_pos;
	} while ( collided( cactus1, house ) || collided( cactus1, hill ) );

	do {
	generate_sprite_coords( 1 );
	cactus2.x = sprite_x_pos;
	cactus2.y = sprite_y_pos;
	} while ( collided( cactus2, house ) || collided( cactus2, hill ) || collided( cactus2, cactus1 ) );
	
	do {
	generate_sprite_coords( 1 );
	oasis.x = sprite_x_pos;
	oasis.y = sprite_y_pos;
	} while ( collided( oasis, house ) || collided( oasis, hill) || collided( oasis, cactus1 ) || collided( oasis, cactus2 ) );
	
	// Ensure obstacles do not spawn on top of car or each other
	do {
		generate_sprite_coords( 2 );
		rock.x = sprite_x_pos;
		rock.y = sprite_y_pos;
	} while ( collided( rock, car ) );
	
	
	do {
		generate_sprite_coords( 2 );
		zombie.x = sprite_x_pos;
		zombie.y = sprite_y_pos;
	} while ( collided( zombie, car ) || collided( zombie, rock ) );
}

// Set up timer 0 (8-bit) - debounce timer
void setup_timer0 (void ) {
	// Overflow period of approx 0.032768s (Normal mode + Prescale of 1024)
	TCCR0A = 0;
	TCCR0B = 5; 
	
	// Enable timer overflow interrupt
	TIMSK0 = 1; 
}

// Set up timer 1 (16-bit) - game timer
void setup_timer1 (void ) {
	// Overflow period of approx 2.097152s (Normal mode + Prescale of 256)
	TCCR1A = 0;
	TCCR1B = 4;
	
	// Enable timer overflow interrupt
	TIMSK1 = 1;
}

// Set up timer 3 (16-bit) - speed and fuel
void setup_timer3 ( void ) { 
	// Overflow period of approx 0.065536s (Normal mode + Prescale of 8) 
	TCCR3A = 0;
	TCCR3B = 2;
	
	// Enable timer overflow interrupt
	TIMSK3 = 1;
}

// Set up timer 4 (fast 10-bit) - PWM
void setup_timer4 (void ) {
	// Use Timer 4 (OC4A) for PWM and enable C7
	TCCR4A = BIT(COM4A1) | BIT(PWM4A);
	SET_BIT(DDRC, 7);

	// Set to No pre-scale and fast PWM
	TCCR4B = BIT(CS42) | BIT(CS41) | BIT(CS40);
	TCCR4D = 0;
}

// Define interrupt service routine for timer 1 (game timer)
ISR(TIMER1_OVF_vect) {
	overflow_count ++;
}

// Define interrupt service routine for timer 0 (debounce timer)
ISR(TIMER0_OVF_vect) {
	debounce( );
}

// Initialise TeensyPewPew
void setup_device( void ) {
	// Set Clock speed
	set_clock_speed( CPU_8MHz );
	
	draw_string( 0, 0, "Connect USB...", FG_COLOUR );
	show_screen( );
	
	// Initialise usb connection
	usb_init();

	while ( !usb_configured() ) {
		// Block until USB is ready.
	}
	
	// Initialise adc
	adc_init( );
	
	// Initialise PWM
	// Set the TOP value for the timer overflow comparator to 1023 - cycle of 1024 ticks per overflow
	TC4H = OVERFLOW_TOP >> 8;
	OCR4C = OVERFLOW_TOP & 0xff;
	
	setup_timer0( ); /* Debounce timer */
	setup_timer1( ); /* Game timer */
	setup_timer4( ); /* PWM timer */
	sei( ); /* Turn on interrupts */
	
	// Configure the Controls for Input
	CLEAR_BIT(DDRD, 1); /* Up joystick */
	CLEAR_BIT(DDRB, 1); /* Left joystick */ 
	CLEAR_BIT(DDRD, 0); /* Right joystick */ 
	CLEAR_BIT(DDRB, 7); /* Down joystick */
	CLEAR_BIT(DDRB, 0); /* Center joystick */
	CLEAR_BIT(DDRF, 6); /* Left button */
	CLEAR_BIT(DDRF, 5); /* Right button */
	SET_BIT(DDRB, 2); /* Left LED */
	SET_BIT(DDRB, 3); /* Right LED */
}

// Determine whether the car is off-road
bool off_road ( void ) {
	if ( !( car.x > road_left_x[29] && car.x < road_left_x[29] + road_width - CAR_WIDTH ) ) {
		return true;
	}
	return false;
}

// Accelerate/decelerate car
void accelerate_car ( ) {
	// Accelerate by 1 if up is pushed on the joystick
	if ( pressed[0] ) {
		if ( !off_road( ) && speed_overflow % 8 == 0 ) { /* If on-road acceleration speed 1 to 10 in 5 secs */
			speed ++;
			speed_overflow = 0;
		}
		else if ( off_road( ) ) { /* If off-road acceleration speed 1 to 3 in 3 secs */
			if (speed_overflow % 38 == 0 ) {
				speed ++;
				speed_overflow = 0;
			}
		}
	}
	// Decelerate by 1 if down is pushed on the joystick
	else if ( pressed[1] ) {
		if ( speed_overflow % 3 == 0 ) { /* Deceleration (on-road and off-road) speed 10 to 0 in 2 secs */
			speed --;
			speed_overflow = 0;
		}
	}
	else if ( !pressed[1] && !pressed[0] ) { /* If neither up or down joystick pressed */
		if ( speed > 1 && !off_road( ) && speed_overflow % 5 == 0 ) { /* If speed > 1 and on-road, speed 10 to 1 in 3 secs */
			speed --;
			speed_overflow = 0;
		} else if ( speed > 1 && off_road( ) && speed_overflow % 23 == 0 ) { /* If speed > 1 and off-road, speed 3 to 1 in 3 secs */
			speed --;
			speed_overflow = 0;
		} else if ( speed < 1 && !off_road( ) && speed_overflow % 31 == 0 ) { /* If speed < 1 and on-road, speed 0 to 1 in 2 secs */
			speed ++;
			speed_overflow = 0;
		} else if ( speed < 1 && off_road( ) && speed_overflow % 46 == 0 ) { /* If speed < 1 and off-road, speed to 0 to 1 in 3 secs */
			speed ++;
			speed_overflow = 0;
		}
	}
	
	// If car off-road, set the maximum speed to 3
	if ( off_road( ) && speed > 3 ) { 
		speed = 3;
	}
	
	// If speed greater than max, apply limit
	if ( speed > 10 ){
		speed = 10;
	}
	
	// If speed less than 0 (reversing), apply limit
	if ( speed < 0 ){
		speed = 0;
	}
}
	
// Function to check if the car is next to the fuel depot
bool check_next_to_fuel ( void ) {
	// Get car's boundary coordinates
	int top_car = round( car.y );	
	int left_car = round( car.x );
	int right_car = left_car + CAR_WIDTH - 1;
	int bottom_car = top_car + CAR_HEIGHT - 1;	
	
	// Get fuel's boundary coordinates
	int top_fuel = round( fuel_depot.y );
	int left_fuel = round( fuel_depot.x );
	int right_fuel = left_fuel + FUEL_WIDTH - 1;
	int bottom_fuel = top_fuel + FUEL_HEIGHT - 1;

	// If the car is within 1 pixel of the fuel depot, return true
	if ( ( top_fuel == top_car || bottom_fuel == bottom_car ) && ( right_fuel + 2 == left_car || left_fuel - 2 == right_car ) ){
		return true;
	}
	return false;
}

// Function to trigger PWM SFX
void pwm_sfx ( void ) {
	// Flicker 5 times
	for (int i = 0; i < 5; i++) {
		set_duty_cycle( ADC_MAX ); /* Turn on backlight at the maximum */
		_delay_ms(5);
		set_duty_cycle( 0 ); /* Turn off backlight */
	}
}

// Function to refuel the car
void refuel_car ( void ) {
	// If car next to depot and car not already full, refuel. From 0 to 100 in 3 seconds.
	if ( check_next_to_fuel( ) && fuel < MAX_FUEL && fuel_overflow % 1 == 0 ){
		fuel += 2.2; /* This gives an exact time of '2.98s' which is the closest possible with 1 decimal place increment */
		// PWM special effect
		if ( fuel < MAX_FUEL ) {
			pwm_sfx( );
		}
		fuel_overflow = 0;
	}
	
	// Limit to maximum fuel (of 100%)
	if ( fuel > MAX_FUEL ) {
		fuel = MAX_FUEL;
	}
}

// Update the fuel remaining in the car
void update_fuel ( void ) {
	// Every update, withdraw fuel proportional to car speed and check to refuel
	fuel = fuel - FUEL_CONSUMPTION * speed;
	
	// Check if the car should be refueled
	refuel_car( );
}

// Define interrupt service routine for timer 3 (speed and fuel timer)
ISR(TIMER3_OVF_vect) {
	speed_overflow ++;
	accelerate_car( );
	fuel_overflow ++;
	update_fuel( );
}

// Function to update the current time
double current_time( void ) {
	double total_time = (overflow_count * 65536.0 + TCNT1 ) * PRESCALE / FREQ;
	return total_time;
}

// Pause game
void pause_game( void ) {
	// Store old speed
	paused_speed = speed;
	
	// While down joystick not pressed, pause game
	while ( !pressed[1] ) {
		// Stop game timer (scale to 0)
		TCCR1B = 0;

		// Erase direct screen write while paused (dashboard)
		//erase_direct( );
		
		clear_screen( );
		draw_string( 27, 8, "PAUSED", FG_COLOUR );
	
		// Set speed to 0
		speed = 0;
		speed_overflow = 0;
	
		// Display time
		char time_disp[50];
		sprintf(time_disp, "Time:%.2fs", cur_time);
		draw_string(19, 16, time_disp, FG_COLOUR);
	
		// Display distance
		char distance_disp[50];
		sprintf(distance_disp, "Dist:%.0fm / %dm", distance, finish_distance);
		draw_string(0, 24, distance_disp, FG_COLOUR);
		
		// Display save/load commands
		draw_string(4, 32, "Left Joy: SAVE", FG_COLOUR);
		draw_string(2, 40, "Right Joy: LOAD", FG_COLOUR);
		show_screen( );
		
		// Check to save - left joystick pressed
		if ( pressed[2] ){
			save_game( );
		}
		
		// Check to load - right joystick pressed
		if ( pressed[3] ){
			load_game( );
			cur_time = current_time( ); /* Update with the saved in-game time */
		}
	}
	
	// Reset to old speed
	speed = paused_speed;	
	
	counter[1] = 0; /* Reset down joystick - unpause button to not unintentionally count towards deceleration */
}

// Change difficulty using adc
void change_difficulty( void ) {
	int right_adc = adc_read(1);
	int difficulty;
	if ( right_adc <= ADC_MAX / 3 ) {
		difficulty = 0; /* 0 - Easy (200m) and wide road */
		finish_distance = 50;
		road_width = 30;
		offset = 26;
	} else if ( right_adc <= 2 * ADC_MAX / 3 ) {
		difficulty = 1; /* 1 - Medium (1000m) and medium road */
		finish_distance = 1000;
		road_width = 26;
		offset = 28;
	}
	else {
		difficulty = 2; /* 2 - Hard (1500m) and narrow road */
		finish_distance = 1500;
		road_width = 22;
		offset = 30;
	}
	draw_string( 0, 40, "DIFFICULTY:", FG_COLOUR );
	if ( difficulty == 0 ) {
		draw_string( 55, 40, "EASY", FG_COLOUR );
	} else if ( difficulty == 1 ) {
		draw_string( 55, 40, "MED ", FG_COLOUR );
	} else if ( difficulty == 2 ) {
		draw_string ( 55, 40, "HARD", FG_COLOUR );
	}
}
	
// Splash Screen
void splash( void ) {
	counter[5] = 0;
	pressed[5] = 0;
	counter[6] = 0;
	pressed[6] = 0; /* Reset left and right continue button to 0 */
	
	while( !pressed[6] && !pressed[5] ){
		draw_string(20, 0,  "Race to", FG_COLOUR);
		draw_string(3,  8,  "Zombie Mountain!", FG_COLOUR);
		draw_string(4, 16,  "Jacinta Roberts", FG_COLOUR);
		draw_string(14, 24, "(n9954619)", FG_COLOUR);
		draw_string(0, 32, "START:SW2 or SW3", FG_COLOUR);
		change_difficulty( );
		show_screen( );
	}
	if( pressed[6] || pressed[5] ){
		// Setup timer 3 for fuel and acceleration
		setup_timer3( );
		
		// Set timers back to overflow 0
		fuel_overflow = 0;
		speed = 0;
		speed_overflow = 0;
		
		counter[5] = 0;
		pressed[5] = 0;
		counter[6] = 0;
		pressed[6] = 0; /* Reset left and right continue button to 0 */
		
		// Use random - timer 1
		srand( current_time( ) );
		
		// Reset timer 1 back to 0 after randomising
		overflow_count = 0;
		
		// Initialise sprite location 
		initialise_location( );
	}
}

// Initialisation for LCD
void new_lcd_init(uint8_t contrast) {
    SET_OUTPUT(DDRD, SCEPIN); // Chip select 
    SET_OUTPUT(DDRB, RSTPIN); // Chip Reset
    SET_OUTPUT(DDRB, DCPIN);  // Data / Command selector
    SET_OUTPUT(DDRB, DINPIN); // Data input to LCD
    SET_OUTPUT(DDRF, SCKPIN); // Clock input to LCD

    CLEAR_BIT(PORTB, RSTPIN); // Reset LCD
    SET_BIT(PORTD, SCEPIN);   // Tell LCD we're not sending data.
    SET_BIT(PORTB, RSTPIN);   // Stop resetting LCD

    LCD_CMD(lcd_set_function, lcd_instr_extended);
    LCD_CMD(lcd_set_contrast, contrast);
    LCD_CMD(lcd_set_temp_coeff, 0);
    LCD_CMD(lcd_set_bias, 3);

    LCD_CMD(lcd_set_function, lcd_instr_basic);
    LCD_CMD(lcd_set_display_mode, lcd_display_normal);
    LCD_CMD(lcd_set_x_addr, 0);
    LCD_CMD(lcd_set_y_addr, 0);
}

// Update the distance travelled variable
void update_distance ( void ) {
	// Update distance proportional to car speed
	double speed_per_update = speed / 5; /* m/s to m/10ms */
	distance += speed_per_update; /* Update distance per 10ms */ 
}

// Function to draw the finish line
void draw_finish ( void ) {
	// Speed per update is 0.5 * speed
	double step_size = 0.5*speed;
	
	if ( distance >= finish_distance - 16 && finish_left == 0) {
		finish_left = road_left_x[0] - 1;
		finish_right = road_right_x[0] - 1;
	}
	
	// Step the finish line
	finish_y += step_size;
	
	// Step finish line
	for (int i = finish_left; i < finish_right; i++ ){
		draw_pixel(i, finish_y, FG_COLOUR);
	}
}

// Function to check the finish line and start drawing when necessary - set game
void check_finish ( void ) {
	 // If finished, set game over to true and display win screen with a recorded time score
	if ( distance >= finish_distance ) {
		finish_time = cur_time;
		game_over = true;
	// If almost finished, start drawing the finish line
	} else if ( distance >= finish_distance - 16 ){ 
		draw_finish( );
	}	
}

// Draw the dashboard
void draw_dashboard( void ) {
	// Draw cover-up
	draw_string( 0, 0, "                  ", FG_COLOUR );
	draw_string( 0, 1, "                  ", FG_COLOUR );
	
	// Condition
	draw_int( 10,0, condition, FG_COLOUR );
	
	// Speed
	int disp_speed = speed; /* Convert speed into an integer for display */
	draw_int( 44, 0, disp_speed, FG_COLOUR ); 
	
	// Fuel
	draw_double( 69, 0, fuel, FG_COLOUR );
}

// Draw the border
void draw_border ( void ) {
	int left = 0; /* Left X-Coordinate of border */
	int right = LCD_X - 1; /* Right X-Coordinate of border */
	int bottom = LCD_Y - 1; /* Lower Y-Coordinate of border */
	int top = TOP_BORDER; /* Top Y-Coordinate of border */
	
	// Draw the border
	draw_line(left, top, right, top, FG_COLOUR); /* Draw top side */
	draw_line(right, top, right, bottom, FG_COLOUR); /* Draw right side */
	draw_line(right, bottom, left, bottom, FG_COLOUR); /* Draw bottom side */
	draw_line(left,top,left,bottom,FG_COLOUR); /* Draw left side */
}

// Draw the curved road, i = 0 is the top
void draw_road( void ) {
	// Pixels 9 to 83 on screen are visible (0 to 9 aid in positioning fuel correctly)
	for (int i = 0; i < 84; i++ ){
		road_left_x[i] = 5*sin(0.04*(i + vary_step)) + 2*sin(0.02*(i + vary_step)) + offset;
		road_right_x[i] = road_left_x[i] + road_width;
		draw_pixel(road_left_x[i], i, FG_COLOUR);
		draw_pixel(road_right_x[i], i, FG_COLOUR);
	}
}

// Function to step fuel depot
void step_fuel ( double step_size ) {
	// If fuel below 75%, step the fuel depot and continue loop
	if ( ( fuel <= 75 ) || ( (fuel_depot.y > TOP_BORDER) && (fuel_depot.y <= LCD_Y ) ) ){
		fuel_depot.y += step_size;
		int current_y_pos = round(fuel_depot.y);
		// When bottom of fuel about to enter screen AND fuel not already on screen
		if ( (current_y_pos <= TOP_BORDER && current_y_pos >= 0) && !fuel_on_screen ){
			do {
				generate_sprite_coords( 3 );
				fuel_depot.x = sprite_x_pos;
			} while ( collided( fuel_depot, house) || collided( fuel_depot, hill ) || collided( fuel_depot, cactus1) || collided( fuel_depot, cactus2 ) || collided( fuel_depot, oasis ) );
			fuel_on_screen = true;
		}
		// Loop the fuel sprite back around if necessary
		if ( fuel_depot.y > LCD_Y ) {
			fuel_depot.y = -1 * rand() % (51); // Place fuel between 0 and -50
			fuel_on_screen = false;
		}
	}
	sprite_draw( &fuel_depot );
}
		
// Step scenery
void step_scenery( double step_size ) {
	house.y += step_size;
	hill.y += step_size;
	cactus1.y += step_size;
	cactus2.y += step_size;
	oasis.y += step_size;
	
	// Loop sprites back around and reposition if necessary
	if ( house.y > LCD_Y ) {
		do {
			generate_sprite_coords( 1 );
			house.x = sprite_x_pos;
			house.y = -1 * rand( ) % (SCENERY_HEIGHT);
		} while ( collided(house, hill) || collided(house, cactus1) || collided(house, cactus2) || collided(house, oasis) );
	}
	
	if ( hill.y > LCD_Y ) {
		do {
			generate_sprite_coords( 1 );
			hill.x = sprite_x_pos;
			hill.y = -1 * rand( ) % (SCENERY_HEIGHT);
		} while ( collided(hill, house) || collided(hill, cactus1) || collided(hill, cactus2) || collided(hill, oasis) );
	}
	
	if ( cactus1.y > LCD_Y ) {
		do {
			generate_sprite_coords( 1 );
			cactus1.x = sprite_x_pos;
			cactus1.y = -1 * rand( ) % (SCENERY_HEIGHT);
		} while ( collided(cactus1, house) || collided(cactus1, hill) || collided(cactus1, cactus2) || collided(cactus1, oasis) );
	}
	
	if ( cactus2.y > LCD_Y ) {
		do {
			generate_sprite_coords( 1 );
			cactus2.x = sprite_x_pos;
			cactus2.y = -1 * rand( ) % (SCENERY_HEIGHT);
		} while ( collided(cactus2, house) || collided(cactus2, hill) || collided(cactus2, cactus1) || collided(cactus2, oasis) );
	}
	
	
	if ( oasis.y > LCD_Y ) {
		do {
			generate_sprite_coords( 1 );
			oasis.x = sprite_x_pos;
			oasis.y = -1 * rand() % (SCENERY_HEIGHT);
		} while ( collided(oasis, house) || collided(oasis, hill) || collided(oasis, cactus1) || collided(oasis, cactus2) );
	}
}

// Step obstacles
void step_obstacles( double step_size ) {
	rock.y += step_size;
	zombie.y += step_size;
	
	// Loop sprites back around if necessary
	if ( rock.y > LCD_Y ){
		do {
			generate_sprite_coords( 2 );
			rock.x = sprite_x_pos;
			rock.y = -1 * rand( ) % (OBSTACLE_HEIGHT);
		} while ( collided( rock, zombie ) );
	}
	
	if (zombie.y > LCD_Y ){
		do { generate_sprite_coords( 2 );
		zombie.x = sprite_x_pos;
		zombie.y =  -1 * rand( ) % (OBSTACLE_HEIGHT);
		} while ( collided( zombie, rock ) );
	}
}

// Function to step sprites
void step_sprites( void ) {
	vary_step -= 0.5*speed; /* Step the road */
	double step_size = 0.5*speed;
	
	step_scenery( step_size );
	step_obstacles( step_size );
	step_fuel( step_size );
}

// Draw sprites (scenery and obstacles)
void draw_sprites( void ) {
	sprite_draw( &house );
	sprite_draw( &cactus1 );
	sprite_draw( &cactus2 );
	sprite_draw( &hill );
	sprite_draw( &oasis );
	sprite_draw( &rock );
	sprite_draw( &zombie );
}

// Move car left and right
void move_car ( void ) {
	// Move left if left is pushed on the joystick, car not stationary and not hit left border
	if ( pressed[2] && car.x >= 2 && speed >= 1 ) {
		car.x -= (int)speed; /* Move proportional to speed */
	}
	// Move right if right is pushed on the joystick, car not stationary and not hit right border
	else if ( pressed[3] && car.x <= LCD_X - CAR_WIDTH - 2 && speed >= 1 ) {
		car.x += (int)speed; /* Move proportional to speed */
	}
}

// Function to update the car's condition
void update_condition ( void ) {
	// If a minor collision occurs, damage = 25
	if ( pixel_collided( car, house ) || pixel_collided( car, hill) || pixel_collided( car, cactus1 ) ||
		pixel_collided( car, cactus2 ) || pixel_collided( car, oasis ) || pixel_collided( car, rock ) || pixel_collided( car, zombie ) ){
		condition -= 25;
		reset_car( );
	}
	
	// A fuel collision or condition reaching 0% would cause the game to be over
	if ( pixel_collided( car, fuel_depot ) || condition == 0 ) {
		game_over = true;
	}
}

//--- SETUP ---//

void setup( void ) {
	setup_device();
	// Turn everything off to start with
	PORTB = 0x00;
	PORTD = 0x00;
	new_lcd_init(LCD_DEFAULT_CONTRAST);
	
	// Create sprites
	create_sprites( );
	
	// Display splash screen
	clear_screen( );
	splash( );
	
	// Reset road position
	vary_step = 0;
	
	// Setup direct screen write
	setup_direct( );
}

//--- END GAME FUNCTIONS ---//
// Function to restart game
void restart_game ( void ) {
	// Reset global variables
	erase_direct( );
	distance = 0;
	condition = 100;
	finish_time = 0;
	finish_left = 0;
	finish_y = TOP_BORDER + 1;
	cur_time = 0;
	fuel_depot.y = 0;
	fuel_on_screen = false;
	game_over = false;
	vary_step = 0;
	draw_road( );
	reset_car( ); /* Speed and fuel also reset in this function */
	clear_screen( );
	splash( );
	// Following this, the program will continue to execute main
}
	
// Function to check if the game is over
void check_game_over ( void ) {
	// If no fuel left, set game over to true
	if ( fuel <= 0 ) {
		game_over = true;
	}
	
	int end = 0; /* Initialise end */
	
	if ( game_over ) {
		while ( end != 1 ){
			clear_screen( );
			turn_leds_on( );
			// Check if the user won (recorded a finish_time) and display score
			if (finish_time != 0) {
				draw_string( 13, 0, "CONGRATS!!", FG_COLOUR );
				draw_string( 4, 8, "Your time:", FG_COLOUR );
				draw_double_2( 54, 8, finish_time, FG_COLOUR );
				draw_string( 0, 16, "Distance:", FG_COLOUR );
				draw_double( 50, 16, distance, FG_COLOUR ); 
			// If they lost, show game over screen
			} else { 
				draw_string( 20, 5, "GAME OVER!", FG_COLOUR );
			}
			// Ask if the user would like to play again
			draw_string( 15, 25, "Play again?", FG_COLOUR );
			draw_string( 0, 34, "SW2: Yes SW3: No", FG_COLOUR );
			draw_string( 0, 41, "Centre Joy: Load", FG_COLOUR );
			show_screen( );
			
			// If centre joystick was pressed, load game
			if ( pressed[4] ){
				clear_screen( );
				turn_leds_off( );
				end = 1; /* Exit loop */
				game_over = false;
				load_game( );
				show_screen( );
			}
			
			// If right button was pressed, exit game
			if ( pressed[6] ){
				clear_screen( );
				show_screen( );
				turn_leds_off( );
				end = 1; /* Exit loop */
			}
			// If left button was pressed, restart game
			else if ( pressed[5] ){
				turn_leds_off( );
				restart_game( );
				end = 1; /* Exit loop */
			}
		}
	}
}

//--- PROCESS ---//
void process( void ) {
	// Clear existing screen
	clear_screen( );
	
	// Move and draw car
	move_car( );
	sprite_draw( &car );

	// Direct screen write
	draw_direct( );
	
	// Step and draw the sprites
	step_sprites( ); 
	draw_sprites( );
	
	// Draw road
	draw_road( );
	
	// Draw dashboard and border
	update_distance( );
	update_condition( );
	draw_dashboard( );
	draw_border( );		
		
	// Check finish
	check_finish( );	
	
	// Update the screen
	show_screen( );
	
	// Get current time and check if the user wishes to pause
	TCCR1B = 4;
	cur_time = current_time( );
	
	// Pause game if centre switch pressed
	if ( pressed[4] ) {
		pause_game( );
	}
	
	// Check if the game is over
	check_game_over( );
}

//--- MAIN ---//

int main( void ) {
	// Setup game
	setup( );

	while ( !game_over ) {
		process( );
		_delay_ms( DELAY );
	}
}

//--- ADDITIONAL MATH - ACCELERATION ---//
// Timer 3 (Normal mode with prescale of 8), overflows every 0.065536s.

// 1,  2,  3,  4,  5,  6,  7,  8,   9,  10 (speed) 
// 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4.5, 5.0 (time) (1 to 10 in 5 seconds on-road)
// This can be calculated using timestep_size = total_time / num_timesteps = 5 / 10 = 0.5s increment
// Hence 0.065536 * X = 0.5, X = 0.5 / 0.065536 = 7.629 (round to 8)
	
// 1,  2,  3
// 0, 2.5, 5 (1 to 3 in 5 seconds off-road)
// timestep_size = total_time / (num_timesteps - 1) = 5 / 2 = 2.5
// X = timestep_size / timer_overflow_period = 2.5 / 0.065536 = 38.15 (round to 38)
	
// 10, 9,   8,   6,   7,   5,   4,   3,   2,   1,   0
// 0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0 (10 to 0 in 2 seconds both)
// timestep_size = total_time / (num_timesteps - 1) = 2 / 10 = 0.2s increment 
// X = timestep_size / timer_overflow_period = 0.2 / 0.065536 = 3.05 (round to 3)
	
// 10, 9,   8,  7,   6,  5,  4,  3,   2,  1
// 0, 1/3, 2/3, 1, 4/3, 5/3, 2, 7/3, 8/3, 3 (10 to 1 in 3 seconds on-road)
// timestep_size = total_time / (num_timesteps - 1) = 3 / 9 = 0.333s increment 
// X = timestep_size / timer_overflow_period = (1/3) / 0.065536 = 5.09 (round to 5)
	
// 3,  2,  1
// 0, 1.5, 3 (3 to 1 in 3 seconds off-road)
// timestep_size = total_time / (num_timesteps - 1) = 3 / 2 = 1.5s increment 
// X = timestep_size / timer_overflow_period = 1.5 / 0.065536 = 22.89 (round to 23)
	
// 0, 1
// 0, 2 (0 to 1 in 2 seconds on-road)
// timestep_size = total_time / (num_timesteps - 1) = 2 / 1 = 2s increment 
// X = timestep_size / timer_overflow_period = 2 / 0.065536 = 30.52 (round to 31)

// 0, 1
// 0, 3 (0 to 1 in 3 seconds off-road)
// timestep_size = total_time / (num_timesteps - 1) = 3 / 1 = 3s increment 
// X = timestep_size / timer_overflow_period = 3 / 0.065536 = 45.78 (round to 46)