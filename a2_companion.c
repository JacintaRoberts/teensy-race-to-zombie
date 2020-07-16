// Jacinta Roberts (n9954619)
// Assignment 2: Desktop Companion Program (Teensy Server File)
//--- INCLUDE FILES ---//
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <cab202_graphics.h>

//--- GLOBAL VARIABLES ---//
FILE *usb_serial, *storage_file;

//--- SET UP FUNCTIONS ---//

void setup_usb_serial( const char * serial_device ) {
	usb_serial = fopen( serial_device, "r+" ); /* Open TEENSY in read and write mode */
	
	// Display error to console if unsuccessful
	if ( usb_serial == NULL ) {
		fprintf( stderr, "Unable to open the device \"%s\"\n", serial_device );
		exit( 1 );
	}
}

void setup( const char * serial_device ) {
	setup_screen( );
	setup_usb_serial( serial_device );
}

//--- PROCESS ---//
void process( void ) {
	clear_screen( );
	unsigned char info_c = fgetc( usb_serial );

	// SAVE - If received string starts with 'S' (begin save flag)
	if( info_c == 'S' ){ 
		storage_file = fopen( "storage_file.txt", "w+" ); /* Create storage_file.txt for both reading and writing */
		while ( info_c != 'T' ){ /* While not end of save ('T' = end save flag), continue to receive chars */ 
			info_c = fgetc( usb_serial );
			fputc( info_c, storage_file );
			fflush( storage_file );
		}
		draw_string( 0, 0, "Saving to file..." ); /* Display console message */
		draw_string( 0, 1,"Done!" );
		show_screen( );
		fclose( storage_file ); /* Close storage_file.txt when complete */
		
	// LOAD - Else if the received string starts with 'L' (begin load flag)
	} else if ( info_c == 'L' ){
		char loadString[1000]; /* Initialise the string to be loaded from storage_file.txt */
		storage_file = fopen("storage_file.txt", "r+"); /* Opens storage_file.txt file for reading and writing - must exist */
		draw_string( 0, 0, "Loading from file..." ); /* Display console message */
		show_screen( );
		fgets( loadString, 1000, storage_file ); /* Get the string from the storage_file.txt */
		const char comma[2] = ",";
		char *token = strtok( loadString, comma ); /* Tokenise the string (separate on commas) */

		// While the token is not empty, send it to the TEENSY
		while ( token != NULL ){
			fputs(token, usb_serial);
			while (fgetc(usb_serial) != 'M'){ /* While the end of Load is not received */
			}
			token = strtok(NULL, comma);
		}
		draw_string( 0, 2, "File contents were sent to Teensy!" ); /* Display console message */
		draw_string( 0, 1,"Done!" );
		show_screen( ); 
		fclose( storage_file ); /* Close storage_file.txt when complete */
	
	// ELSE - No Save or Load flag identified, display appropriate error message to console
	} else {
		draw_string( 0, 0, "No operation to be performed." );
		show_screen( );
	}
}

//--- MAIN ---//
int main(int argc, char *argv[]) {
	if ( argc != 2 ) {
		fprintf(stderr, "Expected 1 command line argument containing serial device name.\n");
		fprintf(stderr, "Example: usb_zdk /dev/ttyS3\n");
		return 1;
	}

	setup(argv[1]);
	for ( ;; ) {
		process();

	}
}