#include <stdlib.h>
#include <string.h>

char string [ 30 ];
char ch1 , coillower, coilupper;
char ch2;

void main ( void ) {
			coillower = 0x01;
			coilupper = 0x20;

			strcat (string, "900901" );
			ch1 = coillower + 0x30;	// convert to ascii
			ch2 = coilupper + 0x30;
			// Now add ch1 and ch2 to string
			strncat ( string, &ch1, 1 );
			strncat ( string, &ch2, 1 );
}
