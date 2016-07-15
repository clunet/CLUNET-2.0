#include "defines.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include "clunet.h"

void data_received(uint8_t src_address, uint8_t command, char* data, uint8_t size)
{
	;
}

int main (void)
{
	clunet_init();
	clunet_set_on_data_received(data_received);
	sei();

	char buffer[1];
	buffer[0] = 1;
	clunet_send(CLUNET_BROADCAST_ADDRESS, CLUNET_PRIORITY_MESSAGE, 50, buffer, sizeof(buffer));
	
	while (1)
	{
	}
	return 0;
}

