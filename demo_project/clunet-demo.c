#include "clunet.h"

void data_received(uint8_t src_address, uint8_t command, char* data, uint8_t size)
{

}

int main (void)
{
	clunet_init();
	clunet_set_on_data_received(data_received);

	char buffer[1];
	buffer[0] = 1;
	clunet_send(CLUNET_BROADCAST_ADDRESS, CLUNET_PRIORITY_MESSAGE, 50, buffer, sizeof(buffer));
	
	while (1)
	{
	}
	return 0;
}
