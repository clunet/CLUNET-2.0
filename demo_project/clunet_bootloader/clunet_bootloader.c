#include "defines.h"
#include "../defines.h"
#include "../clunet_config.h"
#include "clunet.h"
#include "bits.h"
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/interrupt.h>

#define COMMAND_FIRMWARE_UPDATE_START 0
#define COMMAND_FIRMWARE_UPDATE_INIT 1
#define COMMAND_FIRMWARE_UPDATE_READY 2
#define COMMAND_FIRMWARE_UPDATE_WRITE 3
#define COMMAND_FIRMWARE_UPDATE_WRITTEN 4
#define COMMAND_FIRMWARE_UPDATE_DONE 5

#define APP_END (FLASHEND - (BOOTSIZE * 2))
#define COMMAND buffer[CLUNET_OFFSET_COMMAND]
#define SUB_COMMAND buffer[CLUNET_OFFSET_DATA]

#define PAUSE(t) {CLUNET_TIMER_REG = 0; while(CLUNET_TIMER_REG < (t*CLUNET_T));}
#define SEND_BIT(t) {CLUNET_SEND_1; PAUSE(t); CLUNET_SEND_0; PAUSE(1);}

#define FREE_LINE(t) { CLUNET_TIMER_REG = 0; while(CLUNET_TIMER_REG < (t*CLUNET_T)) if(CLUNET_READING) CLUNET_TIMER_REG = 0; }

#if SPM_PAGESIZE > 64
#define MY_SPM_PAGESIZE 64
#else 
#define MY_SPM_PAGESIZE SPM_PAGESIZE
#endif

volatile char update = 0;

char buffer[MY_SPM_PAGESIZE+0x10];
static void (*jump_to_app)(void) = 0x0000;

static char
check_crc(char* data, unsigned char size)
{
      uint8_t crc=0;
      uint8_t i,j;
      for (i=0; i<size;i++) 
      {
            uint8_t inbyte = data[i];
            for (j=0;j<8;j++) 
            {
                  uint8_t mix = (crc ^ inbyte) & 0x01;
                  crc >>= 1;
                  if (mix) 
                        crc ^= 0x8C;
                  
                  inbyte >>= 1;
            }
      }
      return crc;
}

static void
send(char* data, uint8_t size)
{

	uint8_t numBits = 4;
	uint8_t bitIndex = 0;
	uint8_t byteIndex = 0;

	char crc = check_crc(data, size);

	FREE_LINE(7);
	
	CLUNET_SEND_1;

	do
	{
		uint8_t b = (byteIndex < size) ? data[byteIndex] : crc;

		uint8_t xMask = (CLUNET_SENDING) ? 0 : 0x80;
		
		while (((b << bitIndex) & 0x80) ^ xMask)
		{

			if (++bitIndex & 8)
			{
				bitIndex = 0;
				byteIndex++;
			}

			if((++numBits == 5) || (byteIndex > size))
				break;
		}
		
		// Задержка, соответствующая количеству бит
		uint8_t delay = numBits * CLUNET_T;
		CLUNET_TIMER_REG = 0;
		while(CLUNET_TIMER_REG < delay);

		CLUNET_SEND_INVERT;

		numBits = (numBits == 5);

	}
	while (byteIndex <= size);
	
	if (CLUNET_SENDING)
	{
		PAUSE(1);
		CLUNET_SEND_INVERT;
	}
}

static uint8_t
wait_for_impulse()
{
	uint8_t time = 0;

	CLUNET_TIMER_REG = 0;
	CLUNET_TIMER_OVERFLOW_CLEAR;

	while(CLUNET_READING != CLUNET_READING)
		if (CLUNET_TIMER_OVERFLOW)
		{
			// После 256 полных циклов таймера выйдем по таймауту
			if (!++time)
				return 0;
			CLUNET_TIMER_OVERFLOW_CLEAR;
		}

	uint8_t ticks = CLUNET_TIMER_REG;
	
	time = 0;

	while(CLUNET_READING != CLUNET_READING)
		if (CLUNET_TIMER_OVERFLOW)
		{
			// Ошибка: слишком большой период
			if (++time > 1)
				return 0;
			CLUNET_TIMER_OVERFLOW_CLEAR;
		}

	ticks = CLUNET_TIMER_REG - ticks;

	if (ticks < CLUNET_READ1)
		ticks = 1;
	else if (ticks < CLUNET_READ2)
		ticks = 2;
	else if (ticks < CLUNET_READ3)
		ticks = 3;
	else if (ticks < CLUNET_READ4)
		ticks = 4;
	else if (ticks < CLUNET_READ5)
		ticks = 5;
	/* Ошибка: длина импульса должна быть не более 5 */
	else
		return 0;

	// Возвращаем длину импульса в периодах CLUNET_T, если пришли единички, то старший бит равен 1
	return (CLUNET_READING) ? ticks : (ticks | 0x80);
}

static uint8_t
read()
{

	uint8_t bitIndex = 0;
	uint8_t byteIndex = 0;
	uint8_t bitStuff = 0;
	
	FREE_LINE(7);

	uint8_t data = wait_for_impulse();
	
	// Если биты доминантные и их не менее 4 (стартовый + 3 бита приоритета), то этот пакет нам подходит, начнем прием и разбор
	if (data >= 0x84)
	{

		// Если пришло 5 бит, то последний относится к данным, установим его
		if (data & 1)
		{
			buffer[0] = 0x80;
			bitIndex = bitStuff = 1;
		}
		
		do
		{
		
			data = wait_for_impulse();
			
			if (data)
			{
				uint8_t bitNum = data & 7;
				
				if (data & 0x80)
					buffer[byteIndex] |= (255 >> bitIndex);
				else
					buffer[byteIndex] &= ~(255 >> bitIndex);

				// Обновим битовый индекс с учетом битстаффинга
				bitIndex += (bitNum - bitStuff);

				if (bitIndex & 8)
				{
					/* Если данные прочитаны не полностью и мы не выходим за пределы буфера, то присвоим очередной байт и подготовим битовый индекс */
					if (++byteIndex < CLUNET_READ_BUFFER_SIZE)
					{
						bitIndex &= 7;
						buffer[byteIndex] = (data & 0x80) ? 0xFF : 0x00;
					}

					/* Иначе ошибка: нехватка приемного буфера -> игнорируем пакет */
					else return 0;
				
				}
				
				bitStuff = (bitNum == 5);
				
			}
			else return 0;
		}
		while((byteIndex < CLUNET_OFFSET_DATA) && (byteIndex <= buffer[CLUNET_OFFSET_SIZE] + CLUNET_OFFSET_DATA));

		if ((buffer[CLUNET_OFFSET_DST_ADDRESS] == CLUNET_DEVICE_ID) && (buffer[CLUNET_OFFSET_COMMAND] == CLUNET_COMMAND_BOOT_CONTROL) &&
			!check_crc((char*)buffer, byteIndex))

			return buffer[CLUNET_OFFSET_SIZE];
	}

	return 0;
}

static void
write_flash_page(uint32_t address, char* pagebuffer)
{
	eeprom_busy_wait ();

#if MY_SPM_PAGESIZE != SPM_PAGESIZE
	if (address % SPM_PAGESIZE == 0)
#endif
	{
		boot_page_erase (address);
		boot_spm_busy_wait ();      // Wait until the memory is erased.
	}

	int i;
	for (i=0; i<MY_SPM_PAGESIZE; i+=2)
	{
		// Set up little-endian word.
		uint16_t w = *((uint16_t*)(pagebuffer + i));
		boot_page_fill (address + i, w);
	}

	boot_page_write(address);     // Store buffer in flash page.
	boot_spm_busy_wait();            // Wait until the memory is written.

	boot_rww_enable ();
}

static void
send_firmware_command(char b)
{
	char update_start_command[5] = { CLUNET_DEVICE_ID, CLUNET_BROADCAST_ADDRESS, CLUNET_COMMAND_BOOT_CONTROL, 1, b };
	send(update_start_command, 5);
}

static void
firmware_update()
{
	char update_start_command[7] = {CLUNET_DEVICE_ID,CLUNET_BROADCAST_ADDRESS,CLUNET_COMMAND_BOOT_CONTROL,3,COMMAND_FIRMWARE_UPDATE_READY,(MY_SPM_PAGESIZE & 0xFF),(MY_SPM_PAGESIZE>>8)};
	send(update_start_command, 7);
	while(1)
	{
		if (read())
		{
			switch(SUB_COMMAND)
			{
			case COMMAND_FIRMWARE_UPDATE_INIT:
				firmware_update();
				break;
			case COMMAND_FIRMWARE_UPDATE_WRITE:
			{
				uint16_t address = *((uint32_t*)(buffer + (CLUNET_OFFSET_DATA + 1)));
				char* pagebuffer = buffer + (CLUNET_OFFSET_DATA + 5);
				write_flash_page(address, pagebuffer);
				send_firmware_command(COMMAND_FIRMWARE_UPDATE_WRITTEN);
			}
			break;
			case COMMAND_FIRMWARE_UPDATE_DONE:
				jump_to_app();
			}
		}			
	}
}

int main (void)
{
	cli();
 	CLUNET_TIMER_INIT;
	CLUNET_READ_INIT;
	CLUNET_SEND_INIT;
	
	send_firmware_command(COMMAND_FIRMWARE_UPDATE_START);

	if ((read()) && (SUB_COMMAND == COMMAND_FIRMWARE_UPDATE_INIT))
		firmware_update();
	jump_to_app();
	//asm("rjmp 0000");
	return 0;
}
