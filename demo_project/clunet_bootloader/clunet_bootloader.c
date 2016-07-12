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

//#define APP_END (FLASHEND - (BOOTSIZE * 2))
#define COMMAND buffer[CLUNET_OFFSET_COMMAND]
#define SUB_COMMAND buffer[CLUNET_OFFSET_DATA]

#define PAUSE(t) {CLUNET_TIMER_REG = 0; while(CLUNET_TIMER_REG < (t*CLUNET_T));}

#define WAIT_INTERFRAME(t) { CLUNET_TIMER_REG = 0; while(CLUNET_TIMER_REG < (t*CLUNET_T)) if(CLUNET_READING) CLUNET_TIMER_REG = 0; }

#if SPM_PAGESIZE > 64
#define MY_SPM_PAGESIZE 64
#else 
#define MY_SPM_PAGESIZE SPM_PAGESIZE
#endif

volatile uint8_t update = 0;

static uint8_t buffer[MY_SPM_PAGESIZE+0x10];
static void (*jump_to_app)(void) = 0x0000;

static uint8_t
check_crc(const uint8_t* data, const uint8_t size)
{
      uint8_t crc = 0;
      uint8_t i, j;
      for (i = 0; i < size; i++) 
      {
            uint8_t inbyte = data[i];
			
            for (j = 0; j < 8; j++) 
            {
                  uint8_t mix = (crc ^ inbyte) & 1;
                  crc >>= 1;
                  if (mix)
					crc ^= 0x8C;
                  inbyte >>= 1;
            }
      }
      return crc;
}

/*

	CLUNET_TIMER_REG = 0;
	while(CLUNET_TIMER_REG < (t*CLUNET_T))
		if(CLUNET_READING)
			CLUNET_TIMER_REG = 0;
*/




static void
send(const uint8_t* data, const uint8_t size)
{

	uint8_t numBits, bitIndex, byteIndex;

	uint8_t crc = check_crc(data, size);

_repeat:

	WAIT_INTERFRAME(8);			// Ждем межкадровое пространство длиной 8Т и начинаем передачу
	
	CLUNET_SEND_1;

	numBits = 4;				// Начинаем с посылки 4 бит (стартовый и 3 бита приоритета). Наивысший приоритет только у служебных приоритетных пакетов.

	byteIndex = bitIndex = 0;

	uint8_t sendingByte = data[0];
	
	uint8_t xBitMask = 0;		// Битовая маска для целей подсчета бит и получения текущего состояния линии

	do
	{

		while (((sendingByte << bitIndex) & 0x80) ^ xBitMask)
		{

			numBits++;

			if (++bitIndex & 8)
			{

				bitIndex = 0;

				if (++byteIndex < size)
					sendingByte = data[byteIndex];

				else if (byteIndex == size)
					sendingByte = crc;

				// Данные закончились
				else
					break;
			}

			if(numBits == 5)
				break;
		}
		
		// Задержка по количеству передаваемых бит
		uint8_t delay = numBits * CLUNET_T;
		CLUNET_TIMER_REG = 0;
		while(CLUNET_TIMER_REG < delay);
		
		// Конфликт на линии. Ждем и повторяем снова.
		if(xBitMask && CLUNET_READING)
			goto _repeat;
		
		CLUNET_SEND_INVERT;

		xBitMask ^= 0x80;

		numBits = (numBits == 5);

	}
	while (byteIndex <= size);

	// Если линию на финише прижали, то через 1Т отпустим ее
	if (xBitMask)
	{
		PAUSE(1);
		CLUNET_SEND_0;
	}

}


static uint8_t
wait_for_impulse()
{
	uint8_t ticks = 0;
	uint16_t time = 0;

	CLUNET_TIMER_REG = 0;
	CLUNET_TIMER_OVERFLOW_CLEAR;

	uint8_t i;

	for (i = 0; i < 2; i++)
	{

		while(CLUNET_READING != CLUNET_READING)
			if (CLUNET_TIMER_OVERFLOW)
			{
				// После 256 полных циклов таймера выйдем по таймауту
				if (!++time)
					return 0;
				CLUNET_TIMER_OVERFLOW_CLEAR;
			}

		ticks = CLUNET_TIMER_REG - ticks;
	
		time = 254;		// на 2-м проходе переполнение не должно быть более 1 раза, иначе ошибка

	}

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

	WAIT_INTERFRAME(7);	// Ждем межкадровое пространство 7Т и переходим в ожидание сигнала на линии

	uint8_t data = wait_for_impulse();
	
	// Если биты доминантные и их не менее 4 (стартовый + 3 бита приоритета), то этот пакет нам подходит, начнем прием и разбор
	if (data >= 0x84)
	{
	
		uint8_t byteIndex = 0;
		uint8_t bitIndex, bitStuff;

		buffer[0] = data;
		bitIndex = bitStuff = data & 1;

		while(1)
		{
		
			data = wait_for_impulse();
			
			if (data)
			{
				uint8_t bitNum = data & 0x7F;
				uint8_t optByte = (data & 0x80) ? 0xFF : 0x00;
				
				if (optByte)
					buffer[byteIndex] |= (255 >> bitIndex);
				else
					buffer[byteIndex] &= ~(255 >> bitIndex);

				// Обновим битовый индекс с учетом битстаффинга
				bitIndex += (bitNum - bitStuff);

				// Смотрим надо ли применять битстаффинг
				bitStuff = (bitNum == 5);

				if (bitIndex & 8)
				{
					/* Пакет прочитан полностью, выходим из цикла, проверяем и возвращаем управление */
					if ((++byteIndex > CLUNET_OFFSET_SIZE) && (byteIndex > buffer[CLUNET_OFFSET_SIZE] + CLUNET_OFFSET_DATA))
						break;

					/* Если данные прочитаны не полностью и мы не выходим за пределы буфера, то присвоим очередной байт и подготовим битовый индекс */
					else if (byteIndex < CLUNET_READ_BUFFER_SIZE)
					{
						bitIndex &= 7;
						buffer[byteIndex] = optByte;
					}

					/* Иначе ошибка: нехватка приемного буфера -> игнорируем пакет */
					else
						return 0;
				
				}
				
				
			}
			else
				return 0;
		}

		// Пакет принят
		if ((!check_crc(buffer, byteIndex)) && (buffer[CLUNET_OFFSET_DST_ADDRESS] == CLUNET_DEVICE_ID) &&
			(buffer[CLUNET_OFFSET_COMMAND] == CLUNET_COMMAND_BOOT_CONTROL))
			return buffer[CLUNET_OFFSET_SIZE];
	
	}

	return 0;
}

static void
write_flash_page(uint32_t address, uint8_t* pagebuffer)
{
	eeprom_busy_wait();

#if MY_SPM_PAGESIZE != SPM_PAGESIZE
	if (address % SPM_PAGESIZE == 0)
#endif
	{
		boot_page_erase (address);
		boot_spm_busy_wait();		// Wait until the memory is erased.
	}

	int i;
	for (i = 0; i < MY_SPM_PAGESIZE; i += 2)
	{
		// Set up little-endian word.
		uint16_t w = *((uint16_t*)(pagebuffer + i));
		boot_page_fill (address + i, w);
	}

	boot_page_write(address);	// Store buffer in flash page.
	boot_spm_busy_wait();		// Wait until the memory is written.

	boot_rww_enable();
}

static void
send_firmware_command(char b)
{
	uint8_t update_start_command[5] = { CLUNET_DEVICE_ID, CLUNET_BROADCAST_ADDRESS, CLUNET_COMMAND_BOOT_CONTROL, 1, b };
	send(update_start_command, sizeof(update_start_command));
}
/*
static void
firmware_update()
{

	uint8_t update_start_command[7] = {CLUNET_DEVICE_ID,CLUNET_BROADCAST_ADDRESS,CLUNET_COMMAND_BOOT_CONTROL,3,COMMAND_FIRMWARE_UPDATE_READY,(MY_SPM_PAGESIZE & 0xFF),(MY_SPM_PAGESIZE >> 8)};

	send(update_start_command, sizeof(update_start_command));

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
				uint8_t* pagebuffer = buffer + (CLUNET_OFFSET_DATA + 5);
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
*/
int main (void)
{
	cli();
 	CLUNET_TIMER_INIT;
	CLUNET_READ_INIT;
	CLUNET_SEND_INIT;
	
	send_firmware_command(COMMAND_FIRMWARE_UPDATE_START);
	
//	if ((read()) && (SUB_COMMAND == COMMAND_FIRMWARE_UPDATE_INIT))
//		firmware_update();

	jump_to_app();



	uint8_t update_start_command[7] =	{	CLUNET_DEVICE_ID,
											CLUNET_BROADCAST_ADDRESS,
											CLUNET_COMMAND_BOOT_CONTROL,
											3,
											COMMAND_FIRMWARE_UPDATE_READY,
											MY_SPM_PAGESIZE >> 8,
											MY_SPM_PAGESIZE
										};

	send(update_start_command, sizeof(update_start_command));

	while(read())
	{
		switch(SUB_COMMAND)
		{

//		case COMMAND_FIRMWARE_UPDATE_INIT:

//			firmware_update();

//		break;

		case COMMAND_FIRMWARE_UPDATE_WRITE:
		{

			uint16_t address = *((uint32_t*)(buffer + (CLUNET_OFFSET_DATA + 1)));
			uint8_t* pagebuffer = buffer + (CLUNET_OFFSET_DATA + 5);
				
			write_flash_page(address, pagebuffer);
				
			send_firmware_command(COMMAND_FIRMWARE_UPDATE_WRITTEN);

		}
			
		break;

		case COMMAND_FIRMWARE_UPDATE_DONE:

			jump_to_app();

		}
	}
}	
	
	
	
	
	
	
	


	return 0;

}
