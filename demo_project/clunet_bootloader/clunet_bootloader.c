/**************************************************************************************
The MIT License (MIT)
Copyright (c) 2016 Sergey V. DUDANOV
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************************/

#include "defines.h"
#include "../defines.h"
#include "../clunet_config.h"
#include "clunet.h"
#include "bits.h"
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/interrupt.h>

#define COMMAND_FIRMWARE_UPDATE_START	0	// Информируем сеть, что мы в загрузчике
#define COMMAND_FIRMWARE_UPDATE_INIT	1	// Субкоманда инициализации процедуры загрузки прошивки
#define COMMAND_FIRMWARE_UPDATE_READY	2	// Информируем, что мы в режиме прошивки
#define COMMAND_FIRMWARE_UPDATE_WRITE	3	// Субкоманда записи данных во флеш-память (отправитель внешнее устройство)
#define COMMAND_FIRMWARE_UPDATE_WRITTEN	4	// Подтверждение выполнения команды записи (отправитель мы)
#define COMMAND_FIRMWARE_UPDATE_DONE	5	// Субкоманда окончания записи и выполнения записанной программы (отправитель внешнее устройство)

#define APP_END (FLASHEND - (BOOTSIZE * 2))

#define RECEIVED_COMMAND buffer[CLUNET_OFFSET_COMMAND]
#define RECEIVED_SUB_COMMAND buffer[CLUNET_OFFSET_DATA]

/* Вспомогательные макросы для передачи пакетов */
#define PAUSE(t) { uint8_t _top = CLUNET_TIMER_REG + (t * CLUNET_T); while (CLUNET_TIMER_REG != _top); }

// Максимальный размер страницы
#if SPM_PAGESIZE > 128
	#define MY_SPM_PAGESIZE 128
#else
	#define MY_SPM_PAGESIZE SPM_PAGESIZE
#endif

// Таймаут ожидания приемки пакета в циклах переполнения таймера
#define BOOTLOADER_TIMEOUT_OVERFLOWS ((uint16_t)(((float)BOOTLOADER_TIMEOUT / 1000.0f) * ((float)F_CPU / (float)CLUNET_TIMER_PRESCALER / 256.0f)))


static char buffer[MY_SPM_PAGESIZE + 11];

static void (*jump_to_app)(void) = 0x0000;


/*
	Функция ожидания межкадрового интервала заданной длины.
	Блокирует управление до обнаружения интервала.
*/
static void
wait_interframe(const uint8_t periods)
{
	uint8_t stop;
_loop:
	stop = CLUNET_TIMER_REG + periods * CLUNET_T;
	do
		if (CLUNET_READING)
			goto _loop;
	while (CLUNET_TIMER_REG != stop);
}

/* Функция нахождения контрольной суммы Maxim iButton 8-bit */
static char
check_crc(const char* data, const uint8_t size)
{
      char crc = 0;
      uint8_t a = 0;
      do
      {
            uint8_t b = 8;
            char inbyte = data[a];
            do
            {
                  uint8_t mix = (crc ^ inbyte) & 1;
                  crc >>= 1;
                  if (mix) crc ^= 0x8C;
                  inbyte >>= 1;
            }
            while (--b);
      }
      while (++a < size);
      return crc;
}

static void
send(const char* data, const uint8_t size)
{

	uint8_t numBits, bitIndex, byteIndex;

	char crc = check_crc(data, size);

_repeat:

	wait_interframe(8);			// Ждем межкадровое пространство длиной 8Т и начинаем передачу
	
	CLUNET_SEND_1;

	numBits = 4;				// Начинаем с посылки 4 бит (стартовый и 3 бита приоритета). Наивысший приоритет только у служебных приоритетных пакетов.

	byteIndex = bitIndex = 0;

	char sendingByte = data[0];
	
	char xBitMask = 0;		// Битовая маска для целей подсчета бит и получения текущего состояния линии

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

			if (numBits == 5)
				break;
		}
		
		// Задержка по количеству передаваемых бит
		uint8_t stop = CLUNET_TIMER_REG + numBits * CLUNET_T;
		while (CLUNET_TIMER_REG != stop);
		
		// Конфликт на линии. Ждем и повторяем снова.
		if (xBitMask && CLUNET_READING)
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
	uint16_t overflows = 0;

	CLUNET_TIMER_REG = 0;
	CLUNET_TIMER_OVERFLOW_CLEAR;

	uint8_t i = 2;

	do
	{

		while(CLUNET_READING != CLUNET_READING)
			if (CLUNET_TIMER_OVERFLOW)
			{
				// Ожидаем пакет в течение таймаута, заданном в defines.h в параметре BOOTLOADER_TIMEOUT (в милисекундах)
				if (++overflows == BOOTLOADER_TIMEOUT_OVERFLOWS)
					return 0;

				CLUNET_TIMER_OVERFLOW_CLEAR;

			}

		ticks = CLUNET_TIMER_REG - ticks;
	
		// на 2-м проходе переполнение не должно быть более 1 раза, иначе ошибка
		overflows = (BOOTLOADER_TIMEOUT_OVERFLOWS - 2);

	}
	while(--i);


	uint8_t bitNum, period;

	// Цикл подсчета количества бит
	for (bitNum = 0, period = (CLUNET_T / 2); ticks >= period; period += CLUNET_T)
	{
		/* Ошибка: длина импульса должна быть не более 5 */
		if(++bitNum > 5)
			return 0;
	}

	// Возвращаем длину импульса в периодах CLUNET_T, если пришли единички, то старший бит равен 1
	return CLUNET_READING ? bitNum : (bitNum | 0x80);

}

/*
	ЧТЕНИЕ СИСТЕМНЫХ ПАКЕТОВ ОБНОВЛЕНИЯ
	static uint8_t read(void)
	
	Блокирует управление пока линия прижата, при освобождении ожидает межкадровый интервал длительностью 7Т,
	переходит в состояние чтения пакета, читает, проверяет контрольную сумму, удостоверяется что этот пакет системный и предназначен для нас.
	Возвращает длину полученных данных в пакете, в случае ошибки - 0.
*/

static uint8_t
read()
{

	wait_interframe(7);	// Ждем межкадровое пространство 7Т и переходим в ожидание сигнала на линии

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

				// Смотрим надо ли применять битстаффинг
				bitStuff = (bitNum == 5);

			}
			else
				return 0;
		}

		// Пакет принят
		if ((!check_crc(buffer, byteIndex)) && (buffer[CLUNET_OFFSET_DST_ADDRESS] == CLUNET_DEVICE_ID) &&
			(RECEIVED_COMMAND == CLUNET_COMMAND_BOOT_CONTROL))
			return buffer[CLUNET_OFFSET_SIZE];
	
	}

	return 0;
}


static void
#if (FLASHEND > USHRT_MAX)
write_flash_page(uint32_t address, char* pagebuffer)
#else
write_flash_page(uint16_t address, char* pagebuffer)
#endif
{

	eeprom_busy_wait();

#if MY_SPM_PAGESIZE != SPM_PAGESIZE
	if (!(address % SPM_PAGESIZE))
#endif
	{
		boot_page_erase(address);
		boot_spm_busy_wait();		// Wait until the memory is erased.
	}

	uint8_t i;
	for (i = 0; i < MY_SPM_PAGESIZE; i += 2)
		boot_page_fill(address + i, *((uint16_t*)(pagebuffer + i)));

		boot_page_write(address);	// Store buffer in flash page.
		boot_spm_busy_wait();		// Wait until the memory is written.

		boot_rww_enable();
}

static void
send_firmware_command(const uint8_t command)
{
	static char
	update_start_command[5] =	{
									CLUNET_DEVICE_ID,
									CLUNET_BROADCAST_ADDRESS,
									CLUNET_COMMAND_BOOT_CONTROL,
									1,	// размер данных
									0
								};

	update_start_command[4] = command;

	send(update_start_command, sizeof(update_start_command));

}


static void
firmware_update()
{
	static char
	update_start_command[7] =	{
									CLUNET_DEVICE_ID,
									CLUNET_BROADCAST_ADDRESS,
									CLUNET_COMMAND_BOOT_CONTROL,
									3, // размер данных
									COMMAND_FIRMWARE_UPDATE_READY,
									MY_SPM_PAGESIZE,
									MY_SPM_PAGESIZE >> 8
								};

	send(update_start_command, sizeof(update_start_command));
	
	while(1)
	{

		if (read())
		{
			uint8_t subCmd = RECEIVED_SUB_COMMAND;

			switch (subCmd)
			{

			case COMMAND_FIRMWARE_UPDATE_INIT:

				send(update_start_command, sizeof(update_start_command));

				break;

			case COMMAND_FIRMWARE_UPDATE_WRITE:
			{

			#if (FLASHEND > USHRT_MAX)
				uint32_t address = *((uint32_t*)(buffer + (CLUNET_OFFSET_DATA + 1)));
			#else
				uint16_t address = *((uint16_t*)(buffer + (CLUNET_OFFSET_DATA + 1)));	// Адрес страницы памяти берем начиная с 6-го байта (смещение +5). Размер фиксирован - 32 бит.
			#endif

				char* pagebuffer = buffer + (CLUNET_OFFSET_DATA + 5); // с 10-го байта в пакете (смещение +9) начинаются данные. Размер - MY_SPM_PAGESIZE байт.

				write_flash_page(address, pagebuffer); // Пишем во флеш-память
						
				send_firmware_command(COMMAND_FIRMWARE_UPDATE_WRITTEN);	// Отправляем подтверждение записи

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
	CLUNET_PIN_INIT;
	
	send_firmware_command(COMMAND_FIRMWARE_UPDATE_START);

	if (read() && (RECEIVED_SUB_COMMAND == COMMAND_FIRMWARE_UPDATE_INIT))
		firmware_update();

	jump_to_app();

	return 0;
}
