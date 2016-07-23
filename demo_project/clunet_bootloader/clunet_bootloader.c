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
#define FLASHER_ADDRESS buffer[CLUNET_OFFSET_SRC_ADDRESS]

/* Вспомогательные макросы для передачи пакетов */
#define PAUSE(t) { CLUNET_TIMER_REG = 0; while (CLUNET_TIMER_REG < (t * CLUNET_T)); }

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

static char
control_command[5] =		{
								CLUNET_DEVICE_ID,
								CLUNET_BROADCAST_ADDRESS,
								CLUNET_COMMAND_BOOT_CONTROL,
								1,	// размер данных
								0
							};

static char
update_init_command[7] =	{
								CLUNET_DEVICE_ID,
								CLUNET_BROADCAST_ADDRESS,
								CLUNET_COMMAND_BOOT_CONTROL,
								3, // размер данных
								COMMAND_FIRMWARE_UPDATE_READY,
								MY_SPM_PAGESIZE,
								MY_SPM_PAGESIZE >> 8
							};

// Максимально допустимая рассинхронизация между устройствами сети
const uint8_t max_delta = (uint8_t)((float)CLUNET_T * 0.3f);


/*	Функция ожидания межкадрового интервала длительностью 8Т.
	Блокирует управление до обнаружения интервала или начала передачи от другого устройства в пределах допустимой рассинхронизации.
*/
static void
wait_interframe()
{
	uint8_t delta;
_clear:
	CLUNET_TIMER_REG = 0;
	do
	{
		delta = (8 * CLUNET_T) - CLUNET_TIMER_REG;
		if (CLUNET_READING)
		{
			if (delta > max_delta)
				goto _clear;
			break;
		}
	}
	while (delta);
}

/*	Функция ожидания начала кадра (доминантного бита) в течении таймаута,
	заданном в defines.h в параметре BOOTLOADER_TIMEOUT (в милисекундах).
	Возвращает: 0 при успехе, 1 при прошедшем таймауте ожидания (так никто и не начал передачу)
*/
static uint8_t
wait_for_start()
{
	CLUNET_TIMER_REG = 0;
	uint16_t overflows = BOOTLOADER_TIMEOUT_OVERFLOWS;
_clear:
	CLUNET_TIMER_OVERFLOW_CLEAR;
	while (!CLUNET_READING)
		if (CLUNET_TIMER_OVERFLOW)
		{
			if (--overflows)
				goto _clear;
			return 1;
		}
	return 0;
}

/*	Функция чтения битовой задержки до заданного типа сигнала.
	Если signal == 0, то ждем рецессивный (измеряем доминантные биты), иначе наоборот).
	Возвращает количество прочитанных бит, 0 - при ошибке.
*/
static uint8_t
read_signal(const uint8_t signal)
{
	CLUNET_TIMER_REG = 0;
	uint8_t bitNum = 0;
	uint8_t period = CLUNET_T / 2;
	while (!(CLUNET_READING) != !(signal))
		if (CLUNET_TIMER_REG >= period)
		{
			// Ошибка: не может быть больше 5 бит
			if (++bitNum > 5)
				return 0;
			period += CLUNET_T;
		}
	return bitNum;
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

	// Ждем освобождения линии и межкадровое пространство 8Т в блокирующем режиме (в конце концов замкнутая накоротко линия это ненормально)
	wait_interframe();
	CLUNET_SEND_1;
	numBits = 4; // Начинаем с посылки 4 бит (стартовый и 3 бита приоритета)
	byteIndex = bitIndex = 0;
	char sendingByte = data[0];
	char xBitMask = 0; // Битовая маска для целей подсчета бит и получения текущего состояния линии
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
				// Данные закончились. Выходим.
				else
					break;
			}
			if (numBits == 5)
				break;
		}
		// Задержка по количеству передаваемых бит и проверка на конфликт с синхронизацией при передаче
		CLUNET_TIMER_REG = 0;
		uint8_t delta;
		const uint8_t stop = numBits * CLUNET_T;
		do
		{
			const uint8_t now = CLUNET_TIMER_REG;
			delta = stop - now;
			if (xBitMask && CLUNET_READING)
			{
				if (now <= max_delta)
					continue;
				else if (delta > max_delta)
					goto _repeat;
				break;
			}
		}
		while (delta);

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
	// Ждем освобождения линии и межкадровое пространство в блокирующем режиме (в конце концов замкнутая накоротко линия это ненормально)
	wait_interframe();
	// Ожидаем начала передачи в течении таймаута, заданном в defines.h в параметре BOOTLOADER_TIMEOUT (в милисекундах)
	// Если не дожидаемся - выходим
	if (wait_for_start())
		return 0;
	// Читаем доминантные биты
	uint8_t bitNum = read_signal(0);
	
	// Если биты доминантные и их не менее 4 (стартовый + 3 бита приоритета), то этот пакет нам подходит, начнем прием и разбор
	if (bitNum >= 4)
	{
	
		uint8_t bitIndex, bitStuff;
		uint8_t byteIndex = 0;
		uint8_t lineFree = buffer[0] = 0xFF;

		bitIndex = bitStuff = bitNum & 1; // Если приняли 5 бит, то битовый индекс и битстаффинг = 1, иначе 0;

		while(1)
		{
		
			bitNum = read_signal(lineFree);
			
			if (bitNum)
			{
				lineFree = ~lineFree;
				
				if (lineFree)
					buffer[byteIndex] |= (255 >> bitIndex);
				else
					buffer[byteIndex] &= ~(255 >> bitIndex);

				// Обновим битовый индекс с учетом битстаффинга
				bitIndex += bitNum - bitStuff;

				if (bitIndex & 8)
				{
					/* Пакет прочитан полностью, выходим из цикла, проверяем и возвращаем управление */
					if ((++byteIndex > CLUNET_OFFSET_SIZE) && (byteIndex > buffer[CLUNET_OFFSET_SIZE] + CLUNET_OFFSET_DATA))
						break;

					/* Если данные прочитаны не полностью и мы не выходим за пределы буфера, то присвоим очередной байт и подготовим битовый индекс */
					else if (byteIndex < CLUNET_READ_BUFFER_SIZE)
					{
						bitIndex &= 7;
						buffer[byteIndex] = lineFree;
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

static inline void
send_firmware_command(const uint8_t sub_command)
{
	control_command[CLUNET_OFFSET_DATA] = sub_command;
	send(control_command, sizeof(control_command));
}


int main (void)
{

	cli();
 	CLUNET_TIMER_INIT;
	CLUNET_PIN_INIT;
	
	// Посылаем широковещательный пакет, что мы в загрузчике
	send_firmware_command(COMMAND_FIRMWARE_UPDATE_START);

	// Делаем 5 попыток получить в ответ служебный пакет, при успехе переходим в режим прошивки, иначе загружаем основную программу
	uint8_t packets = 5;
	do
	{
		if (read() && (RECEIVED_SUB_COMMAND == COMMAND_FIRMWARE_UPDATE_INIT))
		{

			char flasher_address = FLASHER_ADDRESS; // Запомним, кто инициировал обновление, с тем и будем дальше работать

			// Теперь работаем только с конкретным устройством
			update_init_command[CLUNET_OFFSET_DST_ADDRESS] = flasher_address;
			control_command[CLUNET_OFFSET_DST_ADDRESS] = flasher_address;

			// Говорим устройству, что мы в режиме прошивки и сообщаем наш размер страницы памяти
			send(update_init_command, sizeof(update_init_command));
			
			while(1)
			{

				// Если системный пакет получен и он от нужного устройства, то обработаем его
				if (read() && FLASHER_ADDRESS == flasher_address)
				{
					uint8_t subCmd = RECEIVED_SUB_COMMAND;

					switch (subCmd)
					{

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

					case COMMAND_FIRMWARE_UPDATE_INIT:

						send(update_init_command, sizeof(update_init_command));
						break;


					case COMMAND_FIRMWARE_UPDATE_DONE:

						goto _done;
						
					}
				}
			}			
		}
	}
	while (--packets);

_done:

	jump_to_app();

	return 0;
}
