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

#include "clunet.h"

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

/* Указатели на функции обратного вызова при получении пакетов (должны быть как можно короче) (ОЗУ: 4 байта при МК с 16-битной адресацией) */
static void (*cbDataReceived)(uint8_t src_address, uint8_t command, char* data, uint8_t size) = 0;
static void (*cbDataReceivedSniff)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size) = 0;

/* Global static variables (RAM: 11 bytes) */

/* Sending process variables */
static uint8_t sendingState = CLUNET_SENDING_IDLE; // Current sending state
static uint8_t sendingPriority; // Sending priority (1 to 8)
static uint8_t sendingLength; // Sending data length

/* Reading process variables */
static uint8_t readingState = CLUNET_READING_IDLE; // Current reading state
static uint8_t readingTime; // Reading time
static uint8_t readingPriority; // Receiving packet priority

/* Common variables */
static uint8_t bitIndex; // Bit index
static uint8_t byteIndex; // Byte index
static uint8_t dominantTask; // Dominant task in bits
static uint8_t recessiveTask; // Recessive task in bits

/* Data buffers */
static char sendBuffer[CLUNET_SEND_BUFFER_SIZE]; // Sending data buffer
static char readBuffer[CLUNET_READ_BUFFER_SIZE]; // Reading data buffer

#ifdef CLUNET_DEVICE_NAME
 static const char devName[] = CLUNET_DEVICE_NAME; // Simple and short device name
#endif

#define SEND_IS_ACTIVE (sendingState & 1)
#define SEND_IS_IDLE (!sendingState)

/* Функция нахождения контрольной суммы Maxim iButton 8-bit */
static char
check_crc(const char* data, const uint8_t size)
{
	uint8_t crc = 0;
	uint8_t a = 0;
	do
	{
		uint8_t b = 8;
		uint8_t inbyte = data[a];
		do
		{
			uint8_t mix = crc ^ inbyte;
			crc >>= 1;
			if (mix & 1)
				crc ^= 0x8C;
			inbyte >>= 1;
		}
		while (--b);
	}
	while (++a < size);
	return crc;
}

/* Встраиваемая функция обработки входящего пакета */
static inline void
clunet_data_received(const uint8_t src_address, const uint8_t dst_address, const uint8_t command, char* data, const uint8_t size)
{
	if (cbDataReceivedSniff)
		(*cbDataReceivedSniff)(src_address, dst_address, command, data, size);

	if ((src_address != CLUNET_DEVICE_ID) && ((dst_address == CLUNET_DEVICE_ID) || (dst_address == CLUNET_BROADCAST_ADDRESS)))
	{
		/* Команда перезагрузки */
		if (command == CLUNET_COMMAND_REBOOT)
		{
			wdt_enable(WDTO_15MS);
			while (1);
		}

		if ((sendingState == CLUNET_SENDING_IDLE) || (sendingPriority <= CLUNET_PRIORITY_MESSAGE))
		{
			switch (command)
			{
				/* Ответ на поиск устройств */
				case CLUNET_COMMAND_DISCOVERY:
	
					#ifdef CLUNET_DEVICE_NAME
					clunet_send(src_address, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DISCOVERY_RESPONSE, devName, sizeof(devName) - 1);
					#else
					clunet_send(src_address, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DISCOVERY_RESPONSE, 0, 0);
					#endif
					return;
	
				/* Ответ на пинг */
				case CLUNET_COMMAND_PING:
	
					clunet_send(src_address, CLUNET_PRIORITY_COMMAND, CLUNET_COMMAND_PING_REPLY, data, size);
					return;
			}
		}
		if (cbDataReceived)
			(*cbDataReceived)(src_address, command, data, size);
	}
}

/* Timer output compare interrupt service routine (RAM: 3 bytes) */
ISR(CLUNET_TIMER_COMP_VECTOR)
{
	// Static RAM variables (2 bytes)
	static uint8_t sendingByte, numBits;

	// If in NOT ACTIVE state
	if (!(sendingState & 1))
	{
		// Reset reading state
		readingState = CLUNET_READING_IDLE;

		// If in IDLE state: disable timer output compare interrupt
		if (!sendingState)
		{
_disable:
			CLUNET_DISABLE_TIMER_COMP;
			return;
		}

		// If in WAIT state: starting sending throuth 1Т
		sendingState = CLUNET_SENDING_ACTIVE;
		sendingByte = sendingPriority - 1; // First need send priority 3 bits
		byteIndex = 0; // Data index
		recessiveTask = bitIndex = 5; // Start of priority bits
		numBits = 1; // Start bit
_delay_1t:
		CLUNET_TIMER_REG_OCR += CLUNET_T; // 1T delay
		return;
	}
	
	const uint8_t now = CLUNET_TIMER_REG;
	const uint8_t lineFree = CLUNET_SENDING;

	// If we need to free line - do it
	if (lineFree)
	{
		CLUNET_INT_ENABLE; // Enable external interrupt for collision resolving or maybe sending complete (waiting for new packet)
		CLUNET_SEND_0;
	}

	// If we must pull-down line.
	// In reading ISR we resolve post- & pre- arbitration. In this code we must check that read ISR has been executed.
	// If not - this is 3-rd type of conflict. In this case we must switch to READ mode and stop sending.
	else
	{
		// Если мы будем прижимать линию, то проверим совпадение переданных и полученных бит, если различны, то конфликт на линии - останавливаем передачу и ждем
/*		if (readingActiveBits != lastTaskBits && !(sendingState == CLUNET_SENDING_INIT && !sendingBitIndex))
		{
			sendingState = CLUNET_SENDING_WAIT;
			goto _disable;
		}
*/
		// If no conflict - disable external interrupt and pull down line
		readingTime = now;
		CLUNET_INT_DISABLE;
		CLUNET_SEND_1;
	}

	// If sending data complete: send 1T stop-bit or finish sending process.
	if (sendingBitIndex & 8)
	{
		// If we pull down the line - send 1T stop-bit.
		if (!lineFree)
			goto _delay_1t;

		sendingState = CLUNET_SENDING_IDLE;
		goto _disable;
	}

	/* COLLECTING DATA BITS */
	do
	{
		const uint8_t bitValue = sendingByte & (0x80 >> bitIndex);
		if ((lineFree && !bitValue) || (!lineFree && bitValue))
		{
			numBits++;

			// If sending byte complete: reset bit index and get next byte to send
			if (++bitIndex & 8)
			{
				// If data complete: exit and send this last bits
				if (byteIndex == sendingLength)
					break;
				sendingByte = readingBuffer[byteIndex] = sendBuffer[byteIndex];
				byteIndex++;
				bitIndex = 0;
			}
		}
		else
			break;
	}
	while (numBits != 5);
	
	// Save tasks times
	if (lineFree)
		recessiveTask = numBits;
	else
		dominantTask = numBits;

	// Update OCR
	CLUNET_TIMER_REG_OCR += CLUNET_T * numBits;

	// Bitstuff correction
	numBits = (numBits == 5);
}
/* End of ISR(CLUNET_TIMER_COMP_VECTOR) */

static void
read_switch(void)
{

	CLUNET_DISABLE_TIMER_COMP;
	sendingState = CLUNET_SENDING_WAIT;

	if (bitIndex & 0x80)
	{
		byteIndex--;
		bitIndex += 8;
	}

	if (byteIndex)
	{
		readingPriority = sendingPriority;
		readingByte = sendingBuffer[byteIndex];
	}
	else
	{
		readingPriority = 0;
		readingByte = sendingPriority - 1;
	}

	readingState = CLUNET_READING_ACTIVE;
}

/* External interrupt service routine (RAM: 4 bytes) */
ISR(CLUNET_INT_VECTOR)
{
	// Static variables (RAM: 5 bytes)
	static uint8_t readingByte, bitStuff, crc;

	// Current timer value
	const uint8_t now = CLUNET_TIMER_REG;

	// Многоцелевая переменная состояния линии и заполнения байт соответствующими значениями
	const uint8_t lineFree = CLUNET_READING ? 0x00 : 0xFF;

	/* Reading number of bits */
	uint8_t bitNum = 0; // Number of bits
	uint8_t ticks = now - readingTime;
	if ((ticks >= (CLUNET_T / 2)) && (ticks < (CLUNET_T * 5 + CLUNET_T / 2)))
	{
		uint8_t period = CLUNET_T / 2;
		for ( ; ticks >= period; period += CLUNET_T, bitNum++);
	}

	/* SENDING MODE */
	if (SEND_IS_ACTIVE)
	{
		// Interrupt on rising edge (should always run, if bitNum > sendTaskBits mean arbitration)
		if (lineFree)
		{
			// If number of reading dominant bits greater than we sended:
			if (bitNum > sendingTaskBits)
			{
				// Откатимся индексами на точку доминантного задания с учетом битстаффинга
				bitIndex -= dominantTask - (recessiveTask == 5);
				read_switch();
				goto _reading;
			}
			
		}

		// Interrupt on falling edge (alway mean EARLY ARBITRATION, need save bitNum and switch to READ mode).
		// We in this code only if someone device pull down the line before us.
		else
		{
			const uint8_t delta = CLUNET_TIMER_REG_OCR - now;
			// Conflict: switch to READ mode & stop send process.
			if (delta >= (CLUNET_T / 2))
			{
				// Conflict! Switch to READ mode! Use bitNum variable!
				// Откатимся индексами на точку рецессивного задания с учетом битстаффинга
				bitIndex -= recessiveTask - (dominantTask == 5);
				read_switch();
				goto _reading;
			}
		}

		return;
	}

	if (lineFree)
	{
		// Корректируем время по прочитанным битам
		readingTime += bitNum * CLUNET_T;

		CLUNET_TIMER_REG_OCR = readingTime + (7 * CLUNET_T - 1);
		CLUNET_ENABLE_TIMER_COMP;
	}
	else
	{
		readingTime = now; // Pulldown sync time
		CLUNET_DISABLE_TIMER_COMP;

		// Если в ожидании приема пакета, то переходим к фазе начала приемки кадра, обнуляем счетчики и выходим
		if (!readingState)
		{
			readingState = CLUNET_READING_ACTIVE;
			readingByte = readingPriority = byteIndex = crc = 0;
			bitIndex = 5;
			bitStuff = 1;
			return;
		}

	}

_reading:

	// Если линия освободилась, значит была единичная посылка - установим соответствующие биты
	if (lineFree)
		readingByte |= (0xFF >> bitIndex);

	// Если линия прижалась, значит была нулевая посылка - сбросим соответствующие биты
	else
		readingByte &= ~(0xFF >> bitIndex);

	// Обновим битовый индекс с учетом битстаффинга
	bitIndex += bitNum - bitStuff;

	if (bitIndex & 8)
	{
		// Readed priority
		if (!readingPriority)
			readingPriority = readingByte + 1;
		else
		{
			readBuffer[byteIndex] = readingByte;

			/* Update Maxim iButton 8-bit CRC with received byte */
	
			uint8_t b = 8;
			uint8_t inbyte = readingByte;
	
			do
			{
				uint8_t mix = crc ^ inbyte;
				crc >>= 1;
				if (mix & 1)
					crc ^= 0x8C;
				inbyte >>= 1;
			}
			while (--b);
			
			// Если пакет прочитан полностью, то проверим контрольную сумму
			if ((++byteIndex > CLUNET_OFFSET_SIZE) && (byteIndex > (uint8_t)readBuffer[CLUNET_OFFSET_SIZE] + CLUNET_OFFSET_DATA))
			{
				readingState = CLUNET_READING_IDLE;
				// If CRC is correct - process incoming packet
				if (!crc)
				{
					clunet_data_received (
						readBuffer[CLUNET_OFFSET_SRC_ADDRESS],
						readBuffer[CLUNET_OFFSET_DST_ADDRESS],
						readBuffer[CLUNET_OFFSET_COMMAND],
						readBuffer + CLUNET_OFFSET_DATA,
						readBuffer[CLUNET_OFFSET_SIZE]
					);
				}
			}
	
			// Если данные прочитаны не полностью и мы не выходим за пределы буфера, то присвоим очередной байт и подготовим битовый индекс
			else if (byteIndex < CLUNET_READ_BUFFER_SIZE)
			{
				bitIndex &= 7;
				readBuffer[byteIndex] = lineFree;
			}
	
			// Иначе ошибка: нехватка приемного буфера -> игнорируем пакет
			else
				readingState = CLUNET_READING_IDLE;
		}
	}

	/* Проверка на битстаффинг, учитываем в следующем цикле */
	bitStuff = (bitNum == 5);
}
/* End of ISR(CLUNET_INT_VECTOR) */

void
clunet_init()
{
	sendingLength = MCUSR; // Используем переменную sendingLength для отправки содержимого регистра MCUSR
	MCUSR = 0;
	wdt_disable();
	CLUNET_TIMER_INIT;
	CLUNET_PIN_INIT;
	CLUNET_INT_INIT;
	sei();
	clunet_send (
		CLUNET_BROADCAST_ADDRESS,
		CLUNET_PRIORITY_MESSAGE,
		CLUNET_COMMAND_BOOT_COMPLETED,
		(char*)&sendingLength,
		sizeof(sendingLength)
	);
}

void
clunet_send(const uint8_t address, const uint8_t prio, const uint8_t command, const char* data, const uint8_t size)
{
	/* Если размер данных в пределах буфера передачи (максимально для протокола 250 байт) */
	if (size < (CLUNET_SEND_BUFFER_SIZE - CLUNET_OFFSET_DATA))
	{

		CLUNET_DISABLE_TIMER_COMP;	// Запретим прерывание сравнения, тем самым запретим и отправку и исключим конкурентный доступ к буферу

		/* Заполняем переменные */
		sendingPriority = (prio > 8) ? 8 : prio ? : 1;
		sendBuffer[CLUNET_OFFSET_SRC_ADDRESS] = CLUNET_DEVICE_ID;
		sendBuffer[CLUNET_OFFSET_DST_ADDRESS] = address;
		sendBuffer[CLUNET_OFFSET_COMMAND] = command;
		sendBuffer[CLUNET_OFFSET_SIZE] = size;
		
		/* Есть данные для отправки? Тогда скопируем их в буфер */
		if (size && data)
		{
			uint8_t idx = 0;
			do
				sendBuffer[CLUNET_OFFSET_DATA + idx] = data[idx];
			while (++idx < size);
		}

		sendBuffer[CLUNET_OFFSET_DATA + size] = check_crc(sendBuffer, CLUNET_OFFSET_DATA + size);

		sendingLength = size + CLUNET_OFFSET_DATA + 1;

		sendingState = CLUNET_SENDING_WAIT;	// Фаза ожидания линии

		// Если линия свободна, то запланируем отправку
		if (!CLUNET_READING)
		{
			CLUNET_TIMER_REG_OCR = CLUNET_TIMER_REG + (7 * CLUNET_T - 1);
			CLUNET_ENABLE_TIMER_COMP;
		}

		// Все равно отпустим линию (вдруг задержки на заряд паразитной емкости линии) и процедура внешнего прерывания сама запланирует отправку
		CLUNET_SEND_0;

	}
}
/* Конец void clunet_send(.....) */

/* Возвращает 0, если готов к передаче, иначе приоритет текущей задачи */
uint8_t
clunet_ready_to_send()
{
	return sendingState ? sendingPriority : 0;
}

void
clunet_set_on_data_received(void (*f)(uint8_t src_address, uint8_t command, char* data, uint8_t size))
{
	cbDataReceived = f;
}

void
clunet_set_on_data_received_sniff(void (*f)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size))
{
	cbDataReceivedSniff = f;
}
