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

#define STATE_IDLE 0
#define STATE_ACTIVE 1
#define STATE_WAIT_INTERFRAME 2
#define STATE_PROCESS 4

#define RECEIVED_SRC_ADDRESS (uint8_t)readBuffer[CLUNET_OFFSET_SRC_ADDRESS]
#define RECEIVED_DST_ADDRESS (uint8_t)readBuffer[CLUNET_OFFSET_DST_ADDRESS]
#define RECEIVED_COMMAND (uint8_t)readBuffer[CLUNET_OFFSET_COMMAND]
#define RECEIVED_DATA_PTR readBuffer + CLUNET_OFFSET_DATA
#define RECEIVED_DATA_SIZE (uint8_t)readBuffer[CLUNET_OFFSET_SIZE]


/* Pointers to the callback functions on receiving packet (must be short as possible) */
static void (*cbDataReceived)(uint8_t src_address, uint8_t command, char* data, uint8_t size) = 0;
static void (*cbDataReceivedSniff)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size) = 0;

/* Global static variables (RAM: 7 bytes) */
static uint8_t readingState = STATE_IDLE; // Current reading state
static uint8_t sendingState = STATE_IDLE; // Current sending state
static uint8_t readingPriority; // Receiving packet priority
static uint8_t sendingPriority; // Sending priority (1 to 8)
static uint8_t sendingLength; // Sending data length
static uint8_t dominantTask; // Dominant task (bits)
static uint8_t recessiveTask; // Recessive task (bits)

/* Data buffers */
static char sendBuffer[CLUNET_SEND_BUFFER_SIZE]; // Sending data buffer
static char readBuffer[CLUNET_READ_BUFFER_SIZE]; // Reading data buffer

#ifdef CLUNET_DEVICE_NAME
 static const char devName[] = CLUNET_DEVICE_NAME; // Simple and short device name
#endif

/* Function for calculate Maxim iButton 8-bit checksum */
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

/* Function for process receiving packet */
static void
process_received_packet(void)
{
	const uint8_t src_address = RECEIVED_SRC_ADDRESS;
	const uint8_t dst_address = RECEIVED_DST_ADDRESS;
	const uint8_t command = RECEIVED_COMMAND;
	const uint8_t data_size = RECEIVED_DATA_SIZE;
	char* data_ptr = RECEIVED_DATA_PTR;

	if (cbDataReceivedSniff)
		(*cbDataReceivedSniff)(src_address, dst_address, command, data_ptr, data_size);

	if ((src_address != CLUNET_DEVICE_ID) && ((dst_address == CLUNET_DEVICE_ID) || (dst_address == CLUNET_BROADCAST_ADDRESS)))
	{
		/* Команда перезагрузки */
		if (command == CLUNET_COMMAND_REBOOT)
		{
			wdt_enable(WDTO_15MS);
			while (1);
		}

		if (!sendingState || (sendingPriority <= CLUNET_PRIORITY_MESSAGE))
		{
			switch (command)
			{
				/* Answer for discovery command */
				case CLUNET_COMMAND_DISCOVERY:
	
					#ifdef CLUNET_DEVICE_NAME
					clunet_send(src_address, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DISCOVERY_RESPONSE, devName, sizeof(devName) - 1);
					#else
					clunet_send(src_address, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DISCOVERY_RESPONSE, 0, 0);
					#endif
					return;
	
				/* Answer for ping */
				case CLUNET_COMMAND_PING:
	
					clunet_send(src_address, CLUNET_PRIORITY_COMMAND, CLUNET_COMMAND_PING_REPLY, data_ptr, data_size);
					return;
			}
		}
		if (cbDataReceived)
			(*cbDataReceived)(src_address, command, data_ptr, data_size);
	}
}

/* Timer output compare interrupt service routine (RAM: 4 bytes) */
ISR(CLUNET_TIMER_COMP_VECTOR)
{
	// Static variables
	static uint8_t dataByte, byteIndex, bitIndex, numBits;
	
	// If in NOT ACTIVE state
	if (!(sendingState & STATE_ACTIVE))
	{
		// If we must PROCESS received packet: 
		if (readingState & STATE_PROCESS)
		{
			CLUNET_SEND_1;
			if (!recessiveTask || !check_crc(readBuffer, RECEIVED_DATA_SIZE + CLUNET_OFFSET_DATA + 1))
				process_received_packet();
			CLUNET_SEND_0;
			readingState = STATE_WAIT_INTERFRAME;
			return;
		}
		// Reset reading state
		readingState = STATE_IDLE;
		// If in IDLE state: disable timer output compare interrupt
		if (!sendingState)
		{
			CLUNET_DISABLE_TIMER_COMP;
			return;
		}
		// If in WAIT_INTERFRAME state: starting sending throuth 1Т
		sendingState = STATE_ACTIVE; // Set sending process to ACTIVE state
		dataByte = sendingPriority - 1; // First need send priority (3 bits)
		byteIndex = 0; // Data index
		bitIndex = 5; // Priority bits position
		numBits = 1; // Start bit
		recessiveTask = 0; // Reset recessive task
		CLUNET_TIMER_REG_OCR += CLUNET_T; // 1T delay
		return;
	}
	
	const uint8_t lineFree = CLUNET_SENDING;

	// If we need to free line - do it and check for end data.
	if (lineFree)
	{
		CLUNET_SEND_0;
		// If data sending complete
		if (bitIndex & 8)
		{
			CLUNET_DISABLE_TIMER_COMP;
			sendingState = STATE_IDLE;
			return;
		}
	}

	// If we must pull-down line.
	// In reading ISR we resolve post- & pre- arbitration. In this code we must check that read ISR has been executed.
	// If not - this is 3-rd type of conflict. In this case we must switch to READ mode and stop sending.
	else
	{
		// Check if we not been in reading ISR or first send cycle
		if (recessiveTask)
		{
			CLUNET_DISABLE_TIMER_COMP;
			sendingState = STATE_WAIT_INTERFRAME;
			return;
		}

		CLUNET_SEND_1;

		// If data sending complete
		if (bitIndex & 8)
		{
			CLUNET_TIMER_REG_OCR += CLUNET_T;
			dominantTask = 1;
			return;
		}
	}

	/* COLLECTING DATA BITS */
	do
	{
		const uint8_t bitValue = dataByte & (0x80 >> bitIndex);
		if ((lineFree && !bitValue) || (!lineFree && bitValue))
		{
			numBits++;

			// If sending byte complete: reset bit index and get next byte to send
			if (++bitIndex & 8)
			{
				// If data complete: exit and send this last bits
				if (byteIndex == sendingLength)
					break;
				dataByte = sendBuffer[byteIndex++];
				bitIndex = 0;
			}
		}
		else
			break;
	}
	while (numBits != 5);
	
	// Update OCR
	CLUNET_TIMER_REG_OCR += CLUNET_T * numBits;

	if (lineFree)
		recessiveTask = numBits;
	else
		dominantTask = numBits;

	// Bitstuff correction
	numBits = (numBits == 5);
}
/* End of ISR(CLUNET_TIMER_COMP_VECTOR) */

/* External interrupt service routine (RAM: 2 bytes) */
ISR(CLUNET_INT_VECTOR)
{
	// Static variables (RAM: 6 bytes)
	static uint8_t dataByte, byteIndex, bitIndex, bitStuff, lastTime, crc;

	const uint8_t now = CLUNET_TIMER_REG;
	const uint8_t lineFree = CLUNET_READING ? 0x00 : 0xFF;
	const uint8_t ticks = now - lastTime;

	// Reading bits
	uint8_t bitNum = 0; // Number of reading bits
	const uint8_t t12 = CLUNET_T / 2;
	if ((ticks >= t12) && (ticks < (5 * CLUNET_T + t12)))
	{
		uint8_t period = t12;
		for ( ; ticks >= period; period += CLUNET_T, bitNum++);
	}

	/* SENDING ACTIVE */
	if (sendingState & STATE_ACTIVE)
	{
		if ((lineFree && (bitNum > dominantTask)) || (!lineFree && (bitNum < recessiveTask)))
		{
			CLUNET_DISABLE_TIMER_COMP;
			sendingState = STATE_WAIT_INTERFRAME;
			recessiveTask = 1; // Become CRC check flag.
		}
		// Use as reading interrupt flag (reset it)
		else
			recessiveTask = 0;
	}
	else if (lineFree)
	{
		// Planning reset reading state and start sending if in waiting state
		CLUNET_TIMER_REG_OCR = now + (7 * CLUNET_T - 1);
		CLUNET_ENABLE_TIMER_COMP;
	}
	else
	{
		CLUNET_DISABLE_TIMER_COMP;
	}
	
	// Update reading time value
	if (lineFree)
		lastTime += bitNum * CLUNET_T;
	else
	{
		lastTime = now;
		if (!readingState)
		{
			readingState = STATE_ACTIVE;
			dataByte = readingPriority = byteIndex = crc = 0;
			bitIndex = 5;
			bitStuff = 1;
			recessiveTask = 1;
			return;
		}
	}
	
	if (!bitNum)
		readingState = STATE_WAIT_INTERFRAME;
	
	// Exit if in not active state
	if (!(readingState & STATE_ACTIVE))
		return;
	
	const uint8_t mask = (0xFF >> bitIndex);
	
	// Если линия освободилась, значит была единичная посылка - установим соответствующие биты
	if (lineFree)
		dataByte |= mask;

	// Если линия прижалась, значит была нулевая посылка - сбросим соответствующие биты
	else
		dataByte &= ~mask;

	// Update bit index with bitstuff correction
	bitIndex += bitNum - bitStuff;

	// Byte readed
	if (bitIndex & 8)
	{
		if (!readingPriority)
			readingPriority = dataByte + 1;
		else
			readBuffer[byteIndex++] = dataByte;

		// Whole packet readed
		if ((byteIndex > CLUNET_OFFSET_SIZE) && (byteIndex > (uint8_t)readBuffer[CLUNET_OFFSET_SIZE] + CLUNET_OFFSET_DATA))
		{
			readingState = STATE_WAIT_INTERFRAME;
			// Check CRC only if we not sending (save CPU time)
			if (!recessiveTask || !check_crc(readBuffer, byteIndex))
				process_received_packet();
		}
		
		// Если данные прочитаны не полностью и мы не выходим за пределы буфера, то присвоим очередной байт и подготовим битовый индекс
		else if (byteIndex < CLUNET_READ_BUFFER_SIZE)
		{
			bitIndex &= 7;
			dataByte = lineFree;
		}
		
		// Иначе ошибка: нехватка приемного буфера -> игнорируем пакет
		else
			readingState = STATE_WAIT_INTERFRAME;
	}

	/* Проверка на битстаффинг, учитываем в следующем цикле */
	bitStuff = (bitNum == 5);
}
/* End of ISR(CLUNET_INT_VECTOR) */

void
clunet_init(void)
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
	if (!sendingState && size < (CLUNET_SEND_BUFFER_SIZE - CLUNET_OFFSET_DATA))
	{
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

		clunet_resend_last_packet();
	}
}
/* Конец void clunet_send(.....) */

/* Возвращает 0, если готов к передаче, иначе приоритет текущей задачи */
uint8_t
clunet_ready_to_send(void)
{
	return sendingState ? sendingPriority : 0;
}

void
clunet_resend_last_packet(void)
{
	sendingState = STATE_WAIT_INTERFRAME;

	// If ready to reading: start sending as soon as possible
	if (!readingState)
	{
		CLUNET_ENABLE_TIMER_COMP;
		CLUNET_TIMER_REG_OCR = CLUNET_TIMER_REG;
	}
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
