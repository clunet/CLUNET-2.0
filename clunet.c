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
#include <util/crc16.h>

#define STATE_IDLE 0
#define STATE_ACTIVE 1
#define STATE_WAIT_INTERFRAME 2
#define STATE_PROCESS 4

#define RECEIVED_SRC_ADDRESS (uint8_t)read_buffer[CLUNET_OFFSET_SRC_ADDRESS]
#define RECEIVED_DST_ADDRESS (uint8_t)read_buffer[CLUNET_OFFSET_DST_ADDRESS]
#define RECEIVED_COMMAND (uint8_t)read_buffer[CLUNET_OFFSET_COMMAND]
#define RECEIVED_DATA_PTR read_buffer + CLUNET_OFFSET_DATA
#define RECEIVED_DATA_SIZE (uint8_t)read_buffer[CLUNET_OFFSET_SIZE]

/* Pointers to the callback functions on receiving packet (must be short as possible) */
static void (*cb_data_received)(uint8_t src_address, uint8_t command, char* data, uint8_t size) = 0;
static void (*cb_data_received_sniff)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size) = 0;

/* Global static variables (RAM: 7 bytes) */
static uint8_t reading_state; // Current reading state
static uint8_t sending_state = STATE_IDLE; // Current sending state
static uint8_t reading_priority; // Receiving packet priority
static uint8_t sending_priority; // Sending priority (1 to 8)
static uint8_t sending_length; // Sending data length
static uint8_t dominant_task; // Dominant task (bits)
static uint8_t recessive_task; // Recessive task (bits)

/* Data buffers */
static char send_buffer[CLUNET_SEND_BUFFER_SIZE]; // Sending data buffer
static char read_buffer[CLUNET_READ_BUFFER_SIZE]; // Reading data buffer

#ifdef CLUNET_DEVICE_NAME
 static const char device_name[] = CLUNET_DEVICE_NAME; // Simple and short device name
#endif

/* Function for calculate Maxim iButton 8-bit checksum */
static char
check_crc(const char* data, const uint8_t size)
{
	uint8_t crc = 0;
	uint8_t a = 0;
	do
		crc = _crc_ibutton_update(crc, data[a]);
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

	if (cb_data_received_sniff)
		(*cb_data_received_sniff)(src_address, dst_address, command, data_ptr, data_size);

	if ((src_address != CLUNET_DEVICE_ID) && ((dst_address == CLUNET_DEVICE_ID) || (dst_address == CLUNET_BROADCAST_ADDRESS)))
	{
		/* Команда перезагрузки */
		if (command == CLUNET_COMMAND_REBOOT)
		{
			wdt_enable(WDTO_15MS);
			while (1);
		}

		if (!sending_state || (sending_priority <= CLUNET_PRIORITY_MESSAGE))
		{
			switch (command)
			{
				/* Answer for discovery command */
				case CLUNET_COMMAND_DISCOVERY:
					
					#ifdef CLUNET_DEVICE_NAME
					clunet_send(src_address, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DISCOVERY_RESPONSE, device_name, sizeof(device_name) - 1);
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
		if (cb_data_received)
			(*cb_data_received)(src_address, command, data_ptr, data_size);
	}
}

/* Timer output compare interrupt service routine (RAM: 4 bytes) */
ISR(CLUNET_TIMER_COMP_VECTOR)
{
	// Static variables
	static uint8_t data_byte, byte_index, bit_mask, bit_task;
	
	// If in NOT ACTIVE state
	if (!(sending_state & STATE_ACTIVE))
	{
		// Reset reading state
		reading_state = STATE_IDLE;
		// If in IDLE state: disable timer output compare interrupt
		if (!sending_state)
		{
			CLUNET_DISABLE_TIMER_COMP;
			return;
		}
		// If in WAIT_INTERFRAME state: starting sending throuth 1Т
		sending_state = STATE_ACTIVE; // Set sending process to ACTIVE state
		data_byte = sending_priority - 1; // First need send priority (3 bits)
		byte_index = 0; // Data index
		bit_mask = 0x04; // Init bit mask by priority MSB location
		bit_task = 1; // Start bit
		recessive_task = 0; // Reset recessive task
		CLUNET_TIMER_REG_OCR += CLUNET_T; // 1T delay
		return;
	}
	
	const uint8_t line_pullup = CLUNET_SENDING;

	// If we need to free line - do it and check for end data.
	if (line_pullup)
	{
		CLUNET_SEND_0;
		// If data sending complete
		if (!bit_mask)
		{
			CLUNET_DISABLE_TIMER_COMP;
			sending_state = STATE_IDLE;
			return;
		}
	}

	// If we must pull-down line.
	// In reading ISR we resolve post- & pre- arbitration. In this code we must check that read ISR has been executed.
	// If not - this is 3-rd type of conflict. In this case we must switch to READ mode and stop sending.
	else
	{
		// Check if we not been in reading ISR or first send cycle
		if (recessive_task)
		{
			CLUNET_DISABLE_TIMER_COMP;
			sending_state = STATE_WAIT_INTERFRAME;
			return;
		}

		CLUNET_SEND_1;

		// If data sending complete
		if (!bit_mask)
		{
			CLUNET_TIMER_REG_OCR += CLUNET_T;
			dominant_task = 1;
			return;
		}
	}

	/* COLLECTING DATA BITS */
	do
	{
		const uint8_t bit_value = data_byte & bit_mask;
		if ((line_pullup && !bit_value) || (!line_pullup && bit_value))
		{
			bit_task++;

			// If sending byte complete: reset bit index and get next byte to send
			if (!(bit_mask >>= 1))
			{
				// If data complete: exit and send this last bits
				if (byte_index == sending_length)
					break;
				data_byte = send_buffer[byte_index++];
				bit_mask = 0x80;
			}
		}
		else
			break;
	}
	while (bit_task != 5);
	
	// Update OCR
	CLUNET_TIMER_REG_OCR += CLUNET_T * bit_task;

	if (line_pullup)
		recessive_task = bit_task;
	else
		dominant_task = bit_task;

	// Bit stuffing correction
	bit_task = (bit_task == 5);
}
/* End of ISR(CLUNET_TIMER_COMP_VECTOR) */

/* External interrupt service routine (RAM: 6 bytes) */
ISR(CLUNET_INT_VECTOR)
{
	// Static variables (RAM: 6 bytes)
	static uint8_t data_byte, byte_index, bit_index, bit_stuffing, last_time, crc;
	
	const uint8_t now = CLUNET_TIMER_REG;
	const uint8_t front_edge = CLUNET_READING ? 0 : 255;
	uint8_t num_bits = 0; // Number of reading bits

	if ((reading_state & STATE_ACTIVE) || (sending_state & STATE_ACTIVE))
	{
		// Reading bits
		const uint8_t ticks = now - last_time;
		const uint8_t t12 = CLUNET_T / 2;
		if ((ticks >= t12) && (ticks < (5 * CLUNET_T + t12)))
		{
			uint8_t period = t12;
			for ( ; ticks >= period; period += CLUNET_T, num_bits++);
			if (front_edge)
				last_time += num_bits * CLUNET_T;
		}
	}

	// If sending is active
	if (sending_state & STATE_ACTIVE)
	{
		// Check for conflict on the line
		if ((front_edge && (num_bits > dominant_task)) || (!front_edge && (num_bits < recessive_task)))
		{
			sending_state = recessive_task = STATE_WAIT_INTERFRAME;
			goto _wait_interframe;
		}
		recessive_task = 0; // Used as reading interrupt flag
	}
	// If sending not active
	else
	{
_wait_interframe:
		// If line is pull-up
		if (front_edge)
		{
			CLUNET_TIMER_REG_OCR = now + (7 * CLUNET_T - 1);
			CLUNET_ENABLE_TIMER_COMP;
		}
		// If line is pull-down
		else
			CLUNET_DISABLE_TIMER_COMP;
	}

	// On falling edge
	if (!front_edge)
	{
		last_time = now; // Update time value
		// If reading in IDLE state - start reading process
		if (!reading_state)
		{
			data_byte = reading_priority = byte_index = crc = 0;
			recessive_task = bit_stuffing = 1;
			reading_state = STATE_ACTIVE;
			bit_index = 5;
			return;
		}
	}

	// On error reading bits
	if (!num_bits)
		reading_state = STATE_WAIT_INTERFRAME;

	// Exit if reading is NOT ACTIVE
	if (!(reading_state & STATE_ACTIVE))
		return;

	const uint8_t mask = (0xFF >> bit_index);

	// Если линия освободилась, значит была единичная посылка - установим соответствующие биты
	if (front_edge)
		data_byte |= mask;
	// Если линия прижалась, значит была нулевая посылка - сбросим соответствующие биты
	else
		data_byte &= ~mask;

	// Update bit index with bit stuffing correction
	bit_index += num_bits - bit_stuffing;

	// Whole byte readed
	if (bit_index & 8)
	{
		if (reading_priority)
		{
			read_buffer[byte_index++] = data_byte;
			crc = _crc_ibutton_update(crc, data_byte);
		}
		else
			reading_priority = data_byte + 1;

		// Whole packet readed
		if ((byte_index > CLUNET_OFFSET_SIZE) && (byte_index > RECEIVED_DATA_SIZE + CLUNET_OFFSET_DATA))
		{
			// Packet from another device, line is busy
			if (/*!recessive_task || !check_crc(read_buffer, byte_index)*/ !crc)
				process_received_packet();
			reading_state = STATE_WAIT_INTERFRAME;
		}
		
		// Если данные прочитаны не полностью и мы не выходим за пределы буфера, то присвоим очередной байт и подготовим битовый индекс
		else if (byte_index < CLUNET_READ_BUFFER_SIZE)
		{
			bit_index &= 7;
			data_byte = front_edge;
		}
		
		// Иначе ошибка: нехватка приемного буфера -> игнорируем пакет
		else
			reading_state = STATE_WAIT_INTERFRAME;
	}

	/* Проверка на битстаффинг, учитываем в следующем цикле */
	bit_stuffing = (num_bits == 5);
}
/* End of ISR(CLUNET_INT_VECTOR) */

void
clunet_init(void)
{
	sending_length = MCUSR; // Используем переменную sending_length для отправки содержимого регистра MCUSR
	MCUSR = 0;
	wdt_disable();
	CLUNET_TIMER_INIT;
	CLUNET_PIN_INIT;
	CLUNET_INT_INIT;
	// Start with WAIT_INTERFRAME state
	reading_state = STATE_WAIT_INTERFRAME;
	// If line is free, then planning reset reading state
	if (!CLUNET_READING)
	{
		CLUNET_TIMER_REG_OCR = CLUNET_TIMER_REG + (7 * CLUNET_T - 1);
		CLUNET_ENABLE_TIMER_COMP;
	}
	sei(); // Enable global interrupts
	clunet_send (
		CLUNET_BROADCAST_ADDRESS,
		CLUNET_PRIORITY_MESSAGE,
		CLUNET_COMMAND_BOOT_COMPLETED,
		(char*)&sending_length,
		sizeof(sending_length)
	);
}

void
clunet_send(const uint8_t address, const uint8_t prio, const uint8_t command, const char* data, const uint8_t size)
{
	/* Если размер данных в пределах буфера передачи (максимально для протокола 250 байт) */
	if (size < (CLUNET_SEND_BUFFER_SIZE - CLUNET_OFFSET_DATA))
	{
		if (sending_state)
		{
			CLUNET_DISABLE_TIMER_COMP;
			sending_state = STATE_IDLE;
			CLUNET_SEND_0;
		}
		
		/* Заполняем переменные */
		sending_priority = (prio > 8) ? 8 : prio ? : 1;
		send_buffer[CLUNET_OFFSET_SRC_ADDRESS] = CLUNET_DEVICE_ID;
		send_buffer[CLUNET_OFFSET_DST_ADDRESS] = address;
		send_buffer[CLUNET_OFFSET_COMMAND] = command;
		send_buffer[CLUNET_OFFSET_SIZE] = size;
		
		/* Есть данные для отправки? Тогда скопируем их в буфер */
		if (size && data)
		{
			uint8_t idx = 0;
			do
				send_buffer[CLUNET_OFFSET_DATA + idx] = data[idx];
			while (++idx < size);
		}
		
		send_buffer[CLUNET_OFFSET_DATA + size] = check_crc(send_buffer, CLUNET_OFFSET_DATA + size);
		
		sending_length = size + CLUNET_OFFSET_DATA + 1;
		
		clunet_resend_last_packet();
	}
}
/* Конец void clunet_send(.....) */

/* Возвращает 0, если готов к передаче, иначе приоритет текущей задачи */
uint8_t
clunet_ready_to_send(void)
{
	return sending_state ? sending_priority : 0;
}

void
clunet_resend_last_packet(void)
{
	// If sending in IDLE state
	if (!sending_state)
	{
		sending_state = STATE_WAIT_INTERFRAME; // Set sending to WAIT_INTERFRAME state
	
		// If ready to reading: start sending as soon as possible
		if (!reading_state)
		{
			CLUNET_ENABLE_TIMER_COMP;
			CLUNET_TIMER_REG_OCR = CLUNET_TIMER_REG;
		}

		else if (!CLUNET_READING)
		{
			CLUNET_TIMER_REG_OCR = CLUNET_TIMER_REG + (7 * CLUNET_T - 1);
			CLUNET_ENABLE_TIMER_COMP;
		}

		else
			CLUNET_DISABLE_TIMER_COMP;
	}
}

void
clunet_set_on_data_received(void (*f)(uint8_t src_address, uint8_t command, char* data, uint8_t size))
{
	cb_data_received = f;
}

void
clunet_set_on_data_received_sniff(void (*f)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size))
{
	cb_data_received_sniff = f;
}
