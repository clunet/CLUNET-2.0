/* Name: clunet.c
 * Project: CLUNET network driver
 * Author: Alexey Avdyukhin
 * Creation Date: 2013-09-09
 * License: DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 */

#include "defines.h"
#include "clunet_config.h"
#include "bits.h"
#include "clunet.h"

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void (*on_data_received)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size) = 0;
void (*on_data_received_sniff)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size) = 0;

static volatile uint8_t clunetSendingState = CLUNET_SENDING_IDLE;
static volatile uint8_t clunetSendingDataLength;
static volatile uint8_t clunetSendingByteIndex;
static volatile uint8_t clunetSendingBitIndex;
static volatile uint8_t clunetSendingBitStuff;
static volatile uint8_t clunetReadingState = CLUNET_READING_IDLE;
static volatile uint8_t clunetReadingByteIndex;
static volatile uint8_t clunetReadingBitIndex;
static volatile uint8_t clunetReadingBitStuff;
static volatile uint8_t clunetSyncBits;
static volatile uint8_t clunetCurrentPrio;

static volatile uint8_t clunetTimerStart = 0;

static volatile char dataToSend[CLUNET_SEND_BUFFER_SIZE];
static volatile char dataToRead[CLUNET_READ_BUFFER_SIZE];

static inline void
clunet_read_rst_send()
{
	// подождем 7Т, сбросим статус чтения на готов, потом при необходимости через 1Т запустим передачу
	CLUNET_TIMER_REG_OCR = CLUNET_TIMER_REG + (7*CLUNET_T);
	CLUNET_ENABLE_TIMER_COMP;	// Включаем прерывание сравнения таймера
}

static inline char
check_crc(const char* data, const uint8_t size)
{
      uint8_t crc = 0;
      uint8_t i, j;
      for (i = 0; i < size; i++)
      {
            uint8_t inbyte = data[i];
            for (j = 0 ; j < 8 ; j++)
            {
                  uint8_t mix = (crc ^ inbyte) & 1;
                  crc >>= 1;
                  if (mix) crc ^= 0x8C;
                  inbyte >>= 1;
            }
      }
      return crc;
}

static inline void
clunet_data_received(const uint8_t src_address, const uint8_t dst_address, const uint8_t command, char* data, const uint8_t size)
{
	if (on_data_received_sniff)
		(*on_data_received_sniff)(src_address, dst_address, command, data, size);

	if (src_address == CLUNET_DEVICE_ID) return;	// Игнорируем сообщения от самого себя!

	if ((dst_address != CLUNET_DEVICE_ID) &&
		(dst_address != CLUNET_BROADCAST_ADDRESS)) return;	// Игнорируем сообщения не для нас

	// Команда перезагрузки. Перезагрузим по сторожевому таймеру
	if (command == CLUNET_COMMAND_REBOOT)
	{
		cli();
		set_bit(WDTCR, WDE);
		while(1);
	}

	if ((clunetSendingState == CLUNET_SENDING_IDLE) || (clunetCurrentPrio <= CLUNET_PRIORITY_MESSAGE))
	{
		/* Ответ на поиск устройств */
		if (command == CLUNET_COMMAND_DISCOVERY)
		{
#ifdef CLUNET_DEVICE_NAME
			char buf[] = CLUNET_DEVICE_NAME;
			uint8_t len = 0; while(buf[len]) len++;
			clunet_send(src_address, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DISCOVERY_RESPONSE, buf, len);
#else
			clunet_send(src_address, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DISCOVERY_RESPONSE, 0, 0);
#endif
		}
		/* Ответ на пинг */
		else if (command == CLUNET_COMMAND_PING)
			clunet_send(src_address, CLUNET_PRIORITY_COMMAND, CLUNET_COMMAND_PING_REPLY, data, size);
	}

	if (on_data_received)
		(*on_data_received)(src_address, dst_address, command, data, size);
	
}

/* Процедура прерывания сравнения таймера */
ISR(CLUNET_TIMER_COMP_VECTOR)
{

	/* Если мы в ожидании освобождения линии, то начнем передачу через 1Т, предварительно сбросив статус чтения */
	if (clunetSendingState == CLUNET_SENDING_WAITING_LINE)
	{
		clunetReadingState = CLUNET_READING_IDLE;
		clunetSendingState = CLUNET_SENDING_INIT;
		CLUNET_TIMER_REG_OCR += CLUNET_T;
		return;
	}

	/* Если мы закончили передачу, то освободим передатчик, а условие ниже, сделает все необходимое */
	if (clunetSendingState == CLUNET_SENDING_DONE)
		clunetSendingState = CLUNET_SENDING_IDLE;	// Указываем, что передатчик свободен
	
	/* Иначе если передачу необходимо продолжить, то сначала проверим на конфликт */
	else if (!CLUNET_SENDING && CLUNET_READING)
	{
		CLUNET_DISABLE_TIMER_COMP;					// Выключаем прерывание сравнения таймера
		clunetSendingState = CLUNET_SENDING_WAITING_LINE;		// Переходим в режим ожидания линии
	}

	/* Все в порядке, можем продолжать */
	else
	{
		/* Переменная количества передаваемых бит (сколько периодов Т задержка при следующем вызове прерывания) */
		uint8_t numBits = 0;

		CLUNET_SEND_INVERT;	// Инвертируем значение сигнала
	
		/* Смотрим фазу передачи */
		switch (clunetSendingState)
		{
			// Главная фаза передачи данных!
			case CLUNET_SENDING_DATA:
			
				numBits = clunetSendingBitStuff;

				clunetSendingBitStuff = 0;

				uint8_t xBitMask = (CLUNET_SENDING) ? 0 : 0x80;
				
				/* Если мы прижали линию, то ищем единичные биты, иначе - нулевые */
				while ((0x80 & (dataToSend[clunetSendingByteIndex] << clunetSendingBitIndex)) ^ xBitMask)
				{
				
					/* Если передан байт данных */
					if (++clunetSendingBitIndex & 8)
					{

						/* Если не все данные отосланы */
						if (++clunetSendingByteIndex < clunetSendingDataLength)
							clunetSendingBitIndex = 0;		// начинаем передачу следующего байта с бита 0

						/* Иначе передача всех данных закончена */
						else
						{
							clunetSendingState++;		// переходим к следующей фазе завершения передачи пакета
							numBits++;
							break;
						}

					}

					/* Если будем отправлять 5 одноименных бит, то запланируем битстаффинг в следующей передаче */
					if (++numBits == 5)
					{
						clunetSendingBitStuff = 1;
						break;
					}
					
				}
				
				break;

			// Фаза отправки стартового бита, а также битов приоритета, старшего бита данных при условии их равенства единице
			case CLUNET_SENDING_INIT:

				clunetSendingBitStuff = clunetSendingByteIndex = clunetSendingBitIndex = 0;
				
				uint8_t prio = clunetCurrentPrio - 1;

				/* Считаем сколько бит в приоритете единичных, учитывая стоповый */
				while((prio << ++numBits) & 8);

				// Если отправлены все биты приоритета
				if(numBits & 4)
					clunetSendingState = CLUNET_SENDING_DATA;	// Перепрыгнем стадию отправки приоритета
				else
				{
					clunetSendingState = CLUNET_SENDING_PRIO;	// К следующей фазе
					clunetSendingBitIndex = numBits;		// Запомним на каком бите приоритета остановились
				}

				break;

			// Стадия отправки приоритета и начальных битов данных
			case CLUNET_SENDING_PRIO:
			
				numBits = 0;

				xBitMask = (CLUNET_SENDING) ? 0 : 8;

				prio = clunetCurrentPrio - 1;

				// Если линия прижата, то ищем единичные биты
				while ((8 & (prio << clunetSendingBitIndex)) ^ xBitMask)
				{
					clunetSendingBitIndex++;
					numBits++;
				}

				// Если отправлены все биты приоритета
				if(clunetSendingBitIndex & 4)
				{

					xBitMask <<= 4;

					// Если линия прижата, то ищем нулевые биты
					while((0x80 & (dataToSend[0] << (clunetSendingBitIndex - 4))) ^ xBitMask)
					{
						clunetSendingBitIndex++;
						if (++numBits == 5)
							clunetSendingBitStuff = 1;
					}

					clunetSendingBitIndex -= 4;
					clunetSendingState++;			// К стадии отправки данных

				}

				break;

			case CLUNET_SENDING_STOP:
				
				// Если линия была прижата (последний бит 1), то просто отпускаем и освобождаем передатчик через 7Т
				if (!CLUNET_SENDING)
				{
					numBits = 7;
					clunetSendingState++;
				}
				// Иначе если была отпущена (последний бит 0), то сделаем короткий импульс 1Т, после которого освободим передатчик через 7Т
				else
					numBits = 1;

		}

		CLUNET_TIMER_REG_OCR += CLUNET_T * numBits;

	}

	// Если передатчик освободился, сбросим статус чтения и пока сюда не планируем возвращаться
	if (clunetSendingState == CLUNET_SENDING_IDLE)
	{
		clunetReadingState = CLUNET_READING_IDLE;
		CLUNET_DISABLE_TIMER_COMP;
	}

}


void
clunet_send(const uint8_t address, const uint8_t prio, const uint8_t command, const char* data, const uint8_t size)
{
	/* Если размер данных в пределах буфера передачи (максимально для протокола 250 байт) */
	if (size < (CLUNET_SEND_BUFFER_SIZE - CLUNET_OFFSET_DATA))
	{
		/* Прерываем текущую передачу, если есть такая */
		if (clunetSendingState)
		{
			CLUNET_DISABLE_TIMER_COMP;
			CLUNET_SEND_0;
		}

		/* Заполняем переменные */
		clunetCurrentPrio = (prio > 8) ? 8 : prio ? : 1;	// Ограничим приоритет диапазоном (1 ; 8)
		dataToSend[CLUNET_OFFSET_SRC_ADDRESS] = CLUNET_DEVICE_ID;
		dataToSend[CLUNET_OFFSET_DST_ADDRESS] = address;
		dataToSend[CLUNET_OFFSET_COMMAND] = command;
		dataToSend[CLUNET_OFFSET_SIZE] = size;
		
		/* Копируем данные в буфер */
		uint8_t i;
		for (i = 0; i < size; i++)
			dataToSend[CLUNET_OFFSET_DATA + i] = data[i];

		/* Добавляем контрольную сумму */
		dataToSend[CLUNET_OFFSET_DATA + size] = check_crc((char*)dataToSend, CLUNET_OFFSET_DATA + size);
		
		clunetSendingDataLength = size + (CLUNET_OFFSET_DATA + 1);

		// Если линия свободна, то запланируем передачу сразу
		if (!CLUNET_READING)
			clunet_read_rst_send();
		// Иначе будем ожидать когда освободится в процедуре внешнего прерывания
		else
			clunetSendingState = CLUNET_SENDING_WAITING_LINE;

	}
}

/* Процедура внешнего прерывания по фронту и спаду сигнала */
ISR(CLUNET_INT_VECTOR)
{

	uint8_t now = CLUNET_TIMER_REG;		// Текущее значение таймера

	uint8_t down = (CLUNET_READING);

	uint8_t bitNum = 0;

	if (clunetReadingState)
	{

		uint8_t ticks = clunetTimerStart - now;

		if (ticks < CLUNET_READ1)
			bitNum = 1;
		else if (ticks < CLUNET_READ2)
			bitNum = 2;
		else if (ticks < CLUNET_READ3)
			bitNum = 3;
		else if (ticks < CLUNET_READ4)
			bitNum = 4;
		else if (ticks < CLUNET_READ5)
			bitNum = 5;
		else if (ticks < CLUNET_READ6)
			bitNum = 6;
		else if (ticks < CLUNET_READ7)
			bitNum = 7;
		else if (ticks < CLUNET_READ8)
			bitNum = 8;
		else if (ticks < CLUNET_READ9)
			bitNum = 9;
		else if (ticks < CLUNET_READ10)
			bitNum = 10;
		/* Ошибка: между синхронизациями должно быть не более 10Т */
		else
			clunetReadingState = CLUNET_READING_ERROR;

		bitNum -= clunetSyncBits;
		
		/* Ошибка: нет битстаффинга */
		if (bitNum >= 6)
			clunetReadingState = CLUNET_READING_ERROR;

	}

	/* Если линию прижало к нулю */
	if (down)
	{
		clunetTimerStart = now;	// Синхронизация времени чтения
		clunetSyncBits = 0;		// Сброс прочитанных бит после синхронизации

		/* Если мы в режиме передачи и прижали не мы, то замолкаем и ожидаем, тем более, что наши передаваемые данные уже битые */
		/* Обеспечивается быстрая отработка ошибки на линии во избежание конфликтов */
		if (clunetSendingState && !CLUNET_SENDING)
		{
			CLUNET_DISABLE_TIMER_COMP;
			clunetSendingState = CLUNET_SENDING_WAITING_LINE;
		}

		// Если в ожидании, то переходим к фазе начала приемки кадра и обнуляем счетчики
		if (clunetReadingState == CLUNET_READING_IDLE)
		{
			clunetReadingState++;
			clunetReadingByteIndex = clunetReadingBitIndex = 0;
			return;
		}

	}
	else
	{
		clunet_read_rst_send();

		clunetSyncBits = bitNum;	// Сохраняем количество прочитанных бит после синхронизации
	}

	switch (clunetReadingState)
	{
		/* Главная фаза чтения данных! */
		case CLUNET_READING_DATA:

			if (down)
				dataToRead[clunetReadingByteIndex] &= ~(255 >> clunetReadingBitIndex);
			else
				dataToRead[clunetReadingByteIndex] |= (255 >> clunetReadingBitIndex);

			clunetReadingBitIndex += (bitNum - clunetReadingBitStuff);

			if (clunetReadingBitIndex & 8)
			{

				/* Проверка на окончание чтения пакета */
				if ((++clunetReadingByteIndex > CLUNET_OFFSET_SIZE) && (clunetReadingByteIndex > dataToRead[CLUNET_OFFSET_SIZE] + CLUNET_OFFSET_DATA))
				{

					clunetReadingState = CLUNET_READING_IDLE;

					/* Проверяем CRC, при успехе начнем обработку принятого пакета */
					if (!check_crc((char*)dataToRead, clunetReadingByteIndex))
						clunet_data_received (
							dataToRead[CLUNET_OFFSET_SRC_ADDRESS],
							dataToRead[CLUNET_OFFSET_DST_ADDRESS],
							dataToRead[CLUNET_OFFSET_COMMAND],
							(char*)(dataToRead + CLUNET_OFFSET_DATA),
							dataToRead[CLUNET_OFFSET_SIZE]
						);

				}

				/* Если данные прочитаны не все и не выходим за пределы буфера, то сбросим очередной байт и подготовим битовый индекс */
				else if (clunetReadingByteIndex < CLUNET_READ_BUFFER_SIZE)
				{
					clunetReadingBitIndex &= 7;
					dataToRead[clunetReadingByteIndex] = (down) ? 0 : 0xFF;
				}

				/* Иначе - нехватка приемного буфера -> игнорируем пакет */
				else
					clunetReadingState = CLUNET_READING_ERROR;

			}

			break;

		// Фаза приемки начала кадра
		case CLUNET_READING_START:
			
			clunetReadingBitIndex += bitNum;

			// Если прочитаны биты начала кадра (1-ый стартовый, 2-5 приоритет)
			if (clunetReadingBitIndex >= 4)
			{
				clunetReadingBitIndex -= 4;		// Коррекция индекса чтения бита
				clunetReadingState++;			// К следующей фазе чтения данных
				dataToRead[0] = (down) ? 0 : 0xFF;
			}

			break;
		
		// Если в ожидании, то переходим к фазе начала приемки кадра и обнуляем счетчики
		case CLUNET_READING_IDLE:

			clunetReadingState++;
			clunetReadingByteIndex = clunetReadingBitIndex = 0;

	}

	/* Проверка на битстаффинг, учитываем в следующем цикле */
	clunetReadingBitStuff = (bitNum == 5);

}

void
clunet_init()
{
	sei();
	CLUNET_SEND_INIT;
	CLUNET_READ_INIT;
	CLUNET_TIMER_INIT;
	CLUNET_INIT_INT;
	char reset_source = MCUCSR;
	clunet_send (
		CLUNET_BROADCAST_ADDRESS,
		CLUNET_PRIORITY_MESSAGE,
		CLUNET_COMMAND_BOOT_COMPLETED,
		&reset_source,
		sizeof(reset_source)
	);
	MCUCSR = 0;
}

/* Возвращает 0, если готов к передаче, иначе приоритет текущей задачи */
uint8_t
clunet_ready_to_send()
{
	return clunetSendingState ? clunetCurrentPrio : 0;
}

void
clunet_set_on_data_received(void (*f)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size))
{
	on_data_received = f;
}

void
clunet_set_on_data_received_sniff(void (*f)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size))
{
	on_data_received_sniff = f;
}
