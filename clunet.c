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


/* Функции обратного вызова при получении пакетов (должны быть как можно короче, так как вызываются прерываниями */
static void (*cbDataReceived)(uint8_t src_address, uint8_t command, uint8_t* data, uint8_t size) = 0;
static void (*cbDataReceivedSniff)(uint8_t src_address, uint8_t dst_address, uint8_t command, uint8_t* data, uint8_t size) = 0;

static uint8_t sendingState = CLUNET_SENDING_IDLE;
static uint8_t readingState = CLUNET_READING_IDLE;
static uint8_t sendingLength;
static uint8_t sendingPriority;

static uint8_t sendBuffer[CLUNET_SEND_BUFFER_SIZE];
static uint8_t readBuffer[CLUNET_READ_BUFFER_SIZE];

#ifdef CLUNET_DEVICE_NAME
static uint8_t devName[] = CLUNET_DEVICE_NAME;
#endif

static inline void
clunet_read_rst_send()
{
	/* Если состояние передачи неактивно либо в ожидании, то запланируем сброс чтения и при необходимости начало передачи */
	if (!(sendingState & 7))
	{
		/* подождем 7Т, сбросим статус чтения, потом, при необходимости, через 1Т запустим передачу в прерывании сравнения */
		CLUNET_TIMER_REG_OCR = CLUNET_TIMER_REG + (7*CLUNET_T);
		CLUNET_ENABLE_TIMER_COMP;	// Включаем прерывание сравнения таймера
	}
}

static char
check_crc(const uint8_t* data, const uint8_t size)
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
clunet_data_received(const uint8_t src_address, const uint8_t dst_address, const uint8_t command, uint8_t* data, const uint8_t size)
{
	if (cbDataReceivedSniff)
		(*cbDataReceivedSniff)(src_address, dst_address, command, data, size);

	if ((src_address != CLUNET_DEVICE_ID) && ((dst_address == CLUNET_DEVICE_ID) || (dst_address == CLUNET_BROADCAST_ADDRESS)))
	{
		// Команда перезагрузки
		if (command == CLUNET_COMMAND_REBOOT)
		{
			cli();
			set_bit(WDTCR, WDE);
			while(1);
		}

		if ((sendingState == CLUNET_SENDING_IDLE) || (sendingPriority <= CLUNET_PRIORITY_MESSAGE))
		{
			switch (command)
			{
			/* Ответ на поиск устройств */
			case CLUNET_COMMAND_DISCOVERY:

				#ifdef CLUNET_DEVICE_NAME
				clunet_send(src_address, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DISCOVERY_RESPONSE, devName, sizeof(devName));
				#else
				clunet_send(src_address, CLUNET_PRIORITY_MESSAGE, CLUNET_COMMAND_DISCOVERY_RESPONSE, 0, 0);
				#endif
				break;

			/* Ответ на пинг */
			case CLUNET_COMMAND_PING:

				clunet_send(src_address, CLUNET_PRIORITY_COMMAND, CLUNET_COMMAND_PING_REPLY, data, size);
				break;
				
			default:

				if (cbDataReceived)
					(*cbDataReceived)(src_address, command, data, size);
				
			}
		}
	}
}

/* Процедура прерывания сравнения таймера */
ISR(CLUNET_TIMER_COMP_VECTOR)
{

	static uint8_t bitIndex, byteIndex, bitStuff;

	// Если передатчик освободился, сбросим статус чтения и пока сюда не планируем возвращаться
	if (sendingState == CLUNET_SENDING_IDLE)
	{
		readingState = CLUNET_READING_IDLE;
		CLUNET_DISABLE_TIMER_COMP;
	}

	/* А если мы в ожидании освобождения линии, то начнем передачу через 1Т, предварительно сбросив статус чтения */
	else if (sendingState == CLUNET_SENDING_WAITING_LINE)
	{
		readingState = CLUNET_READING_IDLE;
		sendingState = CLUNET_SENDING_INIT;
		CLUNET_TIMER_REG_OCR += CLUNET_T;
		return;
	}

	/* Проверим на конфликт */
	if (!CLUNET_SENDING && CLUNET_READING)
	{
		CLUNET_DISABLE_TIMER_COMP;					// Выключаем прерывание сравнения таймера
		sendingState = CLUNET_SENDING_WAITING_LINE;		// Переходим в режим ожидания линии
	}

	/* Все в порядке, можем продолжать */
	else
	{
		/* Переменная количества передаваемых бит (сколько периодов Т задержка при следующем вызове прерывания) */
		register uint8_t numBits = 0;

		CLUNET_SEND_INVERT;	// Инвертируем значение сигнала
	
		/* Смотрим фазу передачи */
		switch (sendingState)
		{
			// Главная фаза передачи данных!
			case CLUNET_SENDING_DATA:
			
				numBits = bitStuff;

				bitStuff = 0;

				uint8_t xBitMask = (CLUNET_SENDING) ? 0 : 0x80;
				
				/* Если мы прижали линию, то ищем единичные биты, иначе - нулевые */
				while ((0x80 & (sendBuffer[byteIndex] << bitIndex)) ^ xBitMask)
				{
				
					/* Если передан байт данных */
					if (++bitIndex & 8)
					{

						/* Если не все данные отосланы */
						if (++byteIndex < sendingLength)
							bitIndex = 0;		// начинаем передачу следующего байта с бита 0

						/* Иначе передача всех данных закончена */
						else
						{
							sendingState = CLUNET_SENDING_STOP;
							numBits++;
							break;
						}

					}

					/* Если будем отправлять 5 одноименных бит, то запланируем битстаффинг в следующей передаче */
					if (++numBits == 5)
					{
						bitStuff = 1;
						break;
					}
					
				}
				
				break;

			// Фаза отправки стартового бита, а также битов приоритета, старшего бита данных при условии их равенства единице
			case CLUNET_SENDING_INIT:

				bitStuff = byteIndex = bitIndex = 0;
				
				uint8_t prio = sendingPriority - 1;

				/* Считаем сколько бит в приоритете единичных, учитывая стоповый */
				while(prio & (4 >> numBits++));

				/* Если отправлены все биты приоритета, смотрим старший бит данных, если 1, то отправим и установим битстаффинг в следующей передаче */
				if(numBits & 4)
				{
					sendingState = CLUNET_SENDING_DATA;	// Перепрыгнем стадию отправки приоритета
					if (sendBuffer[0] & 0x80)
					{
						numBits++;
						bitIndex = bitStuff = 1;
					}
				}
				// Если приоритет не наивысший, то будем передавать в специальной стадии
				else
				{
					sendingState = CLUNET_SENDING_PRIO;	// К следующей фазе передачи
					bitIndex = numBits;		// Запомним на каком бите приоритета остановились
				}

				break;

			// Стадия отправки приоритета и начальных битов данных
			case CLUNET_SENDING_PRIO:

				/* Создадим маску XOR в зависимости от состояния линии */
				xBitMask = (CLUNET_SENDING) ? 0 : 8;

				prio = sendingPriority - 1;

				// Если линия прижата, то ищем единичные биты, иначе - нулевые
				// Хотябы один раз мы должны попасть внутрь цикла.
				while ((8 & (prio << bitIndex)) ^ xBitMask)
				{
					numBits++;	// Увеличим отправляемые биты

					// Если отправлены все биты приоритета
					if(++bitIndex & 4)
					{
						xBitMask <<= 4;
						bitIndex = 0;
						sendingState = CLUNET_SENDING_DATA;
	
						// Если линия прижата, то ищем нулевые биты
						while((0x80 & (sendBuffer[0] << bitIndex)) ^ xBitMask)
						{
							bitIndex++;
							// Если набрали 5 одинаковых бит - делаем битстаффинг
							if (++numBits == 5)
							{
								bitStuff = 1;
								break;
							}
						}
						break;
					}
				}

				break;

			case CLUNET_SENDING_STOP:
				
				// Если линия была прижата (последний бит 1), то просто отпускаем, во внешнем прерывании запланируются необходимые действия
				if (!CLUNET_SENDING)
				{
					CLUNET_DISABLE_TIMER_COMP;
					sendingState = CLUNET_SENDING_IDLE;
					return;
				}
				// Иначе если была отпущена (последний бит 0), то сделаем короткий импульс 1Т
				else
					numBits++;

		}

		CLUNET_TIMER_REG_OCR += CLUNET_T * numBits;

	}

}


/* Процедура внешнего прерывания по фронту и спаду сигнала */
ISR(CLUNET_INT_VECTOR)
{

	static uint8_t bitIndex, byteIndex, bitStuff, tickSync, bitNumSync;

	uint8_t now = CLUNET_TIMER_REG;		// Текущее значение таймера

	// Линия освобождена
	uint8_t lineFree = CLUNET_READING ? 0x00 : 0xFF;

	uint8_t bitNum = 0;

	if (readingState)
	{

		uint8_t ticks = tickSync - now;
		uint8_t period;

		// Цикл подсчета количества бит с момента последней синхронизации по спаду
		for (period = CLUNET_READ1, bitNum = 1 ; ticks >= period ; period += CLUNET_T)
		{
			if(++bitNum > 10)
			{
				readingState = CLUNET_READING_ERROR;
				break;
			}
		}

		bitNum -= bitNumSync;
		
		/* Ошибка: нет битстаффинга */
		if (bitNum >= 6)
			readingState = CLUNET_READING_ERROR;

	}

	/* Если линию освободило (пришли единицы) */
	if (lineFree)
	{
		clunet_read_rst_send();
		bitNumSync = bitNum;	// Сохраняем количество прочитанных бит после синхронизации
	}
	/* Если линию прижало к нулю (пришли нули или начало пакета) */
	else
	{
		tickSync = now;		// Синхронизация времени чтения
		bitNumSync = 0;		// Сброс прочитанных бит после синхронизации

		/* Проверка на конфликт передачи */
		if ((sendingState & 7) && !CLUNET_SENDING)
		{
			CLUNET_DISABLE_TIMER_COMP;
			sendingState = CLUNET_SENDING_WAITING_LINE;
		}

		// Если в ожидании приема пакета, то переходим к фазе начала приемки кадра, обнуляем счетчики и выходим
		if (!readingState)
		{
			readingState++;
			bitStuff = byteIndex = bitIndex = 0;
			return;
		}
	}

	switch (readingState)
	{
		/* Главная фаза чтения данных! */
		case CLUNET_READING_DATA:

			/* Если линия освободилась, значит была единичная посылка - установим соответствующие биты */
			if (lineFree)
				readBuffer[byteIndex] |= (255 >> bitIndex);

			/* Если линия прижалась, значит была нулевая посылка - сбросим соответствующие биты */
			else
				readBuffer[byteIndex] &= ~(255 >> bitIndex);

			// Обновим битовый индекс с учетом битстаффинга
			bitIndex += (bitNum - bitStuff);

			if (bitIndex & 8)
			{
				/* Если пакет прочитан полностью, то проверим контрольную сумму */
				if ((++byteIndex > CLUNET_OFFSET_SIZE) && (byteIndex > readBuffer[CLUNET_OFFSET_SIZE] + CLUNET_OFFSET_DATA))
				{
					readingState = CLUNET_READING_IDLE;
					/* Проверяем CRC, при успехе начнем обработку принятого пакета */
					if (!check_crc(readBuffer, byteIndex))
						clunet_data_received (
							readBuffer[CLUNET_OFFSET_SRC_ADDRESS],
							readBuffer[CLUNET_OFFSET_DST_ADDRESS],
							readBuffer[CLUNET_OFFSET_COMMAND],
							readBuffer + CLUNET_OFFSET_DATA,
							readBuffer[CLUNET_OFFSET_SIZE]
						);

				}

				/* Если данные прочитаны не полностью и мы не выходим за пределы буфера, то присвоим очередной байт и подготовим битовый индекс */
				else if (byteIndex < CLUNET_READ_BUFFER_SIZE)
				{
					bitIndex &= 7;
					readBuffer[byteIndex] = lineFree;
				}

				/* Иначе ошибка: нехватка приемного буфера -> игнорируем пакет */
				else
					readingState = CLUNET_READING_ERROR;

			}

			break;

		// Фаза приемки начала кадра
		case CLUNET_READING_START:
			
			bitIndex += bitNum;

			// Если прочитаны биты начала кадра (1-ый стартовый, 2-4 приоритет)
			if (bitIndex >= 4)
			{
				readBuffer[0] = lineFree;
				bitIndex -= 4;					// Коррекция индекса чтения бита
				readingState++;			// К следующей фазе чтения данных
			}

	}

	/* Проверка на битстаффинг, учитываем в следующем цикле */
	bitStuff = (bitNum == 5);

}

void
clunet_init()
{
	sei();
	CLUNET_SEND_INIT;
	CLUNET_READ_INIT;
	CLUNET_TIMER_INIT;
	CLUNET_INIT_INT;
	uint8_t reset_source = MCUCSR;
	clunet_send (
		CLUNET_BROADCAST_ADDRESS,
		CLUNET_PRIORITY_MESSAGE,
		CLUNET_COMMAND_BOOT_COMPLETED,
		&reset_source,
		sizeof(reset_source)
	);
	MCUCSR = 0;
}

void
clunet_send(const uint8_t address, const uint8_t prio, const uint8_t command, const uint8_t* data, const uint8_t size)
{
	/* Если размер данных в пределах буфера передачи (максимально для протокола 250 байт) */
	if (size < (CLUNET_SEND_BUFFER_SIZE - CLUNET_OFFSET_DATA))
	{
		CLUNET_DISABLE_TIMER_COMP;

		/* Заполняем переменные */
		sendingPriority = (prio > 8) ? 8 : prio ? : 1;
		sendBuffer[CLUNET_OFFSET_SRC_ADDRESS] = CLUNET_DEVICE_ID;
		sendBuffer[CLUNET_OFFSET_DST_ADDRESS] = address;
		sendBuffer[CLUNET_OFFSET_COMMAND] = command;
		sendBuffer[CLUNET_OFFSET_SIZE] = size;
		
		if (data && size)
		{
			/* Копируем данные в буфер */
			uint8_t idx = 0;
			do
				sendBuffer[CLUNET_OFFSET_DATA + idx] = data[idx];
			while(++idx < size);
		}

		/* Добавляем контрольную сумму */
		sendBuffer[CLUNET_OFFSET_DATA + size] = check_crc(sendBuffer, CLUNET_OFFSET_DATA + size);
		
		sendingLength = size + (CLUNET_OFFSET_DATA + 1);

		sendingState = CLUNET_SENDING_WAITING_LINE;	// Ждем линию

		// Если линия прижата, то отожмем и процедура внешнего прерывания запланирует отправку сама
		if (CLUNET_READING)
			CLUNET_SEND_0;
		// Если свободна, то запланируем отправку сами
		else
		{
			CLUNET_TIMER_REG_OCR = CLUNET_TIMER_REG + (7*CLUNET_T);
			CLUNET_ENABLE_TIMER_COMP;
		}
	}
}

/* Возвращает 0, если готов к передаче, иначе приоритет текущей задачи */
uint8_t
clunet_ready_to_send()
{
	return sendingState ? sendingPriority : 0;
}

void
clunet_set_on_data_received(void (*f)(uint8_t src_address, uint8_t command, uint8_t* data, uint8_t size))
{
	cbDataReceived = f;
}

void
clunet_set_on_data_received_sniff(void (*f)(uint8_t src_address, uint8_t dst_address, uint8_t command, uint8_t* data, uint8_t size))
{
	cbDataReceivedSniff = f;
}
