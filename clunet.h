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

#ifndef __CLUNET_H__
#define __CLUNET_H__

#include <stdint.h>
#include "clunet_config.h"

#define CLUNET_OFFSET_SRC_ADDRESS 0
#define CLUNET_OFFSET_DST_ADDRESS 1
#define CLUNET_OFFSET_COMMAND 2
#define CLUNET_OFFSET_SIZE 3
#define CLUNET_OFFSET_DATA 4
#define CLUNET_BROADCAST_ADDRESS 255

#define CLUNET_COMMAND_DISCOVERY 0
/* Поиск других устройств, параметров нет */

#define CLUNET_COMMAND_DISCOVERY_RESPONSE 0x01
/* Ответ устройств на поиск, в качестве параметра - название устройства (текст) */

#define CLUNET_COMMAND_BOOT_CONTROL 0x02
/* Работа с загрузчиком. Данные - субкоманда.
<-0 - загрузчик запущен
->1 - перейти в режим обновления прошивки
<-2 - подтверждение перехода, плюс два байта - размер страницы
->3 запись прошивки, 4 байта - адрес, всё остальное - данные (равные размеру страницы)
<-4 блок прошивки записан
->5 выход из режима прошивки */

#define CLUNET_COMMAND_REBOOT 0x03
/* Перезагружает устройство в загрузчик. */

#define CLUNET_COMMAND_BOOT_COMPLETED 0x04
/* Посылается устройством после инициализации библиотеки, сообщает об успешной загрузке устройства. Параметр - содержимое MCU регистра, говорящее о причине перезагрузки. */

#define CLUNET_COMMAND_PING 0xFE
/* Пинг, на эту команду устройство должно ответить следующей командой, возвратив весь буфер */

#define CLUNET_COMMAND_PING_REPLY 0xFF
/* Ответ на пинг, в данных то, что было прислано в предыдущей команде */

#define CLUNET_PRIORITY_NOTICE 1
/* Приоритет пакета 1 - неважное уведомление, которое вообще может быть потеряно без последствий */

#define CLUNET_PRIORITY_INFO 2
/* Приоритет пакета 2 - какая-то информация, не очень важная */

#define CLUNET_PRIORITY_MESSAGE 3
/* Приоритет пакета 3 - сообщение с какой-то важной информацией */

#define CLUNET_PRIORITY_COMMAND 4
/* Приоритет пакета 4 - команда, на которую нужно сразу отреагировать */

#ifndef CLUNET_T
#define CLUNET_T ((F_CPU / CLUNET_TIMER_PRESCALER) / 15625)
#endif
#if CLUNET_T < 8
#  error Timer frequency is too small, increase CPU frequency or decrease timer prescaler
#endif
#if CLUNET_T > 24
#  error Timer frequency is too big, decrease CPU frequency or increase timer prescaler
#endif

#define CLUNET_CONCAT(a, b)            a ## b
#define CLUNET_OUTPORT(name)           CLUNET_CONCAT(PORT, name)
#define CLUNET_INPORT(name)            CLUNET_CONCAT(PIN, name)
#define CLUNET_DDRPORT(name)           CLUNET_CONCAT(DDR, name)

#define CLUNET_SEND_1   CLUNET_OUTPORT(CLUNET_WRITE_PORT) |= (1 << CLUNET_WRITE_PIN)
#define CLUNET_SEND_0   CLUNET_OUTPORT(CLUNET_WRITE_PORT) &= ~(1 << CLUNET_WRITE_PIN)
//#define CLUNET_SEND_INVERT  CLUNET_DDRPORT(CLUNET_PORT) ^= (1 << CLUNET_PIN)
#define CLUNET_SENDING  (CLUNET_OUTPORT(CLUNET_WRITE_PORT) & (1 << CLUNET_WRITE_PIN))
#define CLUNET_READING  (!(CLUNET_INPORT(CLUNET_READ_PORT) & (1 << CLUNET_READ_PIN)))
#define CLUNET_PIN_INIT { CLUNET_DDRPORT(CLUNET_WRITE_PORT) |= (1 << CLUNET_WRITE_PIN); CLUNET_SEND_0; CLUNET_DDRPORT(CLUNET_READ_PORT) &= ~(1 << CLUNET_READ_PIN); CLUNET_OUTPORT(CLUNET_READ_PORT) |= (1 << CLUNET_READ_PIN); }

#ifndef CLUNET_SEND_BUFFER_SIZE
#  error CLUNET_SEND_BUFFER_SIZE is not defined
#endif
#ifndef CLUNET_READ_BUFFER_SIZE
#  error CLUNET_READ_BUFFER_SIZE is not defined
#endif
#if CLUNET_SEND_BUFFER_SIZE > 255
#  error CLUNET_SEND_BUFFER_SIZE must be <= 255
#endif
#if CLUNET_READ_BUFFER_SIZE > 255
#  error CLUNET_READ_BUFFER_SIZE must be <= 255
#endif

// Инициализация
void clunet_init(void);

// Возвращает 0, если готов к передаче, иначе приоритет текущей задачи
uint8_t clunet_ready_to_send(void);

// Resend last sended packet
void clunet_resend_last_packet(void);

// Abort current sending
void clunet_abort_send(void);

// Отправка пакета
void clunet_send(const uint8_t address, const uint8_t prio, const uint8_t command, const char* data, const uint8_t size);

// Установка функций, которые вызываются при получении пакетов
// Эта - получает пакеты, которые адресованы нам
void clunet_set_on_data_received(void (*f)(uint8_t src_address, uint8_t command, char* data, uint8_t size));

// А эта - абсолютно все, которые ходят по сети, включая наши
void clunet_set_on_data_received_sniff(void (*f)(uint8_t src_address, uint8_t dst_address, uint8_t command, char* data, uint8_t size));

#endif
