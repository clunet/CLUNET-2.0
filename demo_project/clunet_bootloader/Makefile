# AVRDUDE config options for 'program' target
LFUSE          = E4
HFUSE          = DA
PROGRAMMER_MCU = m8
PROGRAMMER_TYPE = usbasp
PROGRAMMER_PORT = usb

# MCU
MCU_TARGET     = atmega8

# Bootloader starting address (absolute, datasheet * 2)
BOOTSTART = 0x1C00

# Bootloader section size (words)
BOOTSIZE = 512

# Timeout of waiting packet (miliseconds)
BOOTLOADER_TIMEOUT = 1000UL

# Main frequency
F_CPU = 16000000UL

# Main project path (we need 'clunet_config.h')
PROJECT_PATH		 = ..

# CLUNET library path (we need 'clunet.h')
CLUNET_PATH		 = ../..

# GCC optimize level
OPTIMIZE = s

PRG            = clunet_bootloader
OBJ            = clunet_bootloader.o

DEFS           = -DBOOTSIZE=$(BOOTSIZE) -DBOOTLOADER_TIMEOUT=$(BOOTLOADER_TIMEOUT) -DF_CPU=$(F_CPU)
LIBS           =

# You should not have to change anything below here.

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall -Wextra -O$(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -I$(PROJECT_PATH) -I$(CLUNET_PATH)
LDFLAGS       = -Wl,-Map,$(PRG).map
#override LDFLAGS       =  -Wl,--section-start=.text=$(BOOTSTART)
LDFLAGS += -Wl,--section-start=.text=$(BOOTSTART)

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# dependency:
clunet_bootloader.o: clunet_bootloader.c $(CLUNET_PATH)/clunet.h $(PROJECT_PATH)/clunet_config.h
clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak 
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }

# Every thing below here is used by avr-libc's build system and can be ignored
# by the casual user.

FIG2DEV                 = fig2dev
EXTRA_CLEAN_FILES       = *.hex *.bin *.srec

dox: eps png pdf

eps: $(PRG).eps
png: $(PRG).png
pdf: $(PRG).pdf

%.eps: %.fig
	$(FIG2DEV) -L eps $< $@

%.pdf: %.fig
	$(FIG2DEV) -L pdf $< $@

%.png: %.fig
	$(FIG2DEV) -L png $< $@

program: hex
	avrdude -V -p $(PROGRAMMER_MCU) -c $(PROGRAMMER_TYPE) -P $(PROGRAMMER_PORT) -U flash:w:$(PRG).hex -U lfuse:w:0x$(LFUSE):m -U hfuse:w:0x$(HFUSE):m
