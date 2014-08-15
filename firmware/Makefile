
H		= .
M		= .
S		= $(H)

vpath %.c $(M):$(S)

ifndef GCCPREFIX
    GCCPREFIX   = /usr/local/pic32-tools/bin/pic32-
endif

DEFS		+= -I. -I$(H)

LDRFILE		= bare-metal_32MX150F128B.ld 


DEPFLAGS	= -MT $@ -MD -MP -MF .deps/$*.dep
CFLAGS		= -O $(DEFS) $(DEPFLAGS)
ASFLAGS		= $(DEFS) $(DEPFLAGS)

DEVICE	= 32MX150F128B

AVRDUDE = ../mpide-0023-linux-20120903/hardware/tools/avrdude
AVRDUDECONF = -C ../mpide-0023-linux-20120903/hardware/tools/avrdude.conf -c stk500v2 \
		-p pic32 -P /dev/ttyUSB0 -q -q

PIC32PROG = /usr/local/bin/pic32prog
PIC32PROGCONF = 

CC		= $(GCCPREFIX)gcc -EL -g -mips32r2 -mprocessor=$(DEVICE) -O3
CC		+= -Wextra -Wall -fno-exceptions -ffunction-sections
CC		+= -fdata-sections -mno-smart-io 

LDFLAGS         += -T $(LDRFILE) -lc -lm -Wl,-Map=picnc.map,--gc-sections

SIZE		= $(GCCPREFIX)size
OBJDUMP		= $(GCCPREFIX)objdump
OBJCOPY		= $(GCCPREFIX)objcopy
BIN2HEX		= $(GCCPREFIX)bin2hex

SRCOBJ	= main.o stepgen.o

.SUFFIXES:

.SUFFIXES:	.hex .dis .c .elf .o

all:		.deps picnc.elf picnc.hex
		$(SIZE) picnc.elf
clean:
		rm -rf .deps *.o *.elf *.bin *.dis *.map *.hex *.dep

.deps:
		mkdir .deps


picnc.elf:	$(SRCOBJ) $(LDRFILE)
		$(CC) $(LDFLAGS) $(SRCOBJ) -o $@
		chmod -x $@
		$(OBJDUMP) -d -S $@ > picnc.dis
		$(BIN2HEX) $@

load:           .deps picnc.elf
		$(PIC32PROG) $(PIC32PROGCONF) picnc.hex

.o.dis:
		$(OBJDUMP) -d -z -S $< > $@

ifeq (.deps, $(wildcard .deps))
-include .deps/*.dep
endif
