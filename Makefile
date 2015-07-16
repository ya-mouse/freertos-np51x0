#/*
#    FreeRTOS V7.4.2 - Copyright (C) 2013 Real Time Engineers Ltd.
#	
#
#    ***************************************************************************
#     *                                                                       *
#     *    FreeRTOS tutorial books are available in pdf and paperback.        *
#     *    Complete, revised, and edited pdf reference manuals are also       *
#     *    available.                                                         *
#     *                                                                       *
#     *    Purchasing FreeRTOS documentation will not only help you, by       *
#     *    ensuring you get running as quickly as possible and with an        *
#     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
#     *    the FreeRTOS project to continue with its mission of providing     *
#     *    professional grade, cross platform, de facto standard solutions    *
#     *    for microcontrollers - completely free of charge!                  *
#     *                                                                       *
#     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
#     *                                                                       *
#     *    Thank you for using FreeRTOS, and thank you for your support!      *
#     *                                                                       *
#    ***************************************************************************
#
#
#    This file is part of the FreeRTOS distribution.
#
#    FreeRTOS is free software; you can redistribute it and/or modify it under
#    the terms of the GNU General Public License (version 2) as published by the
#    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
#    >>>NOTE<<< The modification to the GPL is included to allow you to
#    distribute a combined work that includes FreeRTOS without being obliged to
#    provide the source code for proprietary components outside of the FreeRTOS
#    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
#    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
#    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#    more details. You should have received a copy of the GNU General Public
#    License and the FreeRTOS license exception along with FreeRTOS; if not it
#    can be viewed here: http://www.freertos.org/a00114.html and also obtained
#    by writing to Richard Barry, contact details for whom are available on the
#    FreeRTOS WEB site.
#
#    1 tab == 4 spaces!
#
#    http://www.FreeRTOS.org - Documentation, latest information, license and
#    contact details.
#
#    http://www.SafeRTOS.com - A version that is certified for use in safety
#    critical systems.
#
#    http://www.OpenRTOS.com - Commercial support, development, porting,
#    licensing and training services.
#*/
BOARD_NAME=A320
RTOS_SOURCE_DIR=Source
DEMO_SOURCE_DIR=Demo/$(BOARD_NAME)
DEMO_LIB_SOURCE_DIR=Demo/libc
DEMO_COMMON_SOURCE_DIR=Demo/Common
LWIP_SOURCE_DIR=lwip-1.4.0

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
ARCH=arm-none-eabi-ar
CRT0=$(DEMO_SOURCE_DIR)/boot.s
WARNINGS=-Wall -Wextra -Wshadow -Wpointer-arith -Wbad-function-cast -Wcast-align -Wsign-compare \
		-Waggregate-return -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations -Wunused

#
# CFLAGS common to both the THUMB and ARM mode builds
#
CFLAGS=$(WARNINGS) -D $(RUN_MODE) -D $(BOARD_NAME) -I$(DEMO_SOURCE_DIR) -I$(RTOS_SOURCE_DIR)/include \
		-I$(DEMO_COMMON_SOURCE_DIR)/include -I$(LWIP_SOURCE_DIR)/src/include -I$(LWIP_SOURCE_DIR)/src/include/ipv4 \
		-I$(LWIP_SOURCE_DIR)/$(BOARD_NAME)/include \
		 $(DEBUG) -mno-thumb-interwork -march=armv4 -mtune=fa526 -msoft-float -Uarm -T$(LDSCRIPT) $(OPTIM) -fomit-frame-pointer -fno-strict-aliasing -fno-dwarf2-cfi-asm

ifeq ($(USE_THUMB_MODE),YES)
	CFLAGS += -mthumb-interwork -D THUMB_INTERWORK
	THUMB_FLAGS=-mthumb
endif


LINKER_FLAGS= -Xlinker -ortosdemo.elf -Xlinker -M -Xlinker -Map=rtosdemo.map

#
# Source files that can be built to THUMB mode.
#
THUMB_SRC = \
$(addprefix $(DEMO_SOURCE_DIR)/, main.c serial/serial.c ParTest/ParTest.c) \
$(addprefix $(DEMO_COMMON_SOURCE_DIR)/Minimal/, integer.c flash.c PollQ.c flop.c semtest.c dynamic.c BlockQ.c) \
$(addprefix $(RTOS_SOURCE_DIR)/, tasks.c queue.c list.c) \
$(addprefix $(RTOS_SOURCE_DIR)/MemMang/, heap_2.c) \
$(addprefix $(RTOS_SOURCE_DIR)/$(BOARD_NAME)/, port.c)

#
# Source files that must be built to ARM mode.
#
#lwip/src/core/: dhcp.c  dns.c  ipv6
#lwip/src/core/ipv4: autoip.c   igmp.c  inet.c
ARM_SRC = \
$(RTOS_SOURCE_DIR)/$(BOARD_NAME)/portISR.c \
$(DEMO_SOURCE_DIR)/serial/serialISR.c \
$(addprefix $(LWIP_SOURCE_DIR)/src/core/, init.c tcp_out.c mem.c memp.c netif.c pbuf.c raw.c \
					  stats.c sys.c tcp.c tcp_in.c udp.c def.c lwip_timers.c) \
$(addprefix $(LWIP_SOURCE_DIR)/src/core/ipv4/, inet.c ip.c ip_addr.c icmp.c ip_frag.c inet_chksum.c) \
$(addprefix $(LWIP_SOURCE_DIR)/src/api/, tcpip.c api_msg.c err.c api_lib.c netbuf.c netdb.c netifapi.c sockets.c) \
$(addprefix $(LWIP_SOURCE_DIR)/src/netif/, etharp.c) \
$(addprefix $(LWIP_SOURCE_DIR)/A320/, sys_arch.c ftmac100.c) \
apps/init.c \
$(DEMO_LIB_SOURCE_DIR)/stdlib/div64.c \
$(DEMO_LIB_SOURCE_DIR)/stdio/simple_printf.c


#
# Define all object files.
#
ARM_OBJ = $(ARM_SRC:.c=.o)
THUMB_OBJ = $(THUMB_SRC:.c=.o)

rtosdemo.hex : rtosdemo.elf
	$(OBJCOPY) rtosdemo.elf -O ihex rtosdemo.hex
	$(OBJDUMP) -D rtosdemo.elf > rtosdemo.dump

rtosdemo.elf : $(ARM_OBJ) $(THUMB_OBJ) $(CRT0) Makefile
	$(CC) -v $(CFLAGS) $(ARM_OBJ) $(THUMB_OBJ) -nostartfiles $(CRT0) $(LINKER_FLAGS)

$(THUMB_OBJ) : %.o : %.c $(LDSCRIPT) Makefile
	$(CC) -c $(THUMB_FLAGS) $(CFLAGS) $< -o $@

$(ARM_OBJ) : %.o : %.c $(LDSCRIPT) Makefile
	$(CC) -c $(CFLAGS) $< -o $@

clean :
	rm -rf $(ARM_OBJ) $(THUMB_OBJ)
	touch Makefile









	


