export INF_ARM_TOOL_CHAIN = /opt/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf
export BINPREFIX = $(INF_ARM_TOOL_CHAIN)/bin/arm-linux-gnueabihf-
CC		= $(BINPREFIX)gcc
CPP		= $(BINPREFIX)g++
ASM		= $(BINPREFIX)as
AR		= $(BINPREFIX)ar

main: 
	$(CC) *.c -o flash
clean:
	rm -f flash *.o
