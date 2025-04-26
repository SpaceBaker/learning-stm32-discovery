#----------------------------------------------------------------------
#	Makefile
#	Intends to be an easily modifiable makefile template for different 
#	embedded (MCU) projects.
#
#	Note: Originally design around AVR-GCC
#
#	Version: 1.0.1
#	Author: SpaceBaker <https://github.com/SpaceBaker>
#	Date: 2023-09-04
#	Modified on 2024-03-10
#----------------------------------------------------------------------

#------------- Target -------------
TARGET	  ?= main
BIN_NAME  ?= myApp
DEVICE	  ?= STM32L475xx
# ARCH    ?= armv7e-m
CPU	   	  ?= cortex-m4
FPU		  ?= fpv4-sp-d16
FLOAT-ABI ?= hard
MCU 	   = -mcpu=$(CPU) -mthumb -mfpu=$(FPU) -mfloat-abi=$(FLOAT-ABI)
# HSE_VALUE  = 8000000 # External oscillator frequency in Hz
# MSI_VALUE  = 8000000 # Internal oscillator frequency in Hz
# HSI_VALUE  = 8000000 # Internal oscillator frequency in Hz

#------------- Working directories -------------
SRC_DIR	   = src
BUILD_DIR  = build
BIN_DIR    = $(BUILD_DIR)/bin
OBJ_DIR    = $(BUILD_DIR)/obj
# MAKE_SCRIPT_DIR = mk

#------------- Toolchain -------------
# Provide the full path if not found in your environment variables
CC_DIR  = $(HOME)/tools/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin
CC		= $(CC_DIR)/arm-none-eabi-gcc
AS		= $(CC) -x assembler-with-cpp
# AS		= $(CC_DIR)/arm-none-eabi-as
LD		= $(CC_DIR)/arm-none-eabi-ld
OBJCOPY = $(CC_DIR)/arm-none-eabi-objcopy
OBJDUMP = $(CC_DIR)/arm-none-eabi-objdump
SIZE	= $(CC_DIR)/arm-none-eabi-size
CPPCK	= $(HOME)/tools/cppcheck-2.13.0/build/bin/cppcheck
PROG	= openocd
PROG_CONFIG_FLAGS = -f board/stm32l4discovery.cfg
PROG_RUN_FLAGS 	  = verify reset exit


# List source files here
# Exemple : $(wildcard $(SRC_DIR)/bsp/driver/*.c) ...
# Exemple : $(SRC_DIR)/bsp/driver/i2c.c $(SRC_DIR)/bsp/driver/spi.c) ...
C_SRCS =	$(SRC_DIR)/myApp/$(TARGET).c \
			$(SRC_DIR)/myApp/system/irq.c \
			$(SRC_DIR)/myApp/logger.c \
			$(SRC_DIR)/common/ringbuffer.c \
			$(wildcard $(SRC_DIR)/drivers/stm32l475xx/clock/*.c) \
			$(SRC_DIR)/drivers/stm32l475xx/gpio/gpio.c \
			$(SRC_DIR)/drivers/stm32l475xx/usart/uart.c \
			$(SRC_DIR)/drivers/stm32l475xx/dma/dma.c \

AS_SRCS = 

# List header file directories here
# Exemple : $(SRC_DIR)/bsp/utils
C_INCS =	-I$(SRC_DIR) \
			-I$(SRC_DIR)/externals/CMSIS_6/CMSIS/Core/Include \
			-I$(SRC_DIR)/externals/CMSIS_6/CMSIS/Core/Include/m-profile \
			-I$(SRC_DIR)/myApp

AS_INCS =

# List library directories here
# Exemple : TODO
STD_LIB = -nostdlib
LIBS = 
LIB_DIR =

#------------- Defines -------------
C_DEFS = -D$(DEVICE)
AS_DEFS = 

#------------- Optimization Level -------------
OPT	= -Og

#------------- Warnings Options -------------
C_WARNS = -Wall -Wundef -Wextra -Werror
AS_WARNS = -Wall

#------------- Debugging -------------
DEBUG = 1
DEBUG_FORMAT = -gdwarf-2

#------------- C Language ISO Standard -------------
C_STD = -std=gnu11

#------------- cppcheck flags -------------
CPPCK_FLAGS = --quiet --error-exitcode=1 --language=c --std=c11 --suppress=unusedFunction

#------------- Linker Script -------------
LD_SCRIPT = $(SRC_DIR)/myApp/system/stm32l475xx.ld
# LD_SCRIPT = $(SRC_DIR)/drivers/stm32l475xx/startup/stm32l475vgtx_flash.ld


#---------------------------------------------------------------------------------
# DO NOT EDIT BELOW THIS LINE, UNLESS YOU KNOW WHAT YOU ARE DOING
#---------------------------------------------------------------------------------

#------------- C Compiler Flags -------------
C_FLAGS = $(MCU) $(C_STD) $(C_DEFS) $(C_INCS) $(OPT) $(C_WARNS) -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
C_FLAGS += -g $(DEBUG_FORMAT)
endif

#------------- Assembler Flags --------------
AS_FLAGS = $(MCU) $(AS_DEFS) $(AS_WARNS)

#------------- C Linker Flags -------------
#! Not done here !#
# LD_FLAGS = --script=$(LD_SCRIPT) -Map=$(BIN_DIR)/$(BIN_NAME).map --cref -nostdlib --fix-stm32l4xx-629360 #-L $(LIBS)
LD_FLAGS = $(MCU) $(STD_LIB) -T$(LD_SCRIPT) $(LIB_DIR) $(LIBS) -Wl,-Map=$(BIN_DIR)/$(BIN_NAME).map,--cref,--gc-sections,--fix-stm32l4xx-629360

#------------- Objects -------------
C_OBJS   = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(C_SRCS))
AS_OBJS  = $(patsubst $(SRC_DIR)/%.s,$(OBJ_DIR)/%.o,$(AS_SRCS))
AS_OBJS += $(patsubst $(SRC_DIR)/%.S,$(OBJ_DIR)/%.o,$(AS_SRCS))


#------------- PHONY calls -------------
.PHONY: all build elf lst hex srec flash cppcheck clean

#------------- Rules for building the elf file -------------
all: elf hex srec lst
build: elf lst
elf: $(BIN_DIR)/$(BIN_NAME).elf

# Linking
$(BIN_DIR)/$(BIN_NAME).elf: $(AS_OBJS) $(C_OBJS)
	@echo Compiling $<
	@mkdir -p $(@D)
	$(CC) $(LD_FLAGS) -o $@ $^
	@echo Size of your elf file :
	@$(SIZE) $@

# Compiling .s Assembly files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.s
	@echo Compiling $<
	@mkdir -p $(@D)
	$(AS) -c $(AS_FLAGS) $< -o $@

# Compiling .S Assembly files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.S
	@echo Compiling $<
	@mkdir -p $(@D)
	$(AS) -c $(AS_FLAGS) $< -o $@

# Compiling C files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@echo Compiling $<
	@mkdir -p $(@D)
	$(CC) -c $(C_FLAGS) $< -o $@

#------------- Rules for building the listing file -------------
lst: $(BIN_DIR)/$(BIN_NAME).lst

$(BIN_DIR)/%.lst: $(BIN_DIR)/%.elf
	@echo Listing $<
	$(OBJDUMP) -h -S $< > $@

#------------- Rules for converting to hex or srec format -------------
hex:  $(BIN_DIR)/$(BIN_NAME).hex
srec: $(BIN_DIR)/$(BIN_NAME).srec

$(BIN_DIR)/$(BIN_NAME).hex: $(BIN_DIR)/$(BIN_NAME).elf
	@echo Converting .elf to .hex
	$(OBJCOPY) -O ihex $< $@

$(BIN_DIR)/$(BIN_NAME).srec: $(BIN_DIR)/$(BIN_NAME).elf
	@echo Converting .elf to .srec
	$(OBJCOPY) -O srec $< $@

#------------- Rules for openocd -------------
flash: $(BIN_DIR)/$(BIN_NAME).elf
	$(PROG) $(PROG_CONFIG_FLAGS) -c "program $< $(PROG_RUN_FLAGS)"

#------------- Rules for cppcheck -------------
cppcheck: 
	@$(CPPCK) $(CPPCK_FLAGS) $(C_INCS) $(C_SRCS)

#------------- Other Rules -------------
clean:
	@rm -rv $(BUILD_DIR)/*
















# ------------------- NOTES -------------------------------------------------------------
# Automatic Variables
# Automatic variables are set by make after a rule is matched. There include:

# $@: the target filename.
# $*: the target filename without the file extension.
# $<: the first prerequisite filename.
# $^: the filenames of all the prerequisites, separated by spaces, discard duplicates.
# $+: similar to $^, but includes duplicates.
# $?: the names of all prerequisites that are newer than the target, separated by spaces.