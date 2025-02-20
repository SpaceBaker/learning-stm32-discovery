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
TARGET = main
MCU    = STM32L475xx
ARCH   = armv7e-m
CPU	   = cortex-m4
ABI    = hard
FPU	   = vfpv4
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
AS		= $(CC_DIR)/arm-none-eabi-as
CC		= $(CC_DIR)/arm-none-eabi-gcc
LD		= $(CC_DIR)/arm-none-eabi-ld
OBJCOPY = $(CC_DIR)/arm-none-eabi-objcopy
OBJDUMP = $(CC_DIR)/arm-none-eabi-objdump
SIZE	= $(CC_DIR)/arm-none-eabi-size
CPPCK	= $(HOME)/tools/cppcheck-2.13.0/build/bin/cppcheck
PROG	= openocd
PROG_CONFIG_FLAGS = -f interface/stlink.cfg -f board/stm32l4discovery.cfg
PROG_RUN_FLAGS 	  = verify reset exit


# List source files here
# Exemple : $(wildcard $(SRC_DIR)/bsp/driver/*.c) ...
# Exemple : $(SRC_DIR)/bsp/driver/i2c.c $(SRC_DIR)/bsp/driver/spi.c) ...
SRCS =	$(SRC_DIR)/myApp/$(TARGET).c \
		$(wildcard $(SRC_DIR)/bsp/*.c) \
		$(wildcard $(SRC_DIR)/common/*.c) \
		$(wildcard $(SRC_DIR)/drivers/*.c) \
		$(wildcard $(SRC_DIR)/drivers/hts221/*.c) \
		$(wildcard $(SRC_DIR)/drivers/ism43362_m3g_l44/*.c) \
		$(wildcard $(SRC_DIR)/drivers/lis3mdl/*.c) \
		$(wildcard $(SRC_DIR)/drivers/lps22hb/*.c) \
		$(wildcard $(SRC_DIR)/drivers/lsm6dsl/*.c) \
		$(wildcard $(SRC_DIR)/drivers/m24sr/*.c) \
		$(wildcard $(SRC_DIR)/drivers/mp34dt01/*.c) \
		$(wildcard $(SRC_DIR)/drivers/mp34dt01mx25r6435f/*.c) \
		$(wildcard $(SRC_DIR)/drivers/spbtle_rf/*.c) \
		$(wildcard $(SRC_DIR)/drivers/spsgrf/*.c) \
		$(wildcard $(SRC_DIR)/drivers/stsafe_a100/*.c) \
		$(wildcard $(SRC_DIR)/drivers/stsafe_a100vl53l0x/*.c) \
		$(wildcard $(SRC_DIR)/drivers/vl53l0x/*.c) \
		$(wildcard $(SRC_DIR)/drivers/stm32l475xx/*.c) \
		$(wildcard $(SRC_DIR)/drivers/stm32l475xx/*/*.c) \
		$(SRC_DIR)/externals/cmsis_device_l4/Source/Templates/system_stm32l4xx.c
		
# AS_SRCS = $(SRC_DIR)/externals/cmsis_device_l4/Source/Templates/gcc/startup_stm32l475xx.s

# List header file directories here
# Exemple : $(SRC_DIR)/bsp/utils
INCS =	$(SRC_DIR) \
		$(SRC_DIR)/externals/CMSIS_6/CMSIS/Core/Include \
		$(SRC_DIR)/externals/CMSIS_6/CMSIS/Core/Include/m-profile \
		$(SRC_DIR)/externals/cmsis_device_l4/Include \
		$(SRC_DIR)/myApp \
		$(SRC_DIR)/common \
		$(SRC_DIR)/drivers \
		$(SRC_DIR)/drivers/stm32l475xx \
		$(SRC_DIR)/drivers/stm32l475xx/startup \
		$(SRC_DIR)/drivers/stm32l475xx/clock \

# List library directories here
# Exemple : TODO
# LIBS = 

#------------- Defines -------------
DEFS = -D$(MCU)

#------------- Optimization Level -------------
OPT	= -Og

#------------- Warnings Options -------------
WARNS = -Wall -Wundef -Wextra -Werror

#------------- Debugging Format -------------
DEBUG = -g

#------------- C Language ISO Standard -------------
CSTD = -std=gnu11

#------------- cppcheck flags -------------
CPPCK_FLAGS = --quiet --error-exitcode=1 --language=c --std=c11 --suppress=unusedFunction

#------------- Linker Script -------------
LD_SCRIPT = $(SRC_DIR)/drivers/stm32l475xx/startup/stm32l475xx.ld


#---------------------------------------------------------------------------------
# DO NOT EDIT BELOW THIS LINE, UNLESS YOU KNOW WHAT YOU ARE DOING
#---------------------------------------------------------------------------------

#------------- C Compiler Flags -------------
CFLAGS = 	-mcpu=$(CPU) -mfloat-abi=$(ABI) -mfpu=$(FPU) -mthumb $(OPT) -ffunction-sections \
			-fdata-sections $(DEBUG) $(WARNS) $(addprefix -I, $(INCS)) $(CSTD) $(DEFS) -T$(LD_SCRIPT) \
			-Wl,-Map=$(BIN_DIR)/$(TARGET).map -nostdlib #--specs=debug-nano.specs --specs=debug-nosys.specs 

#------------- Assembler Flags --------------
ASFLAGS =	-mcpu=$(CPU) -c -x assembler-with-cpp -mfloat-abi=$(ABI) -mfpu=$(FPU) -mthumb

#------------- C Linker Flags -------------
#! Not done here !#
LDFLAGS = --script=$(LD_SCRIPT) -Map=$(BIN_DIR)/$(TARGET).map --cref -nostdlib --fix-stm32l4xx-629360 #-L $(LIBS)

#------------- Objects -------------
OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS))
AS_OBJS = $(patsubst $(SRC_DIR)/%.s,$(OBJ_DIR)/%.o,$(AS_SRCS))


#------------- Rules for building the elf file -------------
.PHONY: all build elf

all: elf hex srec lst

build: elf lst

elf: $(BIN_DIR)/$(TARGET).elf

$(BIN_DIR)/$(TARGET).elf: $(AS_OBJS) $(OBJS)
	@echo Compiling $<
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -o $@ $^
	@echo Size of your elf file :
	@$(SIZE) $@
# @echo Linking $<
# @mkdir -p $(@D)
# $(LD) $(LDFLAGS) $^ -o $@
# @echo Size of your elf file :
# @$(SIZE) $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.s
	@echo Compiling $<
	@mkdir -p $(@D)
	$(AS) $< -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@echo Compiling $<
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

#------------- Rules for building the listing file -------------
.PHONY: lst

lst: $(BIN_DIR)/$(TARGET).lst

$(BIN_DIR)/%.lst: $(BIN_DIR)/%.elf
	@echo Listing $<
	$(OBJDUMP) -h -S $< > $@

#------------- Rules for converting to hex or srec format -------------
.PHONY: hex srec

hex:  $(BIN_DIR)/$(TARGET).hex
srec: $(BIN_DIR)/$(TARGET).srec

$(BIN_DIR)/$(TARGET).hex: $(BIN_DIR)/$(TARGET).elf
	@echo Converting .elf to .hex
	$(OBJCOPY) -O ihex $< $@

$(BIN_DIR)/$(TARGET).srec: $(BIN_DIR)/$(TARGET).elf
	@echo Converting .elf to .srec
	$(OBJCOPY) -O srec $< $@

#------------- Rules for openocd -------------
.PHONY: flash

flash: $(BIN_DIR)/$(TARGET).elf
	$(PROG) $(PROG_CONFIG_FLAGS) -c "program $< $(PROG_RUN_FLAGS)"

#------------- Rules for cppcheck -------------
.PHONY: cppcheck

cppcheck: 
	@$(CPPCK) $(CPPCK_FLAGS) -I $(INCS) $(SRCS)

#------------- Other Rules -------------
.PHONY: clean

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