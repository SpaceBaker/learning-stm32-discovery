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
TARGET = myApp
MCU    = atmega328p
F_CPU  = 8000000

#------------- Working directories -------------
SRC_DIR	   = src
BUILD_DIR  = build
BIN_DIR    = $(BUILD_DIR)/bin
OBJ_DIR    = $(BUILD_DIR)/obj
# MAKE_SCRIPT_DIR = mk

#------------- Toolchain -------------
# Provide the full path if not found in your environment variables
CC_DIR  = $(HOME)/tools/avr8-gnu-toolchain-linux_x86_64/bin
CC		= $(CC_DIR)/avr-gcc
LD		= $(CC_DIR)/avr-ld
OBJCOPY = $(CC_DIR)/avr-objcopy
OBJDUMP = $(CC_DIR)/avr-objdump
SIZE	= $(CC_DIR)/avr-size
CPPCK	= $(HOME)/tools/cppcheck-2.13.0/build/bin/cppcheck
# PROG	= $(HOME)/tools/avrdude/v7.1/avrdude

# List source files here
# Exemple : $(wildcard $(SRC_DIR)/bsp/driver/*.c) ...
# Exemple : $(SRC_DIR)/bsp/driver/i2c.c $(SRC_DIR)/bsp/driver/spi.c) ...
SRCS =	$(wildcard $(SRC_DIR)/*.c)

# List header file directories here
# Exemple : $(SRC_DIR)/bsp/utils
INCS =	$(SRC_DIR)

# List library directories here
# Exemple : TODO
LIBS =

#------------- Defines -------------
DEFS = -DF_CPU=$(F_CPU)UL

#------------- Optimization Level -------------
OPT	= -Og

#------------- Warnings Options -------------
WARNS = -Wall -Wundef -Wextra -Werror

#------------- Debugging Format -------------
DEBUG = -g

#------------- C Language ISO Standard -------------
CSTD = -std=gnu11

#------------- Verbose -------------
# Comment the line to disable verbose output
# VERB = -v

#------------- cppcheck flags -------------
CPPCK_FLAGS = --quiet --error-exitcode=1 --language=c --std=c11 --suppress=unusedFunction


#---------------------------------------------------------------------------------
# DO NOT EDIT BELOW THIS LINE, UNLESS YOU KNOW WHAT YOU ARE DOING
#---------------------------------------------------------------------------------

#------------- C Compiler Flags -------------
CFLAGS = $(VERB) -mmcu=$(MCU) $(OPT) $(DEBUG) $(WARNS) $(addprefix -I, $(INCS)) $(CSTD) $(DEFS)

#------------- C Linker Flags -------------
LDFLAGS = $(VERB) -mmcu=$(MCU) -Wl,-Map,$(BIN_DIR)/$(TARGET).map,--cref -N $(LIBS)

#------------- Objects -------------
OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS))


#------------- Rules for building the elf file -------------
.PHONY: all build elf

all: elf hex srec lst

build: elf lst

elf: $(BIN_DIR)/$(TARGET).elf

$(BIN_DIR)/$(TARGET).elf: $(OBJS)
	@echo Linking $<
	@mkdir -p $(@D)
	$(CC) $(LDFLAGS) $^ -o $@
	@echo Size of your elf file :
	@$(SIZE) $@

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

#------------- Rules for avrdude (programmer) -------------
# .PHONY: flash read_fuses

# flash: $(BIN_DIR)/$(TARGET).hex
# 	$(PROG) $(VERB) -c usbasp -P usb -qq -p $(MCU) -b 115200 -U $@:w:$<:i

#Reaf fuses
# read_fuses:
# 	$(PROG) $(VERB) -c usbasp -P usb -qq -p $(MCU) -b 115200 -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h

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