TARGET ?= main
TARGET_MCU ?= CH32V002
TARGET_MCU_PACKAGE ?= CH32V002F4U6

CH32V003FUN ?= ./ch32fun/ch32fun
MINICHLINK ?= ./ch32fun/minichlink

PREFIX ?= riscv64-none-elf
NEWLIB ?= /usr/riscv64-none-elf/include/

INCLUDE_DIRS ?= \
	-I./inc \
	-I./u8g2/csrc

PROJECT_C_FILES := $(filter-out ./main.c, $(wildcard ./*.c))
U8G2_C_FILES := $(wildcard ./u8g2/csrc/*.c)
LIB_C_FILES := $(U8G2_C_FILES)

ADDITIONAL_C_FILES ?= $(PROJECT_C_FILES) $(LIB_C_FILES)

include $(CH32V003FUN)/ch32fun.mk

CFLAGS += -Wall -Wextra $(INCLUDE_DIRS)

all: flash
flash: cv_flash
clean: cv_clean
.PHONY: all flash clean

