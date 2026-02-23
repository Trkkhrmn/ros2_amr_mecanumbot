# Makefile for STM32 protocol + stub (host or ARM build)
# Copyright (c) 2025 Tarik Kahraman. SPDX-License-Identifier: MIT
#
# For host (unit test): make build_host
# For ARM (STM32): set CROSS_COMPILE and use build_arm (requires CMSIS/HAL elsewhere)

CC       ?= gcc
CROSS_COMPILE ?=
CFLAGS   := -Wall -Wextra -std=c11 -O2 -I Core
LDFLAGS  :=

SRCS     := Core/protocol.c Core/motor_lift_stub.c
OBJS     := $(SRCS:.c=.o)

HOST_SRCS := $(SRCS) main_host.c

.PHONY: all build_host clean

all: build_host

build_host: main_host
	@echo "Host build done. Run: ./main_host"

main_host: Core/protocol.c main_host.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f Core/*.o main_host.o main_host
