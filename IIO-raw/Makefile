CROSS_COMPILE ?= /home/luban/lubancat/sdk/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-
CC := $(CROSS_COMPILE)gcc
CFLAGS := -O2 -Wall -Wextra -Werror -std=c11

TARGET := iio_buffer_reader

.PHONY: all clean

all: $(TARGET)

$(TARGET): iio_buffer_reader.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
