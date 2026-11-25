# NAM for disting NT
#
#   make                 build plugins/nam_nt.o for the disting NT
#   make test            host test (parser + engine against references)
#   make check           list undefined symbols and section sizes
#
# NT_API_PATH points at a checkout of
# https://github.com/expertsleepersltd/distingNT_API (API v16 or later).

NT_API_PATH ?= ../ext/distingNT_API

CC  = arm-none-eabi-gcc
CXX = arm-none-eabi-g++

DEFS    = -DNAM_A2LITE_MAX_BUFFER_SIZE=48
ARCH    = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -fPIC
COMMON  = $(ARCH) $(DEFS) -ffunction-sections -fdata-sections -fno-unwind-tables -fno-asynchronous-unwind-tables -Wall
CXXFLAGS = $(COMMON) -std=c++11 -Os -fno-rtti -fno-exceptions -I$(NT_API_PATH)/include
# the engine wants speed: TONE3000's recommended flags
ENGINEFLAGS = $(COMMON) -std=c99 -O2 -ffast-math

OUT = plugins/nam_nt.o

all: $(OUT)

build/nam_nt.o: nam_nt.cpp nam_json.h engine/nam_a2lite.h engine/namb_reader.h
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -c -o $@ nam_nt.cpp

build/nam_a2lite.o: engine/nam_a2lite.c engine/nam_a2lite.h
	@mkdir -p build
	$(CC) $(ENGINEFLAGS) -c -o $@ $<

build/namb_reader.o: engine/namb_reader.c engine/namb_reader.h
	@mkdir -p build
	$(CC) $(COMMON) -std=c99 -Os -c -o $@ $<

$(OUT): build/nam_nt.o build/nam_a2lite.o build/namb_reader.o
	@mkdir -p plugins
	$(CXX) $(ARCH) -Wl,--relocatable -nostdlib -o $@ $^
	@arm-none-eabi-size -A $@ | grep -E "^\.text|^\.data|^\.bss|^\.rodata|Total"

check: $(OUT)
	@echo "Undefined symbols (resolved by the disting NT firmware):"
	@arm-none-eabi-nm $(OUT) | grep ' U ' || true
	@arm-none-eabi-size -A $(OUT)

# ---- host test ----
test:
	$(MAKE) -C test

clean:
	rm -rf build plugins
	$(MAKE) -C test clean

.PHONY: all check test clean
