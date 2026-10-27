# GCC build for the MM32SPIN05 hoverboard firmware.
#   make                  build build/hoverboard.{elf,bin,hex}
#   make CROSS=/path/to/arm-none-eabi-    use a toolchain that is not on PATH
#   make flash            flash over SWD with pyOCD (../tools/mm32flash.py, keeps the board config)
#   make clean

CROSS   ?= arm-none-eabi-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size

BUILD   := build
TARGET  := $(BUILD)/hoverboard
LDSCRIPT := Device/MM32SPIN0x/Source/mm32spin05.ld

SRC := $(wildcard Src/*.c) \
       $(wildcard Library/MM32SPIN05/HAL_Lib/Src/*.c) \
       Device/MM32SPIN0x/Source/system_mm32spin05.c \
       Device/MM32SPIN0x/Source/startup_mm32spin0x_gcc.c
ASRC := $(wildcard Src/*.S)

# vendor and CMSIS headers as system headers: their warnings are not ours to fix
INC := -IInc -isystem Library/MM32SPIN05/HAL_Lib/Inc -isystem Device/MM32SPIN0x/Include -isystem Device/CMSIS/Include

ARCH    := -mcpu=cortex-m0 -mthumb
# -fno-strict-aliasing: the vendor HAL type-puns peripheral pointers (*(u32*)&gpio)
# -flto: 3 KB less flash (of 30 KB); the SWD tools still find the globals by name
CFLAGS  := $(ARCH) -Os -flto -g3 -std=gnu11 -ffunction-sections -fdata-sections -fno-common \
           -fno-strict-aliasing -Wall $(INC)
# make TRACE=1: the FOC capture buffer for SWD tools (foc.c, 768 bytes of RAM)
ifeq ($(TRACE),1)
CFLAGS  += -DFOC_TRACE
endif
# stricter for our own code (Src/, startup); the vendor HAL sources keep -Wall
WARN    := -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef
$(BUILD)/Src/%.o $(BUILD)/Device/MM32SPIN0x/Source/startup_mm32spin0x_gcc.o: CFLAGS += $(WARN)
# -nostartfiles: startup_mm32spin0x_gcc.c is the startup; newlib's crt0 pulled in exit() and stdio
LDFLAGS := $(ARCH) -Os -flto -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs -nostartfiles -Wl,--gc-sections \
           -Wl,-Map=$(TARGET).map -Wl,--print-memory-usage

OBJ := $(addprefix $(BUILD)/,$(SRC:.c=.o) $(ASRC:.S=.o))
DEP := $(OBJ:.o=.d)

all: $(TARGET).bin $(TARGET).hex

# objects and the link depend on this Makefile and on the flags used (TRACE=1, CROSS on the command line):
# a flag change rebuilds everything
FLAGS_STAMP := $(BUILD)/flags.txt
$(shell mkdir -p $(BUILD); echo '$(CC) $(CFLAGS)' | cmp -s - $(FLAGS_STAMP) || echo '$(CC) $(CFLAGS)' > $(FLAGS_STAMP))
$(BUILD)/%.o: %.c Makefile $(FLAGS_STAMP)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.S Makefile $(FLAGS_STAMP)
	@mkdir -p $(dir $@)
	$(CC) $(ARCH) -c $< -o $@

$(TARGET).elf: $(OBJ) $(LDSCRIPT) Makefile $(FLAGS_STAMP)
	$(CC) $(LDFLAGS) $(OBJ) -lm -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

PYTHON ?= python3
flash: $(TARGET).bin
	$(PYTHON) ../tools/mm32flash.py $<

# PinFinder: a separate image (the main firmware has no room for it) built from PinFinder/ and the shared
# hardware, config, uart and startup code. make pinfinder / make flash-pinfinder
PF       := $(BUILD)/pinfinder
PF_SRC   := $(wildcard PinFinder/*.c) Src/hardware.c Src/initialize.c Src/sim_eeprom.c Src/uart.c \
            $(wildcard Library/MM32SPIN05/HAL_Lib/Src/*.c) \
            Device/MM32SPIN0x/Source/system_mm32spin05.c Device/MM32SPIN0x/Source/startup_mm32spin0x_gcc.c
PF_OBJ   := $(addprefix $(BUILD)/,$(PF_SRC:.c=.o) Src/hwdiv.o)
$(BUILD)/PinFinder/%.o: CFLAGS += $(WARN)
$(BUILD)/PinFinder/%.o: CFLAGS += -IPinFinder

pinfinder: $(PF).bin

$(PF).elf: $(PF_OBJ) $(LDSCRIPT) Makefile $(FLAGS_STAMP)
	$(CC) $(subst $(TARGET).map,$(PF).map,$(LDFLAGS)) $(PF_OBJ) -o $@
	$(SIZE) $@

$(PF).bin: $(PF).elf
	$(OBJCOPY) -O binary $< $@

flash-pinfinder: $(PF).bin
	$(PYTHON) ../tools/mm32flash.py $<

clean:
	rm -rf $(BUILD)

.PHONY: all clean flash pinfinder flash-pinfinder
-include $(PF_OBJ:.o=.d)
-include $(DEP)
