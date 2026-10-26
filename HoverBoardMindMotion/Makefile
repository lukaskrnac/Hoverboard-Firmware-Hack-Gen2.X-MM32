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

# vendor and CMSIS headers as system headers: their warnings are not ours to fix
INC := -IInc -isystem Library/MM32SPIN05/HAL_Lib/Inc -isystem Device/MM32SPIN0x/Include -isystem Device/CMSIS/Include

ARCH    := -mcpu=cortex-m0 -mthumb
# -fno-strict-aliasing: the vendor HAL type-puns peripheral pointers (*(u32*)&gpio)
CFLAGS  := $(ARCH) -Os -g3 -std=gnu11 -ffunction-sections -fdata-sections -fno-common \
           -fno-strict-aliasing -Wall $(INC)
# stricter for our own code (Src/, startup); the vendor HAL sources keep -Wall
WARN    := -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef
$(BUILD)/Src/%.o $(BUILD)/Device/MM32SPIN0x/Source/startup_mm32spin0x_gcc.o: CFLAGS += $(WARN)
# -nostartfiles: startup_mm32spin0x_gcc.c is the startup; newlib's crt0 pulled in exit() and stdio
LDFLAGS := $(ARCH) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs -nostartfiles -Wl,--gc-sections \
           -Wl,-Map=$(TARGET).map -Wl,--print-memory-usage

OBJ := $(addprefix $(BUILD)/,$(SRC:.c=.o))
DEP := $(OBJ:.o=.d)

all: $(TARGET).bin $(TARGET).hex

# objects and the link depend on this Makefile: a flag change rebuilds everything
$(BUILD)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(TARGET).elf: $(OBJ) $(LDSCRIPT) Makefile
	$(CC) $(LDFLAGS) $(OBJ) -lm -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

PYTHON ?= python3
flash: $(TARGET).bin
	$(PYTHON) ../tools/mm32flash.py $<

clean:
	rm -rf $(BUILD)

.PHONY: all clean flash
-include $(DEP)
