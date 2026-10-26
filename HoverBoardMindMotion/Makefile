# GCC build for the MM32SPIN05 hoverboard firmware.
#   make                  build build/hoverboard.{elf,bin,hex}
#   make CROSS=/path/to/arm-none-eabi-    use a toolchain that is not on PATH
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

INC := -IInc -ILibrary/MM32SPIN05/HAL_Lib/Inc -IDevice/MM32SPIN0x/Include -IDevice/CMSIS/Include

ARCH    := -mcpu=cortex-m0 -mthumb
CFLAGS  := $(ARCH) -Os -g3 -std=gnu11 -ffunction-sections -fdata-sections -fno-common \
           -Wall $(INC)
LDFLAGS := $(ARCH) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections \
           -Wl,-Map=$(TARGET).map -Wl,--print-memory-usage

OBJ := $(addprefix $(BUILD)/,$(SRC:.c=.o))
DEP := $(OBJ:.o=.d)

all: $(TARGET).bin $(TARGET).hex

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(TARGET).elf: $(OBJ) $(LDSCRIPT)
	$(CC) $(LDFLAGS) $(OBJ) -lm -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
-include $(DEP)
