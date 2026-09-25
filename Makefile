TARGET  := ebb42_heater
BUILD   := build
TUSB    := vendor/tinyusb/src

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

MCUFLAGS := -mcpu=cortex-m0plus -mthumb
CFLAGS   := $(MCUFLAGS) -std=gnu11 -O2 -g3 -Wall -Wextra -Wno-unused-parameter \
            -ffunction-sections -fdata-sections -fno-common \
            -DSTM32G0B1xx -DCFG_TUSB_MCU=OPT_MCU_STM32G0 -DCFG_TUSB_OS=OPT_OS_NONE \
            -Isrc -Ivendor/cmsis_device_g0/Include -Ivendor/cmsis_core -I$(TUSB)
ASFLAGS  := $(MCUFLAGS) -x assembler-with-cpp
LDFLAGS  := $(MCUFLAGS) -specs=nano.specs -specs=nosys.specs \
            -Wl,-T,stm32g0b1.ld -Wl,--gc-sections \
            -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--print-memory-usage
LDLIBS   := -lm

SRCS_C := src/main.c src/usb.c src/usb_descriptors.c src/oled.c src/ui.c src/settings.c \
          $(TUSB)/tusb.c \
          $(TUSB)/common/tusb_fifo.c \
          $(TUSB)/device/usbd.c \
          $(TUSB)/class/cdc/cdc_device.c \
          $(TUSB)/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \
          $(TUSB)/portable/st/stm32_fsdev/fsdev_common.c
SRCS_S := src/startup_stm32g0b1xx.s
OBJS   := $(patsubst %.c,$(BUILD)/%.o,$(SRCS_C)) $(patsubst %.s,$(BUILD)/%.o,$(SRCS_S))

all: $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) stm32g0b1.ld
	$(CC) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $@
	$(SIZE) $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

flash: all
	./flash.sh

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
.PHONY: all flash clean
