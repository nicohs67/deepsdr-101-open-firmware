TARGET     = firmware
BUILD_DIR  = build

# --- Toolchain ---
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
LD      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

# --- MCU ---
CPU    = -mcpu=cortex-m4
FPU    = -mfpu=fpv4-sp-d16
FLOAT  = -mfloat-abi=hard
MCUFLAGS = $(CPU) -mthumb $(FPU) $(FLOAT)

# --- Defines ---
DEFS = -DGD32F450 -DUSE_STDPERIPH_DRIVER -DHXTAL_VALUE=12288000U

# --- Includes ---
INCLUDES  = -ICMSIS/Include
INCLUDES += -ICMSIS/GD/GD32F4xx/Include
INCLUDES += -IFirmware/Include
INCLUDES += -IUser

# --- Sources ---
C_SOURCES   = $(wildcard User/*.c)
C_SOURCES  += $(wildcard Firmware/Source/*.c)
C_SOURCES  += CMSIS/GD/GD32F4xx/Source/system_gd32f4xx.c

ASM_SOURCES = CMSIS/GD/GD32F4xx/Source/GCC/startup_gd32f450_470.S

# --- Linker ---
LDSCRIPT = GD32F450VE_FLASH.ld
LDFLAGS  = $(MCUFLAGS) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

CFLAGS  = $(MCUFLAGS) $(DEFS) $(INCLUDES) -Wall -O0 -g3 -ffunction-sections -fdata-sections
ASFLAGS = $(MCUFLAGS) -g3

OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

.PHONY: all clean flash erase

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

# --- Programming with ST-Link + OpenOCD ---
# We use our own target/gd32f450.cfg (in openocd/) instead of the
# stock target/stm32f4x.cfg: the GD32F450's silicon ID makes OpenOCD
# detect it as a dual-bank, 2048KB STM32F42x/43x, when the real VET6
# part is 512KB single-bank. See openocd/gd32f450.cfg.
#
# IMPORTANT: no address at the end of the program command. The .elf
# already carries the load address embedded in each section
# (0x08020000, set by the linker script). Passing an extra address
# here makes OpenOCD treat it as a "relocation offset" that gets ADDED
# to each section's address in the .elf (a feature meant for .bin
# files, which carry no address of their own) - that's what used to
# happen: it tried to write at 0x08020000+0x08020000 = 0x10040000,
# outside the flash range (0x08000000-0x08080000), silently failing to
# program anything.
flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f openocd/gd32f450.cfg \
		-c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

erase:
	openocd -f interface/stlink.cfg -f openocd/gd32f450.cfg \
		-c "init; reset halt; stm32f2x mass_erase 0; reset; exit"

# --- Generate update4.bin for the real bootloader ---
# The bootloader requires a fixed 8-byte signature at offset 0x40000
# (256KB) of the file, or it hangs with "APP not Programmed". This
# signature is constant (the same across any valid vendor firmware,
# independent of content), extracted by analyzing the bootloader. This
# target pads our .bin to 256KB and appends it.
UPDATE4_MAGIC := 8f25c865599c5531
update4: $(BUILD_DIR)/$(TARGET).bin
	python3 -c "\
firmware = open('$(BUILD_DIR)/$(TARGET).bin','rb').read(); \
magic = bytes.fromhex('$(UPDATE4_MAGIC)'); \
padded = firmware + b'\x00' * (0x40000 - len(firmware)) + magic; \
open('update4.bin','wb').write(padded); \
print('update4.bin generated:', len(padded), 'bytes')"

