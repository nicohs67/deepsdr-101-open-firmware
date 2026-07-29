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

# --- Fuentes ---
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

# --- Programacion con ST-Link + OpenOCD ---
# Usamos un target/gd32f450.cfg propio (en openocd/) en vez del
# target/stm32f4x.cfg de serie: el ID de silicio del GD32F450 hace que
# OpenOCD lo detecte como STM32F42x/43x de doble banco y 2048KB, cuando
# la VET6 real es de 512KB en banco unico. Ver openocd/gd32f450.cfg.
#
# IMPORTANTE: sin direccion al final. El .elf ya lleva la direccion de
# carga incrustada en cada seccion (0x08020000, puesta por el linker
# script). Si se pasa una direccion extra aqui, OpenOCD la trata como
# "offset de reubicacion" que se SUMA a la de cada seccion del .elf
# (funcion pensada solo para .bin, que no llevan direccion propia) -
# eso es lo que pasaba antes: intentaba escribir en 0x08020000+0x08020000
# = 0x10040000, fuera del rango de flash (0x08000000-0x08080000), y por
# eso "no programaba" nada.
flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f openocd/gd32f450.cfg \
		-c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

erase:
	openocd -f interface/stlink.cfg -f openocd/gd32f450.cfg \
		-c "init; reset halt; stm32f2x mass_erase 0; reset; exit"

# --- Generar update4.bin para el bootloader real ---
# El bootloader exige una firma fija de 8 bytes en el offset 0x40000
# (256KB) del archivo, o se cuelga con "APP not Programmed". Esta
# firma es constante (la misma en cualquier firmware valido del
# fabricante, no depende del contenido), extraida por analisis del
# bootloader. Este target rellena nuestro .bin hasta 256KB y la añade.
UPDATE4_MAGIC := 8f25c865599c5531
update4: $(BUILD_DIR)/$(TARGET).bin
	python3 -c "\
firmware = open('$(BUILD_DIR)/$(TARGET).bin','rb').read(); \
magic = bytes.fromhex('$(UPDATE4_MAGIC)'); \
padded = firmware + b'\x00' * (0x40000 - len(firmware)) + magic; \
open('update4.bin','wb').write(padded); \
print('update4.bin generado:', len(padded), 'bytes')"

