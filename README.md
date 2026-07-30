This project is a collaboration between UA6YKK Alexandr and EA8DGL Esteban, aiming to create open firmware for the DEEPSDR 101 with the GD32F450 MCU. It is currently under development, and there is no usable firmware yet. We would welcome any collaboration or assistance with its development. Regards.

# GD32F450 QSD SDR Receiver

Bare-metal firmware for a custom GD32F450VET6-based SDR receiver
board: an 800x480 RM68120 TFT display, resistive touch, and a
TLV320AIC3204 audio codec used as the I/Q ADC/DAC front-end for a QSD
(Quadrature Sampling Detector) mixer stage clocked by an MS5351 local
oscillator.

No RTOS, no HAL abstraction beyond GigaDevice's standard peripheral
library - direct register access where it matters (clocks, DMA, I2S).

## Hardware

- MCU: GD32F450VET6 (Cortex-M4F, 512KB Flash single bank, 192KB main
  SRAM + separate TCM)
- Crystal: 12.288MHz HXTAL (not the 25MHz the vendor's stock clock
  branch assumes - see `CMSIS/GD/GD32F4xx/Source/system_gd32f4xx.c`)
- Display: RM68120 480x800 TFT over EXMC, pinout confirmed by
  continuity (RESET=PC9, CS=PD7, RS=PD11, WR=PD5, RD=PD4, D0-D15 on
  ports D/E), driven via `rm68120_exmc.c`
- Touch: resistive panel, bit-banged via `touch.c`
- Audio codec: TLV320AIC3204, connected over I2S1/I2S1_ADD (full
  duplex) + I2C (control)
- RF front-end: QSD with an MS5351 (Si5351-compatible) local
  oscillator (LO frequency control not yet implemented in firmware)

## Confirmed clock tree

These values were measured on real hardware with an oscilloscope, not
predicted from a formula - see the comments in `gd32_i2s.c` and
`aic3204.c` for the full derivation:

- System clock (SYSCLK/HCLK): ~199.68MHz (main PLL, PSC=8, N=260, P=2)
- I2S1 (SPI1) master clock chain: PLLI2S N=128, R=4 -> i2sclock =
  49.152MHz
- MCLK (PC6, to the AIC3204): 12.288MHz
- BCLK (PB13): 1.536MHz
- WCLK / sample rate (PB12): 48kHz

`PSC` is shared between the main system PLL and PLLI2S (both read the
same `RCU_PLL` field) - if you ever change one, the other must be
recalculated to match, or the I2S clock chain will silently break.

## AIC3204 I/Q input routing

- I (left ADC channel): `IN2_L` (+) / `IN2_R` (-), differential.
  **Confirmed on real hardware.**
- Q (right ADC channel): `IN3_R` (+) / `IN3_L` (-), differential.
  Routing confirmed at the architecture level against TI's SLAA557
  Application Reference Guide, but the exact register value
  (`P1_R55`/`P1_R57`) is extrapolated from an earlier Arduino driver
  for this same board and is **pending independent verification**.
  See `SDR_SHOW_CHANNEL_Q` in `main.c` and the I/Q min/max debug
  output for a way to verify it by injecting a known signal into
  `IN3_R`/`IN3_L`.

## Firmware layout

| File | Purpose |
|---|---|
| `main.c` | Startup sequence, main loop, demo UI, SDR spectrum/waterfall tick |
| `gd32_i2s.c/h` | I2S1 master clock setup (WS/BCLK/MCLK), TX test tone via DMA |
| `sdr_rx.c/h` | I2S1_ADD RX capture via circular DMA (ping-pong), I/Q de-interleaving |
| `aic3204.c/h` | AIC3204 driver: I2C bring-up (phase 1), clock/ADC/routing config (phase 2) |
| `fft.c/h` | Self-contained radix-2 FFT (no libm), Hann window, dB conversion |
| `spectrum.c/h` | Instantaneous spectrum drawing + shared dB colormap |
| `waterfall.c/h` | Scrolling waterfall history buffer and blit |
| `rm68120_exmc.c/h` | Display controller init and EXMC bus driver |
| `touch.c/h` | Resistive touch panel driver |
| `gfx.c/h`, `ui.c/h` | Framebuffer-less drawing primitives and a minimal widget/screen system |
| `i2c_bitbang.c/h` | Bit-banged I2C master, used for the AIC3204 control interface |
| `debug_uart.c/h` | USART0 logging (115200 8N1) |

Other directories:

- `CMSIS/` - ARM CMSIS core + GD32F4xx startup/support headers
  (vendor-provided, not hand-written).
- `Firmware/` - the full GD32F4xx Standard Peripheral Library (all
  peripherals, not just the ones currently used).
- `GD32F450VE_FLASH.ld` - linker script for the VET6 variant (512KB
  Flash, 192KB RAM + 64KB TCM). The bootloader on this board expects
  the application at `0x08020000`, not `0x08000000` - `SCB->VTOR` is
  set explicitly in `main()` to match.
- `.vscode/` - build/flash tasks and a Cortex-Debug launch
  configuration.

## Building

Requires `arm-none-eabi-gcc` and OpenOCD with an ST-Link:

```sh
sudo apt install gcc-arm-none-eabi openocd
```

```sh
make            # build build/firmware.elf/.hex/.bin
make flash      # flash + verify + reset via OpenOCD
make erase      # mass-erase via OpenOCD
make update4    # pad + sign build/firmware.bin as update4.bin for the
                # vendor bootloader (see Makefile for the fixed magic
                # signature this requires)
make clean
```

`openocd/gd32f450.cfg` is a custom target config, not the stock
`target/stm32f4x.cfg`: the GD32F450's silicon ID makes OpenOCD
misdetect it as a dual-bank 2048KB STM32F42x/43x, when the real part
is 512KB single-bank. Without this you'll see `Error: checksum
mismatch` after programming.

### Debugging in VS Code

With the Cortex-Debug extension installed (suggested in
`.vscode/extensions.json`), F5 launches the "Debug GD32F450 (OpenOCD +
ST-Link)" configuration already set up in `.vscode/launch.json`.

## Known open items

- Q-channel (`IN3_R`/`IN3_L`) routing: pending independent
  verification (see above).
- `SDR_DB_MIN`/`SDR_DB_MAX` in `main.c`: an uncalibrated working dB
  range for the spectrum/waterfall display, not a referenced
  measurement.
- `i2c_bitbang.c`'s `delay_i2c()` uses a fixed CPU-cycle NOP loop
  rather than a clock-independent delay - if `SystemCoreClock` is ever
  raised further, this should be revisited.
- MS5351 LO frequency control (with a rotary encoder) is not yet
  implemented.
- PGA gain is currently fixed at 0dB on both channels - will need
  tuning once a real signal chain (QSD + LO) is connected.
- Touch panel calibration: `touch.c` currently uses an identity
  mapping (raw 0-4095 -> screen 0-800/0-480), not a real calibration
  against the panel's 4 corners.
