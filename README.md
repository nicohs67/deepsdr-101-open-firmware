# Descrption

This project is a collaboration between EA8DGL Esteban, UA6YKK Alexandr and 
EA7GIB Blas, aiming to create open firmware for the DEEPSDR 101 with the 
GD32F450 MCU. 

It is currently in the development phase, and programming is being carried 
out primarily using AI. The current version is functional and supports most 
DEEPSDR radio features.

We would welcome any collaboration or assistance with its development. 
Regards.

# License

GD32F450 QSD SDR Receiver
Copyright (c) 2026 Jorge

This work is licensed under a Creative Commons
Attribution-NonCommercial 4.0 International License (CC BY-NC 4.0).

You are free to:
  - Share   — copy and redistribute the material in any medium or format
  - Adapt   — remix, transform, and build upon the material

Under the following terms:
  - Attribution   — You must give appropriate credit, provide a link to
    the license, and indicate if changes were made. You may do so in any
    reasonable manner, but not in any way that suggests the licensor
    endorses you or your use.
  - NonCommercial — You may not use the material for commercial purposes.

No additional restrictions — You may not apply legal terms or
technological measures that legally restrict others from doing
anything the license permits.

Full legal code:      https://creativecommons.org/licenses/by-nc/4.0/legalcode
Human-readable deed:  https://creativecommons.org/licenses/by-nc/4.0/

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
  oscillator, tuned by firmware (see "MS5351 quadrature LO" below and
  the encoder-driven tuning under "User interface")
- Speaker PA enable/mute: PB7, active-HIGH - confirmed hardware fact
  (already used and driven in `demod_am.c`), now also exposed as a
  software mute toggled from the settings menu (SPK tile, HW page -
  see "User interface" below). See "Known open items" for a small
  duplicate-init caveat between `demod_am.c` and `main.c` around this
  pin.
- Rotary encoder: PD13 (A) / PD12 (B) quadrature + PC9 push button
  (active HIGH), 1kHz SysTick sampling - drives tuning, and doubles as
  the adjustment knob for whichever setting is currently selected (see
  "User interface")
- Battery: VBAT sensed via ADC0 channel 18 (`battery.c`)
- Backlight: PWM brightness on PA3/TIMER1_CH3 (`backlight.c`)

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
- Q (right ADC channel): `IN3_R` (+) / `IN3_L` (-), differential.


## MS5351 quadrature LO

The QSD's local oscillator is an MS5351M (Si5351A-compatible clone) at
I2C address `0x60`, sharing the bit-banged bus with the codec. 

- **Reference: 26MHz.** Inferred from the capture: PLLA is programmed
  to x32 integer and MS2 to /104, which yields exactly 8.000MHz only
  with a 26MHz reference.
- **PLLA = 832MHz** fixed, feeding only CLK2 (8MHz auxiliary output,
  configured but left powered down by the original firmware -
  `ms5351_clk2_8mhz()` can switch it on).
- **PLLB** is the tunable fractional PLL feeding CLK0/CLK1 through
  identical even output dividers. Quadrature is produced with the
  phase-offset registers: CLK0 offset = divider value (= 90 degrees),
  CLK1 offset = 0, followed by a PLLB soft reset to latch the phases.
- The captured tune is **LO = 90.800MHz** (divider 10, VCO at 908MHz -
  marginally above the 900MHz datasheet ceiling, which this clone
  demonstrably tolerates). `ms5351_set_lo_freq()` reproduces those
  exact register bytes when asked for 90.8MHz (verified byte-for-byte
  against the capture), and generalizes to roughly 4.8-225MHz, the
  range where the phase-offset quadrature trick is possible.
- Retunes that keep the same divider skip the PLL reset (no click);
  the reset only happens when the divider - and with it the phase
  offset - changes.

Startup order matches the capture: MS5351 base init before the codec
configuration, quadrature tune right after `aic3204_phase2_init()`.

### Front-end low-pass filter bank

The RF input has a 4-position low-pass filter bank switched by PA1,
PA2 and PA5 (levels listed in that order):

| Range | Frequencies | PA1 PA2 PA5 |
|---|---|---|
| 1 | 0 - 36 MHz | 0 1 0 |
| 2 | 37 - 60 MHz | 1 1 0 |
| 3 | 61 - 120 MHz | 0 0 0 |
| 4 | 121 - 180 MHz | 1 0 0 |

PA5 is low in all four positions (PA1/PA2 alone encode the range);
it is still driven low explicitly in case it controls an additional
front-end function. `rf_lpf_select()` is called from the MS5351
driver on every tune, so the filter always follows the LO; above
180MHz it clamps to range 4 with a UART warning.

### AM demodulation + audio out

Audio path (30/07/2026): `demod_am.c` turns baseband I/Q into speaker
audio. Chain per sample at the full 192kHz (no decimation - the DAC
shares the same I2S clock): envelope via alpha-max-beta-min |I+jQ|
(no sqrt/libm), one-pole DC blocker (~15Hz) removing the carrier
term, two cascaded one-pole LPFs (~6kHz audio bandwidth), and an
instant-attack / ~180ms-release AGC with bounded gain (silence stays
silent). Same sample to L and R. PB7 (speaker enable, active-HIGH -
see "Speaker PA" under Hardware above) is unconditionally driven high
in `demod_am_init()`; since 03/08/2026 it's ALSO owned by
`main.c`'s `speaker_pa_set_enabled()`, toggled from the settings
menu's SPK tile (HW page) - see "Known open items" for the resulting
duplicate-init caveat.

**Execution model - the important part**: demodulation runs in the RX
DMA interrupt, not the main loop. `sdr_rx` now delivers blocks via
the DMA0/CH3 half/full-transfer ISR, which calls a registered hook
(`sdr_rx_set_block_hook()`) with each raw half; the display path
still polls, consuming a "pending half" note from the ISR. Rationale:
one spectrum frame takes longer to draw than one 2.67ms block, so
main-loop audio would gap ~30x/s; in the ISR (~100us/block, ~4% CPU)
audio is immune to the display. The demodulated block goes out
through a 2-half ping-pong TX stream (`gd32_i2s_dma_start_stream()` /
`gd32_i2s_stream_write_half()`) on the same DMA channel that carried
the bring-up test tone; RX and TX share the codec's BCLK/WS, so their
rates are hardware-locked and the write-half choice self-corrects
from the live DMA position.

**Channel selection**: a cascaded one-pole LPF applied identically to
I and Q before the detector acts as a complex low-pass centered on
the VFO (~+/-4kHz composite corner), so the receiver demodulates AT
THE CENTER LINE of the panadapter rather than hearing the strongest
station anywhere in the +/-96kHz window. Measured on host with an
EQUAL-strength interferer: 45.7dB of selectivity at 30kHz spacing,
23.3dB at 10kHz (adjacent channel). Bandwidth is one define
(`CHF_K` in demod_am.c: 0.12 ~= +/-2kHz, 0.35 ~= +/-6.5kHz). Within
the channel, envelope detection remains tuning-tolerant (a 3kHz
off-tune carrier demodulates identically). Caveat inherent to
envelope+AGC: with NOTHING on-channel, a lone strong station outside
the filter is still pulled back up toward audibility by the AGC (the
filter attenuates its carrier, not its modulation depth) - the moment
any signal is present on-channel it wins by the filter margin.

If the speaker stays silent with a confirmed-good stream: the ported
`aic3204_phase2_init()` is described as ADC-baseline - the codec's
DAC/output-driver routing (page 1 registers) may still need porting
from the original I2C capture. The bring-up test tone being audible
would rule this out.

### I/Q panadapter (complex FFT)

Since 30/07/2026 the display uses a COMPLEX FFT of I + jQ
(`fft_compute_db_iq()`), not a real FFT of one channel. With a real
transform, baseband 0Hz (= the VFO) sat at the LEFT EDGE and +/-f
were indistinguishable - a signal tuned on the VFO showed up far
left. The complex transform separates them; output comes fftshifted
(index 0 = -96kHz, center = VFO, top = +96kHz at 192kHz sampling), so
the spectrum reads like a proper panadapter: VFO on the center line
(dim red marker), lower frequencies left, higher right, "-96k / VFO /
+96k" labels under the trace. The DC bin (+/-2 neighbors) is blanked
cosmetically - the QSD/codec DC offset would otherwise put a
permanent spike on the center line.

Note for RF validation: with real hardware, expect a MIRRORED faint
copy of each signal on the opposite side of center - that's analog
I/Q gain/phase imbalance, not a firmware bug (the FFT itself measures
>130dB image rejection on synthetic input). Correcting it digitally
is a future DSP step.

### Spectrum/waterfall rendering pipeline

Rebuilt (30/07/2026) for speed and readability. The old path called a
float colormap function per lit pixel (~80k float calls/frame) and
redrew spectrum + waterfall for every 512-sample block (up to 375
attempts/s); the waterfall additionally memmoved ~94KB per line.

Current model:

- **Palette LUT**: 256-entry RGB565 table built once by
  `spectrum_init()`; shared by trace and waterfall. The per-pixel
  inner loop is integer-compare + store only.
- **Frame throttle + averaging**: FFTs still run per block but are
  summed; every ~33ms the average is drawn once (spectrum + one
  waterfall line). Averaging N FFTs/frame lowers displayed noise
  variance - calmer floor, smoother waterfall. FFT work is capped at
  6/frame so it can't starve touch/encoder polling.
- **Column folding**: each screen column averages all FFT bins that
  land in it (no more nearest-neighbor shimmer).
- **Asymmetric EMA** per column (fast attack / slow decay) plus
  decaying **peak-hold** markers; vertical **gradient fill**, 1px
  white trace, dim gridline every 20dB. All tunable via the defines
  at the top of `spectrum.h`.
- **Ring-buffer waterfall**: push is one 1.6KB row copy; scroll is
  index bookkeeping; blit is two contiguous segments.

### Encoder tuning + on-screen frequency readout

A rotary encoder on PD13 (A), PD12 (B) and PC9 (push button, active
HIGH - Vcc when pressed, 0V at rest, internal pull-down; A/B use
internal pull-ups) drives the LO:

- **Rotate**: tune up/down by the current step. Range clamped to
  4.8-180MHz (quadrature floor / LPF ceiling).
- **Push**: cycles the tune step (100Hz / 1k / 5k / 10k / 12.5k / 25k
  / 100k / 1M - see "Settings menu" below for the full picker).
- The frequency ("XXX.XXX.XXX" format) and the active step are drawn
  in the right side of the title bar and repainted only on change.

Decoding: `encoder_tick()` runs at 1kHz from `SysTick_Handler` and
feeds A/B through a full-quadrature Gray-transition table - valid
transitions count quarter-steps, invalid ones (bounce/noise) are
inherently ignored, so no timing-based debounce is needed for
rotation. 4 quarter-steps = 1 detent, with sub-detent remainder
carried between reads. The button uses a 20ms integrating debounce.
The main loop drains accumulated detents once per pass, so fast
spins during a slow FFT pass coalesce into a single retune instead of
queueing stale intermediate ones. If tuning direction comes out
reversed on the real knob, flip `ENCODER_DIRECTION` in `encoder.c`;
if each detent moves two steps instead of one, set
`QUARTER_STEPS_PER_DETENT` to 2.

## User interface

Touch UI (`ui.c`/`gfx.c`, no framebuffer - direct EXMC draws), laid
out as: title bar (frequency / mode / step / time / battery) on top,
the spectrum+waterfall panadapter as the main area (drag left/right on
it to tune, quantized to 1kHz steps), a right-hand column with the
S-meter and status badges (AGC profile, audio bandwidth, PRE), and a
6-button bar along the bottom.

### Bottom bar

| Button | Action |
|---|---|
| MODE  | Opens the demod mode picker (AM/USB/LSB/NFM/WFM) |
| VOL   | Toggles the encoder between TUNE and VOLUME |
| STEP  | Opens the tune-step picker (100Hz-1MHz, 8 entries) |
| NR    | Cycles the AGC profile (MAN/SLW/MED/FST) - same action as tapping the AGC badge in the right column |
| BANDS | Opens the band preset picker |
| MENU  | Opens the settings menu (see below) |

### Encoder

Same rotary knob used for tuning (PD13/PD12 quadrature, PC9 push, see
"Encoder tuning" above) doubles as the adjustment control for
whichever target is currently selected:

- **Rotate**: adjusts the current target - frequency by default, or
  VOLUME/SQUELCH/BACKLIGHT/SCALE/PGA/SMOOTH once one of those is
  picked (from the settings menu, or VOL on the bottom bar).
- **Short press**: cycles the tune step when the target is TUNE,
  toggles the LO/HI bound when the target is SCALE; no effect for the
  other targets.
- **Long press**: unconditionally hands the knob back to TUNE and
  closes any open menu/picker screen - the universal "get me out of
  here" gesture, from anywhere.

### Band / mode / step pickers

Reachable directly from the bottom bar (not nested inside the
settings menu):

- **BANDS** (12 presets): SW 49M/41M/31M/19M, FM BCST, AIRBAND, 2M,
  VHF HI, 80M/40M/20M, 11M (CB) - each one-tap-sets frequency + demod
  mode + tune step together.
- **MODE** (5): AM, USB, LSB, NFM, WFM.
- **STEP** (8): 100Hz, 1kHz, 5kHz, 10kHz, 12.5kHz, 25kHz, 100kHz, 1MHz.

### Settings menu (MENU button)

Redesigned 03/08/2026 into a PAGED tile grid (4 columns x 3 rows,
confined to the panadapter area - the title bar, S-meter/badges, and
bottom bar all stay live underneath):

- **Column 0** (all 3 rows): a fixed page selector - **RADIO / UI /
  HW** - visually distinct (orange) from every option tile, so
  "switches page" reads differently at a glance from "is a page's
  option". The active page is filled solid; the other two are
  outlined.
- **Columns 1-3** (3x3 = 9 slots): the selected page's options.
  Bottom-right is always **EXIT**, on every page, closing the whole
  menu.

| Page | Options |
|---|---|
| RADIO | AGC (profile), SQUELCH, VOL, BW (AM/SSB audio filter width, 4K0/2K3/1K8), PGA (input gain, 0-47.5dB) |
| UI | BL (backlight %), SCALE (spectrum dB range), SPT (spatial smoothing passes), SMOOTH (frame-to-frame smoothing, see "Spectrum/waterfall rendering pipeline" above), SPC (trace style, heatmap/line), ZOOM (1x/2x/4x/8x, see "Spectrum/waterfall ZOOM" comment in `main.c`) |
| HW | SPK (speaker PA enable/mute, PB7 - see Hardware above); the remaining 7 slots are free for future hardware-related settings |

Some tiles cycle/toggle directly on tap and stay on the grid (AGC,
BW, SPC, ZOOM, SPK); the rest (SQUELCH, VOL, BL, SCALE, PGA, SMOOTH)
open a full-screen DETAIL view where the encoder adjusts the value
live and a BACK tile returns to the grid.

## Firmware layout

| File | Purpose |
|---|---|
| `main.c` | Startup sequence, main loop, demo UI, SDR spectrum/waterfall tick |
| `gd32_i2s.c/h` | I2S1 master clock setup (WS/BCLK/MCLK), TX test tone via DMA |
| `sdr_rx.c/h` | I2S1_ADD RX capture via circular DMA (ping-pong), I/Q de-interleaving |
| `aic3204.c/h` | AIC3204 driver: I2C bring-up (phase 1), clock/ADC/routing config (phase 2) |
| `ms5351.c/h` | MS5351 (Si5351 clone) quadrature LO driver, replayed from a captured I2C init |
| `rf_lpf.c/h` | Front-end low-pass filter bank selector (PA1/PA2/PA5), tracks the LO frequency |
| `encoder.c/h` | Rotary encoder driver (PD13/PD12 quadrature + PC9 button), 1kHz SysTick sampling |
| `demod_am.c/h` | AM envelope demodulator (ISR context), PB7 speaker enable, AGC |
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

- `SDR_DB_MIN`/`SDR_DB_MAX` in `main.c`: an uncalibrated working dB
  range for the spectrum/waterfall display, not a referenced
  measurement.
- `i2c_bitbang.c`'s `delay_i2c()` uses a fixed CPU-cycle NOP loop
  rather than a clock-independent delay - if `SystemCoreClock` is ever
  raised further, this should be revisited.
- PGA gain now defaults to 20dB and is live-adjustable per channel
  pair from the settings menu's PGA tile (RADIO page) - still needs
  real-world tuning once a full signal chain (QSD + LO) is connected
  and receiving actual signals, the default is just a starting guess.
- Touch panel calibration: `touch.c` currently uses an identity
  mapping (raw 0-4095 -> screen 0-800/0-480), not a real calibration
  against the panel's 4 corners.

