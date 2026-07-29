# Intro

This project is a collaboration between UA6YKK Alexandr and EA8DGL Esteban, aiming to create open firmware for the DEEPSDR 101 with the GD32F450 MCU. It is currently under development, and there is no usable firmware yet. We would welcome any collaboration or assistance with its development. Regards.

# Proyecto GD32F450VET6 + RM68120 (EXMC)

Proyecto verificado: **compila sin errores** con `arm-none-eabi-gcc` (13.2.1)
contra la GD32F4xx Standard Peripheral Library oficial de GigaDevice.

## Contenido

- `CMSIS/` — Core ARM CMSIS + cabeceras/soporte de arranque de GD32F4xx
  (obtenidos de los repos oficiales/comunidad, no escritos a mano).
- `Firmware/` — GD32F4xx Standard Peripheral Library completa (todos los
  periféricos, no solo EXMC, por si los necesitas mas adelante).
- `User/` — `main.c` y el driver `rm68120_exmc.c/.h` con el pinout que
  verificaste por multimetro (RESET=PC9, CS=PD7, RS=PD11, WR=PD5, RD=PD4,
  D0-D15 en puertos D/E).
- `GD32F450VE_FLASH.ld` — Linker script para la variante VET6 (Flash 512K,
  RAM 192K + 64K TCM).
- `Makefile` — build con `arm-none-eabi-gcc`, mas targets `flash`/`erase`
  vía OpenOCD + ST-Link.
- `.vscode/` — tareas de build/flash y config de debug con Cortex-Debug.

## Requisitos en tu maquina Linux

```
sudo apt install gcc-arm-none-eabi openocd
```

(en VSCode instala tambien la extension `marus25.cortex-debug`, ya esta
sugerida en `.vscode/extensions.json`)

## Compilar

```
make
```

Genera `build/firmware.elf`, `.hex` y `.bin`.

## Flashear con tu ST-Link

```
make flash
```

Usa `openocd/gd32f450.cfg` (propio, no el `target/stm32f4x.cfg` de serie de
OpenOCD). Motivo: el ID de silicio que reporta el GD32F450 coincide con el
de STM32F42x/43x en la tabla de deteccion de OpenOCD, que asume flash de
2048KB en DOBLE banco. La VET6 real tiene 512KB en banco UNICO, así que
`openocd/gd32f450.cfg` fuerza el tamaño correcto en vez de dejarlo en
autoprobe. Sin esto veras `Error: checksum mismatch` al verificar tras
programar. Esto se ha probado en este mismo entorno (parseo del script
TCL correcto con OpenOCD 0.12.0), pero sin hardware real conectado aqui
no he podido verificar el `program`/`verify` de extremo a extremo — si
te sigue fallando el checksum, dimelo con el log completo.

## Debug en VSCode

Con la extension Cortex-Debug instalada, F5 lanza la config
"Debug GD32F450 (OpenOCD + ST-Link)" ya preparada en `.vscode/launch.json`.

## IMPORTANTE — lo que SI y lo que NO esta verificado

- El bus EXMC (GPIO, timing base, funciones de comando/dato) usa el pinout
  que confirmaste por continuidad. Alta confianza.
- La secuencia de inicializacion propia del panel RM68120 (registros de
  power/driving/gamma, normalmente 0xB0-0xEF) NO esta incluida con valores
  reales — se ha dejado un TODO en `rm68120_init()` en `rm68120_exmc.c`.
  Esos valores dependen del fabricante concreto de tu panel de 480x800 y
  necesitas el datasheet o el ejemplo de tu proveedor para rellenarlos.
- Los tiempos de `timing_init_struct` en `rm68120_exmc_bus_init()` son de
  partida/conservadores, no verificados contra osciloscopio. Ajustalos si
  ves corrupcion de imagen o el panel no responde.
- El pin de LED en `main.c` (PC13) es un placeholder generico para
  comprobar que el firmware arranca — cambialo al LED real de tu placa si
  difiere, o quitalo si no tienes uno accesible.
