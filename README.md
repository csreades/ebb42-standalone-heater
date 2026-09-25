# EBB42 v1.2 standalone heater controller

Bare-metal firmware that turns a BIGTREETECH EBB42 CAN v1.2 toolhead board
into a self-contained temperature controller: it holds the thermistor at a
fixed setpoint (default 80 °C) with no host, no CAN, and no USB needed after
flashing.

## Hardware assumptions

| Function   | Pin  | Notes                                                   |
|------------|------|---------------------------------------------------------|
| Heater     | PB13 | HE0 MOSFET output, active high (same as Klipper config) |
| Thermistor | PA3  | TH0 input, 4.7 kΩ pull-up, EPCOS 100K B57560G104F table |
| MCU        | STM32G0B1CBT6, run from internal HSI16 at 16 MHz            |

* The heater MOSFET switches the board's **24 V input**. USB only powers the
  MCU. For standalone use, feed 24 V/GND into the power/CAN connector and
  leave USB unplugged.
* Any "standard" 100 kΩ NTC (3950 type) will read within a couple of degrees
  of the EPCOS table around 80 °C. Edit `THERM_*` in `src/main.c` if you want
  a different table.

## Behaviour

* PID control (Klipper-style gains, output on 0..255 scale), 250 ms loop,
  100 ms time-proportioning window on the heater pin.
  Set `CONTROL_MODE_PID 0` for simple bang-bang with ±1 °C hysteresis.
* Safety: the same checks Klipper applies to a toolhead heater, all of which
  latch the heater **off until reset / power cycle** (see table below).
* Heater is held low from the first instruction and stays off for 1 s at
  boot while the ADC filter settles.

## Protection, compared with Klipper

| Klipper check                          | Here                                                                 |
|----------------------------------------|----------------------------------------------------------------------|
| `min_temp` / `max_temp`                | 0 °C / 130 °C on the filtered reading (`MIN_TEMP_C`, `MAX_TEMP_C`)   |
| MCU `adc_range` (raw sample window, 4 consecutive misses) | same: raw 5..4090 counts, 4 misses -> `FAULT_ADC_OPEN/SHORT` |
| `verify_heater` heating gain           | must rise `VERIFY_GAIN_C` (1.5 °C) every `VERIFY_TIME_MS` (45 s)      |
| `verify_heater` `max_error`            | 120 °C·s below target-5 °C while at temp -> `FAULT_HEATER_NOT_MAINTAINING_TEMP` |
| `verify_heater` `hysteresis`           | 5 °C                                                                 |
| overshoot                              | target + 25 °C -> fault (Klipper relies on `max_temp` for this)      |
| heater `max_duration` (MCU turns PWM off if host stops refreshing) | SysTick forces the pin low if the control loop has not run for 1 s |
| MCU watchdog                           | independent watchdog, 2 s                                            |
| heater off at boot / on shutdown       | pin driven low before it becomes an output; gate has a 20k pull-down; PWM window only opens after the first control update |
| `max_power`                            | `MAX_POWER`                                                          |

Klipper's hotend defaults are 2 °C gain in 20 s; this firmware ships looser
(1.5 °C in 45 s) because the heater is unknown. Tighten `VERIFY_*` once you
know how fast yours responds.

## USB telemetry

While USB is connected the board enumerates as a CDC serial port
(`/dev/serial/by-id/usb-EBB42_standalone_*`, any baud rate) and prints one
line per second:

```
t=21s temp=20.66C raw=3942 duty=100% target=80.0 en=1 oled=0 state=HEATING
```

`state` is `STARTUP`, `HEATING`, `AT_TEMP`, `STANDBY` or a `FAULT_*` reason.
Run `./monitor.sh` to watch it. Single-character commands:

| Key | Action                                              |
|-----|-----------------------------------------------------|
| `+` / `-` | setpoint up / down 1 °C                       |
| `e` | toggle heater on/off                                |
| `r` | reset the MCU (clears a latched fault)              |
| `b` | reboot into the ROM DFU bootloader (used by flash.sh) |

To send one: `printf 'e' > /dev/ttyACM0`.

## Optional OLED and buttons

Any 128x64 SSD1306 I2C module (0.96", address 0x3C) plugs into the **I2C**
header; set `-DOLED_SH1106=1` in the Makefile for 1.3" SH1106 modules. The
firmware probes for the display every 2 s, so it works with or without one.

| I2C header pin | Signal | MCU pin | OLED pin |
|----------------|--------|---------|----------|
| 1              | 5 V    | -       | VCC      |
| 2              | GND    | -       | GND      |
| 3              | SCL    | PB3     | SCL      |
| 4              | SDA    | PB4     | SDA      |

PB3/PB4 are 5 V tolerant ("FT") pins, so a 5 V-powered module with its
pull-ups to 5 V is fine. Check your module's pin order; many are GND-VCC-SCL-SDA.

Three momentary buttons to GND on the **Endstop** header give local control
(the board already has 10k pull-ups and 1k series resistors on these):

| Endstop header pin | Signal | MCU pin | Function                              |
|--------------------|--------|---------|---------------------------------------|
| 3                  | Stop1  | PB5     | setpoint up (hold to repeat)          |
| 2                  | Stop2  | PB6     | setpoint down (hold to repeat)        |
| 1                  | Stop3  | PB7     | heater on/off; when faulted: reset    |
| 4                  | GND    | -       | common for all three                  |

Setpoint range is 20..100 °C in 1 °C steps. Setpoint and on/off state are
saved to the last flash page 3 s after the last change and restored at boot.

Screen layout: setpoint and status on the top line, temperature large in the
middle, heater power bar at the bottom. A fault shows `FAULT` with the reason.

## Status LED

The blue "Status" LED (PA13, shared with SWDIO) shows the controller state so
the board is readable without USB:

| Pattern                  | Meaning                                  |
|--------------------------|------------------------------------------|
| slow blink (1 s period)  | heating, more than 5 °C below target     |
| solid on                 | at temperature                           |
| fast blink (5 Hz)        | fault latched, heater off; reset to clear|
| short blip every 2 s     | heater disabled (standby)                |

The green LED next to the regulator is a plain 3.3 V power indicator.
Using PA13 as a GPIO disables SWD debugging; DFU flashing is unaffected.

## Build

Requires `gcc-arm-none-eabi` (already installed here). Device headers live in
`vendor/` (STM32 CMSIS device G0 + CMSIS 5 core headers).

```sh
make            # -> build/ebb42_heater.bin / .hex / .elf
```

## Flash (USB DFU)

1. Put the board in DFU mode: hold **BOOT**, tap **RESET** (or plug USB in
   while holding BOOT). `lsusb` shows `0483:df11 STMicroelectronics STM Device
   in DFU Mode`.
2. Flash:

```sh
./flash.sh          # or: make flash
```

If the board is already running this firmware, `flash.sh` sends it the `b`
command so it drops into DFU by itself; no buttons needed. `flash.sh` writes
to 0x08000000 and issues `:leave`, so the board resets straight into the new
firmware. This overwrites whatever was there before (Klipper / Katapult). To
go back, re-enter DFU mode and flash that image.

`dfu-util` and a udev rule (`/etc/udev/rules.d/70-stm32-dfu.rules`, mode 0666
for 0483:df11) are installed on this machine, so no sudo is needed.

## Tuning

Everything user-facing is at the top of `src/main.c`:

* `DEFAULT_TARGET_C` – setpoint used until one is saved from the buttons/USB
* `SETPOINT_MIN_C` / `SETPOINT_MAX_C` – adjustment range
* `PID_KP / PID_KI / PID_KD` – if the heater is much bigger or smaller than a
  ~40 W hotend cartridge you may want to retune, or just switch to bang-bang
* `MAX_POWER` – cap the duty cycle (e.g. 0.5 for a heater that is
  over-powered for the mass it heats)
* `VERIFY_*` – loosen if a slow, large heater trips the verify check

## Getting the source on another PC

```sh
git clone --recursive https://github.com/csreades/ebb42-standalone-heater.git
cd ebb42-standalone-heater && make
```

`vendor/tinyusb` and `vendor/cmsis_device_g0` are git submodules;
`vendor/cmsis_core` is a copy of the ARM CMSIS 5 core headers.
