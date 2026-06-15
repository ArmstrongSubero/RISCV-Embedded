# RISCV-Embedded

Bare metal C and C++ examples for the CH32V003F4P6 RISC-V microcontroller, built on the Rovari SDK.

Part of the [Rovari](https://rvembedded.com) platform.

## About

The CH32V003F4P6 is a 32-bit RISC-V microcontroller from WCH. QingKe V2A core, 48 MHz, 16 KB flash, 2 KB SRAM, under $0.10 in volume. Full peripheral set: UART, SPI, I2C, 10-bit ADC, timers, DMA, analog comparator.

Every example in this repository uses the Rovari SDK (`#include "rovari.h"`). Each one is provided in both C and C++ style. The C API uses standalone functions. The C++ API wraps the same hardware in lightweight objects (`Gpio`, `Pwm`, `Uart`, `Spi`, `I2c`, `Adc`, `Servo`, `Oled`). Both produce identical binary output.

## Examples

| # | Project | Description |
|---|---------|-------------|
| 00 | Blink | GPIO output, LED toggle |
| 01 | Dual LEDs | Source and sink current on a single pin |
| 02 | Pushbutton | Digital input with internal pull-up, debounced read |
| 03 | Millis | Non-blocking timing with millis() |
| 04 | Button Interrupt | Hardware interrupt on falling edge |
| 05 | PWM | LED fade using hardware PWM on TIM1 |
| 06 | Tone Gen | Musical note playback on a piezo buzzer |
| 07 | Sound Effects | Built-in sound library: beep, alarm, siren, melody |
| 08 | UART | Serial echo with uart_read_line() |
| 09 | ADC | Analog read with raw and millivolt output |
| 10 | Analog RW | Potentiometer controlling LED brightness via ADC and PWM |
| 11 | Smoothing | Running average filter on ADC readings |
| 12 | Calibration | Auto-calibrate sensor range, map to PWM output |
| 13 | Timer Interrupt | Periodic callback using timer_start() |
| 14 | Watchdog Timer | IWDG setup with optional feed |
| 15 | SPI | MCP41010 digital potentiometer over SPI |
| 16 | I2C | 24LC16B EEPROM write/read verification |
| 17 | DMA Mem2Mem | Memory to memory block copy via DMA |
| 18 | DMA ADC | Continuous ADC sampling into a buffer via DMA |
| 19 | Comparator | Analog comparator with UART output |
| 20 | Sine Wave | DMA-driven sine wave generation through PWM |
| 21 | Servo | Sweep and smooth S-curve servo motion |
| 22 | Servo Knob | Potentiometer controlling servo position |
| 23 | DC Motor | Bidirectional DC motor with speed ramp via L293D |
| 24 | Display | SSD1306 128x32 OLED over I2C |
| 25 | SD Card | Data logger writing CSV to microSD over SPI |

Each project folder contains `app.rova` (your code), `rovari.blocks` (Guvari visual blocks), `rovari.py` (Python companion script), and the full build output.

## Requirements

**Hardware:**
- CH32V003F4P6 board or bare chip on a breadboard
- WCH-LinkE programmer
- USB cable
- Components as listed in each example header

**Software:**
- [Rovari Studio](https://rvembedded.com/products/rovari-studio/) (bundled toolchain, drivers, and SDK)

## Build and Flash

1. Clone the repo:
    ```
    git clone https://github.com/ArmstrongSubero/RISCV-Embedded.git
    ```

2. Open Rovari Studio, select CH32V003 as target, open the project folder.

3. Press Run. The IDE compiles, links, and flashes in one step.

## Project Structure

```
00_Blink/
    app.rova            # Your C/C++ source
    rovari.blocks       # Guvari visual block file
    rovari.py           # Python companion script
    data/config.json    # Peripheral and library config
    src/                # Interrupt handlers
    system/             # Linker script, startup, build output
```

## Arduino Migration

If you are coming from Arduino, see the [Arduino to RISC-V migration guide](https://rvembedded.com/guides/arduino-migration/) on the website.

## Legacy Examples

The `Legacy_WCH_SPL/` folder contains the original bare metal examples written directly against the WCH standard peripheral library without the Rovari SDK. These cover additional topics including software UART, NFC (PN532), GPS, IMU (roll/pitch/yaw), nRF24L01 wireless, function generator, and RTC. They compile with MounRiver Studio.

## License

Apache License 2.0. See [LICENSE](LICENSE).

## Contact

- Website: [rvembedded.com](https://rvembedded.com)
- GitHub: [ArmstrongSubero](https://github.com/ArmstrongSubero)
- Email: armstrongsubero@gmail.com
