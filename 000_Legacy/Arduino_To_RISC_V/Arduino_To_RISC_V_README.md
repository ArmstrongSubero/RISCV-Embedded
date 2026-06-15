# Arduino to RISC-V: CH32V003 Lab Guide (Legacy)

20 bare metal C projects and a 116 page lab manual for Arduino developers learning the CH32V003F4P6.

These examples use the WCH standard peripheral library directly (register level code, no abstraction layer). For the current Rovari SDK examples with `#include "rovari.h"`, see the main repository root.

Part of the [Rovari](https://rvembedded.com) platform.

## The Lab Guide PDF

**RV_Embedded_Arduino_To_RISC_V_Lab_Guide.pdf** is the companion document for this folder. It covers the electronics, schematics, and theory behind each project and explains the code line by line. Read the PDF alongside the `.c` files.

The guide is structured around Arduino concepts you already know. If you can use `digitalRead()`, `analogWrite()`, or `Serial.begin()`, the PDF shows you how to do the same thing at the register level on RISC-V.

## CH32V003 vs ATmega328P

| | CH32V003F4P6 | ATmega328P (Arduino UNO) |
|---|---|---|
| Architecture | 32-bit RISC-V (RV32EC) | 8-bit AVR |
| Clock | 48 MHz | 16 MHz |
| Flash / SRAM | 16 KB / 2 KB | 32 KB / 2 KB |
| Peripherals | UART, SPI, I2C, ADC, DMA, Comparator | UART, SPI, I2C, ADC |
| Unit Cost | ~$0.10 | ~$2.50 |
| Programming | WCH-LinkE (external) | USB bootloader (built-in) |

Despite having half the flash, the 32-bit compressed instruction set means you often need fewer instructions per task. 16 KB goes further than you would expect.

## Hardware

- CH32V003F4P6 (TSSOP-20)
- WCH-LinkE programmer
- Breadboard, jumper wires, 3.3V or 5V supply
- LEDs, resistors (330, 1k, 10k), pushbuttons, 2N3904/2N3906
- For communication labs: MCP41010 (SPI), 24LC16B EEPROM (I2C)

## Software

These legacy examples were built with **MounRiver Studio** and the WCH RISC-V GCC toolchain. Extract the included `CH32V003F4P6.zip` project template as your base.

For new projects, use [Rovari Studio](https://rvembedded.com/products/rovari-studio/) with the Rovari SDK instead.

## Project Files

| File | Topic | Arduino Equivalent | Labs |
|------|-------|--------------------|------|
| `00_Blank.c` | Minimum super-loop template | `setup()` + `loop()` | 2, 3, 4 |
| `01_Blink.c` | Blink an LED on PD0 | `digitalWrite()`, `delay()` | 8 |
| `02_Dual_LEDs.c` | Two LEDs on one pin (sink/source) | | 9 |
| `03_Bidirectional_Knightrider.c` | LED sweep across PORTC | | 10 |
| `04_Complimentary_LED_Drive.c` | Charlieplexing 6 LEDs on 3 pins | | 11 |
| `05_Read_Pushbutton.c` | Digital input with pull-up | `digitalRead()` | 18 |
| `06_Pushbutton_On_Off.c` | Toggle with debounce | | 19 |
| `07_Double_Click_Detection.c` | Timing-based double-click | | 20 |
| `08_Pushbutton_Long_Press.c` | Hold-to-toggle detection | | 21 |
| `09_Timer_Interrupt.c` | TIM2 interrupt-driven blink | `attachInterrupt()` | 22 |
| `10_Systick_Timer.c` | SysTick periodic interrupt | `millis()` | 23 |
| `11_Watchdog_Timer.c` | Independent watchdog (IWDG) | `wdt_enable()` | 24 |
| `12_UART.c` | Serial TX/RX with echo | `Serial.begin()` | 25 |
| `13_SPI.c` | SPI master driving MCP41010 | `SPI.begin()` | 26 |
| `14_I2C.c` | I2C master with 24LC16B EEPROM | `Wire.begin()` | 27 |
| `15_PWM.c` | TIM1 PWM output on PD2 | `analogWrite()` | 28 |
| `16_ADC.c` | 10-bit ADC with averaging | `analogRead()` | 29 |
| `17_Comparators.c` | On-chip comparator | | 30 |
| `18_DMA.c` | Memory-to-memory DMA transfer | | 31 |
| `19_Sine_Wave_Generation.c` | DMA + PWM sine wave synthesis | | 32 |

## Guide Structure

The PDF builds sequentially across seven parts:

**Part 1 (p. 1 to 14):** Platform background. Embedded systems, AVR vs RISC-V, QingKe cores, CH32V003 vs ATmega328P.

**Part 2 (p. 15 to 21):** Setup. WCH-LinkE wiring, MounRiver Studio, first build, super-loop structure. Labs 1 through 4.

**Part 3 (p. 22 to 54):** GPIO outputs. Push-pull, open-drain, pin speeds, current limits, Charlieplexing, transistor switching, optocouplers, inductive load protection. Labs 5 through 17.

**Part 4 (p. 55 to 69):** GPIO inputs. Polling, debounce theory, toggle, double-click, long-press. Labs 18 through 21.

**Part 5 (p. 70 to 80):** Timers and interrupts. TIM2, SysTick, independent watchdog. Labs 22 through 24.

**Part 6 (p. 81 to 97):** Communication. UART, SPI with MCP41010, I2C with 24LC16B. Labs 25 through 27.

**Part 7 (p. 98 to 116):** Analog and advanced. PWM, ADC, comparator, DMA, DMA-driven sine wave. Labs 28 through 32.

## Quick Start

1. Install MounRiver Studio from [mounriver.com](http://www.mounriver.com/)
2. Extract `CH32V003F4P6.zip` (the preconfigured project template)
3. Open the PDF, follow Lab 1 to wire up hardware
4. Replace `main.c` in the template with any `.c` file from this folder
5. Build and flash via WCH-LinkE

Start with `00_Blank.c` to verify the toolchain, then `01_Blink.c` to confirm wiring. After that, follow the PDF in order or jump to the peripheral you need.

## Other Files

| File | Description |
|------|-------------|
| `CH32V003F4P6.zip` | MounRiver Studio project template |
| `RV_Embedded_Arduino_To_RISC_V_Lab_Guide.pdf` | 116 page lab manual (32 labs) |

## License

Apache License 2.0. See [LICENSE](../LICENSE) in the repository root.

## Contact

- Website: [rvembedded.com](https://rvembedded.com)
- GitHub: [ArmstrongSubero](https://github.com/ArmstrongSubero)
- Email: armstrongsubero@gmail.com
