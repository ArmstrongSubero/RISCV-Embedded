> From `digitalWrite()` to bare-metal registers — a hands-on lab series for Arduino developers moving to RISC-V

# Arduino to RISC-V: CH32V003 Lab Guide

This folder contains **20 self-contained C projects** and a **116-page lab manual** that teach you how to program the WCH CH32V003F4P6 RISC-V microcontroller by building on what you already know from Arduino. Each `.c` file maps to one or more labs in the guide and covers a specific peripheral or concept.

Part of the [rvembedded.com](https://rvembedded.com) ecosystem for RISC-V embedded systems education.

---

## 📖 Start Here — The Lab Guide PDF

**[RV_Embedded_Arduino_To_RISC_V_Lab_Guide.pdf](RV_Embedded_Arduino_To_RISC_V_Lab_Guide.pdf)** is the companion document for everything in this folder. It walks you through the underlying electronics and theory behind each project, shows you the schematics and circuits to build, and explains the code line by line. The `.c` files in this folder are the finished source listings from the guide you are meant to read the PDF alongside the code.

The guide is structured so that if you can use `digitalRead()`, `analogWrite()`, or `Serial.begin()`, you already know the *concept* the PDF just shows you how to do the same thing at the register level on RISC-V.

---

## Why Move from Arduino to RISC-V?

Arduino hides the hardware behind abstraction layers like `pinMode()`, `digitalWrite()`, and `analogRead()`. This guide strips those layers away and teaches you to work directly with the CH32V003's registers and peripherals giving you full control of the silicon while keeping the learning curve manageable.

| | CH32V003F4P6 | ATmega328P (Arduino UNO) |
|---|---|---|
| **Architecture** | 32-bit RISC-V (RV32EC) | 8-bit AVR |
| **Clock Speed** | 48 MHz | 16 MHz |
| **Flash / SRAM** | 16 KB / 2 KB | 32 KB / 2 KB |
| **Peripherals** | UART, SPI, I2C, ADC, DMA, Op-Amp, Comparator | UART, SPI, I2C, ADC |
| **Unit Cost** | ~$0.10 | ~$2.50 |
| **Programming** | WCH-LinkE (external) | USB bootloader (built-in) |

Despite having half the flash, the CH32V003's 32-bit compressed instructions and wider registers mean you often need fewer instructions per task. In practice, 16 KB on this device goes further than you might expect.

---

## What You Need

### Hardware

- **CH32V003F4P6** microcontroller (TSSOP-20 package)
- **WCH-LinkE** programmer/debugger — low-cost, available directly from WCH
- Breadboard, jumper wires, and a 3.3V or 5V power supply (any jellybean regulator works)
- Basic components: LEDs, resistors (330Ω, 1kΩ, 10kΩ), pushbuttons, 2N3904/2N3906 transistors
- For communication labs: MCP41010 (SPI), 24LC16B EEPROM (I2C)

### Software

- **[MounRiver Studio II](http://www.mounriver.com/)** — the recommended IDE (Eclipse-based, includes the RISC-V GCC toolchain and flash utility)
- The **CH32V003F4P6.zip** project template included in this folder

---

## Project Files

Each `.c` file corresponds to labs in the PDF guide. The Arduino function equivalent is shown so you can find the topic you're looking for.

| File | Topic | Arduino Equivalent | PDF Labs |
|------|-------|--------------------|----------|
| `00_Blank.c` | Minimum super-loop template | `setup()` + `loop()` | #2 – #4 |
| `01_Blink.c` | Blink an LED on PD0 | `digitalWrite()`, `delay()` | #8 |
| `02_Dual_LEDs.c` | Two LEDs on one pin (sink/source) | — | #9 |
| `03_Bidirectional_Knightrider.c` | LED sweep across PORTC | — | #10 |
| `04_Complimentary_LED_Drive.c` | Charlieplexing 6 LEDs on 3 pins | — | #11 |
| `05_Read_Pushbutton.c` | Digital input with pull-up | `digitalRead()` | #18 |
| `06_Pushbutton_On_Off.c` | Toggle with debounce | — | #19 |
| `07_Double_Click_Detection.c` | Timing-based double-click | — | #20 |
| `08_Pushbutton_Long_Press.c` | Hold-to-toggle detection | — | #21 |
| `09_Timer_Interrupt.c` | TIM2 interrupt-driven blink | `attachInterrupt()` | #22 |
| `10_Systick_Timer.c` | SysTick periodic interrupt | `millis()` | #23 |
| `11_Watchdog_Timer.c` | Independent watchdog (IWDG) | `wdt_enable()` | #24 |
| `12_UART.c` | Serial TX/RX with echo | `Serial.begin()` | #25 |
| `13_SPI.c` | SPI master driving MCP41010 | `SPI.begin()` | #26 |
| `14_I2C.c` | I2C master with 24LC16B EEPROM | `Wire.begin()` | #27 |
| `15_PWM.c` | TIM1 PWM output on PD2 | `analogWrite()` | #28 |
| `16_ADC.c` | 10-bit ADC with averaging | `analogRead()` | #29 |
| `17_Comparators.c` | On-chip op-amp as comparator | — | #30 |
| `18_DMA.c` | Memory-to-memory DMA transfer | — | #31 (bonus) |
| `19_Sine_Wave_Generation.c` | DMA + PWM sine wave synthesis | — | #32 (bonus) |

---

## Guide Structure

The PDF is organized into sections that build on each other. Here's the roadmap:

### Part 1 — Understanding the Platform (p. 1–14)
Background on embedded systems, the AVR architecture you're leaving behind, the RISC-V ISA, WCH's QingKe cores, and a side-by-side comparison of the CH32V003 vs the ATmega328P.

### Part 2 — Getting Set Up (p. 15–21)
The two "big changes" from Arduino: using an external programmer (WCH-LinkE) and working in a professional IDE (MounRiver Studio). Labs #1–#4 walk you through connecting hardware, building your first project, understanding the build output, and structuring a super-loop application.

### Part 3 — GPIO Outputs (p. 22–54)
Deep dive into how GPIO actually works — push-pull vs open-drain, pin speeds, current limits, 5V-tolerant pins, and the full GPIO block diagram. Labs #5–#17 cover everything from basic LED blink through Charlieplexing, transistor switching (NPN, PNP, power, MOSFET), optocoupler isolation, and inductive load protection.

### Part 4 — GPIO Inputs (p. 55–69)
Polling, debouncing theory, and practical input techniques. Labs #18–#21 cover basic digital input, toggle buttons, double-click detection, and long-press handling.

### Part 5 — Timers and Interrupts (p. 70–80)
Moving from polling to interrupt-driven design. Labs #22–#24 cover timer interrupts, the SysTick timer, and the independent watchdog.

### Part 6 — Communication Protocols (p. 81–97)
Serial vs parallel communication, then hands-on with the three main protocols. Labs #25–#27 cover UART, SPI (with a digital potentiometer), and I2C (with an EEPROM).

### Part 7 — Analog and Advanced Peripherals (p. 98–116)
PWM generation, ADC reading, the on-chip comparator, DMA transfers, and DMA-driven sine wave synthesis. Labs #28–#32 round out the guide.

---

## Quick Start

1. **Install MounRiver Studio II** from [mounriver.com](http://www.mounriver.com/)
2. **Extract `CH32V003F4P6.zip`** this is the preconfigured project template
3. **Open the PDF** and follow Lab #1 to wire up your CH32V003 and WCH-LinkE
4. **Replace `main.c`** in the template project with any `.c` file from this folder
5. **Build and flash** click the build button, then program via the WCH-LinkE

> **Tip:** Start with `00_Blank.c` to verify your toolchain works, then move to `01_Blink.c` to confirm your hardware is wired correctly. After that, follow the PDF sequentially or jump to whichever peripheral you need.

---

## Other Files in This Folder

| File | Description |
|------|-------------|
| `CH32V003F4P6.zip` | MounRiver Studio project template — extract and use as your base project |
| `RV_Embedded_Arduino_To_RISC_V_Lab_Guide.pdf` | The 116-page companion lab manual (32 labs) |
| `Readme.txt` | Original readme notes |

---

## FAQ

**Do I need to read the entire PDF before writing code?**
No. The PDF is designed so you can jump to any lab that interests you. That said, Labs #1–#4 (setup and super-loop basics) are recommended for everyone since they cover the project structure all other labs build on.

**Can I use PlatformIO or the command-line toolchain instead of MounRiver Studio II?**
Yes, but MounRiver Studio II is the path of least resistance as it bundles the compiler, flasher, and debug support. The `.c` files themselves are IDE-agnostic.

**What if I don't have a specific component for a lab?**
The guide was written to use common "jellybean" parts available worldwide. The only specialized hardware you truly need is the CH32V003 itself and the WCH-LinkE programmer. Most labs use basic LEDs, resistors, and pushbuttons.

**How does this relate to the other projects in the parent repository?**
This folder is the starting point. Once you're comfortable with the peripherals covered here, the other project folders in the repository build on these foundations with more advanced applications and sensor integrations.

---

## Resources

- [rvembedded.com](https://rvembedded.com) — tutorials, documentation, and more RISC-V content
- [WCH Official Site](https://www.wch-ic.com/) — datasheets, reference manuals, and tools
- [MounRiver Studio](http://www.mounriver.com/) — IDE download
- [CH32V003 Reference Manual](https://www.wch-ic.com/products/CH32V003.html) — full peripheral documentation
- [RISC-V ISA Specification](https://riscv.org/technical/specifications/) — for going deeper into the architecture

---

## License

This project is licensed under the MIT License. See the [LICENSE](../LICENSE) file in the repository root for details.

## Author

**Armstrong Subero** — RV Embedded 

For more RISC-V embedded content, visit **[rvembedded.com](https://rvembedded.com)**.
