# RISCV-Embedded
> Bare-Metal Libraries and Projects for the CH32V003F4P6 RISC-V Microcontroller written in C/C++!

## CH32V003 Projects

Welcome to the **CH32V003 Projects** repository! This repository contains a collection of bare-metal libraries, projects, and code examples for the WCH **CH32V003F4P6** RISC-V microcontroller. Whether you're transitioning from Arduino or looking to dive deep into RISC-V embedded development, this repository has you covered.

Part of the **[rvembedded.com](https://rvembedded.com)** ecosystem for RISC-V embedded systems education and development.

## Table of Contents
- [Introduction](#introduction)
- [Arduino to RISC-V Guide](#arduino-to-risc-v-guide)
- [Project List](#project-list)
- [Getting Started](#getting-started)
- [Requirements](#requirements)
- [How to Use](#how-to-use)
- [Contributing](#contributing)
- [License](#license)

## Introduction

The CH32V003F4P6 is an ultra-low-cost 32-bit RISC-V microcontroller from WCH, featuring a QingKe V2A core running at up to 48 MHz with 16 KB flash and 2 KB SRAM. Despite its tiny footprint and price point, it packs a surprising number of peripherals including UART, SPI, I2C, ADC, timers, and DMA.

This repository provides bare-metal C drivers and example projects that demonstrate the full range of the CH32V003's capabilities — from basic GPIO and timer operations through communication protocols, sensor interfaces, waveform generation, and beyond.

## Arduino to RISC-V Guide

New to RISC-V? Coming from an Arduino background? The **`Arduino_To_RISC_V`** folder is designed specifically for you. It includes a comprehensive **PDF guide with over 30 hands-on labs** that walk you through the transition from Arduino-style development to bare-metal RISC-V programming on the CH32V003.

The guide covers fundamental concepts and progressively builds your skills, making the jump from the Arduino ecosystem to RISC-V development as smooth as possible.

## Project List

| # | File | Description |
|---|------|-------------|
| — | **Arduino_To_RISC_V/** | Beginner's guide with 30+ lab PDF for transitioning from Arduino to RISC-V |
| 01 | **01_Blink.c** | Basic LED blink |
| 02 | **02_Dual_Pin_Drive.c** | Driving two GPIO pins simultaneously |
| 03 | **03_Complementary_Drive_Circuit.c** | Complementary push-pull drive circuit |
| 04 | **04_Bidirectional_Knightrider.c** | Bidirectional Knight Rider LED effect |
| 05 | **05_Read_Pushbutton.c** | Reading a pushbutton input |
| 06 | **06_Pushbutton_Debounce.c** | Software debouncing for pushbuttons |
| 07 | **07_Double_Click_Detection.c** | Detecting double-click button presses |
| 08 | **08_Pushbutton_Long_Press.c** | Long press detection |
| 09 | **09_Timer_Interrupt.c** | Timer interrupt configuration |
| 10 | **10_Smart_Button.c** | Smart button with multiple input modes |
| 11 | **11_Pushbutton_Int_Push_On_Push_Off.c** | Interrupt-driven push on/push off toggle |
| 12 | **12_Pulse_Width_Modulation.c** | PWM signal generation |
| 13 | **13_Systick.c** | SysTick timer usage |
| 14 | **14_Watchdog_Timer.c** | Watchdog timer implementation |
| 15 | **15_UART_Debug_Port.c** | UART debug port setup |
| 16 | **16_UART_Echo_Add.c** | UART echo with data manipulation |
| 17 | **17_SPI.c** | SPI communication |
| 18 | **18_I2C.c** | I2C communication |
| 19 | **19_RGB_LED.c** | RGB LED control |
| 20 | **20_OLED.c** | OLED display driver (SSD1306) |
| 21 | **21_ADC.c** | Analog-to-Digital Conversion |
| 22 | **22_Comparator.c** | Analog comparator usage |
| 23 | **23_One_Wire_Protocol.c** | One-Wire protocol implementation |
| 24 | **24_Sound_Library.c** | Sound generation library |
| 25 | **25_Direct_Memory_Access.c** | DMA transfers |
| 26 | **26_LRSR_PRNG.c** | LFSR-based pseudo-random number generator |
| 30 | **30_DC_Motor.c** | DC motor control |
| 31 | **31_Servo_Library.c** | Servo motor control library |
| 33 | **33_Temp_Sensor.c** | Temperature sensor interface |
| 34 | **34_Ultrasonic_Library.c** | Ultrasonic distance sensor (HC-SR04) library |
| 35 | **35_Software_UART.c** | Software UART implementation |
| 36 | **36_RTC.c** | Real-Time Clock |
| 37 | **37_Ultraviolet.c** | UV sensor interface |
| 38 | **38_nRF24L01_ISM.c** | nRF24L01 ISM band wireless communication |
| 50 | **50_NFC_Library.c** | NFC library (PN532) |
| 51 | **51_GPS_Library.c** | GPS module library |
| 52 | **52_Accel_Gyro.c** | Accelerometer and gyroscope interface |
| 53 | **53_Inclinometer.c** | Inclinometer implementation |
| 54 | **54_Roll_Pitch_Yaw.c** | Roll, pitch, and yaw calculations |
| 55 | **55_Sine_Wave.c** | Sine wave generation |
| 56 | **56_Triangle_Wave.c** | Triangle wave generation |
| 57 | **57_Adj_Sine.c** | Adjustable sine wave output |
| 58 | **58_Func_Gen.c** | Function generator |

## Getting Started

### Prerequisites

To work with the projects in this repository, you will need:

- A **CH32V003F4P6** development board or breakout.
- **MounRiver Studio** IDE or the WCH toolchain for RISC-V.
- **WCH-LinkE** programmer/debugger (or compatible).
- Basic electronic components: breadboard, resistors, LEDs, sensors, etc.

### Setting Up

1. Clone the repository to your local machine:
    ```sh
    git clone https://github.com/ArmstrongSubero/CH32V003-Projects.git
    cd CH32V003-Projects
    ```

2. If you're new to RISC-V, start with the `Arduino_To_RISC_V` folder and follow the PDF guide.
3. Open the desired project file in MounRiver Studio or your preferred editor.
4. Compile and flash to your CH32V003 using the WCH-LinkE.

## Requirements

- **Hardware**: CH32V003F4P6 development board, WCH-LinkE programmer, breadboard, resistors, capacitors, LEDs, sensors, etc.
- **Software**: MounRiver Studio IDE (or WCH RISC-V toolchain).
- **Programmer**: WCH-LinkE or compatible debugger.

## How to Use

- Each `.c` file is a self-contained project demonstrating a specific peripheral or concept.
- If you are coming from Arduino, start with the **Arduino_To_RISC_V** folder which includes a PDF guide with over 30 labs to get you up to speed.
- Review the comments within each source file for pin assignments and configuration details.
- Build and flash the project onto your CH32V003F4P6.

### Example: Running the Blink Project

1. Connect an LED with a resistor to the GPIO pin specified in `01_Blink.c`.
2. Open the file in MounRiver Studio.
3. Build and flash the project to your CH32V003F4P6 via WCH-LinkE.
4. Observe the LED blinking on your board.

## Contributing

Contributions are welcome! If you have a new project, improvements, or bug fixes
plse reach out to me!

Feel free to suggest new projects or improvements to existing ones!

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Contact

For more RISC-V embedded content and resources, visit **[rvembedded.com](https://rvembedded.com)**.

- **GitHub**: [ArmstrongSubero](https://github.com/ArmstrongSubero)
- **Email**: [armstrongsubero@gmail.com](mailto:armstrongsubero@gmail.com)
- **Website**: [rvembedded.com](https://rvembedded.com)

If you find this repository helpful, please give it a star ⭐ and share it with others interested in RISC-V embedded development!
