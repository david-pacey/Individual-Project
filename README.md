Embedded C++ firmware for a wearable physiotherapy feedback device. Monitors joint movement in real time across three flex sensors and delivers proportional haptic feedback when unsafe extension is detected.

Hardware
ComponentDetailsMCUSTM32F401RE Nucleo (84 MHz Cortex-M4)Flex sensors ×3Bend angle measurement via ADC channels 0, 1, 4HM-10 BLE moduleCompanion app communication (USART1, 9600 baud)Vibration motorHaptic feedback output (GPIO PB3)Onboard LEDVisual calibration confirmation (GPIO PA5)Debug serialUSART2, 115200 baud, 8N1

How It Works
Calibration
On startup, all three sensor limits default to 1.0 (full scale). The user calibrates the device by moving their joint to the desired safe maximum position and sending the C command over Bluetooth. The firmware saves the current normalised readings as the personal safe-range limits for each sensor.
Movement detection
The main loop runs at approximately 100 Hz. On each iteration:

All three sensors are read via ADC
The current reading is compared against a 30-sample circular buffer (~300 ms window) to compute a velocity delta
If the delta exceeds the rapid-movement threshold (default 0.2), the motor triggers a solid 1-second buzz
If any sensor reading exceeds its calibrated limit, the motor pulses at a dynamic frequency proportional to the degree of overextension — the further beyond the limit, the faster the pulse
Each new overextension event (rising edge only) increments a counter
A CSV data packet is transmitted over Bluetooth every 50 ms

Bluetooth command protocol
Commands are received as single ASCII characters via interrupt-driven UART (HAL_UART_RxCpltCallback):
CommandActionCCalibrate — save current readings as safe-range limits. LED toggles as confirmation.PPause — suspend detection and silence the motorRResume — re-enable detectionXReset overextension event counter to zero
Bluetooth data format
Transmitted every 50 ms while unpaused:
<angle>,<overextensionCount>\n
For example: 134.52,7
angle maps Sensor 1's normalised reading to a 90°–180° range. overextensionCount is the cumulative number of overextension events since the last reset or power cycle.

Software Structure
Classes
Motor (Core/Src/main.cpp) — GPIO actuator driver with three operating modes:

on() / off() — direct control
trigger(ms) + update() — non-blocking timed activation; must be called every loop iteration
pulse(currentTime, period) — dynamic-period pulsing for proportional overextension feedback

Sensor (Core/Src/main.cpp) — ADC wrapper for on-demand single-channel reads. Exposes:

amplitudeRaw() — raw 12-bit ADC value (0–4095)
amplitudeVolts() — calibrated voltage (0–VDD)
amplitudeNorm() — normalised float (0.0–1.0)

Repository structure
Core/
  Inc/
    main.h                     — peripheral handles and pin definitions
    stm32f4xx_hal_conf.h       — HAL module enable configuration
    stm32f4xx_it.h             — interrupt handler declarations
  Src/
    main.cpp                   — application logic (start here)
    stm32f4xx_it.c             — interrupt handlers
    stm32f4xx_hal_msp.c        — HAL MSP initialisation (CubeMX generated)
    syscalls.c / sysmem.c      — newlib system calls (CubeMX generated)
    system_stm32f4xx.c         — system clock initialisation (CubeMX generated)
cmake/
  CMakeLists.txt               — sources and library definitions
  gcc-arm-none-eabi.cmake      — GCC toolchain configuration
  starm-clang.cmake            — Clang toolchain configuration (alternative)
CMakeLists.txt                 — root build definition
CMakePresets.json              — build preset configuration
startup_stm32f401xe.s          — device startup assembly
STM32F401XX_FLASH.ld           — linker script
Supposed_Final_Success.ioc     — STM32CubeMX project file

Note: The Drivers/ folder is not included in this repository as it contains ST-owned HAL and CMSIS libraries. It must be generated locally before building — see setup instructions below.


Building and Flashing
Prerequisites

STM32CubeIDE 1.13 or later, or a manual arm-none-eabi-gcc toolchain with CMake 3.22+
ST-LINK/V2 programmer (onboard on the Nucleo-F401RE)

Step 1 — Generate the HAL drivers
The Drivers/ folder is not included in this repository. It must be generated once before the project will build:

Open STM32CubeMX (bundled with STM32CubeIDE)
Open Supposed_Final_Success.ioc from the project root
Click Project → Generate Code
CubeMX will populate Drivers/ with the STM32F4xx HAL and CMSIS libraries

Step 2 — Build and flash
Using STM32CubeIDE:

Open STM32CubeIDE → File → Open Projects from File System → select the cloned folder
Build: Project → Build All (Ctrl+B)
Flash and debug: Run → Debug (F11)

Using CMake directly:
bashgit clone <repository-url>
cd <repository-folder>
cmake --preset Debug        # configure (uses CMakePresets.json)
cmake --build build/Debug   # compile
# Flash with STM32CubeProgrammer or openocd
Debug serial output
Connect a serial terminal (e.g. PuTTY, CoolTerm) to the Nucleo's virtual COM port at 115200 baud, 8N1. The firmware prints sensor readings, velocity deltas, calibration limits, and Bluetooth receive events on every loop iteration.
