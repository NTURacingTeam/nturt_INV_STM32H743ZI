# NTURT INV STM32H743ZI

Firmware for the NTU Racing Team inverter main board based on STM32H743ZI microcontroller.

## Overview

This repository contains the embedded software for controlling the inverter system in the vehicle's powertrain. It handles motor control, power management, and communication protocols.

## Compatible Hardware

- **Main Board**: INV_MainBoard_STM32 V2.1
- **Microcontroller**: STM32H743ZI

For hardware schematics and PCB design, see: [nturt_kicad - INV_MainBoard_STM32](https://github.com/NTURacingTeam/nturt_kicad/tree/main/EP6/Powertrain_Group/INV_MainBoard_STM32)

## Requirements

- STM32 for VSCode
- ARM GCC Compiler
- STM32H7 HAL Libraries

## Getting Started

1. Clone this repository
2. Adjust `.stm32env` file to match your environment configuration
3. Import the `.ioc` file to STM32CubeMX and generate code
4. change Core/Src/tim.c line 51 to htim1.Init.Period = CCR;
4. Build and run using STM32 for VSCode

## .stm32env example code

```
# .stm32env
ARM_GCC_PATH = $HOME\AppData\Roaming\Code\User\globalStorage\bmd.stm32-for-vscode\@xpack-dev-tools\arm-none-eabi-gcc\14.2.1-1.1.1\.content\bin
OPENOCD = $HOME\AppData\Roaming\Code\User\globalStorage\bmd.stm32-for-vscode\@xpack-dev-tools\openocd\0.12.0-6.1\.content\bin\openocd.EXE
```

## License

NTU Racing Team