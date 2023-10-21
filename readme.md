# Summary
The MPMCM is a DIN rail module of the DINFox project. It embeds the following features:
* **Mains measurements**: true RMS voltage, true RMS current, active power, apparent power, power factor and frequency.
* **4 independent current channels**.
* **Linky TIC** input interface.
* **Real time control** of 4 loads (triac).
* **RS485** communication.

# Hardware
The board was designed on **Circuit Maker V2.0**. Hardware documentation and design files are available @ https://circuitmaker.com/Projects/Details/Ludovic-Lesur/MPMCMHW1-0

# Embedded software

## Environment
The embedded software was developed under **Eclipse IDE** version 2019-06 (4.12.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target
The board is based on the **STM32G441CBT6** microcontroller of the STMicroelectronics G4 family. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected target.

## Structure
The project is organized as follow:
* `inc` and `src`: **source code** split in 7 layers:
    * `registers`: MCU **registers** address definition.
    * `peripherals`: internal MCU **peripherals** drivers.
    * `utils`: **utility** functions.
    * `dsp`: Optimized Cortex-M4 **DSP** functions (from ARM).
    * `components`: external **components** drivers.
    * `nodes` : Node **descriptor**.
    * `applicative`: high-level **application** layers.
* `startup`: MCU **startup** code (from ARM).
* `linker`: MCU **linker** script (from ARM).
