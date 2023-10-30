# Description

The **MPMCM** is a DIN rail module of the **DINFox** project. It embeds the following features:

* **Mains measurements**: true RMS voltage, true RMS current, active power, apparent power, power factor and frequency.
* **4 independent current channels**.
* **Real time control** of 4 loads (triac).
* **Linky TIC** input interface.
* **RS485** communication.

# Hardware

The board was designed on **Circuit Maker V2.0**. Below is the list of hardware revisions:

| Hardware revision | Description | Status |
|:---:|:---:|:---:|
| [MPMCM HW1.0](https://365.altium.com/files/DD635FDD-1D00-456C-9219-78701675DC01) | Initial version. | :white_check_mark: |

# Embedded software

## Environment

The embedded software was developed under **Eclipse IDE** version 2019-06 (4.12.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target

The board is based on the **STM32G441CBT6** microcontroller of the STMicroelectronics G4 family. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected board version.

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

## Dependencies

The `inc/dinfox` and `src/dinfox` folders of the [XM project](https://github.com/Ludovic-Lesur/xm) must linked to the MPMCM project, as they contain common data definition related to the DINFox system.
