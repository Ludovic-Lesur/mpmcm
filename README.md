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

The embedded software is developed under **Eclipse IDE** version 2024-09 (4.33.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

> [!WARNING]
> To compile any version under `sw5.0`, the `git_version.sh` script must be patched when `sscanf` function is called: the `SW` prefix must be replaced by `sw` since Git tags have been renamed in this way.

## Target

The board is based on the **STM32G441CBT6** microcontroller of the STMicroelectronics G4 family. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected board version.

## Structure

The project is organized as follow:

* `drivers` :
    * `device` : MCU **startup** code and **linker** script.
    * `registers` : MCU **registers** address definition.
    * `peripherals` : internal MCU **peripherals** drivers.
    * `mac` : **medium access control** driver.
    * `components` : external **components** drivers.
    * `utils` : **utility** functions.
* `middleware` :
    * `analog` : High level **analog measurements** driver.
    * `node` : **UNA** nodes interface implementation.
    * `power` : Board **power tree** manager.
* `application` : Main **application**.
