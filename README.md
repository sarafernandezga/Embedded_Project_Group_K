# FINAL PROJECT - Implementation of the Embedded Platform for a Drone Tracking and Telemetry IoT System using the ARM-based Microcontroller STM32WL55JC and Zephyr OS

# Table of Contents

1. [Introduction](#1-introduction)
   1. [System Overview](#11-system-overview)
   2. [Sensor Monitoring and Operating States](#12-sensor-monitoring-and-operating-states)
   3. [Operating Modes](#13-operating-modes)
      1. [TEST Mode](#test-mode)
      2. [NORMAL Mode](#normal-mode)
      3. [SIMU Mode](#simu-mode)
2. [Visual Studio Code Configuration](#2-visual-studio-code-configuration)
   1. [Configuration Files](#21-configuration-files)
   2. [Requirements](#22-requirements)
   3. [Environment Variables](#23-environment-variables)
   4. [Configure Environment Variables](#24-configure-environment-variables)
   5. [Available Tasks](#25-available-tasks)
   6. [Build and Debug Workflow](#26-build-and-debug-workflow)
3. [System Architecture](#3-system-architecture)
   1. [System Overview](#31-system-overview)
   2. [Hardware Connections](#32-hardware-connections)
   3. [Software Architecture](#33-software-architecture)
4. [Firmware Implementation](#4-firmware-implementation)
   1. [Operating Modes, Tracker States and Data Reporting](#41-operating-modes-tracker-states-and-data-reporting)
      1. [Operating Mode Management](#411-operating-mode-management)
      2. [Tracker States](#412-tracker-states)
      3. [Main Application Data Handling](#413-main-application-data-handling)
      4. [Mode Functionality](#414-mode-functionality)
   2. [Sensor Acquisition and Processing](#42-sensor-acquisition-and-processing)
      1. [MPU6050 – Inertial Measurement Unit (IMU)](#421-mpu6050--inertial-measurement-unit-imu)
      2. [GPS – Global Positioning System](#422-gps--global-positioning-system)
      3. [Environmental Sensors – Si7021 and LDR](#423-environmental-sensors--si7021-and-ldr)
5. [Power Management](#5-power-management)
   1. [Power Management Configuration](#51-power-management-configuration)
   2. [Hardware Configuration](#52-hardware-configuration)
   3. [GPS Power Management Implementation](#53-gps-power-management-implementation)
   4. [RGB LED Power Optimization](#54-rgb-led-power-optimization)
   5. [Final Operating Configuration and Limitations](#55-final-operating-configuration-and-limitations)


# 1. Introduction

## 1.1. System Overview

The objective of this project is to develop an embedded IoT system capable of monitoring and tracking a drone or drone-like mobile platform. The system is based on the STM32WL55JC development board and Zephyr RTOS, and acts as an independent monitoring and telemetry unit installed on the drone.

The platform collects information from several sensors in order to obtain an overview of the drone's position, movement, and surrounding environmental conditions. The main components used for this purpose are a GPS module, an Inertial Measurement Unit (IMU), a temperature and relative humidity sensor (Si7021), and a light-dependent resistor (LDR).

The measurements collected by these sensors are processed by the embedded application to determine the operating condition of the drone. Based on the available information, the system classifies its status into one of three possible states: **IDLE**, **FLYING**, or **PROBLEM**. These states provide a simple representation of whether the drone is stationary, performing movement or flight-related activity, or experiencing an abnormal condition that requires attention.

The system also includes an RGB LED to provide local visual feedback about its operating mode or global status, depending on the selected configuration.

## 1.2. Sensor Monitoring and Operating States

Each sensor provides complementary information about the drone and its environment.

The GPS module provides global positioning information, including latitude, longitude, altitude, satellite availability, and UTC time when a valid fix is available. This information allows the system to track the drone's geographical position and timestamp its measurements.

The MPU6050 IMU measures acceleration along the three spatial axes and angular velocity around them. These measurements are used to characterize the drone's movement and estimate its orientation through roll, pitch, and yaw angles.

The Si7021 sensor measures ambient temperature and relative humidity, providing information about the environmental conditions in which the drone is operating. Additionally, the LDR circuit measures ambient light intensity, allowing the system to identify changes in surrounding illumination.

By combining the information obtained from these sensors, the embedded application evaluates the operating condition of the drone and assigns one of the following states:

- **IDLE:** The drone is considered stationary or not performing significant movement.
- **FLYING:** The measurements indicate movement or activity compatible with drone operation.
- **PROBLEM:** An abnormal condition or significant sensor-related problem has been detected.

The classification is based on the available measurements and predefined detection criteria. The system is intended for monitoring purposes only and does not perform any flight-control operation.

## 1.3. Operating Modes

The application provides three main operating modes: **TEST**, **NORMAL**, and **SIMU**. Each mode has a different purpose and defines how measurements are acquired, processed, and transmitted through the serial interface.

### TEST Mode

TEST mode is intended to verify the correct operation of the hardware and sensors. In this mode, the system periodically reads the available sensors and sends a complete diagnostic report through the USB Virtual COM port every 2 seconds.

The report includes the GPS status and position, IMU acceleration and angular velocity, estimated orientation, ambient light level, temperature, relative humidity, and the detected operating state. The output is human-readable, making it easier to inspect sensor values and identify possible connection or measurement problems during system testing.

### NORMAL Mode

NORMAL mode represents the standard monitoring operation of the system. In this mode, sensors are sampled at different frequencies according to their characteristics and the requirements of the application. For example, inertial measurements are acquired at a higher frequency, while GPS and environmental measurements are updated less frequently.

A complete telemetry report is transmitted through the serial interface every 5 seconds. In addition, the application calculates statistical information over successive 30-second periods. These statistics include the minimum, maximum, and mean values of selected measurements, such as ambient light, temperature, relative humidity, acceleration magnitude, and angular velocity magnitude.

The purpose of this mode is to provide continuous monitoring of the drone while keeping the acquisition and reporting frequencies appropriate for each sensor.

### SIMU Mode

SIMU mode allows the embedded platform to communicate with an external graphical flight simulator running on a computer. In this mode, the system periodically sends telemetry frames through the USB Virtual COM port using a predefined serial data format.

The simulator receives and interprets these frames to represent the drone's movement and operating conditions graphically. This allows the user to visualize information such as the drone's position, orientation, movement, and environmental measurements.

Unlike TEST and NORMAL modes, the serial output in SIMU mode is intended for automatic processing by the simulator rather than direct human inspection. Therefore, the transmitted frames must follow the specified format, and no additional diagnostic messages can be sent through the same serial interface.

Overall, the three operating modes provide different ways to interact with the system: TEST mode focuses on hardware verification, NORMAL mode provides continuous monitoring and statistical analysis, and SIMU mode connects the embedded platform to an external graphical representation of the drone.

# 2. Visual Studio Code Configuration

The project includes a `.vscode` directory with configuration files for building, flashing, monitoring and debugging the firmware on the **NUCLEO-WL55JC** board.

The configuration files contain **no absolute paths**. The project location is resolved with `${workspaceFolder}`, and the location of each tool is read from Windows user environment variables. Each team member only has to define these variables once on their own machine.

## 2.1. Configuration Files

| File | Description |
|---|---|
| `launch.json` | Configures Cortex-Debug and GDB for debugging the firmware. |
| `settings.json` | Defines the terminal environment variables (`ZEPHYR_TOOLCHAIN_VARIANT` and `PROJECT_DIR`). |
| `tasks.json` | Defines build, flash, serial monitor, cleanup and debug server tasks. |

## 2.2. Requirements

The following tools are required to use the provided VS Code tasks:

- Visual Studio Code with the **Cortex-Debug** extension.
- The Zephyr SDK (1.0.x), including the ARM toolchain, GDB and OpenOCD.
- A Zephyr workspace with the `west` tool installed in its Python virtual environment.
- STM32CubeProgrammer for flashing the firmware.
- PuTTY's `plink.exe` for serial communication.

## 2.3. Environment Variables

The following user environment variables must be defined before using the tasks:

| Variable | Example | Purpose |
|---|---|---|
| `ZEPHYR_SDK_INSTALL_DIR` | `C:\Users\<user>\zephyr-sdk-1.0.1` | Zephyr SDK installation directory (toolchain, GDB, OpenOCD). |
| `ZEPHYR_BASE` | `C:\Users\<user>\zephyrproject\zephyr` | Zephyr source directory inside the west workspace. |
| `ZEPHYR_WORKSPACE` | `C:\Users\<user>\zephyrproject` | Root of the west workspace. Used as the working directory of the build. |
| `WEST_EXE` | `C:\Users\<user>\zephyrproject\.venv\Scripts\west.exe` | `west` executable. |
| `STM32_PROGRAMMER_CLI` | `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe` | STM32CubeProgrammer command-line tool used for flashing. |
| `PLINK_EXE` | `C:\Program Files\PuTTY\plink.exe` | PuTTY command-line client used as serial monitor. |

## 2.4. Configure Enviroment Variables

Open PowerShell, replace `<user>` and the paths with the ones from your machine, and run:

```powershell
[Environment]::SetEnvironmentVariable("ZEPHYR_SDK_INSTALL_DIR", "C:\Users\<user>\zephyr-sdk-1.0.1", "User")
[Environment]::SetEnvironmentVariable("ZEPHYR_BASE", "C:\Users\<user>\zephyrproject\zephyr", "User")
[Environment]::SetEnvironmentVariable("ZEPHYR_WORKSPACE", "C:\Users\<user>\zephyrproject", "User")
[Environment]::SetEnvironmentVariable("WEST_EXE", "C:\Users\<user>\zephyrproject\.venv\Scripts\west.exe", "User")
[Environment]::SetEnvironmentVariable("STM32_PROGRAMMER_CLI", "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe", "User")
[Environment]::SetEnvironmentVariable("PLINK_EXE", "C:\Program Files\PuTTY\plink.exe", "User")
```

The `"User"` scope makes the variables persistent and does not require administrator privileges.

> **Important:** Close Visual Studio Code completely (not only the window) and open it again. VS Code only reads the environment variables when it starts. Otherwise, `${env:...}` will be empty and the tasks will fail.

#### Verifying the configuration

Open a **new** terminal inside VS Code and run:

```powershell
echo $env:ZEPHYR_SDK_INSTALL_DIR
echo $env:ZEPHYR_BASE
echo $env:ZEPHYR_WORKSPACE
echo $env:WEST_EXE
echo $env:STM32_PROGRAMMER_CLI
echo $env:PLINK_EXE

Test-Path "$env:ZEPHYR_SDK_INSTALL_DIR\cmake"      # must return True
& "$env:ZEPHYR_SDK_INSTALL_DIR\gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-gcc.exe" --version
```

All variables must show a value, `Test-Path` must return `True`, and the compiler must print its version.

## 2.5. Available Tasks

The following tasks are available from **Terminal → Run Task** in Visual Studio Code:

| Task | Description |
|---|---|
| `West Build (auto)` | Builds the firmware for the `nucleo_wl55jc` board. |
| `Flash (CubeProgrammer, UR+HWrst)` | Flashes the generated HEX firmware using STM32CubeProgrammer. |
| `List COM Ports` | Lists available Windows serial ports. |
| `Serial Monitor` | Opens a serial connection at 115200 baud. |
| `Clean Build` | Removes the build directory. |
| `Clean Cache` | Removes the project-specific Zephyr cache directory. |
| `Zephyr: Debug Server` | Starts the Zephyr debug server after building the firmware. |
| `Zephyr: Stop Debug Server` | Stops the OpenOCD debug server process. |

### 2.6. Build and Debug Workflow

1. Open the project root directory in Visual Studio Code.
2. Check that the Zephyr environment and tool paths are configured correctly.
3. Run **West Build (auto)** to compile the firmware.
4. Run **Flash (CubeProgrammer, UR+HWrst)** to program the board.
5. Open **Serial Monitor** to view serial output.
6. Start the **Debug Zephyr - NUCLEO-WL55JC** launch configuration to debug the application.

The debug configuration uses the generated ELF file at `build/zephyr/zephyr.elf` and connects to the GDB server on port `3333`.

# 3. System Architecture

## 3.1. System Overview

The proposed drone tracking system is built around the STM32WL55JC development board, which acts as the central processing unit of the embedded platform. The board runs Zephyr RTOS and is responsible for managing the connected sensors, processing the acquired measurements, determining the operating state of the drone, and transmitting telemetry data to an external computer.

The system integrates four main sensing elements: an MPU6050 Inertial Measurement Unit (IMU), a GPS module, a Si7021 temperature and relative humidity sensor, and an LDR-based ambient light sensor. Each component provides different information about the drone and its surrounding environment. Additionally, an RGB LED is used to provide local visual feedback about the operating mode or the detected system state.

The sensors are connected to the STM32WL55JC through their corresponding hardware interfaces. The MPU6050 and Si7021 communicate through the I2C bus, the GPS module uses a UART interface, and the LDR is connected to an analog-to-digital converter (ADC) input. The RGB LED is controlled through GPIO or PWM outputs.

The embedded application processes the measurements collected from these peripherals and generates the corresponding telemetry information. Depending on the selected operating mode, this information is either presented as human-readable diagnostic reports or formatted into structured serial frames for communication with the graphical flight simulator.

Figure X shows the general block diagram of the proposed system, including the main hardware components, their communication interfaces, and the connection to the external computer.


*Figure X. Block diagram of the drone tracking and telemetry system.*

## 3.2. Hardware Connections

The hardware architecture is based on the communication interfaces available on the STM32WL55JC development board. Each sensor or actuator is connected to the microcontroller through the appropriate peripheral interface, allowing the embedded application to acquire measurements and control the system's visual indicators.

The MPU6050 and Si7021 share the same I2C bus, using the SDA and SCL signals for data transmission and clock synchronization. The GPS module communicates with the microcontroller through a UART serial interface, providing positioning and timing information. The LDR circuit is connected to an ADC input, allowing the microcontroller to measure the voltage generated by the light-dependent resistor voltage divider. Finally, the RGB LED uses three independent GPIO or PWM outputs to control its red, green, and blue channels.

Table X summarizes the hardware connections used by the system.

| Component | Interface | Signal | STM32WL55JC pin |
| --- | --- | --- | --- |
| MPU6050 | I2C | SDA | PA11 |
| MPU6050 | I2C | SCL | PA12 |
| Si7021 | I2C | SDA | PA11 |
| Si7021 | I2C | SCL | PA12 |
| GPS | USART1 | TX | PB7 (RX microcontrolador) |
| GPS | USART1 | RX | PB6 (TX microcontrolador) |
| LDR | ADC | Analog input | PB1 (ADC1 Channel 5) |
| RGB LED | GPIO / PWM | Red channel | PB5 |
| RGB LED | GPIO / PWM | Green channel | PC1 |
| RGB LED | GPIO / PWM | Blue channel | PC2 |
| BUTTON |  |  |  |

Both I2C sensors share the same communication lines, while the GPS, LDR, and RGB LED use independent microcontroller peripherals. This configuration allows the different hardware elements to operate as part of a single embedded monitoring platform.

The USB connection provided through the ST-LINK interface is also used to establish the Virtual COM port communication with the external computer. This connection is essential for transmitting diagnostic reports and telemetry frames, particularly when operating in SIMU mode.

## 3.3. Software Architecture

The firmware is structured into independent software modules running under Zephyr RTOS. Each module is responsible for a specific task, while the main application (`main.c`) coordinates the overall system operation.

The architecture separates sensor data acquisition from application-level processing. Sensor modules collect and process their measurements independently, then publish the resulting data to the main application through Zephyr message queues. This allows the main application to access the latest available sensor information without directly managing the acquisition process of each peripheral.

### 3.3.1. Software modules

The firmware is organized into the following main components:

| Module | Main responsibility |
| --- | --- |
| main.c | Coordinates the application, manages operating modes and collects sensor data. |
| imu.c | Acquires motion measurements from the MPU6050. |
| gps.c | Receives and processes GPS data from the PA6H module. |
| enviroment.c | Acquires light, temperature and humidity measurements. |
| normal.c | Determines the tracker state and manages NORMAL mode reporting and state indication. |
| rgb.c | Controls the RGB LED. |
| button.c | Handles user button input. |
| statistics.h | Provides statistical data structures and calculation support. |
| simu.c | Builds and transmits telemetry frames to the external simulator. |

### 3.3.2. Message queues

Communication between modules is implemented using Zephyr message queues (`k_msgq`). Each queue transfers a specific message structure between a producer and a consumer.

The following table summarizes the queues used by the firmware and the information carried by each message structure.

| Queue | Producer → Consumer | Message structure | Main data carried |
| --- | --- | --- | --- |
| imu_q | IMU → Main | struct imu_m | Acceleration (X, Y, Z), angular velocity (X, Y, Z), orientation (roll, pitch, yaw), motion state and measurement status. |
| gps_msgq | GPS → Main | struct gps_data | Latitude, longitude, altitude, satellite information, UTC time, GPS fix and status. |
| env_q | Environmental → Main | struct env_m | Ambient light level, temperature, relative humidity and measurement status. |
| imu_stats_q | IMU → Main | struct imu_stats | Statistical information about acceleration and angular velocity, including mean, minimum, maximum and sample count. |
| env_stats_q | Environmental → Main | struct env_stats | Statistical information about light, temperature and humidity, including mean, minimum, maximum and sample count. |
| simu_msgq | Main → SIMU thread | struct system_messages | Aggregated sensor measurements, validity indicators and protocol flags for simulator transmission. |

The main application combines the sensor messages into `struct system_messages`, which acts as a common container for the latest available system information. In addition to the sensor structures, it contains validity indicators for the IMU, GPS and environmental measurements, as well as additional flags.

The sensor queues are configured to prioritize the latest available measurements rather than storing a history of previous readings. The main application retrieves messages without blocking; if no new message is available, the previously stored data is retained.

### 3.3.3. Synchronization mechanisms

The firmware uses Zephyr events, timers and semaphores to coordinate the execution of its different tasks. These mechanisms are summarized below.

#### Events

The main application uses the `main_events` event object to receive notifications and determine which operations must be executed.

| Event | Trigger / purpose | Main application action |
| --- | --- | --- |
| EVT_BUTTON | User button press. | Changes the operating mode and reconfigures the corresponding timers. |
| EVT_TIMER | Expiration of the main periodic timer. | Retrieves available sensor messages and executes the current mode routine. |
| EVT_DRONE | Notification that the tracker state needs to be updated. | Refreshes the sensor data and updates the tracker state and RGB LED in NORMAL mode. |

#### Timers

Timers are used to schedule periodic operations. 

| Timer | Main purpose |
| --- | --- |
| main_timer | Triggers periodic application execution through EVT_TIMER. |
| imu_timer | Schedules IMU acquisition cycles by releasing the corresponding semaphore. |
| enviroment_timer | Controls the periodic environmental sensor acquisition. |

Their periods are configured according to the selected operating mode.

| Operating mode | GPS update period | Environmental update period | IMU update period | Main application period |
| --- | --- | --- | --- | --- |
| TEST | 2000 ms | 2000 ms | 50 ms | 2000 ms |
| NORMAL | 1000 ms | 2500 ms | 20 ms | 5000 ms |
| SIMU | 1000 ms | 1000 ms | 20 ms | 100 ms |

#### Semaphores

Semaphores synchronize the execution of sensor acquisition threads with their corresponding timer callbacks.

| Semaphore | Defined in | Released by | Purpose |
| --- | --- | --- | --- |
| imu_sem | IMU module | imu_sem_timer | Wakes the IMU acquisition thread when a new measurement cycle is due. |
| sem_env_gate | Enviroment module | enviroment_timer_handler | Wakes the enviroment acquisition thread when an new measurement cycle is due | 
| sem_gps_gate | GPS module | gps_uart_isr | Wakes the gps thread when an interruption is received to read data


## 3.3.4. Overall data flow

Sensor modules acquire and process their measurements independently. The resulting messages are published to their respective queues and retrieved by the main application, which maintains the latest system snapshot in `struct system_messages`.

This information is used to generate reports, update the tracker state and control the RGB LED. In SIMU mode, the same aggregated structure is transferred to a dedicated thread, which builds and transmits telemetry frames through `LPUART1`.

# 4. Firmware Implementation

## 4.1 Operating Modes, Tracker States and Data Reporting

The firmware provides three operating modes: TEST, NORMAL and SIMU. Each mode configures different sensor acquisition periods and determines how the collected data is processed and presented. The operating mode management is implemented in `main.c`, while the corresponding reporting and simulation functions are implemented in `states.c` and `simu.c`.

### 4.1.1. Operating Mode Management

The application starts in TEST mode. After initializing the RGB LED and the user button, `main()` initializes the `system_messages` structure and calls `enter_mode()` to configure the initial operating mode.

The `next_mode()` function determines the next operating mode following the sequence TEST → NORMAL → SIMU → TEST. Whenever the button callback posts `EVT_BUTTON`, the main application loop processes the event, updates `op_mode` and calls `enter_mode()` again.

The `enter_mode()` function performs the following operations:

1. Retrieves the timing configuration corresponding to the selected mode from `mode_table`.
2. Calls `gps_set_update_rate()` to configure the GPS update period.
3. Starts the environmental and IMU timers using the periods defined in the configuration table.
4. Starts the main application timer, which controls how frequently the current mode is executed.
5. Calls `normal_exit()` to disable NORMAL mode statistics collection before configuring the new mode.
6. Executes the initialization actions associated with the selected mode, including the corresponding RGB LED configuration.

The first main timer expiration is delayed by `BOOT_SETTLE_MS` (1200 ms) during startup, allowing the sensor modules time to initialize and publish their initial measurements.

The acquisition and execution periods configured for each operating mode are shown below:

| Module / operation | TEST | NORMAL | SIMU |
| --- | ---: | ---: | ---: |
| GPS update period | 2000 ms | 1000 ms | 1000 ms |
| Environmental acquisition period | 2000 ms | 2500 ms | 1000 ms |
| IMU acquisition period | 50 ms | 20 ms | 20 ms |
| Main application period | 2000 ms | 5000 ms | 100 ms |

### 4.1.2. Tracker States

The tracker uses three states: `IDLE`, `FLYING` and `PROBLEM`. The IMU module estimates the motion state using the acceleration and angular velocity measurements. The overall tracker state is evaluated by `get_drone_state()` in `states.c`.

The function first checks the validity indicators of all sensor modules. If any sensor reports an error, or if the IMU has classified its own state as `PROBLEM`, the overall tracker state is set to `PROBLEM`. Otherwise, the state estimated by the IMU is returned.

| State | Description |
| --- | --- |
| `IDLE` | No significant motion is detected by the IMU. |
| `FLYING` | The IMU detects motion according to its acceleration and angular velocity thresholds. |
| `PROBLEM` | At least one sensor reports an error, or the IMU reports a `PROBLEM` state. |

The classification thresholds and the IMU state estimation algorithm are described in the IMU sensor section.

The GPS communication status and positioning fix are treated separately. A GPS receiver without a valid positioning fix (`fix_quality == 0`) does not automatically cause a `PROBLEM` state, provided that GPS communication and data processing are working correctly (`gps_valid == 0`).

In NORMAL mode, the RGB LED indicates the overall tracker state:

| Tracker state | RGB LED |
| --- | --- |
| `IDLE` | Green |
| `FLYING` | Yellow |
| `PROBLEM` | Red |

TEST mode uses a blue LED, while SIMU mode uses purple.

### 4.1.3. Main Application Data Handling

The main application collects the latest available sensor measurements through the `queue_update()` function. Each sensor module publishes its measurements through a message queue, allowing the modules to operate independently.

The `system_messages` structure acts as a container for the latest data from all sensor modules:

| Field | Data structure | Description |
| --- | --- | --- |
| `imu` | `struct imu_m` | Inertial measurements, motion estimates and IMU status information. |
| `gps` | `struct gps_data` | GPS position, satellite information, UTC time and GPS status. |
| `env` | `struct env_m` | Environmental measurements and sensor validity information. |
| `flags` | Additional status flags | Stores status information used by the simulation module. |

The individual fields contained in each sensor structure are described in their corresponding sensor sections:

- **IMU:** See the [IMU data structure](#imu-data-structure).
- **GPS:** See the [GPS data structure](#gps-data-structure).
- **Environmental sensors:** See the [Environmental sensors data structure](#environment-data-structure).

The `queue_update()` function performs non-blocking reads from `imu_q`, `gps_msgq` and `env_q`, updating the corresponding members of `system_messages` whenever a new message is available. If a queue is empty, the previously stored measurements are retained.

The main application waits for events using `k_event_wait_safe()`. Three events control its execution:

| Event | Value | Function |
| --- | --- | --- |
| `EVT_BUTTON` | `0x01` | Changes the operating mode. |
| `EVT_TIMER` | `0x02` | Triggers data collection and execution of the current operating mode. |
| `EVT_DRONE` | `0x04` | Indicates a change in tracker state or sensor health. In NORMAL mode, it triggers an immediate LED update. |

When `EVT_TIMER` is received, the application calls `queue_update()` and executes the function associated with the current mode: `run_test_mode()`, `run_normal_mode()` or `run_simu_mode()`.

When `EVT_DRONE` is received in NORMAL mode, the application retrieves the latest available sensor data and calls `change_drone_mode()` to refresh the RGB LED without waiting for the next periodic report.

### 4.1.4. Mode Functionality

#### TEST Mode

TEST mode is intended for sensor verification and debugging. The main application executes `run_test_mode()` every 2 seconds. This function calls `print_report()` to display the latest available measurements from all sensor modules.

The report includes:

| Section | Information displayed |
| --- | --- |
| GPS | Fix quality, number of satellites, latitude, longitude, altitude and UTC time. |
| IMU | Acceleration along the X, Y and Z axes (`m/s²`). |
| IMU | Angular velocity along the X, Y and Z axes (`deg/s`). |
| Attitude | Roll, pitch and yaw angles (`deg`). |
| LIGHT | Calibrated light level (`%`). |
| ENV | Temperature (`°C`) and relative humidity (`%`). |
| STATUS | Overall tracker state. |

The GPS position is displayed only when `fix_quality > 0`. If there is no positioning fix, the report displays the satellite count and UTC time instead. GPS communication errors are reported separately.

For the IMU, light sensor and environmental sensor, measurements are displayed only when their corresponding validity indicators are zero. Otherwise, an error message is printed for the affected sensor.

An example of the TEST mode output is shown below:

```text
MODE: TEST
GPS: fix=0 sats=00 time=19:19:46 (no fix)
IMU: ax=+001.34 m/s2 ay=-000.01 m/s2 az=+010.03 m/s2
IMU: gx=-0001.6 deg/s gy=-0000.5 deg/s gz=+0001.5 deg/s
ATT: roll=-045.80 deg pitch=-013.37 deg yaw=+044.87 deg
LIGHT: 059.2 %
ENV: temp=+021.3 C hum=066.0 %
STATUS: IDLE
```

The RGB LED remains blue throughout TEST mode. No statistical report is generated in this mode.

#### NORMAL Mode

NORMAL mode provides periodic monitoring of the latest sensor measurements and statistical information. The main application executes `run_normal_mode()` every 5 seconds.

When NORMAL mode is entered, `enter_mode()` calls `normal_enter()`. This function clears the previous contents of `imu_stats_q` and `env_stats_q` and sets `normal_active` to `true`, allowing the sensor modules to start collecting data for a new statistical window.

Each execution of `run_normal_mode()` performs three operations:

1. `print_report()` displays the latest sensor measurements and the overall tracker state.
2. `change_drone_mode()` updates the RGB LED according to the current tracker state.
3. `stats_report()` retrieves and displays any available statistical results from the IMU and environmental modules.

An example of the periodic sensor report is shown below:

```text
MODE: NORMAL
GPS: fix=0 sats=00 time=19:20:22 (no fix)
IMU: ax=+000.00 m/s2 ay=+000.00 m/s2 az=+000.00 m/s2
IMU: gx=+0000.0 deg/s gy=+0000.0 deg/s gz=+0000.0 deg/s
ATT: roll=-101.16 deg pitch=-029.64 deg yaw=+097.58 deg
LIGHT: 054.4 %
```

**Statistical reporting**

The IMU and environmental sensor modules calculate statistics over 30-second windows. The reported parameters are:

| Sensor module | Parameter | Unit | Statistics |
| --- | --- | --- | --- |
| IMU | Acceleration magnitude (`\|ACC\|`) | m/s² | Mean, maximum and minimum |
| IMU | Angular velocity magnitude (`\|GYR\|`) | deg/s | Mean, maximum and minimum |
| Environmental | Light level | % | Mean, maximum and minimum |
| Environmental | Temperature | °C | Mean, maximum and minimum |
| Environmental | Relative humidity | % | Mean, maximum and minimum |

The `stats_report()` function checks `imu_stats_q` and `env_stats_q` using non-blocking reads. When a new statistical result is available, it retrieves and prints the corresponding values. If no valid samples have been collected for a parameter, the report displays `no data`.

An example of the statistical output is shown below:

```text
STATS IMU (last 30 s):
  |ACC| : mean=  +10.11 max=  +10.60 min=   +9.20 m/s2
  |GYR| : mean=   +2.25 max=   +5.63 min=   +1.37 deg/s

STATS ENV (last 30 s):
  LIGHT : mean=  +57.84 max=  +58.41 min=  +57.40 %
  TEMP  : mean=  +21.40 max=  +21.43 min=  +21.38 C
  HUM   : mean=  +65.79 max=  +65.80 min=  +65.77 %
```

Although the statistics are calculated over 30-second windows, they are checked whenever a NORMAL report is generated, every 5 seconds. Therefore, statistical results are not necessarily available in every report.

When the application leaves NORMAL mode, `normal_exit()` sets `normal_active` to `false`, disabling further NORMAL-mode statistics collection.

#### SIMU Mode

SIMU mode is designed to transmit the tracker's telemetry data to an external graphical simulator running on a PC. Unlike TEST and NORMAL modes, which display information through the serial console, SIMU mode sends data using a predefined ASCII communication protocol.

The main application period is configured to 100 ms, corresponding to a nominal update rate of 10 Hz. The main application collects the latest sensor data and provides it to the simulation module, while `simu.c` handles message queuing, frame generation and UART transmission.

**Data transfer and message queue**

Every time the main timer expires and `EVT_TIMER` is received, the main application calls `queue_update()` to retrieve the latest available sensor measurements. It then calls `run_simu_mode()`, passing the current `system_messages` structure.

The `run_simu_mode()` function checks that the received pointer is valid, removes any pending message from `simu_msgq` using `k_msgq_purge()` and inserts the latest system data snapshot into the queue.

This queue has a capacity of one message. Purging it before inserting a new snapshot prevents older pending measurements from accumulating when the simulation thread has not yet processed them.

**UART configuration and simulation thread**

The simulation module uses `LPUART1`, selected through the `DT_NODELABEL(lpuart1)` Device Tree node. The UART is configured at 115200 baud.

A dedicated thread, created using `K_THREAD_DEFINE()`, processes the messages received through `simu_msgq`.

| Parameter | Configuration |
| --- | --- |
| UART peripheral | `LPUART1` |
| Baud rate | 115200 baud |
| Thread function | `my_thread_simu()` |
| Thread stack size | 2048 bytes |
| Thread priority | 4 |
| Message queue | `simu_msgq` |

The thread waits indefinitely for a new message using `k_msgq_get()` with `K_FOREVER`. Once a message is received, it calls `simu_build_frame()` to generate the corresponding telemetry frame and then calls `simu_send_frame()` to transmit it.

**Simulation frame format**

The communication protocol uses a fixed-length ASCII frame with the following general structure:

```text
$SIMU1,<fields>*<checksum>\r\n
```

| Element | Description |
| --- | --- |
| `$` | Start-of-frame character |
| `SIMU1` | Protocol identifier and version |
| `<fields>` | Comma-separated telemetry and status information |
| `*` | Separator between the data fields and checksum |
| `<checksum>` | Two-character hexadecimal XOR checksum |
| `\r\n` | Frame termination characters |

The complete frame has a fixed length of 221 bytes. The data prefix, from `$` up to the last character before `*`, occupies 216 bytes. The remaining five bytes contain the checksum separator, the two hexadecimal checksum characters and the carriage return and line feed characters.

The frame buffer is defined as follows:

```c
#define SIMU_FRAME_LENGTH       221
#define SIMU_FRAME_BUFFER_SIZE  (SIMU_FRAME_LENGTH + 1)
```

The additional byte provides space for the null terminator added by `snprintf()`. It is not included in the transmitted frame.

**Frame fields**

The frame is generated using `snprintf()`. Each field has a predefined format specifying its width, sign and decimal precision. This ensures that the generated prefix has the expected fixed length.

| Field | Description | Format | Unit |
| --- | --- | --- | --- |
| Protocol | Frame identifier (`SIMU1`) | Fixed text | — |
| Sequence | Incremental frame counter | `%05u` | — |
| Timestamp | System uptime when the frame is generated | `%010u` | ms |
| GPS fix | `1` if `fix_quality == 1`, otherwise `0` | `%u` | — |
| Satellites | Number of satellites reported by the GPS receiver | `%02u` | — |
| Latitude | GPS latitude | `%+010.6f` | ° |
| Longitude | GPS longitude | `%+011.6f` | ° |
| Altitude | GPS altitude | `%+08.1f` | m |
| X, Y, Z | Estimated position along the three axes | `%+08.2f` | m |
| VX, VY, VZ | Estimated velocity along the three axes | `%+07.2f` | m/s |
| Roll, Pitch, Yaw | Estimated orientation angles | `%+07.2f` | ° |
| AX, AY, AZ | Acceleration along the three axes | `%+07.1f` | m/s² |
| GX, GY, GZ | Angular velocity around the three axes | `%+07.1f` | °/s |
| Light | Calibrated light intensity | `%05.1f` | % |
| Temperature | Ambient temperature | `%+06.1f` | °C |
| Humidity | Relative humidity | `%05.1f` | % |
| State | Tracker state, padded to eight characters with `_` | 8 characters | — |
| Flags | Sensor validity and additional status information | `%04X` | — |

The sequence number is stored in a 16-bit unsigned variable and is incremented each time the simulation thread builds a new frame. The timestamp is obtained using `k_uptime_get_32()`, representing the elapsed system uptime in milliseconds.

The GPS fix field is generated from the condition `fix_quality == 1`. Therefore, it indicates whether the receiver reports a valid position fix. This field is independent of `gps_valid`, which indicates whether GPS communication and data processing have encountered an error.

Before formatting the position estimates, the `clamp_float()` function limits the X, Y and Z coordinates to the range from -9999.99 to +9999.99. This prevents values outside the supported range from exceeding the expected field width.

The tracker state is obtained using `get_drone_state()`. The resulting string is copied into an eight-character field and padded with underscores when necessary. For example, `IDLE` is transmitted as `IDLE____`.

**Status flags**

The flags field is transmitted as a four-character hexadecimal value. The generated value uses the lower eight bits to represent sensor validity and additional status information.

| Bit | Mask | Description |
| --- | --- | --- |
| 0 | `0x0001` | IMU data invalid |
| 1 | `0x0002` | GPS communication or processing error |
| 2 | `0x0004` | LDR measurement invalid |
| 3 | `0x0008` | Si7021 measurement invalid |
| 4–5 | `0x0030` | Preserved from the input `flags` field |
| 6 | `0x0040` | Inertial estimates are not corrected using GPS |
| 7 | `0x0080` | Preserved from the input `flags` field |
| 8–15 | `0xFF00` | Always zero in the generated flags value |

The flags value is initially obtained by applying the mask `0x00F0` to the flags received in `system_messages`. The module then attempts to set the sensor error bits according to the validity fields of the corresponding sensor structures.

Bit 6 is always activated by the firmware because the position and velocity estimates are generated through inertial integration without GPS correction.

**Checksum calculation**

The checksum is calculated using an 8-bit XOR operation over all characters after `$` and before `*`. The start-of-frame character is excluded.

The calculation starts with a checksum value of zero. Each character in the data prefix is then combined with the current checksum using the XOR operator:

```c
checksum = 0;

for (int i = 1; i < len; i++) {
    checksum ^= (uint8_t)frame[i];
}
```

The resulting value is converted into two hexadecimal characters and appended to the frame using the format:

```c
"*%02X\r\n"
```

This produces the final five-byte trailer. The receiving simulator can perform the same calculation to verify the integrity of the received frame.

**Frame construction and transmission**

The `simu_build_frame()` function is responsible for generating and validating the complete frame. Its execution follows these steps:

1. It checks that the input data structure and output buffer are valid.
2. It verifies that the buffer has enough space for the complete frame and its null terminator.
3. It obtains the current tracker state and formats it as an eight-character field.
4. It generates the status flags from the sensor validity fields and the input flags.
5. It formats all telemetry measurements into the data prefix using `snprintf()`.
6. It verifies that the prefix length is exactly 216 bytes.
7. It calculates the XOR checksum and appends the final trailer.
8. It verifies that the complete frame length is exactly 221 bytes.

If the frame is successfully constructed, the function returns `SIMU_FRAME_LENGTH`. Otherwise, it returns a negative error code.

Once the frame has been generated, `simu_send_frame()` checks that the UART device is ready and transmits the frame using `uart_poll_out()`. This function sends the data one byte at a time.

The `my_thread_simu()` thread coordinates the complete process. After receiving a message from `simu_msgq`, it obtains the current system uptime, increments the sequence counter, builds the frame and transmits it if its length matches the expected value.

This approach separates data collection from communication. The main application provides the latest system measurements, while the simulation thread is responsible for formatting, checksum calculation and UART transmission.

## 4.2. Sensor Acquisition and Processing

### 4.2.1. MPU6050 – Inertial Measurement Unit (IMU)

The MPU6050 is a six-axis inertial measurement unit that combines a three-axis accelerometer and a three-axis gyroscope. The accelerometer measures linear acceleration along the X, Y and Z axes, while the gyroscope measures angular velocity around those same axes.

The sensor communicates with the STM32WL55JC through the I2C2 interface. Its I2C address is `0x68`.

#### Sensor configuration

Before acquiring measurements, the firmware configures the MPU6050 by writing specific values to its internal registers. The configuration is performed in the `IMU_initialization()` function.

The following table summarises the registers used:

| Register | Address | Configuration |
| --- | --- | --- |
| PWR_MGMT_1 | 0x6B | 0x09 – Wakes up the sensor, disables its internal temperature measurement and selects the X-axis gyroscope PLL as the clock source. |
| CONFIG | 0x1A | 0x03 – Enables the digital low-pass filter with a bandwidth configuration of 42 Hz for the gyroscope and 44 Hz for the accelerometer. |
| SMPLRT_DIV | 0x19 | 0x13 (19 decimal) – Sets the output sampling rate to 50 Hz. |
| GYRO_CONFIG | 0x1B | 0x08 – Selects a gyroscope measurement range of ±500 °/s. |
| ACCEL_CONFIG | 0x1C | 0x08 – Selects an accelerometer measurement range of ±4 g. |

The `PWR_MGMT_1` register is used to wake up the device and select its clock source. The value `0x09` sets the SLEEP bit to zero, disables the internal temperature sensor and selects the phase-locked loop (PLL) clock based on the X-axis gyroscope reference.

The `CONFIG` register controls the digital low-pass filter (DLPF). This filter reduces high-frequency noise in the measurements. With `DLPF_CFG = 3`, the configured bandwidths are 42 Hz for the gyroscope and 44 Hz for the accelerometer.

The sampling frequency is controlled through the `SMPLRT_DIV` register. Since the DLPF is enabled, the gyroscope output rate is 1 kHz. The resulting sampling frequency is:

$$
f_s = \frac{1000}{1+SMPLRT\_DIV}
$$

With `SMPLRT_DIV = 19`:

$$
f_s = \frac{1000}{1+19}=50\text{ Hz}
$$

Therefore, the sensor provides a new measurement approximately every 20 ms.

The `GYRO_CONFIG` and `ACCEL_CONFIG` registers define the measurement ranges. In both cases, setting bit 3 (`0x08`) selects the second available range:

- **Gyroscope:** ±500 °/s, with a sensitivity of 65.5 LSB/(°/s).
- **Accelerometer:** ±4 g, with a sensitivity of 8192 LSB/g.

These sensitivity values are essential for converting the raw sensor readings into physical units.

#### Data acquisition and physical conversion

The function `IMU_read_and_process()` retrieves the sensor measurements using the `i2c_burst_read()` function.

The acquisition starts at register `0x3B`, corresponding to the first accelerometer output register. A total of 14 consecutive bytes are read in a single I2C transaction.

The returned data is organised as follows:

| Bytes | Measurement |
| --- | --- |
| 0–1 | Accelerometer X |
| 2–3 | Accelerometer Y |
| 4–5 | Accelerometer Z |
| 6–7 | Internal temperature (ignored) |
| 8–9 | Gyroscope X |
| 10–11 | Gyroscope Y |
| 12–13 | Gyroscope Z |

Each measurement is represented by a signed 16-bit integer, transmitted in big-endian format: the most significant byte comes first. The firmware reconstructs each value by combining the two corresponding bytes.

For example, the X-axis accelerometer reading is reconstructed as:

$$
raw_{ax} = (buf[0] \ll 8) \;|\; buf[1]
$$

The same operation is applied to the remaining axes.

Once the raw values have been obtained, they are converted into physical units using the sensitivities selected during initialization.

For the accelerometer, the conversion to metres per second squared is:

$$
a_x = \frac{raw_{ax}}{8192}\cdot 9.81
$$

The same formula is applied to the Y and Z axes. The factor 9.81 converts the acceleration from units of g to m/s².

For the gyroscope, the conversion to degrees per second is:

$$
g_x = \frac{raw_{gx}}{65.5}
$$

The same conversion is applied to the other two axes.

After these operations, the firmware has six physical measurements: three acceleration components in m/s² and three angular velocity components in °/s.

#### Velocity, position and orientation estimation

The firmware also estimates velocity, position and orientation by numerically integrating the sensor measurements over time.

The elapsed time between consecutive measurements is calculated using `k_uptime_get()`. The difference between the current and previous timestamps is converted from milliseconds to seconds:

$$
dt = \frac{t_{now}-t_{previous}}{1000}
$$

**Gravity estimation and acceleration correction**

Accelerometer measurements include both the acceleration caused by movement and the effect of gravity. To reduce the influence of gravity on the velocity and position estimates, the firmware implements a low-pass filter that estimates the gravity component independently for each axis.

The filter is controlled by the `GRAVITY_FILTER_ALPHA` constant, configured to `0.98`. The estimated gravity component is updated using the following equation:

$$
g_{est}(t)=\alpha \cdot g_{est}(t-1)+(1-\alpha)\cdot a(t)
$$

where:

- \(g_{est}(t)\) is the estimated gravity component for the current sample.
- \(g_{est}(t-1)\) is the previous estimate.
- \(a(t)\) is the current measured acceleration.
- \(\alpha\) is the filter coefficient, set to 0.98.

The filter is applied independently to the X, Y and Z axes. At the first measurement, each gravity estimate is initialized using the corresponding accelerometer reading. This avoids an abrupt initial difference between the measured acceleration and the estimated gravity.

The linear acceleration used for integration is then calculated by subtracting the estimated gravity component from the measured acceleration:

$$
a_{linear,x}(t)=a_x(t)-g_{est,x}(t)
$$

Equivalent calculations are performed for the Y and Z axes.

**Velocity and position estimation**

Once the acceleration has been corrected, the firmware estimates velocity by numerical integration:

$$
v_x(t)=v_x(t-1)+a_{linear,x}(t)\cdot dt
$$

The same calculation is performed for the Y and Z axes.

Position is then estimated by integrating velocity:

$$
x(t)=x(t-1)+v_x(t)\cdot dt
$$

Equivalent calculations are used for the Y and Z coordinates.

**Zero-Velocity Update (ZUPT) for drift mitigation**

Numerical integration of acceleration can cause errors to accumulate over time. Even small residual acceleration errors can produce an increasing velocity error, which subsequently affects the estimated position.

To mitigate this effect when the tracker is stationary, the firmware implements a Zero-Velocity Update (ZUPT) mechanism.

The mechanism uses the state calculated by the `calcular_estado()` function. A counter, represented by the `idle_time` variable, accumulates the elapsed time (`dt`) while the calculated state remains `IDLE`. If the state changes to `FLYING` or `PROBLEM`, this counter is reset to zero.

The implementation uses the `ZUPT_HOLD_S` constant, configured to `0.5` seconds. Therefore, the tracker must remain continuously classified as `IDLE` for at least 0.5 seconds before the zero-velocity correction is applied.

The condition is expressed as:

$$
idle\_time =
\begin{cases}
idle\_time + dt, & \text{if state = IDLE}\\
0, & \text{otherwise}
\end{cases}
$$

When the accumulated time reaches the configured threshold:

$$
idle\_time \geq 0.5\text{ s}
$$

the firmware resets the three velocity components:

$$
v_x=v_y=v_z=0
$$

While this condition remains satisfied, velocity integration and position updates are skipped. This prevents residual acceleration from continuing to increase the estimated velocity and position while the tracker is considered stationary.

If the tracker leaves the `IDLE` state, the counter is reset and the normal velocity and position integration resumes.

This correction reduces the accumulation of velocity drift during stationary periods. However, it does not directly correct the position estimate or remove errors accumulated before the ZUPT condition is reached. The resulting position therefore remains an approximate inertial estimate.

**Orientation estimation**

The orientation angles are estimated separately by integrating the angular velocity measured by the gyroscope:

$$
roll(t)=roll(t-1)+g_x(t)\cdot dt
$$

$$
pitch(t)=pitch(t-1)+g_y(t)\cdot dt
$$

$$
yaw(t)=yaw(t-1)+g_z(t)\cdot dt
$$

The resulting angles are expressed in degrees.

These calculations provide a basic inertial estimate. The low-pass filter reduces the influence of gravity on the acceleration used for velocity and position estimation. However, this is an approximate compensation: it assumes that gravity changes slowly compared with the movement being measured. During sustained acceleration, part of the actual movement may be incorporated into the gravity estimate. Similarly, changes in sensor orientation can affect how gravity is distributed across the three axes.

The implementation does not perform a full coordinate-frame transformation or use a sensor-fusion algorithm to separate gravity from movement. Consequently, velocity and position estimates remain approximate and accumulate errors over time. The ZUPT mechanism mitigates velocity drift during stationary periods, but it does not eliminate accumulated position error. The orientation estimates also experience drift because they are obtained by directly integrating gyroscope measurements, particularly in the yaw angle.

#### Tracker state estimation

After processing the measurements, the firmware determines the current tracker state through the `calcular_estado()` function.

First, it calculates the magnitude of the acceleration and angular velocity vectors:

$$
|a|=\sqrt{a_x^2+a_y^2+a_z^2}
$$

$$
|g|=\sqrt{g_x^2+g_y^2+g_z^2}
$$

These magnitudes are used to evaluate the movement of the drone independently of the individual sensor axes.

The implementation uses three heuristic thresholds:

- Acceleration magnitude threshold for detecting a possible impact: 25 m/s².
- Acceleration deviation from gravity: 0.5 m/s².
- Angular velocity magnitude threshold: 5 °/s.

The state is determined according to the following conditions:

1. **PROBLEM:** if the acceleration magnitude exceeds 25 m/s², a possible impact or abnormal acceleration is detected.
2. **FLYING:** if the acceleration magnitude differs from 9.81 m/s² by more than 0.5 m/s², or the angular velocity magnitude exceeds 5 °/s.
3. **IDLE:** if neither of the previous conditions is met.

The `IDLE` state is also used by the ZUPT mechanism described above. Remaining in this state for at least 0.5 seconds triggers the velocity reset and temporarily stops position integration.

These calculations use the original accelerometer measurements, before the gravity-filter correction used for velocity and position estimation.

These thresholds provide a simple indication of movement and possible abnormal events. They are heuristic values and do not constitute a reliable flight-detection or impact-classification algorithm.

#### Measurement thread and data publication

The `IMU_measurement()` function runs in a dedicated Zephyr thread. It is responsible for initializing the sensor, periodically acquiring measurements, processing the data and publishing the latest result.

The thread uses a semaphore (`imu_sem`) to synchronize its execution with `imu_timer`. Each time the timer expires, its callback releases the semaphore. The measurement thread waits for this semaphore before performing the next acquisition.

The sampling period is configured by the main application according to the selected operating mode.

If initialization fails, the firmware marks the IMU measurement as invalid, sets the tracker state to `PROBLEM`, publishes the error information and retries initialization every two seconds.

During normal operation, the thread also monitors consecutive I2C communication errors. An isolated error does not immediately replace the last published measurement. After five consecutive failures, the IMU is marked as invalid and the tracker state is changed to `PROBLEM`.

After each successful measurement, or when the error limit is reached, the latest data is published through `imu_q`. This message queue has a capacity of one element. The firmware purges the previous message before inserting the new one, ensuring that the queue stores the most recent available IMU data.

The `imu_change_mode()` function also checks whether the calculated tracker state has changed. If a transition occurs, it posts the `EVT_DRONE` event to notify the main application.

#### IMU statistics

When the system operates in NORMAL mode, the IMU thread additionally calculates statistics over 30-second windows.

Two independent statistical accumulators are used:

- Acceleration magnitude: calculated from the three original acceleration axes.
- Angular velocity magnitude: calculated from the three gyroscope axes.

For each quantity, the firmware records the mean, minimum and maximum values, together with the number of samples.

At the end of each 30-second window, the results are published through `imu_stats_q`. The accumulators are then reset to start a new window.

These statistics are only collected while NORMAL mode is active. When the system enters this mode, the current statistical window is reset to ensure that measurements from previous operating modes are not included.

#### Data structure

The IMU measurements and motion estimates are stored in the `struct imu_m` structure. This structure is published through the `imu_q` message queue and contains the following fields:

| Field | Description | Unit |
| --- | --- | --- |
| `ax` | Acceleration along the X axis | m/s² |
| `ay` | Acceleration along the Y axis | m/s² |
| `az` | Acceleration along the Z axis | m/s² |
| `gx` | Angular velocity around the X axis | °/s |
| `gy` | Angular velocity around the Y axis | °/s |
| `gz` | Angular velocity around the Z axis | °/s |
| `roll` | Estimated rotation around the X axis | ° |
| `pitch` | Estimated rotation around the Y axis | ° |
| `yaw` | Estimated rotation around the Z axis | ° |
| `vx` | Estimated velocity along the X axis | m/s |
| `vy` | Estimated velocity along the Y axis | m/s |
| `vz` | Estimated velocity along the Z axis | m/s |
| `x` | Estimated position along the X axis | m |
| `y` | Estimated position along the Y axis | m |
| `z` | Estimated position along the Z axis | m |
| `state` | Estimated drone state: `IDLE`, `FLYING` or `PROBLEM` | — |
| `valid_imu` | IMU measurement status: `0` = valid, `-1` = I2C error | — |

### 4.2.2. GPS – Global Positioning System

The GPS module (PA6H) provides geographical positioning information for the drone. It communicates with the STM32WL55JC through the USART1 interface using UART serial communication.

Unlike the MPU6050, which provides binary measurement data, the GPS transmits information as ASCII text organized into NMEA sentences. These sentences contain different types of navigation and positioning information, such as position, time, satellite information and altitude.

The NMEA protocol defines several sentence types. Each one starts with a specific identifier that determines the information it contains. Some common examples are:

| Sentence | Description |
| --- | --- |
| GGA | Position, altitude, fix quality and satellite information. |
| RMC | Recommended minimum navigation data, including position, speed and date. |
| GSA | Satellite selection and positioning accuracy information. |
| GSV | Information about satellites in view. |
| VTG | Course over ground and ground speed. |

In this project, only GGA sentences are enabled and processed because they provide the positioning information required by the application.

#### Sensor configuration

The GPS communicates with the microcontroller through USART1. The UART configuration is defined in the Device Tree:

| Parameter | Configuration |
| --- | --- |
| UART peripheral | USART1 |
| Baud rate | 9600 baud |
| TX pin (MCU) | PB6 |
| RX pin (MCU) | PB7 |
| Communication type | UART serial communication |

The GPS module is configured by sending PMTK commands through the UART interface. These commands allow the firmware to select the NMEA sentences transmitted by the module and configure their update interval.

The following commands are used:

| Command | Purpose | Configuration |
| --- | --- | --- |
| PMTK314 | Selects the NMEA sentence types transmitted by the GPS. | Enables GGA sentences only. |
| PMTK220 | Configures the position update interval. | Initially set to 2000 ms. |

The PMTK314 command is sent during initialization:

`$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29`

The value `1` in the fourth sentence-selection field enables GGA output, while the remaining fields disable the other supported sentence types.

The update interval is configured using the PMTK220 command. The firmware generates this command dynamically according to the requested interval. Initially, the interval is set to 2000 ms, corresponding to an update frequency of:

$$
f = \frac{1000}{2000}=0.5\text{ Hz}
$$

Therefore, the GPS initially provides a new position update every two seconds. The main application can subsequently modify this interval according to the selected operating mode.

Both PMTK commands include a checksum. The firmware calculates it by applying the XOR operation to all characters between `$` and `*`, and appends the resulting hexadecimal value followed by `\r\n`.

**NMEA sentence format and data acquisition**

An NMEA sentence is transmitted as a sequence of ASCII characters. It starts with `$`, contains comma-separated fields, and ends with a checksum preceded by `*`, followed by the line-ending characters `\r\n`.

The following is an example of a GGA sentence:

`$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47`

The sentence contains the following fields:

| Field | Example value | Description |
| --- | --- | --- |
| 0 | `$GPGGA` | Sentence identifier. |
| 1 | `123519` | UTC time (12:35:19). |
| 2 | `4807.038` | Latitude in degrees and decimal minutes. |
| 3 | `N` | Latitude direction: North. |
| 4 | `01131.000` | Longitude in degrees and decimal minutes. |
| 5 | `E` | Longitude direction: East. |
| 6 | `1` | Fix quality. A value greater than zero indicates a valid fix. |
| 7 | `08` | Number of satellites used. |
| 8 | `0.9` | Horizontal Dilution of Precision (HDOP). |
| 9 | `545.4` | Altitude above mean sea level. |
| 10 | `M` | Altitude unit: metres. |
| 11 | `46.9` | Geoidal separation. |
| 12 | `M` | Geoidal separation unit: metres. |
| 13 | Empty | Age of differential GPS data. |
| 14 | Empty | Differential reference station ID. |
| - | `*47` | Checksum. |

The firmware does not process the sentence directly when it arrives. Instead, UART reception is handled asynchronously through an interrupt routine.

The `gps_uart_isr()` callback reads the available bytes from the UART FIFO using `uart_fifo_read()`, in blocks of up to 32 bytes. The received bytes are then stored in a circular buffer using `ring_buf_put()`.

This circular buffer separates byte reception from sentence processing, allowing the interrupt routine to remain short. If the buffer does not have enough free space, the number of bytes that could not be stored is counted as an overflow.

Whenever at least one byte has been successfully stored, the ISR releases `gps_sem`. This semaphore wakes up the GPS thread, which processes the received data outside the interrupt context.

**NMEA sentence reconstruction and checksum validation**

The GPS thread retrieves the available bytes from the circular buffer using `ring_buf_get()` and passes them one by one to `gps_process_byte()`.

The sentence reconstruction follows these rules:

1. The character `$` identifies the beginning of a new NMEA sentence and resets the sentence buffer index.
2. Bytes received before `$` are ignored.
3. Each subsequent byte is stored in `gps_nmea_buffer`.
4. When `\n` is received, the sentence is considered complete and a null terminator (`\0`) is added to form a valid C string.
5. If the sentence exceeds the available buffer capacity, it is discarded.

Before extracting the fields, the firmware verifies the integrity of the complete sentence through `gps_validate_checksum()`.

The NMEA checksum is calculated by applying the bitwise XOR operation to all characters between `$` and `*`, excluding both delimiters.

For the example sentence, the calculation can be represented as:

$$
CS = C_1 \oplus C_2 \oplus \cdots \oplus C_n
$$

where \(C_1\) to \(C_n\) represent the ASCII characters between `$` and `*`.

The calculated result is then compared with the two hexadecimal characters received after `*`. In the example, the received checksum is `47` in hexadecimal.

The firmware first locates the `*` character, calculates the XOR checksum and converts the two received hexadecimal characters into a numerical value. If both values match, the sentence is considered valid.

The checksum validation function returns:

- `1` if the checksum is correct.
- `0` if the checksum does not match.
- A negative value if the sentence is malformed.

The GPS processing thread also counts consecutive sentences with invalid checksums. After five consecutive checksum failures, it reports a processing error. A valid sentence resets this counter.

#### Field extraction and position conversion

After checksum validation, the firmware separates the sentence into individual fields using `gps_split_nmea()`. Each comma is replaced with a null terminator (`\0`), allowing the fields to be accessed as independent strings within the original sentence buffer.

The resulting field array is passed to `gps_process_fields()`, which first checks the sentence identifier. Only `$GPGGA` and `$GNGGA` sentences are accepted.

The relevant fields are then converted and stored in the `gps_data` structure:

| Structure field | NMEA source | Conversion |
| --- | --- | --- |
| `utc_time` | Field 1 | Formatted as `HH:MM:SS`. |
| `latitude` | Fields 2 and 3 | Converted to decimal degrees, including hemisphere sign. |
| `longitude` | Fields 4 and 5 | Converted to decimal degrees, including hemisphere sign. |
| `fix_quality` | Field 6 | Converted to an integer. |
| `satellites` | Field 7 | Converted to an integer. |
| `hdop` | Field 8 | Converted to a floating-point value. |
| `altitude` | Field 9 | Converted to a floating-point value in metres. |

NMEA coordinates use the degrees and decimal minutes format (`ddmm.mmmm` for latitude and `dddmm.mmmm` for longitude), rather than decimal degrees.

To convert them, the firmware separates the degrees from the minutes and applies:

$$
\text{Decimal Degrees} = \text{Degrees}+\frac{\text{Minutes}}{60}
$$

For example, the latitude `4807.038,N` is interpreted as 48 degrees and 7.038 minutes:

$$
48+\frac{7.038}{60}=48.1173^\circ
$$


The same conversion is applied to longitude. The hemisphere determines the sign of the resulting coordinate: South (`S`) and West (`W`) produce negative values, while North (`N`) and East (`E`) produce positive values.

The UTC time is extracted from the first data field and formatted as hours, minutes and seconds. The remaining numerical fields are converted using functions such as `atoi()`, `strtod()` and `strtof()`, depending on their data type.

The firmware also retains the last valid latitude, longitude and altitude. If the fix quality becomes zero, the previous position is preserved instead of being overwritten with zeros. However, `fix_quality` continues to indicate that the GPS does not currently have a valid position fix.

#### Data structure

The GPS measurements are stored in the `struct gps_data` structure. This structure is published through the `gps_msgq` message queue and contains the following fields:

| Field | Description | Unit |
| --- | --- | --- |
| `latitude` | Geographic latitude | degrees (°) |
| `longitude` | Geographic longitude | degrees (°) |
| `altitude` | Altitude above mean sea level, as reported by the GGA sentence | m |
| `hdop` | Horizontal dilution of precision | — |
| `satellites` | Number of satellites used for the position fix | — |
| `fix_quality` | GPS fix quality indicator reported by the receiver | — |
| `utc_time` | UTC time received from the GPS module | hhmmss.ss |
| `gps_valid` | GPS communication and processing status: `0` = valid, `-1` = error | — |

The `gps_valid` field indicates whether the GPS data processing is operating correctly. It is independent of `fix_quality`, which indicates whether the receiver has obtained a valid position fix.


### 4.2.3. Environmental Sensors – Si7021 and LDR

The environmental monitoring system uses two sensors to measure the surrounding conditions of the drone:

- **Si7021:** a digital temperature and relative humidity sensor.
- **LDR (Light Dependent Resistor):** a light sensor whose resistance varies according to the incident light intensity.

The Si7021 communicates with the STM32WL55JC through the I2C2 interface, while the LDR is connected to an analogue input and measured using the ADC peripheral.

#### Si7021 – Sensor configuration

The Si7021 is a digital humidity and temperature sensor with an I2C interface. Its 7-bit I2C address is `0x40`.

The communication interface is configured as follows:

| Parameter | Configuration |
| --- | --- |
| Sensor | Si7021 |
| I2C peripheral | I2C2 |
| I2C address | `0x40` |
| I2C clock frequency | 100 kHz |
| Humidity resolution | 12 bits |
| Temperature resolution | 14 bits |
| Internal heater | Disabled |

Before taking measurements, the firmware checks that the I2C device is ready and reads the sensor's User Register 1 using command `0xE7`.

This register controls the measurement resolution and the internal heater. The relevant bits are:

| Bit | Function | Required value |
| --- | --- | --- |
| D7 | Humidity and temperature resolution selection (RES1) | 0 |
| D2 | Internal heater enable (HTRE) | 0 |
| D0 | Humidity and temperature resolution selection (RES0) | 0 |

The combination `RES1 = 0` and `RES0 = 0` selects 12-bit humidity and 14-bit temperature resolution. Setting `HTRE = 0` disables the internal heater.

If any of these bits is set incorrectly, the firmware writes the modified register using command `0xE6`. The operation clears the corresponding bits while preserving the remaining register configuration.

The following table summarises the I2C commands used by the firmware:

| Command | Code | Purpose |
| --- | --- | --- |
| Read User Register 1 | `0xE7` | Reads the current sensor configuration. |
| Write User Register 1 | `0xE6` | Updates the resolution and heater settings. |
| Measure Relative Humidity | `0xE5` | Starts a humidity measurement and reads its result. |
| Read Temperature from Previous RH Measurement | `0xE0` | Reads the temperature measured during the previous humidity conversion. |

#### Si7021 – Data acquisition and physical conversion

The firmware obtains humidity and temperature measurements using two consecutive I2C operations.

**Relative humidity measurement**

The humidity measurement starts by sending command `0xE5` to the sensor. This command operates in Hold Master Mode, meaning that the sensor holds the I2C clock low while the measurement is being performed.

The firmware uses `i2c_write_read()` to send the command and retrieve three bytes in the same I2C operation.

The returned data is organised as follows:

| Byte | Content |
| --- | --- |
| 0 | Most Significant Byte (MSB) of the humidity measurement. |
| 1 | Least Significant Byte (LSB) of the humidity measurement. |
| 2 | CRC checksum. |

The two measurement bytes are combined into a single unsigned 16-bit value:

$$
RH_{raw}=(data[0]\ll 8)\;|\;data[1]
$$

Before converting the measurement, the firmware verifies the CRC checksum. The received CRC byte is compared with the value calculated from the two measurement bytes.

The CRC uses the polynomial:

$$
P(x)=x^8+x^5+x^4+1
$$

Its hexadecimal representation is `0x31`, and the initial CRC value is `0x00`.

The calculation processes each received byte using XOR operations and eight bitwise iterations. At each iteration, the CRC is shifted one bit to the left. If its most significant bit was set before the shift, the result is also XORed with the polynomial `0x31`.

If the calculated CRC does not match the received byte, the measurement is considered invalid and the function returns an I2C/data error.

Once the checksum has been validated, the raw humidity value is converted into relative humidity using the formula specified in the datasheet:

$$
RH(\%)=125\cdot\frac{RH_{raw}}{65536}-6
$$

The result is expressed as a percentage of relative humidity. Since the datasheet allows values slightly outside the physical range, the firmware limits the final result to 0–100%.

**Temperature measurement**

After obtaining the humidity measurement, the firmware reads the temperature associated with that same conversion using command `0xE0`.

This command retrieves the temperature value from the previous relative humidity measurement, avoiding the need to initiate an independent temperature conversion.

Two bytes are received:

| Byte | Content |
| --- | --- |
| 0 | Most Significant Byte (MSB) of the temperature measurement. |
| 1 | Least Significant Byte (LSB) of the temperature measurement. |

The raw temperature value is reconstructed in the same way as the humidity measurement:

$$
T_{raw}=(data[0]\ll 8)\;|\;data[1]
$$

The temperature is then converted into degrees Celsius using the datasheet formula:

$$
T(^{\circ}C)=175.72\cdot\frac{T_{raw}}{65536}-46.85
$$

Unlike the humidity measurement, the temperature read using command `0xE0` does not include a CRC byte.

After both conversions, the resulting humidity and temperature values are stored in the `env_m` structure.

#### LDR – Light measurement

The LDR is an analogue sensor whose electrical resistance changes depending on the amount of incident light. The firmware measures its output voltage using the STM32 ADC peripheral.

The ADC channel is defined through the Zephyr Device Tree configuration:

| Parameter | Configuration |
| --- | --- |
| ADC peripheral | ADC1 |
| ADC channel | Channel 5 |
| Resolution | 12 bits |
| Gain | `ADC_GAIN_1` |
| Reference voltage | Internal reference |
| Acquisition time | Default |

The ADC provides a digital value between 0 and 4095, corresponding to its 12-bit resolution.

The firmware initializes the ADC channel and configures an ADC sequence using `adc_sequence_init_dt()`. During each acquisition, `adc_read_dt()` stores the raw conversion result in a 16-bit variable.

**Raw measurement and calibration**

The raw ADC value is first converted into an initial light percentage using:

$$
L_{raw}=100-\frac{ADC_{raw}\cdot100}{4095}
$$

The subtraction from 100 reverses the ADC scale, so that a lower ADC reading corresponds to a higher initial light percentage.

The firmware then applies a linear calibration using two experimentally defined limits:

| Parameter | Value |
| --- | --- |
| `MIN_CALIBRATE_LCD` | 25 |
| `MAX_CALIBRATE_LCD` | 95 |

The calibrated light percentage is calculated as:

$$
L_{calibrated}=
\frac{L_{raw}-25}{95-25}\cdot100
$$

Finally, the result is limited to the range from 0% to 100%:

$$
L=
\begin{cases}
0 & \text{if } L_{calibrated}<0\\
100 & \text{if } L_{calibrated}>100\\
L_{calibrated} & \text{otherwise}
\end{cases}
$$

This calibration maps the initial values of 25 and 95 to 0% and 100%, respectively. Values outside these limits are saturated to the corresponding endpoint.

The calibration constants are manually selected and determine the relationship between the ADC reading and the light percentage reported by the firmware.

#### Data structure

The environmental measurements are stored in the `struct env_m` structure. This structure is published through the `env_q` message queue and contains the following fields:

| Field | Description | Unit |
| --- | --- | --- |
| `light` | Calibrated light intensity | % |
| `temp` | Ambient temperature | °C |
| `hum` | Relative humidity | % |
| `ldr_valid` | LDR measurement status: `0` = valid, negative value = error | — |
| `si7021_valid` | Si7021 measurement status: `0` = valid, negative value = error | — |

# 5.  Power Management

The firmware incorporates preliminary low-power management functionality based on Zephyr's Power Management subsystem. The objective is to provide the infrastructure required to reduce the energy consumption of the STM32WL55JC and to explore the coordination between peripheral activity and the microcontroller's power states.

## 4.1 Power management configuration

The following options are enabled in `prj_nucleo_wl55jc.conf`:

| Configuration option | Function |
|---|---|
| `CONFIG_PM=y` | Enables Zephyr's system power management framework, allowing the kernel to select suitable power states according to the system conditions. |
| `CONFIG_PM_DEVICE=y` | Enables power management support for hardware devices, allowing peripherals to be suspended and resumed. |
| `CONFIG_PM_DEVICE_RUNTIME=y` | Enables runtime device power management, allowing device power states to be managed while the application is running. |
| `CONFIG_TICKLESS_KERNEL=y` | Allows the kernel to suppress periodic timer ticks when they are not required, reducing unnecessary wake-ups during idle periods. |
| `CONFIG_STM32_LPTIM_TIMER=y` | Enables the STM32 Low-Power Timer driver, providing a timer peripheral suitable for low-power timing. |

These options establish the software infrastructure for power management. In particular, device power management is relevant when peripherals do not need to remain active continuously, while tickless operation helps avoid unnecessary CPU wake-ups.

## 5.2. Hardware configuration

The Device Tree overlay provides the clock and CPU power-state configuration required by the STM32WL55JC:

| Device Tree node | Configuration | Purpose |
|---|---|---|
| `&clk_lse` | Enabled | Enables the Low-Speed External oscillator (LSE) as a low-frequency clock source. |
| `&lptim1` | Enabled and clocked from LSE | Configures Low-Power Timer 1 to use the LSE clock source. |
| `&cpu0` | `cpu-power-states = <&stop0 &stop1>` | Declares the STOP0 and STOP1 power states as available CPU power states. |

Using LSE as the LPTIM1 clock source provides a low-frequency timing reference for low-power operation. This is particularly relevant when timing needs to be maintained without relying exclusively on the main high-frequency clock sources.

Declaring STOP0 and STOP1 makes these power states available to the system power management framework. However, their declaration does not guarantee that the CPU will enter either state. The actual state selection depends on the power management policy, pending activity and active power-state constraints.

## 5.3. GPS power management implementation

A preliminary GPS duty-cycle mechanism was also implemented to investigate the possibility of reducing power consumption during operation.

The mechanism is controlled through:

```c
gps_set_duty_cycle(uint32_t standby_ms);
```

The `gps_standby_ms` variable determines the standby interval between GPS acquisition windows:

- A value of `0` selects continuous GPS operation.
- A value greater than `0` enables the standby mechanism, specifying the duration of the standby interval in milliseconds.

During the active phase, the GPS thread acquires and processes incoming NMEA data. A power-state lock is acquired using:

```c
pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
```

This prevents the power management policy from selecting the specified suspend-to-idle state while the GPS is active. This restriction is important because the GPS continues transmitting data through USART1, which cannot receive that data if the system enters the relevant STOP state.

When the standby condition is reached, the firmware performs the following operations:

1. **GPS standby command:** The firmware sends the `$PMTK161,0*28` command to request the GPS module to enter standby.
2. **UART shutdown:** After allowing time for the command transmission to finish, UART receive interrupts are disabled and USART1 is suspended using `pm_device_action_run()`.
3. **Power-state lock release:** The suspend-to-idle lock is released using `pm_policy_state_lock_put()`, allowing the power management policy to consider the available low-power states.
4. **Standby interval:** The GPS thread waits for the configured standby interval or for a wake-up request.
5. **GPS reactivation:** When the standby interval ends, USART1 is resumed, receive interrupts are enabled again and the GPS module is awakened by transmitting a byte. The previous GPS update rate is then reapplied.

The implementation also includes a maximum acquisition time of 30 seconds (`GPS_ACQ_TIMEOUT_MS`) for each active acquisition window. This prevents the GPS thread from waiting indefinitely for a valid fix before entering standby.

## 5.4. RGB LED power optimization

The RGB LED control has also been modified to reduce its power consumption while maintaining visual status indication.

Instead of keeping the selected LED continuously illuminated, the firmware implements a periodic pulsing mechanism using two Zephyr kernel timers:

- `rgb_period_timer`: controls the repetition interval and activates the selected LED colour.
- `rgb_off_timer`: switches the LEDs off after the configured illumination time.

The timing parameters are defined as follows:

| Parameter | Value | Description |
|---|---:|---|
| `RGB_ON_MS` | 3 ms | Time during which the selected LED remains illuminated. |
| `RGB_PERIOD_MS` | 10 ms | Time between consecutive LED activations. |

This configuration produces a repetition frequency of 100 Hz, with an LED on-time of 3 ms per 10 ms period. Therefore, the resulting duty cycle is 30%.

The selected RGB colour is stored in `rgb_current`, allowing the firmware to preserve the requested status indication between timer callbacks.

The behaviour is controlled through `rgb_set_pulsed(bool en)`. When pulsed operation is enabled, the periodic timer is started. Each period activates the selected colour and schedules the second timer to switch the LEDs off after 3 ms. When pulsed operation is disabled, both timers are stopped and the selected colour is applied continuously.

In the main application, pulsed operation is enabled in TEST and NORMAL modes, while SIMU mode uses continuous LED illumination.

This approach reduces the time during which the LEDs are powered while preserving a visible status indication. The implementation uses software timers rather than a dedicated hardware PWM peripheral.

## 5.5. Final operating configuration and limitations

Although the duty-cycle mechanism was implemented, tests showed that the selected standby intervals did not provide satisfactory operation while maintaining the system requirements. Shorter intervals also resulted in incorrect behaviour during testing.

For this reason, the final configuration uses a standby interval of zero. The GPS therefore remains continuously active, and the standby and UART suspension sequence is not executed during normal operation.

The duty-cycle implementation is retained in the firmware as preliminary work for future power-consumption optimisation. The current implementation provides the required Zephyr and hardware configuration, but no reduction in power consumption from periodic GPS standby is claimed for the final operating configuration.

