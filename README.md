# FINAL PROJECT - Implementation of the Embedded Platform for a Drone Tracking and Telemetry IoT System using the ARM-based Microcontroller STM32WL55JC and Zephyr OS

Final project of *Embedded Platforms and Communications for IoT* (UPM, IoT Master) — Group K.

The firmware reads a GPS receiver, an IMU, a temperature and humidity sensor and a light sensor. From these measurements it determines the state of the drone (idle, flying or problem), indicates it on the RGB LED and prints a report through the serial console. The user button selects the operating mode.

**Version 1.0.0.** The TEST and NORMAL modes are practically complete. The SIMU mode (the `$SIMU1` frame sent to the simulator at 10 Hz) is not implemented yet: the mode can be selected and the LED turns purple, but no data is transmitted.

## 1. System Overview

Each sensor runs in its own Zephyr thread and publishes its most recent measurement in a message queue of size 1, so a new value always replaces the previous one and the main thread never reads outdated data. The main thread remains blocked until an event arrives, then collects the latest value of every sensor and acts according to the current mode.

```
 GPS thread ──┐
 IMU thread ──┼─ k_msgq (latest value) ──► main thread ──► serial report + RGB LED
 ENV thread ──┘                               ▲
 Button ISR ── EVT_BUTTON ────────────────────┤
 Main timer ── EVT_TIMER  ────────────────────┤
 Sensor health change ── EVT_DRONE ───────────┘
```

| Event | Source | Action of the main thread |
|---|---|---|
| `EVT_BUTTON` | Button B1 interrupt | Selects the next mode (TEST → NORMAL → SIMU → TEST) |
| `EVT_TIMER` | Periodic timer | Executes the report of the current mode |
| `EVT_DRONE` | A sensor changes between healthy and faulty | Refreshes the LED (NORMAL mode only) |

### 1.1 Operating Modes

| Mode | LED | Behaviour |
|---|---|---|
| TEST | Blue | Prints a report of all sensors every 2 s |
| NORMAL | Green / Yellow / Red | Prints the report every 5 s, shows the tracker state on the LED and computes 30 s statistics |
| SIMU | Purple | Not implemented yet |

The sampling periods (in ms) of each mode are defined in `mode_table` in `main.c`:

| Mode | GPS | Environment (T / RH / light) | IMU | Main report |
|---|---|---|---|---|
| TEST | 2000 | 2000 | 50 | 2000 |
| NORMAL | 1000 | 2500 | 20 | 5000 |
| SIMU | 1000 | 1000 | 20 | 100 |

### 1.2 Tracker State (NORMAL Mode)

- **IDLE** (green): all sensors operate correctly and the drone is not moving.
- **FLYING** (yellow): the IMU detects movement.
- **PROBLEM** (red): an impact is detected or a sensor does not provide valid data. This state has priority over the other two. The absence of a GPS fix does not constitute a problem by itself.

While NORMAL mode is active, the sensor threads also report the mean, maximum and minimum values of their measurements every 30 s.

## 2. Hardware and Sensors

| Element | Interface | Notes |
|---|---|---|
| MPU6050 (IMU) | I2C2, address `0x68` | 50 Hz, ±4 g, ±500 °/s |
| Si7021 (temperature and humidity) | I2C2, address `0x40` | 12-bit humidity, 14-bit temperature |
| Adafruit Ultimate GPS (PA6H) | USART1, PB6 (TX) / PB7 (RX), 9600 baud, 8N1 | NMEA GGA sentences |
| LDR (light) | ADC1, channel 5, 12-bit | Converted to a percentage |
| RGB LED | GPIO PC2 (blue), PC1 (green), PB5 (red) | Active low |
| User button B1 | PA0 | Interrupt on the active edge |

All peripherals are described in `boards/nucleo_wl55jc.overlay`, and the application code accesses them through devicetree aliases.

### 2.1 Main Registers and Commands

**MPU6050.** The sensor is configured once at start-up and then read in a single 14-byte burst starting at `0x3B` (accelerometer, temperature and gyroscope).

| Register | Value | Meaning |
|---|---|---|
| `PWR_MGMT_1` (`0x6B`) | `0x09` | Wake-up, internal oscillator, temperature sensor disabled |
| `CONFIG` (`0x1A`) | `0x03` | Digital low-pass filter |
| `SMPLRT_DIV` (`0x19`) | `19` | 50 Hz sample rate |
| `GYRO_CONFIG` (`0x1B`) | `0x08` | ±500 °/s (65.5 LSB per °/s) |
| `ACCEL_CONFIG` (`0x1C`) | `0x08` | ±4 g (8192 LSB per g) |

**Si7021.** Commands sent over I2C:

| Command | Purpose |
|---|---|
| `0xE5` | Humidity measurement (hold master mode). Returns two data bytes followed by a CRC-8 (polynomial `0x31`), which is verified |
| `0xE0` | Reads the temperature acquired during the previous humidity measurement (two bytes, no CRC) |
| `0xE6` / `0xE7` | Writes / reads the user register. The resolution is forced to 12/14 bits and the heater is disabled (mask `0x85`) |

The conversions are RH = 125 · raw / 65536 − 6 (limited to 0–100 %) and T = 175.72 · raw / 65536 − 46.85 °C.

**GPS.** The UART interrupt stores the received bytes in a ring buffer. The GPS thread rebuilds each NMEA sentence, verifies its XOR checksum and parses only the GGA sentence (latitude, longitude, altitude, fix quality and number of satellites). At start-up the module is configured with `PMTK314` (output of GGA sentences only) and `PMTK220` (update rate).

## 3. Error Handling

The sensor threads are designed to tolerate faults without stopping the system:

- **Initialization retries:** if a sensor does not respond at start-up, its thread keeps retrying the initialization.
- **Error tolerance:** a small number of consecutive invalid readings is ignored before the sensor is declared faulty (3 for the environment sensors, 5 for GPS checksums and for the IMU). The counters saturate and cannot overflow.
- **Last valid value:** when a reading fails, the previous valid value is kept. In the case of the GPS, the last known position is retained when the fix is lost.
- **Notification on change:** a thread notifies the main thread (`EVT_DRONE`) only when its health state changes, not on every sample.

## 4. Project Structure

```
src/main.c            main thread, operating modes, timer and events
src/sensors/          IMU.c, gps.c, enviroment.c
src/states/           tests.c, normal.c, simu.c (stub)
src/inputs/button.c   B1 interrupt
src/outputs/rgb.c     RGB LED control
includes/             header files
boards/               devicetree overlay
confs/                Kconfig of the Nucleo board
```

---

## A.1 Visual Studio Code Configuration

The project includes a `.vscode` directory with configuration files for building, flashing, monitoring and debugging the firmware on the **NUCLEO-WL55JC** board.

The configuration files contain **no absolute paths**. The project location is resolved with `${workspaceFolder}`, and the location of each tool is read from Windows user environment variables. Each team member only has to define these variables once on their own machine.

### A.1.1 Configuration Files

| File | Description |
|---|---|
| `launch.json` | Configures Cortex-Debug and GDB for debugging the firmware. |
| `settings.json` | Defines the terminal environment variables (`ZEPHYR_TOOLCHAIN_VARIANT` and `PROJECT_DIR`). |
| `tasks.json` | Defines build, flash, serial monitor, cleanup and debug server tasks. |

### A.1.2 Requirements

The following tools are required to use the provided VS Code tasks:

- Visual Studio Code with the **Cortex-Debug** extension.
- The Zephyr SDK (1.0.x), including the ARM toolchain, GDB and OpenOCD.
- A Zephyr workspace with the `west` tool installed in its Python virtual environment.
- STM32CubeProgrammer for flashing the firmware.
- PuTTY's `plink.exe` for serial communication.

### A.1.3 Environment Variables

The following user environment variables must be defined before using the tasks:

| Variable | Example | Purpose |
|---|---|---|
| `ZEPHYR_SDK_INSTALL_DIR` | `C:\Users\<user>\zephyr-sdk-1.0.1` | Zephyr SDK installation directory (toolchain, GDB, OpenOCD). |
| `ZEPHYR_BASE` | `C:\Users\<user>\zephyrproject\zephyr` | Zephyr source directory inside the west workspace. |
| `ZEPHYR_WORKSPACE` | `C:\Users\<user>\zephyrproject` | Root of the west workspace. Used as the working directory of the build. |
| `WEST_EXE` | `C:\Users\<user>\zephyrproject\.venv\Scripts\west.exe` | `west` executable. |
| `STM32_PROGRAMMER_CLI` | `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe` | STM32CubeProgrammer command-line tool used for flashing. |
| `PLINK_EXE` | `C:\Program Files\PuTTY\plink.exe` | PuTTY command-line client used as serial monitor. |

### A.1.4 Configure Enviroment Variables

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

### A.1.5 Available Tasks

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

### A.1.6 Build and Debug Workflow

1. Open the project root directory in Visual Studio Code.
2. Check that the Zephyr environment and tool paths are configured correctly.
3. Run **West Build (auto)** to compile the firmware.
4. Run **Flash (CubeProgrammer, UR+HWrst)** to program the board.
5. Open **Serial Monitor** to view serial output.
6. Start the **Debug Zephyr - NUCLEO-WL55JC** launch configuration to debug the application.

The debug configuration uses the generated ELF file at `build/zephyr/zephyr.elf` and connects to the GDB server on port `3333`.