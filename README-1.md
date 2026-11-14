# Drone Tracking and Telemetry on the STM32WL55JC (Zephyr OS)

Final project of *Embedded Platforms and Communications for IoT* (UPM, IoT Master) — Group K.

The board reads a GPS, an IMU, a temperature/humidity sensor and a light sensor, decides what the "drone" is doing (idle, flying, in trouble), shows it on the RGB LED and prints a report over the serial console. A button switches between operating modes.

**Status: v1.0.0.** TEST and NORMAL modes are complete. SIMU mode (the `$SIMU1` frame sent to the simulator at 10 Hz) is **not implemented yet**; the mode exists and the LED turns purple, but it sends nothing.

## How it works

Each sensor lives in its own Zephyr thread and publishes its latest reading in a message queue of size 1 (a new value replaces the old one, so nobody ever reads stale data by accident). The main thread sleeps until something happens, wakes up through event flags, collects the latest value of every sensor and reacts.

```
 GPS thread ──┐
 IMU thread ──┼─ k_msgq (latest value) ──► main thread ──► serial report + RGB LED
 ENV thread ──┘                               ▲
 Button ISR ── EVT_BUTTON ────────────────────┤
 Main timer ── EVT_TIMER  ────────────────────┤
 Sensor health change ── EVT_DRONE ───────────┘
```

| Event | Source | What main does |
|---|---|---|
| `EVT_BUTTON` | Button B1 interrupt | Moves to the next mode (TEST → NORMAL → SIMU → TEST) |
| `EVT_TIMER` | Periodic timer | Runs the report of the current mode |
| `EVT_DRONE` | A sensor became healthy/faulty | Refreshes the LED (NORMAL only) |

### Modes

| Mode | LED | What it does |
|---|---|---|
| TEST | Blue | Prints a report of all sensors every 2 s |
| NORMAL | Green / Yellow / Red | Same report every 5 s, tracker state on the LED, 30 s statistics |
| SIMU | Purple | Placeholder — **pending** |

Sampling periods (ms) are set per mode in `mode_table` in `main.c`:

| Mode | GPS | Env (T/RH/light) | IMU | Main report |
|---|---|---|---|---|
| TEST | 2000 | 2000 | 50 | 2000 |
| NORMAL | 1000 | 2500 | 20 | 5000 |
| SIMU | 1000 | 1000 | 20 | 100 |

### Tracker state (NORMAL mode)

- **IDLE** (green): all sensors fine, the drone is not moving.
- **FLYING** (yellow): the IMU detects motion.
- **PROBLEM** (red): an impact is detected or a sensor is not delivering valid data. It has priority over the other two. Not having a GPS fix is *not* a problem by itself.

While in NORMAL, every 30 s the sensor threads also report mean / max / min of their measurements.

## Hardware and sensors

| Sensor | Interface | Notes |
|---|---|---|
| MPU6050 (IMU) | I2C2, address `0x68` | 50 Hz, ±4 g, ±500 °/s |
| Si7021 (temp. + humidity) | I2C2, address `0x40` | 12-bit RH, 14-bit temperature |
| Adafruit Ultimate GPS (PA6H) | USART1, PB6 (TX) / PB7 (RX), 9600 8N1 | NMEA GGA sentences |
| LDR (light) | ADC1 channel 5, 12-bit | Converted to a percentage |
| RGB LED | GPIO PC2 (blue), PC1 (green), PB5 (red) | Active low |
| Button B1 | PA0 | Interrupt, no debounce |

Everything is declared in `boards/nucleo_wl55jc.overlay`; the code only uses devicetree aliases.

### The registers that matter

**MPU6050** — configured once at start-up, then read in a single 14-byte burst starting at `0x3B` (accel, temperature, gyro):

| Register | Value | Meaning |
|---|---|---|
| `PWR_MGMT_1` `0x6B` | `0x09` | Wake up, internal oscillator, temperature sensor off |
| `CONFIG` `0x1A` | `0x03` | Digital low-pass filter |
| `SMPLRT_DIV` `0x19` | `19` | 50 Hz sample rate |
| `GYRO_CONFIG` `0x1B` | `0x08` | ±500 °/s (65.5 LSB per °/s) |
| `ACCEL_CONFIG` `0x1C` | `0x08` | ±4 g (8192 LSB per g) |

**Si7021** — commands over I2C:

| Command | Use |
|---|---|
| `0xE5` | Measure humidity (hold master). Returns 2 data bytes + CRC-8 (poly `0x31`), which we check |
| `0xE0` | Read the temperature taken during that humidity measurement (2 bytes, no CRC) |
| `0xE6` / `0xE7` | Write / read the user register. We force 12/14-bit resolution and heater off (mask `0x85`) |

RH = 125 · raw / 65536 − 6 (clamped to 0–100 %), T = 175.72 · raw / 65536 − 46.85 °C.

**GPS** — the UART interrupt fills a ring buffer, the GPS thread rebuilds each NMEA sentence, verifies the XOR checksum and parses only `$..GGA` (latitude, longitude, altitude, fix quality, satellites). At start-up we send `PMTK314` (output GGA only) and `PMTK220` (update rate).

## Error handling

Sensors fail in real life, so the threads are written not to panic:

- **Initialisation retries:** if a sensor does not answer at boot, its thread keeps trying instead of dying.
- **Error tolerance:** a few consecutive bad reads (3 for the environment sensors, 5 for GPS checksums and IMU) are ignored before the sensor is declared faulty. The counters saturate, so they cannot overflow.
- **Last valid value:** on a bad read the previous good value is kept and flagged as stale, and the GPS keeps the last known position when the fix is lost.
- **Notify only on change:** a thread wakes main (`EVT_DRONE`) only when its health changes, not on every sample.

## Known limitations

- SIMU mode is not implemented.
- The IMU was seen once returning zeros after a cold power-up; a reset fixed it. Cause not confirmed.
- Position integration from the accelerometer does not remove gravity yet, so it is only a rough estimate.
- Impact detection has no latch/hysteresis: PROBLEM clears as soon as the acceleration drops.
- The button has no debounce.
- Some `k_sleep` calls remain in the initialisation retry loops (IMU, GPS, environment).

## Project layout

```
src/main.c            main thread, modes, timer, events
src/sensors/          IMU.c, gps.c, enviroment.c
src/states/           tests.c, normal.c, simu.c (stub)
src/inputs/button.c   B1 interrupt
includes/             headers (states.h, IMU.h, gps.h, enviroment.h, button.h)
boards/               devicetree overlay
confs/                Kconfig for the Nucleo board
```

## Build and flash

See the VS Code section below for the environment variables and tasks. In short:

```
west build -b nucleo_wl55jc -p auto
west flash
```

Open the board's serial console (115200 baud) to see the reports.

---

## X. Visual Studio Code Configuration

The project includes a `.vscode` directory with configuration files for building, flashing, monitoring and debugging the firmware on the **NUCLEO-WL55JC** board.

The configuration files contain **no absolute paths**. The project location is resolved with `${workspaceFolder}`, and the location of each tool is read from Windows user environment variables. Each team member only has to define these variables once on their own machine.

### 7.1 Configuration Files

| File | Description |
|---|---|
| `launch.json` | Configures Cortex-Debug and GDB for debugging the firmware. |
| `settings.json` | Defines the terminal environment variables (`ZEPHYR_TOOLCHAIN_VARIANT` and `PROJECT_DIR`). |
| `tasks.json` | Defines build, flash, serial monitor, cleanup and debug server tasks. |

### 7.2 Requirements

The following tools are required to use the provided VS Code tasks:

- Visual Studio Code with the **Cortex-Debug** extension.
- The Zephyr SDK (1.0.x), including the ARM toolchain, GDB and OpenOCD.
- A Zephyr workspace with the `west` tool installed in its Python virtual environment.
- STM32CubeProgrammer for flashing the firmware.
- PuTTY's `plink.exe` for serial communication.

### 7.3 Environment Variables

The following user environment variables must be defined before using the tasks:

| Variable | Example | Purpose |
|---|---|---|
| `ZEPHYR_SDK_INSTALL_DIR` | `C:\Users\<user>\zephyr-sdk-1.0.1` | Zephyr SDK installation directory (toolchain, GDB, OpenOCD). |
| `ZEPHYR_BASE` | `C:\Users\<user>\zephyrproject\zephyr` | Zephyr source directory inside the west workspace. |
| `ZEPHYR_WORKSPACE` | `C:\Users\<user>\zephyrproject` | Root of the west workspace. Used as the working directory of the build. |
| `WEST_EXE` | `C:\Users\<user>\zephyrproject\.venv\Scripts\west.exe` | `west` executable. |
| `STM32_PROGRAMMER_CLI` | `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe` | STM32CubeProgrammer command-line tool used for flashing. |
| `PLINK_EXE` | `C:\Program Files\PuTTY\plink.exe` | PuTTY command-line client used as serial monitor. |

### Configure Enviroment Variables

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

### 7.4 Available Tasks

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

### 7.5 Build and Debug Workflow

1. Open the project root directory in Visual Studio Code.
2. Check that the Zephyr environment and tool paths are configured correctly.
3. Run **West Build (auto)** to compile the firmware.
4. Run **Flash (CubeProgrammer, UR+HWrst)** to program the board.
5. Open **Serial Monitor** to view serial output.
6. Start the **Debug Zephyr - NUCLEO-WL55JC** launch configuration to debug the application.

The debug configuration uses the generated ELF file at `build/zephyr/zephyr.elf` and connects to the GDB server on port `3333`.