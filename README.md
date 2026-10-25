# FINAL PROJECT - Implementation of the Embedded Platform for a Drone Tracking and Telemetry IoT System using the ARM-based Microcontroller STM32WL55JC and Zephyr OS








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