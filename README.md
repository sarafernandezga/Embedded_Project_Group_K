# FINAL PROJECT - Implementation of the Embedded Platform for a Drone Tracking and Telemetry IoT System using the ARM-based Microcontroller STM32WL55JC and Zephyr OS


### 5.X. Environmental Monitoring Module (`enviroment.c` / `enviroment.h`)

**Status: Implemented.**

The environmental monitoring module acquires ambient light, relative humidity, and temperature measurements using the ADC and the Si7021 sensor. The measurements are processed by a dedicated Zephyr thread and sent to other application threads through a message queue.

The module is divided into two files:

- **`enviroment.c`** — Implements peripheral initialization, sensor measurements, data conversion, error handling, and thread execution.
- **`enviroment.h`** — Defines the sensor commands, configuration constants, measurement data structure, error status enumeration, and declarations of the synchronization objects shared with other modules.

#### Devicetree Overlay Configuration

The module uses the Devicetree configuration to access the ADC channel and the I²C interface connected to the Si7021 sensor.

**ADC configuration**

The ADC channel is obtained from the `zephyr,user` node using the following definitions:

```c
#define ADC_NODE DT_PATH(zephyr_user)
static const struct adc_dt_spec adc_ch =
    ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 0);
```

`DT_PATH(zephyr_user)` identifies the `zephyr,user` node, while `ADC_DT_SPEC_GET_BY_IDX()` retrieves the first ADC channel listed in its `io-channels` property. This allows Zephyr to provide the ADC configuration defined in the overlay without hardcoding the peripheral configuration in the application code.

**I²C configuration**

The Si7021 sensor is accessed through the following definitions:

```c
#define I2C_NODE DT_ALIAS(si7021)
static const struct device *i2c = DEVICE_DT_GET(I2C_NODE);
```

The `si7021` alias identifies the sensor's I²C controller through the Devicetree configuration. The sensor uses the 7-bit I²C address `0x40`, which is defined in `enviroment.h` as `SI7021_ADDR`.

#### Header File and Shared Definitions (`enviroment.h`)

The `enviroment.h` file centralizes the definitions required by the environmental monitoring module and other parts of the application.

**Sensor configuration constants**

The header defines the Si7021 I²C address and the commands used to communicate with the sensor:

| Constant | Value | Description |
|---|---|---|
| `SI7021_ADDR` | `0x40` | Si7021 I²C address |
| `SI7021_READ_USER_REG` | `0xE7` | Read the user register |
| `SI7021_WRITE_USER_REG` | `0xE6` | Write the user register |
| `SI7021_MEASURE_RH` | `0xE5` | Measure relative humidity using hold-master mode |
| `SI7021_MEASURE_TEMP` | `0xE3` | Measure temperature using hold-master mode |
| `SI7021_READ_TEMP_FROM_RH` | `0xE0` | Read the temperature associated with the previous humidity measurement |
| `CRC_POLINOMIAL` | `0x31` | Polynomial used for CRC-8 verification |

The header also defines the calibration limits used by the light measurement function. These constants establish the reference values used to map the measured light level to a percentage.

**Measurement data structure**

The `env_m` structure groups the results of a measurement cycle into a single message:

```c
struct env_m {
    float temp;
    float hum;
    float light;
    enum env_status status;
};
```

Its fields are:

- `temp`: measured temperature in degrees Celsius.
- `hum`: measured relative humidity as a percentage.
- `light`: calibrated light level expressed as a percentage.
- `status`: result of the measurement cycle, indicating whether the acquisition succeeded or an error occurred.

Grouping the measurements and their status into a single structure allows the module to transfer all the results through one message queue operation.

**Error status enumeration**

The `env_status` enumeration defines the possible states reported by the environmental monitoring module:

| Status | Description |
|---|---|
| `ALL_OK` | All measurements completed successfully. |
| `LIGHT_FAIL` | Light measurement failed. |
| `SI7021_FAIL` | Humidity measurement failed. |
| `TEMP_FAIL` | Temperature measurement failed. |
| `MULTIPLE_FAIL` | Multiple measurement operations failed. |
| `ADC_INIT_FAIL` | ADC initialization failed. |
| `SI7021_INIT_FAIL` | Si7021 I²C device initialization check failed. |

These values allow the receiving thread to distinguish successful measurements from different types of errors.

**Shared synchronization objects**

The header declares the semaphore and message queue as external objects:

```c
extern struct k_msgq env_q;
extern struct k_sem sem_env_gate;
```

Their actual definitions are located in `enviroment.c`. Declaring them with `extern` allows other source files to access the same synchronization objects without defining duplicate instances.

#### ADC Initialization and Light Measurement

The ADC is initialized by `initialize_adc()`. This function performs three checks and configuration steps:

1. `adc_is_ready_dt()` verifies that the ADC device is ready.
2. `adc_channel_setup_dt()` configures the selected ADC channel.
3. `adc_sequence_init_dt()` initializes the ADC measurement sequence.

If any operation fails, the function returns the corresponding negative error code.

The `take_LDC_measure()` function reads the ADC using `adc_read_dt()`. The conversion result is stored in the `sample` variable, which is referenced by the ADC sequence buffer.

The raw measurement is converted into an initial percentage using the following formula:

\[
L_{\mathrm{raw}}=100-\frac{\mathrm{sample}\times100}{4095}
\]

The value `4095` represents the maximum reading of a 12-bit ADC. The calculation inverts the raw reading so that a lower ADC value corresponds to a higher initial light percentage.

The result is then calibrated using the reference values defined at `enviroment.h`:

\[
L_{\mathrm{cal}}=
\frac{(L_{\mathrm{raw}}-MIN_CALIBRATE_LCD)\times100}{MAX_CALIBRATE_LCD-MIN_CALIBRATE_LCD}
\]

Finally, the result is limited to the range from 0% to 100%. This prevents the calibrated value from exceeding the expected range.

#### Si7021 Initialization and Sensor Communication

The `initialize_Si7021()` function checks whether the I²C device is ready using `device_is_ready()`. If the device is unavailable, the function returns `-ENODEV`.

This check verifies the readiness of the I²C device, but it does not independently confirm that the physical Si7021 sensor is connected and responding.

**Humidity measurement**

The `measure_humidity()` function obtains the relative humidity using the Si7021 sensor.

First, the function sends the command `SI7021_MEASURE_RH` (`0xE5`) through `i2c_write()`. The command uses hold-master mode, allowing the sensor to perform the conversion while keeping the I²C transaction active.

The function then reads three bytes using `i2c_read()`:

- First byte: humidity measurement MSB.
- Second byte: humidity measurement LSB.
- Third byte: CRC-8 checksum.

The first two bytes are combined into a 16-bit raw value:

\[
RH_{\mathrm{raw}}=(\mathrm{MSB}\ll8)\;|\;\mathrm{LSB}
\]

The relative humidity is calculated using the sensor conversion formula:

\[
RH=125\times\frac{RH_{\mathrm{raw}}}{65536}-6
\]

Before converting the result, the function verifies the checksum. If the received CRC does not match the calculated value, it returns `-EIO`.

**CRC-8 verification**

The `si7021_crc8()` function implements the checksum calculation using the polynomial `0x31`.

The CRC starts at `0x00`. Each data byte is XORed with the current CRC value, after which the function processes its eight bits. For each bit, the CRC is shifted left; if the most significant bit was set, the polynomial is XORed with the shifted result.

This verification helps detect errors in the two bytes received from the sensor.

**Temperature measurement**

The `measure_temperature()` function retrieves the temperature associated with the previous humidity measurement by sending the command `SI7021_READ_TEMP_FROM_RH` (`0xE0`).

The function reads two bytes, combines them into a 16-bit raw value, and converts the result to degrees Celsius:

\[
T=175.72\times\frac{T_{\mathrm{raw}}}{65536}-46.85
\]

The temperature is acquired only if the humidity measurement succeeds. This follows the selected measurement sequence, in which the temperature reading is associated with the preceding humidity conversion.

#### Thread Architecture

The environmental monitoring thread is created using `K_THREAD_DEFINE()`:

```c
K_THREAD_DEFINE(
    my_thread_enviroment_id,
    1024,
    my_thread_enviroment,
    NULL, NULL, NULL,
    4,
    0,
    0
);
```

The thread uses a stack size of 1024 bytes and priority 4. Its entry function, `my_thread_enviroment()`, coordinates initialization, measurements, error handling, and message transmission.

At startup, the thread initializes the ADC and checks the I²C device readiness. It then enters an infinite loop and waits for a measurement request by calling `k_sem_take()` with `K_FOREVER`.

When the semaphore is released, the thread resets the measurement variables and performs the following operations:

1. Reads and calibrates the ambient light level.
2. Measures relative humidity using the Si7021.
3. Reads the associated temperature if the humidity measurement succeeds.
4. Determines the measurement status based on the returned error codes.
5. Groups the results into an `env_m` structure and sends the message to `env_q`.

This event-driven approach allows the thread to remain blocked while waiting for a measurement request instead of continuously polling the sensors.

#### Thread Synchronization and Message Queue

The module uses a binary semaphore and a message queue to coordinate measurement requests and transfer the acquired data.

**Measurement semaphore**

The semaphore is defined using:

```c
K_SEM_DEFINE(sem_env_gate, 0, 1);
```

The initial count is zero and the maximum count is one. Therefore, the environmental thread blocks at `k_sem_take()` until another part of the application releases the semaphore using `k_sem_give()`.

The current implementation does not define a timer to release this semaphore. Consequently, measurement requests must be triggered by another thread or by an additional mechanism implemented elsewhere in the application.

**Measurement message queue**

The message queue is defined using:

```c
K_MSGQ_DEFINE(env_q, sizeof(struct env_m), 1, 4);
```

The queue stores one message of type `struct env_m`, with a memory alignment of four bytes. After each measurement cycle, the thread sends the message using `k_msgq_put()` with `K_NO_WAIT`.

This prevents the environmental thread from blocking while trying to send a message. However, if the queue is full, the operation can fail and the new message will not be stored. The return value should therefore be checked to detect unsuccessful transmissions.

#### Main Application Integration and Testing

The main application coordinates the measurement process by releasing `sem_env_gate` to request a new acquisition cycle. It then retrieves the resulting `env_m` message from `env_q` and checks its `status` field.

When the status is `ALL_OK`, the application can print the measured temperature, humidity, and light level to the serial console. If an error status is received, it can report the corresponding failure instead of treating the measurements as valid.

This test verifies the complete data path between the environmental monitoring thread and the receiving application: measurement request, sensor acquisition, result packaging, queue transmission, and status evaluation.

#### Implementation Considerations

The current implementation has several aspects to consider during error handling:

- Initialization errors are stored in `env_status`, but the thread continues into the measurement loop. Subsequent measurement cycles can overwrite the initialization status.
- A failed humidity measurement prevents the temperature reading from being attempted.
- The current `MULTIPLE_FAIL` condition specifically detects simultaneous ADC and humidity measurement failures. It does not cover every possible combination of errors, such as ADC and temperature failures when humidity succeeds.
- The return value of `k_msgq_put()` is stored in `ret_queue`, but it is not currently checked.
- The light calibration limits are defined in the header, but the current calibration formula uses the literal values defined in `enviroment.h`.

These points should be considered when refining the error-reporting logic and making the module easier to maintain.

### 5.X. GPS Module (`gps.c` / `gps.h`)

**Status: Initial implementation.**

The GPS module acquires geographical information using the Adafruit Ultimate GPS Breakout v3, based on the PA6H GPS receiver. The module communicates with the microcontroller through a UART interface and receives NMEA sentences containing GPS information.

The received data is processed by a dedicated Zephyr thread. This thread reconstructs the NMEA sentences, extracts the relevant information from GGA messages, converts geographical coordinates into decimal degrees, and sends valid GPS measurements to the rest of the application through a message queue.

The module is divided into two files:

- **`gps.c`** — Implements UART initialization, GPS configuration, interrupt-driven reception, buffering, NMEA sentence processing, coordinate conversion and thread execution.
- **`gps.h`** — Defines the GPS data structure, configuration constants, buffer parameters and declarations of the synchronization objects shared with other modules.

#### Hardware and Devicetree Configuration

The GPS receiver communicates with the microcontroller using UART. The USART1 peripheral is configured in the Devicetree overlay.

The relevant configuration is:

```dts
&usart1 {
    status = "okay";
    current-speed = <9600>;
    pinctrl-0 = <&usart1_tx_pb6 &usart1_rx_pb7>;
    pinctrl-names = "default";
};
```

The UART interface uses a baud rate of 9600 bits per second, corresponding to the default communication speed of the GPS receiver.

The connection between the GPS module and the NUCLEO-WL55JC board is:

| GPS Breakout | STM32WL55JC |
|---|---|
| TX | PB7 (USART1_RX) |
| RX | PB6 (USART1_TX) |
| GND | GND |
| VIN | 3.3 V |

The GPS TX pin is connected to the microcontroller RX pin because the receiver transmits data to the microcontroller. Similarly, the GPS RX pin is connected to the microcontroller TX pin to allow the microcontroller to send configuration commands.

The UART configuration is accessed in the application using the following definitions:

```c
#define GPS_UART_NODE DT_NODELABEL(usart1)

static const struct device *gps_uart =
    DEVICE_DT_GET(GPS_UART_NODE);
```

`DT_NODELABEL(usart1)` identifies the USART1 peripheral in the Devicetree, while `DEVICE_DT_GET()` obtains the corresponding Zephyr device instance.

#### Header File and Shared Definitions (`gps.h`)

The `gps.h` file centralizes the definitions required by the GPS module and the rest of the application.

**GPS measurement data structure**

The `gps_data` structure groups the information extracted from a valid GPS measurement:

```c
struct gps_data {
    float latitude;
    float longitude;
    float altitude;
    float hdop;
    int fix_quality;
    int satellites;
};
```

Its fields are:

- `latitude`: geographical latitude expressed in decimal degrees.
- `longitude`: geographical longitude expressed in decimal degrees.
- `altitude`: altitude above mean sea level, expressed in metres.
- `hdop`: Horizontal Dilution of Precision, an indicator of horizontal positioning quality.
- `fix_quality`: indicates the type or availability of the GPS fix. A value of zero means that no valid fix is available.
- `satellites`: number of satellites used to calculate the position.

Grouping the GPS measurements into a single structure allows the module to transfer the complete set of information through one message queue operation.

**Shared synchronization objects**

The GPS module uses a semaphore and a message queue to coordinate data reception and transfer the processed measurements to other application threads.

These objects are declared in the header using `extern` declarations:

```c
extern struct k_msgq gps_msgq;
```

Their definitions are located in `gps.c`. This allows other source files, such as `main.c`, to access the same message queue without creating additional instances.

#### UART Initialization and GPS Configuration

The `gps_uart_init()` function initializes the UART interface used by the GPS receiver.

The initialization process includes:

1. Checking whether the USART1 device is ready using `device_is_ready()`.
2. Registering the UART interrupt callback.
3. Enabling UART reception interrupts.
4. Sending configuration commands to the GPS receiver.

The GPS receiver uses PMTK commands to configure its output.

The current configuration requests GGA sentences and sets the output update rate to 1 Hz. This reduces the amount of incoming data and focuses the application on the GPS information required by the project.

The commands are transmitted through the UART interface using `uart_poll_out()`.

The GPS receiver continues transmitting NMEA sentences automatically after initialization.

#### UART Reception and Interrupt Handling

The GPS module uses interrupt-driven UART reception to receive data without continuously polling the peripheral.

The UART callback is responsible for reading the received bytes from the UART FIFO and storing them in a ring buffer.

The reception process follows these steps:

1. The UART interrupt is triggered when data is available.
2. The callback reads the received bytes using `uart_fifo_read()`.
3. The bytes are copied into the GPS ring buffer using `ring_buf_put()`.
4. A semaphore is released to notify the GPS processing thread that data is available.

The interrupt callback does not perform NMEA parsing or lengthy processing operations. Its responsibility is limited to receiving and buffering the incoming bytes.

This approach reduces the processing time inside the interrupt context and allows the GPS thread to handle the received data independently.

**Ring buffer**

The ring buffer temporarily stores the incoming UART bytes until the GPS thread processes them.

This is particularly useful because the GPS receiver transmits complete NMEA sentences as a continuous stream of characters. The UART callback may receive only part of a sentence during a single interrupt.

The ring buffer allows the application to reconstruct the original byte stream even when the data arrives in several reception events.

#### NMEA Sentence Processing

The GPS receiver transmits information using the NMEA 0183 format. NMEA sentences are ASCII messages that begin with the `$` character and contain comma-separated fields.

For example, a GGA sentence has the following structure:

```text
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
```

The application uses the `gps_process_byte()` function to reconstruct complete NMEA sentences from the bytes stored in the ring buffer.

The function performs the following operations:

1. Detects the beginning of a new sentence when the `$` character is received.
2. Stores incoming characters in the NMEA buffer.
3. Checks that the buffer does not exceed its maximum capacity.
4. Detects the end of the sentence when the newline character (`\n`) is received.
5. Adds the null terminator to obtain a valid C string.
6. Passes the complete sentence to the parsing functions.

Once a complete sentence has been reconstructed, the `gps_split_nmea()` function separates its fields.

This function replaces commas with null terminators and stores pointers to the individual fields in an array.

Empty fields are preserved, which is important because NMEA sentences may contain unavailable measurements, especially when the GPS receiver has not acquired a valid position.

#### GGA Message Interpretation

The current implementation focuses on GGA sentences, which provide information about the GPS position and its quality.

The GGA fields are interpreted as follows:

| Field | Description |
|---|---|
| `fields[1]` | UTC time |
| `fields[2]` | Latitude |
| `fields[3]` | Latitude direction (N/S) |
| `fields[4]` | Longitude |
| `fields[5]` | Longitude direction (E/W) |
| `fields[6]` | Fix quality |
| `fields[7]` | Number of satellites used |
| `fields[8]` | Horizontal Dilution of Precision (HDOP) |
| `fields[9]` | Altitude |
| `fields[10]` | Altitude unit |
| `fields[11]` | Geoid separation |
| `fields[12]` | Geoid separation unit |

The `gps_process_fields()` function checks whether the received sentence is a supported GGA message (`$GPGGA` or `$GNGGA`) and extracts the corresponding information.

The function also checks the fix quality to determine whether the receiver has obtained a valid position.

A fix quality of zero indicates that no valid GPS position is available. In this case, the application does not send the measurement to the GPS message queue.

This prevents invalid coordinates from being treated as valid positioning information.

#### Coordinate Conversion

The latitude and longitude values provided by the GPS receiver use the NMEA degrees-and-minutes format.

For example:

- Latitude: `4807.038,N`
- Longitude: `01131.000,E`

The application converts these values into decimal degrees using the `gps_convert_coordinate()` function.

The conversion separates the degrees from the minutes and divides the minutes by 60:

\[
\text{Decimal degrees} =
\text{Degrees} + \frac{\text{Minutes}}{60}
\]

The direction field determines the sign of the resulting coordinate:

- North (`N`) and East (`E`) produce positive values.
- South (`S`) and West (`W`) produce negative values.

This conversion makes the coordinates easier to use and display in the rest of the application.

#### GPS Thread Architecture

The GPS processing thread is created using `K_THREAD_DEFINE()`.

```c
K_THREAD_DEFINE(
    my_thread_gps_id,
    1024,
    my_thread_gps,
    NULL, NULL, NULL,
    4,
    0,
    0
);
```

The thread uses a stack size of 1024 bytes and priority 4.

Its entry function, `my_thread_gps()`, coordinates UART initialization, reception, and NMEA processing.

At startup, the thread initializes the ring buffer and calls `gps_uart_init()` to configure the UART interface and enable reception.

After initialization, the thread enters an infinite loop and waits for the reception semaphore using `k_sem_take()` with `K_FOREVER`.

When the semaphore is released by the UART callback, the thread retrieves the available bytes from the ring buffer using `ring_buf_get()`.

Each received byte is passed to `gps_process_byte()`, which reconstructs complete NMEA sentences and processes them.

The thread repeats this operation until there are no more bytes available in the ring buffer.

This architecture separates UART reception from GPS data processing and prevents the application from continuously polling the UART peripheral.

#### Thread Synchronization and Message Queue

The GPS module uses a semaphore and a message queue to coordinate reception and transfer processed measurements.

**GPS reception semaphore**

The semaphore is defined as a binary semaphore:

```c
K_SEM_DEFINE(gps_sem, 0, 1);
```

Its initial count is zero and its maximum count is one.

The GPS processing thread blocks while waiting for the semaphore. When the UART callback stores new data in the ring buffer, it releases the semaphore using `k_sem_give()`.

This mechanism notifies the processing thread that new bytes are available without requiring continuous polling.

**GPS message queue**

The GPS message queue stores processed GPS measurements using the `gps_data` structure.

The queue is defined using Zephyr's `K_MSGQ_DEFINE()` mechanism.

After a valid GGA message has been processed, the GPS module sends the resulting measurement to the queue using `k_msgq_put()`.

The queue allows the GPS processing thread to operate independently from the main application thread.

Only measurements with a valid fix quality are currently sent to the queue. This ensures that the receiving application does not process a position when the GPS receiver has not acquired a valid fix.

#### Main Application Integration and Testing

The main application retrieves GPS measurements from `gps_msgq` and processes them independently from the environmental measurements.

The GPS message queue is accessed using `k_msgq_get()` with `K_NO_WAIT`, allowing the main application to check whether a GPS measurement is available without blocking while waiting for new data.

When a message is available, the application can access the latitude, longitude, altitude, HDOP, fix quality, and number of satellites stored in the `gps_data` structure.

The current implementation has been tested by checking the UART reception and the reconstruction of complete NMEA sentences.

A GGA sentence containing empty position fields, a fix quality of zero, and zero satellites was successfully received and parsed. This confirms that the application can correctly handle a receiver that has not yet acquired a valid GPS position.

The parsing functions can also be tested using a simulated GGA sentence containing known coordinates:

```text
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
```

This sentence can be used to verify the extraction of the fields and the conversion of latitude and longitude. It represents simulated test data and does not correspond to a position measured by the physical receiver.

A test with a real GPS fix still requires suitable satellite visibility.

#### Implementation Considerations

The current GPS implementation has several aspects to consider during future development:

- The GPS receiver can transmit NMEA sentences without having acquired a valid position. Therefore, receiving a complete sentence does not necessarily mean that valid coordinates are available.
- The current implementation focuses on GGA messages. Other NMEA sentence types, such as RMC, have not yet been integrated into the position-processing logic.
- The NMEA checksum is present in the received sentences, but checksum validation is not yet implemented in the current parsing process.
- Empty NMEA fields must be handled carefully to avoid using invalid or outdated measurement values.
- The ring buffer has a limited capacity. If incoming data exceeds the available space, bytes may be lost.
- The return value of `k_msgq_put()` should be checked to detect cases where the GPS message queue is full and a measurement cannot be stored.
- A real GPS positioning test is still required to verify the complete acquisition process under suitable satellite reception conditions.

These considerations will be addressed in future iterations to improve the reliability of the GPS module and its integration with the rest of the application.



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