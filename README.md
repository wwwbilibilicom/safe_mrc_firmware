# Safe MRC - STM32H7 Magnetorheological Clutch Controller

## Overview

Safe MRC is an embedded control system for Magnetorheological Clutch (MRC) devices using STM32H7 microcontrollers. The system provides precise torque, current, and voltage control, collision detection, and real-time communication capabilities.

## Features

- **Precise Torque/Current/Voltage Control**: Supports both voltage and current (feedforward+PI) control modes, with easy switching.
- **Collision Detection**: Real-time collision detection and safety response.
- **DMA Communication**: High-efficiency UART communication using DMA.
- **Request-Response Protocol**: Master-slave communication pattern to avoid bus conflicts.
- **Multi-sensor Support**: Encoder feedback, voltage monitoring, and temperature sensing.
- **Safety Features**: Automatic demagnetization on collision detection.
- **Modular Design**: Clean separation of hardware drivers and application logic.
- **Comprehensive Filtering**: Built-in moving average, low-pass, Kalman, and band-pass filters.
- **PWM Nonlinearity Compensation**: Lookup table (LUT) and interpolation for accurate low-voltage output.

## Project Structure

```
safe-MRC/
├── Common/           # Common utilities (filters, PID controllers)
├── Core/            # STM32H7 core files and main application
├── Device/          # Device drivers and communication modules
│   ├── Inc/         # Header files
│   └── Src/         # Source files
├── Drivers/         # STM32H7 HAL drivers
└── MDK-ARM/         # Keil MDK-ARM project files
```

## 1. Control Modes & Key Logic

### Control Modes

| Enum Name               | Value | Description                |
|------------------------|-------|----------------------------|
| `MRC_VOLTAGE_CONTROL`  | 0     | Voltage control mode       |
| `MRC_CURRENT_CONTROL`  | 1     | Coil current control mode  |

### Key Reaction Logic

- **KEY1**:  
  - Voltage mode: Decrease voltage by 1V per press  
  - Current mode: Decrease target current by 0.1A per press
- **KEY2**:  
  - Voltage mode: Increase voltage by 1V per press  
  - Current mode: Increase target current by 0.1A per press
- **LED1/LED2**: Toggle on key press for feedback.
- **UART**: Prints current set value after each key press.

## 2. Coil Current Control (Feedforward + PI)

- **Function**: `float MRC_CoilCurrentControl_Update(Device_MRC_t *MRC, float i_ref, float i_meas, float R_coil, float L_coil, float Ts);`
- **Parameters**:
  - `i_ref`: Target current (A)
  - `i_meas`: Measured current (A)
  - `R_coil`: Coil resistance (Ohm, use DC value from multimeter)
  - `L_coil`: Coil inductance (H, use 20kHz value from LCR meter)
  - `Ts`: Sample time (seconds)
- **PI Parameters**: Configured via macros in `drv_mrc.h`:
  - `COIL_PID_KP`, `COIL_PID_KI`, `COIL_PID_KD`, `COIL_PID_TS`, `COIL_PID_MAX_OUT`, `COIL_PID_MIN_OUT`
- **Tuning Advice**: Start with recommended values, increase `Kp` for response, then add `Ki` for zero steady-state error. Reduce `Kp`/`Ki` if overshoot occurs.

## 3. Filters

- **Simple Low-Pass Filter**: `SimpleLowPassFilter` (exponential smoothing, supports cutoff frequency or alpha initialization)
- **Moving Average**: `movingAverage_t` (removes high-frequency noise)
- **First-Order Kalman**: `FirstOrderKalmanFilter` (for noisy signals)
- **Band-Pass**: `BandPassFilter` (extracts specific frequency bands)

**Usage**: See `filter.h` for initialization and update functions. All filters have Doxygen-style comments and are easy to use.

## 4. PWM Target Voltage vs Output Voltage Lookup Table

- **Function**: `float LookupMeasuredVoltageByTarget(float v_target);`
- **How it works**: For |v_target| ≤ 5V, uses LUT and linear interpolation; for |v_target| > 5V, uses v_target directly.
- **Purpose**: Compensates for H-bridge dead-zone and nonlinearity at low voltages.
- **Table Data**: (Current is for reference only; focus on voltage mapping)

| Target Voltage (V) | Measured Voltage (V) | Duty Cycle (0~1) | Output Current (A) |
| ------------------ | -------------------- | ---------------- | ------------------ |
| 0                  | 0.0017               | 0                | -0.062             |
| 0.1                | 0.0017               | 0.00833          | -0.062             |
| 0.2                | 0.0017               | 0.01667          | 0.062              |
| 0.3                | 0.00295              | 0.025            | 0.062              |
| 0.4                | 0.00735              | 0.03333          | 0.062              |
| 0.5                | 0.0142               | 0.04167          | 0.067              |
| 0.6                | 0.0434               | 0.05             | 0.073              |
| 0.7                | 0.1336               | 0.05833          | 0.103              |
| 0.8                | 0.2831               | 0.06667          | 0.147              |
| 0.9                | 0.50175              | 0.075            | 0.201              |
| 1                  | 0.66635              | 0.08333          | 0.242              |
| 1.1                | 0.828                | 0.09167          | 0.289              |
| ...                | ...                  | ...              | ...                |
| 5.0                | 5.752                | 0.41667          | 1.200              |

**Note**: The full table is in code and can be updated as needed.

## 5. Device Structure & Main Interfaces

### Device_MRC_t Main Members

| Member                  | Type/Macro             | Description                        |
|------------------------|------------------------|-------------------------------------|
| `mode`                 | `MRC_Mode`             | Work mode                           |
| `state_phase`          | `MRC_State`            | State machine phase                 |
| `VNH7040`              | `Device_VNH7040_t`     | H-bridge driver structure           |
| `coil_pid`             | `PID_Controller`       | Coil current PI controller          |
| `coil_current_filter`  | `SimpleLowPassFilter`  | Coil current low-pass filter        |
| `control_mode`         | `MRC_ControlMode`      | Control mode (voltage/current)      |
| ...                    | ...                    | ...                                 |

**All members and functions are documented in the header files with clear comments.**

## 6. Communication Protocol (Detailed)

### Hardware Interface

- **Physical Layer**: RS-485 differential bus
- **Baudrate**: 4 Mbps (4000000 bps)
- **Topology**: Multi-drop (one master, multiple slaves)
- **Connector**: Standard 485-A/B differential pair
- **Termination**: 120Ω resistors recommended at both ends
- **Cable**: Shielded twisted pair, length < 50m for 4Mbps

### Protocol Overview

- **Pattern**: Request-Response (Master sends command, slave responds)
- **Frame Format**: Binary, fixed length
- **Error Checking**: CRC-16-CCITT (poly=0x1021, init=0xFFFF, no xorout, no reflection)
- **Bus Arbitration**: Only one device transmits at a time (slave only responds after receiving a command)

#### Command Message (Host → Device, 8 bytes)

| Byte Index | Field Name    | Size (bytes) | Description                          |
| ---------- | ------------- | ------------ | ------------------------------------ |
| 0-1        | Header        | 2            | Frame header, fixed 0xFE, 0xEE       |
| 2          | Device ID     | 1            | Target device address (0-255)        |
| 3          | Mode          | 1            | Work mode (see below)                |
| 4-5        | Target Torque | 2            | Target torque, uint16, little-endian |
| 6-7        | CRC-16-CCITT  | 2            | CRC of bytes 0-5, little-endian      |

#### Feedback Message (Device → Host, 13 bytes)

| Byte Index | Field Name     | Size (bytes) | Description                            |
| ---------- | -------------- | ------------ | -------------------------------------- |
| 0-1        | Header         | 2            | Frame header, fixed 0xFE, 0xEE         |
| 2          | Device ID      | 1            | Responding device address              |
| 3          | Mode           | 1            | Current work mode                      |
| 4          | Collision Flag | 1            | 0: safe, 1: collision detected         |
| 5-8        | Encoder Value  | 4            | Encoder reading, uint32, little-endian |
| 9-10       | Present Torque | 2            | Current torque, uint16, little-endian  |
| 11-12      | CRC-16-CCITT   | 2            | CRC of bytes 0-10, little-endian       |

#### CRC-16-CCITT Calculation

- **Polynomial**: 0x1021
- **Initial Value**: 0xFFFF
- **Input/Output Reflection**: None
- **Final XOR**: None
- **Range**: For command, CRC covers bytes 0-5; for feedback, CRC covers bytes 0-10

**Example (C code):**

```c
uint16_t crc_ccitt(uint16_t crc, const uint8_t *data, size_t len);
```

#### Mode Field Definition (MRC_Mode)

| Value | Name       | Description           |
| ----- | ---------- | --------------------- |
| 0     | FREE       | Free mode (no output) |
| 1     | FIX_LIMIT  | Fixed limit mode      |
| 2     | ADAPTATION | Adaptation mode       |

#### Example Communication Sequence

1. The host sends an 8-byte command frame to the RS-485 bus.
2. The target device verifies the ID and CRC, then parses the command.
3. The device replies with a 13-byte feedback frame containing its current status.
4. The host verifies the feedback CRC and reads the status.

#### Physical Layer Notes

- All nodes should use RS-485 transceivers; termination resistors (e.g., 120Ω) are recommended on A/B lines.
- At 4 Mbps, cable length should be limited to several tens of meters, and shielded twisted pair is recommended.
- In case of communication errors, the host should retransmit the command.

## 7. Timer Resource Allocation

| Timer | Purpose/Function                       | Channel(s) | Notes/Details                                     |
| ----- | -------------------------------------- | ---------- | ------------------------------------------------- |
| TIM1  | Encoder PWM capture (input capture)    | CH4        | Used for reading encoder PWM signal               |
| TIM2  | PWM generation for VNH7040 driver      | CH1        | Main PWM output for motor driver                  |
| TIM3  | (Available/Reserved for PWM)           | CH1, CH2   | Initialized for PWM, not actively used            |
| TIM4  | (Available/Reserved for base timing)   | -          | Initialized, not actively used                    |
| TIM6  | System periodic tasks (key scan, loop) | -          | Periodic interrupt for keyscan, control loop flag |
| TIM7  | (Available/Reserved for base timing)   | -          | Initialized, not actively used                    |

## Example: Main Loop Usage

```c
Device_MRC_t mrc_device;
MRC_Init("MRC_Device", &mrc_device, 0x01);

while(1) {
    MRC_Com_Process(&mrc_device); // Handle communication
    MRC_Key1_Reaction(&mrc_device); // Handle key1
    MRC_Key2_Reaction(&mrc_device); // Handle key2
    if (mrc_device.control_mode == MRC_CURRENT_CONTROL) {
        float i_meas = MRC_Update_Coil_Current(&mrc_device);
        float v_cmd = MRC_CoilCurrentControl_Update(&mrc_device, mrc_device.des_coil_current, i_meas, 4.22f, 0.0014868f, 0.001f);
        // v_cmd is sent to PWM driver automatically
    } else {
    MRC_set_voltage(&mrc_device);
}
}
```

## Reflection & Suggestions

- **Documentation Sync**: Always update README after code changes, especially for LUT, parameters, and interfaces.
- **Configurable Parameters**: Consider making PI parameters and LUT updatable via communication protocol in the future.
- **Comments & Examples**: Keep Doxygen-style comments and provide simple usage examples for beginners.

---

**If you have any questions or need further help, just ask!**

## Building and Flashing

### Prerequisites

- Keil MDK-ARM v5.37 or later
- STM32H7xx HAL drivers
- STM32CubeMX (for configuration)

### Build Steps

1. Open`MDK-ARM/safeMRC.uvprojx` in Keil MDK-ARM
2. Configure target settings if needed
3. Build the project (F7)
4. Flash to STM32H7 device

### Configuration

The system can be configured through the following defines:

```c
#define MRC_COIL_MAX_VOLTAGE 12.0f    // Maximum coil voltage
#define PWM_FREQ 20000                // PWM frequency
#define ENCODER_RESOLUTION 4096       // Encoder pulses per revolution
```

## API Reference

### MRC Device Functions

#### `MRC_Init(dev_name, mrc, id)`

Initialize MRC device with communication support.

**Parameters:**

- `dev_name`: Device name string
- `mrc`: MRC device structure pointer
- `id`: Device ID (0-255)

#### `MRC_set_voltage(mrc)`

Set coil voltage based on target torque.

**Parameters:**

- `mrc`: MRC device structure pointer

#### `MRC_Com_Process(mrc)`

Handle communication exchange using request-response pattern.

**Parameters:**

- `mrc`: MRC device structure pointer

#### `MRC_collision_detect(mrc1, mrc2, param, threshold)`

Detect collision between two MRC devices.

**Parameters:**

- `mrc1`,`mrc2`: MRC device structure pointers
- `param`: Detection parameter
- `threshold`: Collision threshold

### Communication Functions

#### `MRC_Com_Init(mrc_com, huart, id)`

Initialize communication module with DMA idle reception support.

#### `MRC_Com_UnpackCmd(mrc_com)`

Unpack and validate command from DMA buffer (device side).

#### `MRC_Com_PackFbk(mrc_com, mode, encoder, torque, collision)`

Pack feedback message with current device status (device side).

#### `MRC_Com_SendFbk(mrc_com)`

Send feedback response using DMA (device side).

#### `MRC_Com_RestartReception(mrc_com)`

Restart DMA reception for next command (device side).

#### `MRC_Com_PackCmd(mrc_com, mode, torque)` (Host side)

Pack command message for transmission.

## Safety Features

1. **Automatic Demagnetization**: On collision detection, coils are automatically demagnetized
2. **Voltage Limiting**: Coil voltage is limited to safe operating range
3. **CRC Validation**: All communication messages are validated using CRC-16
4. **Timeout Protection**: DMA operations include timeout protection
5. **Error Recovery**: Automatic recovery from communication errors
6. **Bus Conflict Prevention**: Request-response pattern prevents multiple devices from transmitting simultaneously

## Troubleshooting

### Common Issues

1. **Communication Not Working**

   - Check UART configuration in STM32CubeMX
   - Verify DMA settings for USART2
   - Ensure IDLE interrupt is enabled
   - Confirm request-response timing
2. **CRC Errors**

   - Verify protocol implementation on host side
   - Check byte order and data types
   - Ensure consistent CRC calculation
3. **DMA Issues**

   - Verify DMA stream configuration
   - Check interrupt priorities
   - Ensure proper buffer alignment
4. **Bus Conflicts**

   - Ensure only one device responds at a time
   - Check device ID configuration
   - Verify request-response timing

### Debug Output

The system provides debug output through UART1:

- Device initialization status
- Communication errors
- Collision detection events
- Voltage changes

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## Support

For technical support or questions, please open an issue on the project repository.

## 8. UART4 String Command Set (Debug/Control)

You can send string commands via UART4 (baud rate 921600) to switch control modes and set target values at runtime. **This feature is fully decoupled from the main device driver and is initialized in `main.c`, so it is always active after device startup.**

### Supported Command Formats

- Switch control mode:
  - `MODE VOLTAGE`   Switch to voltage control mode
  - `MODE CURRENT`   Switch to current control mode
- Set target value:
  - `SET VOLTAGE <value>`   Set target voltage (unit: V, range: -12.0 ~ 12.0)
  - `SET CURRENT <value>`   Set target current (unit: A, range: -5.0 ~ 5.0)

### Usage Examples

- Switch to current mode:
  ```
  MODE CURRENT
  ```
- Set target current to 0.5A:
  ```
  SET CURRENT 0.5
  ```
- Switch to voltage mode and set target voltage to 2.0V:
  ```
  MODE VOLTAGE
  SET VOLTAGE 2.0
  ```

### Command Feedback and Error Handling

- After each command, the system prints feedback via UART1, for example:
  - `[UART4] Switched to CURRENT control mode.`
  - `[UART4] Set target voltage: 2.000 V`
  - `[UART4] Voltage out of range (-12.0~12.0 V)!`
  - `[UART4] Unknown command: ...`
- If you enter an invalid command (missing parameter, out of range, typo, etc.), you will get a clear error message.
- Commands and parameters are case-insensitive.
- Each command should end with Enter or newline.

### Notes

- Only the four commands above are supported. Any other input will result in an "Unknown command" message.
- The CLI is initialized in `main.c` and does not require manual activation.
- Using the CLI does not affect the main communication protocol or normal device operation.
