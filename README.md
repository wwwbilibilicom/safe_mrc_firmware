# Safe MRC - STM32H7 Magnetorheological Clutch Controller

## Overview

Safe MRC is an embedded control system for Magnetorheological Clutch (MRC) devices using STM32H7 microcontrollers. The system provides precise torque control, collision detection, and real-time communication capabilities.

## Features

- **Precise Torque Control**: PID-based control system for accurate torque regulation
- **Collision Detection**: Real-time collision detection and safety response
- **DMA Communication**: High-efficiency UART communication using DMA
- **Request-Response Protocol**: Master-slave communication pattern to avoid bus conflicts
- **Multi-sensor Support**: Encoder feedback, voltage monitoring, and temperature sensing
- **Safety Features**: Automatic demagnetization on collision detection
- **Modular Design**: Clean separation of hardware drivers and application logic

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

## Key Components

### MRC Communication Module (`mrc_com.c/h`)

The MRC communication module provides DMA-based UART communication with request-response pattern:

- **DMA Idle Reception**: Automatic command reception using UART IDLE interrupt
- **CRC Validation**: CCITT CRC-16 checksum verification
- **Request-Response**: Device only responds after receiving a command
- **Bus Conflict Prevention**: Master-slave pattern ensures no bus conflicts
- **Error Handling**: Robust error detection and recovery

#### Communication Flow

1. **DMA Idle Reception**: Device continuously receives data via DMA
2. **IDLE Interrupt**: When UART becomes idle, interrupt sets RxFlag=1
3. **Command Unpacking**: Main loop detects RxFlag and unpacks command
4. **Parameter Update**: Device updates mode and target torque from command
5. **Feedback Preparation**: Device packs current status (encoder, torque, collision)
6. **Response Transmission**: Device sends feedback via DMA
7. **Reception Restart**: DMA reception restarted for next command

#### Usage Example

```c
// Initialize MRC device with communication
Device_MRC_t mrc_device;
MRC_Init("MRC_Device", &mrc_device, 0x01);

// Main loop - handle communication
while(1) {
    // Process communication exchange (request-response)
    MRC_Com_Process(&mrc_device);
    
    // Handle key events
    MRC_Key1_Reaction(&mrc_device);
    MRC_Key2_Reaction(&mrc_device);
    
    // Update control loop
    MRC_set_voltage(&mrc_device);
}
```

### Communication Protocol

The system uses a binary protocol with request-response pattern:

#### Command Message (8 bytes) - Host → Device
```
[0-1] Header: 0xFE 0xEE
[2]   Device ID
[3]   Mode
[4-5] Target Torque (16-bit)
[6-7] CRC-16
```

#### Feedback Message (13 bytes) - Device → Host
```
[0-1] Header: 0xFE 0xEE
[2]   Device ID
[3]   Mode
[4-7] Encoder Value (32-bit)
[8-9] Present Torque (16-bit)
[10]  Collision Flag
[11-12] CRC-16
```

### Hardware Drivers

- **VNH7040 Driver**: Motor driver chip control with PWM and ADC
- **Encoder Driver**: Position and velocity feedback
- **LED/Key Driver**: User interface control
- **PID Controller**: Closed-loop control implementation

## Building and Flashing

### Prerequisites

- Keil MDK-ARM v5.37 or later
- STM32H7xx HAL drivers
- STM32CubeMX (for configuration)

### Build Steps

1. Open `MDK-ARM/safeMRC.uvprojx` in Keil MDK-ARM
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
- `mrc1`, `mrc2`: MRC device structure pointers
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


