# SafeMRC Host UI

A Python-based graphical user interface (GUI) for communicating with the SafeMRC embedded controller via serial port. This tool allows you to send control commands, view real-time feedback, and visualize key parameters such as encoder angle, velocity, and coil current.

## Features
- **Serial Port Communication**: Auto-detects available serial ports, supports high baud rates (default 4 Mbps).
- **Control Panel**: Set control mode, command send frequency, and target current.
- **Feedback Display**: Shows all fields from the device feedback message, including CRC, current, mode, encoder angle, and velocity.
- **Real-Time Visualization**: Three high-performance plots (angle, velocity, current) with adjustable time window, using pyqtgraph for smooth, real-time updates.
- **Robust Multithreading**: Serial communication runs in a separate thread to ensure UI responsiveness.

## Installation & Environment Setup

1. **Install [Miniconda/Anaconda](https://docs.conda.io/en/latest/miniconda.html) if not already installed.**
2. **Create the environment:**

```bash
conda env create -f environment.yml
conda activate safemrc_host
```

3. **Install additional dependencies if needed:**

```bash
conda install pyqt pyqtgraph pyserial numpy
```

## How to Run

```bash
python main.py
```

## UI Overview

- **Top Row**: Serial port dropdown (auto-detect), Connect/Disconnect buttons (leftmost)
- **Control Panel**: Select control mode, set send frequency (Hz), and target current (A)
- **Feedback Area**: Displays all fields from the feedback message:
  - CRC (received/calculated)
  - Current (A)
  - Mode
  - Encoder Angle (deg)
  - Encoder Velocity (deg/s)
  - Collision flag
- **Plot Controls**: Adjust the time window (in seconds) for real-time plots
- **Plots**: Three separate real-time plots for encoder angle, velocity, and current. Default window is 1 second, adjustable up to 30 seconds.

## Communication Protocol

### Command Frame (Host → Device, 8 bytes)
| Bytes | Field         | Description                |
|-------|--------------|----------------------------|
| 0-1   | Header       | 0xFE, 0xEE                 |
| 2     | Device ID    | 1 (default)                |
| 3     | Mode         | 0:FREE, 1:FIX_LIMIT, etc.  |
| 4-5   | Current      | int16, mA (current*1000)   |
| 6-7   | CRC-16-CCITT | Little-endian              |

### Feedback Frame (Device → Host, 17 bytes)
| Bytes   | Field           | Description                        |
|---------|----------------|------------------------------------|
| 0-1     | Header         | 0xFE, 0xEE                         |
| 2       | Device ID      |                                    |
| 3       | Mode           |                                    |
| 4       | Collision      | 0: safe, 1: collision               |
| 5-8     | Encoder Angle  | int32, deg*1000                    |
| 9-12    | Encoder Vel.   | int32, deg/s*1000                  |
| 13-14   | Current        | int16, mA (current*1000)           |
| 15-16   | CRC-16-CCITT   | Little-endian                      |

## Notes & Troubleshooting
- Ensure your user account has permission to access the serial port.
- The device must be powered and connected before launching the UI.
- If no ports are listed, click the dropdown to refresh.
- For best performance, use a high-quality USB-to-serial adapter capable of 4 Mbps.
- If the UI freezes or no data is shown, check the serial connection and device status.

## Author / Contact
- (Fill in as needed)

## Data Export (CSV)
- Click the **Export CSV** button to save all recorded data to a CSV file.
- Exported fields include: high-precision timestamp, all feedback fields (angle, velocity, current, mode, collision flag, CRC, etc.), and the original hex message.
- Timestamp is unique, strictly increasing, and aligned with the plot's time axis (uses high-precision `time.perf_counter`).

## Data Recording Controls
- **Start Recording**: Begin saving all incoming data for later export.
- **Stop Recording**: Pause data recording; data up to this point can be exported.
- **Clear Plots**: Clears all cached data and resets the time axis to zero (for a fresh measurement window).

## MRC ID Switching
- The UI provides an input box for MRC Device ID.
- You can switch between multiple device IDs; changes take effect immediately.
- Only the device with the matching ID will respond to commands.

## High-Frequency Sampling Support
- Supports command send/receive rates up to **1000Hz** (1ms interval).
- Real-time plotting and data recording remain smooth at high frequencies.

## Hexadecimal Message Display
- The TX/RX message area displays all sent/received frames in hexadecimal format for easy protocol debugging.
- Format is consistent with embedded protocol documentation.

## Time Axis Reset
- After clicking **Clear Plots**, the time axis of all plots resets to zero, ensuring new data is aligned from t=0.

## Common Bugs & Fixes
- **Serial port thread/parameter sync**: Fixed issues with parameter passing and ID sync between UI and serial thread.
- **Ring buffer overflow**: Improved buffer management to prevent data loss at high rates.
- **Timestamp duplication**: Ensured all timestamps are unique and strictly increasing.
- **Time axis not resetting**: Fixed so that clearing plots always resets the time axis.
- **UI freeze/port refresh**: Improved serial port refresh and error handling for better stability.

## User Experience & Details
- All controls are disabled until the serial port is connected; user is prompted if actions are attempted while disconnected.
- UI is responsive and supports window resizing.
- All code is well-commented in English for easy maintenance.
- For best results, use a high-quality USB-to-serial adapter and ensure the device is powered before connecting. 