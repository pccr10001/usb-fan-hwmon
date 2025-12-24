# USB Fan Controller for Liquidctl (ATmega32U4)

A USB HID Fan Controller firmware for the Pro Micro (ATmega32U4), supporting 4 independent PWM fans with RPM monitoring.

## Features

- **4 Independent Fan Channels** with PWM control.
- **25kHz PWM Frequency** for silent operation (compatible with standard 4-pin fans).
- **RPM Monitoring** for all 4 channels.
- **No Drivers Needed**: Uses standard USB HID protocol (RawHID).
- **Python Support**: Includes a test script and a driver class ensuring compatibility with tools like `liquidctl`.

## Hardware Setup

**Board**: Pro Micro (5V/16MHz) or compatible ATmega32U4 board.

### Pinout Configuration

| Channel   | PWM Pin (Arduino) | PWM Pin (AVR) | Tacho Pin (Arduino) | Tacho Pin (AVR) | Interrupt |
| :-------- | :---------------- | :------------ | :------------------ | :-------------- | :-------- |
| **Fan 1** | **9**             | PB5           | **2**               | PD1             | INT1      |
| **Fan 2** | **10**            | PB6           | **0 (RX)**          | PD2             | INT2      |
| **Fan 3** | **5**             | PC6           | **1 (TX)**          | PD3             | INT3      |
| **Fan 4** | **6**             | PD7           | **7**               | PE6             | INT6      |

## Building and Flashing

This project uses [PlatformIO](https://platformio.org/).

1.  **Build Firmware**:

    ```bash
    platformio run
    ```

2.  **Flash to Device**:
    ```bash
    platformio run -t upload
    ```

## Logic & Protocol

- **VID**: `0x1B4F` (SparkFun)
- **PID**: `0x9206` (Pro Micro)
- **Protocol**: Custom HID (64-byte packets).
- **RPM Calculation**: The firmware counts tacho pulses between read requests. When the host requests RPM, the firmware calculates the RPM based on the exact time elapsed since the last read.

## Usage

### Test Tool

Use the included `test_fan.py` to test the device on Windows/Linux:

```bash
pip install hidapi
python test_fan.py
```

### Python Driver

`32u4fan.py` provides a `FanController` class that can be integrated into other Python projects or used to extend `liquidctl`.

## Linux Kernel Module

A native Linux Kernel Module (`usb_fan_hwmon`) is included to expose the fans via the standard `hwmon` interface (sysfs/lm-sensors).

### Compilation & Installation

#### Option 1: DKMS (Recommended)

Dynamic Kernel Module Support ensures the driver is automatically rebuilt when you update your kernel.

1.  Current directory should be project root.
2.  Copy driver to source tree:
    ```bash
    sudo cp -R linux_driver /usr/src/usb-fan-hwmon-0.1
    ```
3.  Add, Build, and Install via DKMS:
    ```bash
    sudo dkms add -m usb-fan-hwmon -v 0.1
    sudo dkms build -m usb-fan-hwmon -v 0.1
    sudo dkms install -m usb-fan-hwmon -v 0.1
    ```
4.  Load the module:
    ```bash
    sudo modprobe usb-fan-hwmon
    ```

#### Option 2: Manual Build

```bash
cd linux_driver
make
sudo insmod usb_fan_hwmon.ko
```

### Usage (HWMON)

Once loaded, the device appears in `/sys/class/hwmon/`. You can use standard tools:

```bash
sensors
```

Or control manually via sysfs:

```bash
# Read Fan 1 RPM
cat /sys/class/hwmon/hwmonX/fan1_input

# Set Fan 1 to 50% PWM (0-255 range -> 128)
echo 128 > /sys/class/hwmon/hwmonX/pwm1
```

## License

MIT
