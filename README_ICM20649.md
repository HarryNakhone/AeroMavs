# ICM20649 Driver for Zephyr RTOS

This project contains a custom driver for the ICM20649 6-axis motion sensor (accelerometer + gyroscope) adapted from the ICM42605 driver for Zephyr RTOS.

## Project Structure

```
zephyrproject/
├── custom_driver/
│   ├── drivers/sensor/icm20649/
│   │   ├── icm20649.c              # Main driver implementation
│   │   ├── icm20649.h              # Driver header
│   │   ├── icm20649_setup.c/.h     # Sensor initialization
│   │   ├── icm20649_spi.c/.h       # SPI communication
│   │   ├── icm20649_trigger.c      # Interrupt handling
│   │   ├── icm20649_reg.h          # Register definitions
│   │   ├── invensense,icm20649.yaml # Device tree binding
│   │   ├── Kconfig                  # Driver configuration
│   │   └── CMakeLists.txt          # Build configuration
│   ├── CMakeLists.txt              # Module build file
│   ├── Kconfig                     # Module configuration
│   └── module.yml                  # Module definition
├── boards/
│   └── icm20649.overlay           # Device tree overlay
├── applications/
│   └── main.c                     # Sample application
├── prj.conf                       # Project configuration
└── CMakeLists.txt                 # Main build file
```

## Hardware Requirements

- STM32 microcontroller (or any Zephyr-supported board)
- ICM20649 sensor connected via SPI
- GPIO pin for interrupt (optional, for trigger support)

## Configuration

### 1. Device Tree Configuration

Update your board's device tree or use the provided overlay (`boards/icm20649.overlay`):

```dts
&spi1 {
    status = "okay";
    
    icm20649@0 {
        compatible = "invensense,icm20649";
        reg = <0>;
        spi-max-frequency = <1000000>;
        
        /* Adjust GPIO pin according to your hardware */
        int-gpios = <&gpioa 0 GPIO_ACTIVE_HIGH>;
        
        /* Sensor configuration */
        accel-hz = <100>;      /* Accelerometer sample rate */
        gyro-hz = <100>;       /* Gyroscope sample rate */
        accel-fs = <16>;       /* Accelerometer full scale (16g) */
        gyro-fs = <2000>;      /* Gyroscope full scale (2000 dps) */
    };
};
```

### 2. Project Configuration

The `prj.conf` file includes the necessary configurations:

```
CONFIG_GPIO=y
CONFIG_SPI=y
CONFIG_SENSOR=y
CONFIG_ICM20649=y
CONFIG_ICM20649_TRIGGER=y
CONFIG_SENSOR_LOG_LEVEL_DBG=y
```

## Building and Running

### 1. Build the Project

```bash
# Navigate to your project directory
cd /Users/abhisheklal/zephyrproject

# Build for your target board (replace with your board)
west build -p auto -b stm32f4_disco

# Or if you have a different board:
west build -p auto -b <your_board_name>
```

### 2. Flash and Run

```bash
# Flash the firmware
west flash

# Monitor output
west debug
```

## Usage Example

The sample application in `applications/main.c` demonstrates how to use the ICM20649 driver:

```c
#include <zephyr/drivers/sensor.h>

// Get the sensor device
static const struct device *icm20649 = DEVICE_DT_GET(DT_NODELABEL(icm20649));

// Read sensor data
struct sensor_value accel[3], gyro[3], temp;

// Fetch sample
sensor_sample_fetch(icm20649);

// Read accelerometer
sensor_channel_get(icm20649, SENSOR_CHAN_ACCEL_XYZ, accel);

// Read gyroscope
sensor_channel_get(icm20649, SENSOR_CHAN_GYRO_XYZ, gyro);

// Read temperature
sensor_channel_get(icm20649, SENSOR_CHAN_DIE_TEMP, &temp);
```

## Driver Features

- **Accelerometer**: 3-axis acceleration measurement
- **Gyroscope**: 3-axis angular velocity measurement
- **Temperature**: Die temperature measurement
- **SPI Interface**: Full SPI communication support
- **Interrupt Support**: Data ready and tap detection triggers
- **Configurable**: Sample rates and full-scale ranges
- **FIFO Support**: Hardware FIFO for continuous data streaming

## Configuration Options

### Sample Rates
- **Accelerometer**: 1, 3, 6, 12, 25, 50, 100, 200, 500, 1000, 2000, 4000, 8000 Hz
- **Gyroscope**: 12, 25, 50, 100, 200, 500, 1000, 2000, 4000, 8000 Hz

### Full Scale Ranges
- **Accelerometer**: 2g, 4g, 8g, 16g
- **Gyroscope**: 15, 32, 62, 125, 250, 500, 1000, 2000 dps

## Troubleshooting

### Common Issues

1. **Build Errors**: Ensure all dependencies are installed and Zephyr environment is set up
2. **SPI Communication**: Verify SPI bus configuration and connections
3. **GPIO Interrupts**: Check interrupt pin configuration if using triggers
4. **Device Tree**: Ensure device tree overlay is properly applied

### Debug Output

Enable debug logging by setting `CONFIG_SENSOR_LOG_LEVEL_DBG=y` in `prj.conf`.

## Contributing

This driver is based on the ICM42605 driver from Zephyr RTOS, adapted for the ICM20649 sensor. The main differences are in register definitions and some configuration parameters.

## License

Copyright (c) 2020 TDK Invensense
SPDX-License-Identifier: Apache-2.0
