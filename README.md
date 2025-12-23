# HX711 Load Cell with GP8413 DAC Output

ESP32-C6 firmware for reading HX711 load cell data and outputting it as 0-10V analog signal via GP8413 DAC module.

## Features

- **HX711 Load Cell Reading**: High-speed sampling (up to 80Hz) from HX711 load cell amplifier
- **GP8413 DAC Output**: Converts load cell readings to 0-10V analog output
- **USB CDC Serial Output**: CSV data stream over USB for monitoring and logging
- **Auto-Tare**: Automatic zero calibration on startup
- **ESP32-C6 Support**: Updated for ESP32-C6 processor

## Hardware Requirements

1. **ESP32-C6** development board
2. **HX711** load cell amplifier module
3. **GP8413** I2C DAC module (0-10V output)
4. Load cell sensor

## Pin Configuration

### HX711 Load Cell
- **CLK**: GPIO 3
- **DATA**: GPIO 4

### GP8413 DAC (I2C)
- **SCL**: GPIO 6
- **SDA**: GPIO 7
- **I2C Address**: 0x58 (default)

## Wiring Diagram

```
ESP32-C6          HX711
--------          -----
GPIO 3    ---->   CLK
GPIO 4    <----   DOUT
GND       ---->   GND
3.3V      ---->   VCC

ESP32-C6          GP8413
--------          ------
GPIO 6    ---->   SCL
GPIO 7    ---->   SDA
GND       ---->   GND
5V        ---->   VCC
                  CH0 (0-10V output)
                  CH1 (0-10V output, unused)
```

## Configuration

### Calibration Parameters (in `main.c`)

Located in the `app_main()` function, adjust these values for your specific load cell:

```c
// Zero offset - automatically calculated during tare
int32_t zero_offset = 0;

// Scale factor - adjust based on your load cell calibration
float scale_factor = 1.0f;

// DAC output range
float min_load = 0.0f;      // Minimum load (maps to 0V)
float max_load = 10000.0f;  // Maximum load (maps to 10V)
```

### How to Calibrate

1. **Tare (Zero Offset)**: Done automatically on startup with no load applied
   
2. **Scale Factor**: 
   - Apply a known weight to your load cell
   - Note the raw_count value from serial output
   - Calculate: `scale_factor = known_weight / (raw_count - zero_offset)`
   - Update the value in code

3. **DAC Range**:
   - Set `max_load` to the maximum weight your system should measure
   - At this weight, the DAC will output 10V
   - Linear scaling from 0V (no load) to 10V (max load)

## Building and Flashing

### Prerequisites
- ESP-IDF v6.1 or later
- Configured ESP-IDF environment

### Build Commands

```bash
# Configure for ESP32-C6 (already done)
idf.py set-target esp32c6

# Build the project
idf.py build

# Flash to device
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor

# Or combine flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

## Output Format

The firmware outputs CSV data over USB CDC serial:

```csv
timestamp_ms,raw_count,voltage
0,8388607,0.000
12,8388650,0.043
25,8388698,0.091
...
```

- **timestamp_ms**: Time since startup in milliseconds
- **raw_count**: Raw 24-bit value from HX711 (signed integer)
- **voltage**: Calculated DAC output voltage (0.0 - 10.0V)

## GP8413 DAC Module

The GP8413 is a dual-channel, 15-bit I2C DAC with 0-10V output range.

- **Resolution**: 15-bit (0-32767)
- **Output Range**: 0-10V (configurable to 0-5V via register)
- **I2C Address**: 0x58 (default, can be changed via hardware jumper)
- **Update Rate**: Suitable for 80Hz HX711 sampling rate

### DAC Features Used
- Channel 0: Active (load cell output)
- Channel 1: Unused (available for future use)
- Output Range: 0-10V (configured in `gp8413_init()`)

## Project Structure

```
/workspaces/load/
├── main/
│   ├── main.c              # Main application code
│   ├── CMakeLists.txt      # Component build config
│   └── idf_component.yml   # Component dependencies
├── build/                  # Build output (generated)
├── CMakeLists.txt          # Project CMake config
├── sdkconfig               # ESP-IDF configuration
└── README.md               # This file
```

## Troubleshooting

### HX711 Not Ready
- Check wiring connections
- Ensure HX711 has stable power supply
- Wait a few seconds after power-on for stabilization

### GP8413 Initialization Failed
- Verify I2C wiring (SCL/SDA)
- Check pull-up resistors on I2C lines (usually built into GP8413 module)
- Confirm I2C address (default 0x58)
- Program will continue with USB output even if DAC fails

### Unstable Readings
- Add capacitor (0.1µF) near HX711 power pins
- Keep wiring short and away from noise sources
- Shield load cell wires if possible
- Adjust averaging if needed

### DAC Output Not Linear
- Verify calibration parameters
- Check that `max_load` matches your expected range
- Ensure load cell is properly mounted and not binding

## Advanced Configuration

### Change I2C Speed
Default is 100kHz. Can be increased for faster updates:

```c
#define I2C_MASTER_FREQ_HZ  400000  // 400kHz fast mode
```

### Adjust Sampling
The main loop samples as fast as HX711 allows (10Hz or 80Hz depending on RATE pin).
Add delays if slower sampling is desired.

### Use Both DAC Channels
Modify `app_main()` to output to Channel 1:

```c
gp8413_set_voltage(1, voltage);  // Channel 1
```

## Migration from ESP32-C3

This project has been migrated from ESP32-C3 to ESP32-C6. The GPIO pins and peripherals are compatible. If reverting:

```bash
idf.py set-target esp32c3
idf.py build
```

## License

This project is provided as-is for educational and commercial use.

## Support

For hardware support:
- HX711: https://www.analog.com/en/products/hx711.html
- GP8413: Check with your module supplier
- ESP32-C6: https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/

## Changelog

### v2.0 - Current
- Added GP8413 DAC support for 0-10V output
- Migrated from ESP32-C3 to ESP32-C6
- Added auto-tare functionality
- Enhanced CSV output with voltage data

### v1.0 - Original
- Basic HX711 reading over USB CDC serial
- ESP32-C3 support
