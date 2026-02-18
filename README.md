# BRIDGEplate C Library

C implementation of the PI-Plates BRIDGEplate interface for serial communication with multiple plate types.

## Overview

This library provides a complete C interface to control PI-Plates hardware via USB CDC serial communication. It replicates the functionality of the original Python implementation with cross-platform support for Windows and Linux.

## Supported Plate Types

- **ADC** - Analog-to-Digital Converter plate (44 functions)
- **BRIDGE** - Main bridge controller (6 functions)
- **CURRENT** - 4-20mA current input plate (9 functions)
- **DAQC** - Data Acquisition and Control plate (27 functions)
- **DAQC2** - Enhanced DAQC plate (54 functions)
- **DIGI** - Digital I/O plate (7 functions: basic info and LED control only)
- **RELAY** - 7-channel relay plate (12 functions)
- **RELAY2** - 8-channel relay plate (12 functions)
- **THERMO** - Thermocouple interface plate (24 functions)

## Features

- Cross-platform: Windows and Linux support
- Automatic device enumeration by VID/PID (2E8A:10E3)
- Clean C API matching Python functionality
- Blocking and non-blocking I/O operations
- Array-based data retrieval for multi-channel reads

## Building

### Windows

```bash
gcc -Wall -O2 -o example.exe example.c BRIDGEplate.c -lsetupapi
```

### Linux

```bash
gcc -Wall -O2 -o example example.c BRIDGEplate.c
```

### Using Make

```bash
make                # Build example program
make clean          # Clean build artifacts
make install        # Install library (Linux only, requires sudo)
```

## Usage

### Basic Initialization

```c
#include "BRIDGEplate.h"

int main(void) {
    // Initialize connection
    if (BRIDGEplate_init() != 0) {
        fprintf(stderr, "Failed to connect to BRIDGEplate\n");
        return 1;
    }
    
    // Your code here
    
    // Clean up
    BRIDGEplate_close();
    return 0;
}
```

### Poll for Connected Plates

```c
// Scan all addresses for all plate types
POLL();
```

Output example:
```
ADCplates:     0-------
CURRENTplates: -1------
DAQCplates:    0-------
DAQC2plates:   --------
DIGIplates:    --------
RELAYplates:   0-------
RELAY2plates:  --------
THERMOplates:  --------
```

### ADC Operations

```c
// Read single channel
double voltage = ADC_getADC(0, 2);  // addr=0, channel=2
printf("Voltage: %.3f V\n", voltage);

// Read all channels
double values[8];
int count = ADC_getADCall(0, values, 8);
for (int i = 0; i < count; i++) {
    printf("Channel %d: %.3f V\n", i, values[i]);
}

// Configure and read in single mode (default rate)
ADC_configINPUT(0, 3, 5);  // Configure channel 3, sample rate 5
ADC_enableINPUT(0, 3);     // Enable channel 3
double result = ADC_readSINGLE(0, 3);

// Configure with explicit enable status
ADC_configINPUT_enable(0, 3, 5, 1);  // Channel 3, rate 5, enable=true

// Read with specific sample rate (0-18, 0=slowest, 18=fastest)
double fast = ADC_readSINGLE_rate(0, 3, 18);  // Fastest rate
double slow = ADC_readSINGLE_rate(0, 3, 0);   // Slowest rate

// Display sample rate table
ADC_srTable(0);
```

### Relay Control

```c
// Control individual relays
RELAY_relayON(0, 1);      // Turn on relay 1
RELAY_relayOFF(0, 2);     // Turn off relay 2
RELAY_relayTOGGLE(0, 3);  // Toggle relay 3

// Set all relays at once (0-127 for RELAY, 0-255 for RELAY2)
RELAY_relayALL(0, 0b00001111);  // Relays 1-4 on, 5-7 off

// Get relay state
int state = RELAY_relaySTATE(0);
printf("Relay state: 0x%02X\n", state);
```

### Digital I/O (DAQC)

```c
// Digital outputs
DAQC_setDOUTbit(0, 0);     // Set bit 0
DAQC_clrDOUTbit(0, 1);     // Clear bit 1
DAQC_toggleDOUTbit(0, 2);  // Toggle bit 2
DAQC_setDOUTall(0, 0xFF);  // Set all bits

// Digital inputs
int bit = DAQC_getDINbit(0, 0);      // Read single bit
int all_bits = DAQC_getDINall(0);    // Read all bits
```

### PWM and DAC (DAQC/DAQC2)

```c
// PWM (0-1023 range)
DAQC_setPWM(0, 0, 512);    // 50% duty cycle on channel 0
int pwm = DAQC_getPWM(0, 0);

// DAC (0-4.095V range)
DAQC_setDAC(0, 0, 2.5);    // 2.5V on channel 0
double dac = DAQC_getDAC(0, 0);
```

### DAQC2 Advanced Features

#### Function Generator
```c
DAQC2_fgFREQ(0, 1000.0);   // Set 1 kHz
DAQC2_fgTYPE(0, 0);        // Sine (0=sine, 1=triangle, 2=square, 3=sawtooth)
DAQC2_fgLEVEL(0, 2.5);     // 2.5V amplitude
DAQC2_fgON(0);             // Start generator
DAQC2_fgOFF(0);            // Stop generator
```

#### Stepper Motor Control
```c
DAQC2_motorRATE(0, 1, 200);    // 200 steps/second
DAQC2_motorDIR(0, 1, 0);       // 0=forward, 1=reverse
DAQC2_motorENABLE(0, 1);       // Enable motor 1
DAQC2_motorMOVE(0, 1, 400);    // Move 400 steps
DAQC2_motorJOG(0, 1);          // Continuous movement
DAQC2_motorSTOP(0, 1);         // Stop movement
```

#### Oscilloscope
```c
DAQC2_setOSCchannel(0, 0);     // Select channel 0
DAQC2_setOSCsweep(0, 100);     // 100 µs/div
DAQC2_startOSC(0);             // Start capture

double trace[1024];
int count = DAQC2_getOSCtraces(0, trace, 1024);
DAQC2_stopOSC(0);
```

#### Multi-Color LED
```c
// DAQC2 has RGB LED with 8 colors (0-7)
DAQC2_setLED(0, 0);  // Off
DAQC2_setLED(0, 1);  // Red
DAQC2_setLED(0, 2);  // Green
DAQC2_setLED(0, 3);  // Yellow
DAQC2_setLED(0, 4);  // Blue
DAQC2_setLED(0, 5);  // Magenta
DAQC2_setLED(0, 6);  // Cyan
DAQC2_setLED(0, 7);  // White
```

### Temperature Measurement (DAQC/THERMO)

```c
// DAQC onboard temperature sensor (defaults to Celsius)
double temp_c = DAQC_getTEMP(0);
double temp_f = DAQC_getTEMP_scale(0, 'F');
double temp_k = DAQC_getTEMP_scale(0, 'K');

// THERMO plate with thermocouples (defaults to Celsius)
double tc_temp = THERMO_getTEMP(0, 1);  // Channel 1, Celsius
double tc_temp_f = THERMO_getTEMP_scale(0, 1, 'F');  // Channel 1, Fahrenheit
double cold_junction = THERMO_getCOLD(0);  // Celsius
double cold_junction_f = THERMO_getCOLD_scale(0, 'F');  // Fahrenheit
```

### Current Measurement (CURRENT plate)

```c
// Read single 4-20mA channel
double current = CURRENT_getI(0, 1);  // Channel 1
printf("Current: %.3f mA\n", current);

// Read all channels
double currents[8];
int count = CURRENT_getIall(0, currents, 8);
for (int i = 0; i < count; i++) {
    printf("Channel %d: %.3f mA\n", i + 1, currents[i]);
}
```

### LED Control

```c
// Most plates have LED control
DAQC_setLED(0, 0);      // Turn on LED 0 (DAQC has 2 LEDs: 0 and 1)
DAQC_clrLED(0, 1);      // Turn off LED 1
DAQC_toggleLED(0, 0);   // Toggle LED 0
int state = DAQC_getLED(0, 0);  // Get LED 0 state

// Other plates have single LED (no LED parameter needed)
ADC_setLED(0);          // Turn on LED
ADC_clrLED(0);          // Turn off LED
RELAY_toggleLED(0);     // Toggle LED
```

### BRIDGE Control Functions

```c
// Get BRIDGE information
char* id = BRIDGE_getID();
double hw_rev = BRIDGE_getHWrev();
double fw_rev = BRIDGE_getFWrev();

// Stack management
BRIDGE_resetSTACK();
int srq = BRIDGE_getSRQ();
BRIDGE_resetBRIDGE();
```

## API Functions by Category

### Common Functions (Most Plates)
- `getADDR(addr)` - Verify plate at address
- `getID(addr)` - Get plate ID string
- `getHWrev(addr)` - Get hardware revision
- `getFWrev(addr)` - Get firmware revision
- `setLED(addr)` - Turn on LED
- `clrLED(addr)` - Turn off LED
- `toggleLED(addr)` - Toggle LED

### ADC-Specific Functions
See function declarations in `BRIDGEplate.h` for complete list of:
- Basic ADC reads (`getADC`, `getADCall`)
- Mode configuration (`setMODE`, `getMODE`)
- Input configuration (`configINPUT`, `enableINPUT`)
- Advanced acquisition (`readSINGLE`, `startSCAN`, `getBLOCK`, `startSTREAM`)
- Digital inputs (`getDINbit`, `getDINall`)
- Trigger control (`configTRIG`, `startTRIG`, `triggerFREQ`)
- Event handling (`enableEVENTS`, `getEVENTS`)

### DAQC-Specific Functions
- ADC, Digital I/O, PWM, DAC, Temperature, Range finder
- Interrupt management
- Full list in header file

## Platform-Specific Notes

### Windows
- Requires `setupapi.lib` for COM port enumeration
- COM ports are opened as `\\.\COMx` for ports >= 10
- Uses Windows API for serial communication

### Linux
- Scans `/sys/class/tty` for USB devices
- Uses POSIX termios for serial communication
- Requires read/write permissions on `/dev/ttyACMx`
- May need to add user to `dialout` group: `sudo usermod -a -G dialout $USER`

## Error Handling

Most functions return:
- `double` for measurement values (check for 0.0 and verify with string response if needed)
- `int` for status/state values
- `char*` for string responses (static buffer, copy if needed to persist)
- `void` for commands with no return value

Check `BRIDGEplate_init()` return value:
- Returns 0 on success
- Returns -1 on failure (device not found or port open failed)

## Connection Details

- **VID:PID** - 2E8A:10E3 (Raspberry Pi Pico)
- **Baud Rate** - 115200
- **Data Bits** - 8
- **Parity** - None
- **Stop Bits** - 1
- **Timeout** - 20 seconds (configurable via TIMEOUT_MS)

## Limitations

1. Single global serial port connection
2. Blocking I/O operations
3. Static response buffers (copy strings if persistence needed)
4. No thread safety (add mutex if using multiple threads)

## Future Enhancements

- Non-blocking I/O option
- Thread-safe operation
- macOS support for device enumeration
- Dynamic buffer allocation
- Error code returns instead of silent failures

## Example Output

```
Scanning for connected plates...
ADCplates:     0-------
CURRENTplates: -1------
DAQCplates:    0-------
DAQC2plates:   --------
DIGIplates:    --------
RELAYplates:   0-------
RELAY2plates:  --------
THERMOplates:  --------

BRIDGE Information:
  ID: BRIDGEplate
  HW Rev: 1.0
  FW Rev: 1.2

Reading ADC channels from address 0:
  Channel 0: 0.523 V
  Channel 1: 1.234 V
  Channel 2: 2.456 V
  ...
```

## License

This implementation matches the functionality of the original PI-Plates Python library.

## Author

C implementation created for PI-Plates BRIDGEplate hardware interface.
