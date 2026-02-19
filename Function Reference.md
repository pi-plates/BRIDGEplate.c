# BRIDGEplate C Library - Complete Function Reference

**Version:** 1.0  
**Date:** February 2026  
**Platform Support:** Windows, Linux

---

## Table of Contents

1. [Introduction](#introduction)
2. [Compilation Instructions](#compilation-instructions)
3. [Library Initialization](#library-initialization)
4. [Common Function Patterns](#common-function-patterns)
5. [ADC Functions](#adc-functions)
6. [BRIDGE Functions](#bridge-functions)
7. [CURRENT Functions](#current-functions)
8. [DAQC Functions](#daqc-functions)
9. [DAQC2 Functions](#daqc2-functions)
10. [DIGI Functions](#digi-functions)
11. [RELAY Functions](#relay-functions)
12. [RELAY2 Functions](#relay2-functions)
13. [THERMO Functions](#thermo-functions)
14. [Example Programs](#example-programs)

---

## Introduction

The BRIDGEplate C Library provides a complete interface to control Pi-Plates hardware via USB CDC serial communication. The library supports nine different plate types with over 200 functions for data acquisition, control, and monitoring.

### Key Features

- **Cross-platform**: Works on Windows and Linux
- **Automatic device detection**: Finds BRIDGEplate by VID/PID (2E8A:10E3)
- **Multiple plate support**: Control up to 8 plates per stack (addresses 0-7)
- **115200 baud serial communication**
- **8N1 serial format** (8 data bits, no parity, 1 stop bit)

### Hardware Connection

- Connect BRIDGEplate via USB
- Device appears as a CDC serial port (COM port on Windows, /dev/ttyACM on Linux)
- Library automatically finds and opens the port

---

## Compilation Instructions

### Windows

**Requirements:**
- GCC (MinGW-w64 or similar)
- Windows SDK headers (included with MinGW)

**Basic compilation:**
```bash
gcc -Wall -O2 -o myprogram.exe myprogram.c BRIDGEplate.c -lsetupapi
```

**Explanation:**
- `-Wall` - Enable all warnings
- `-O2` - Optimization level 2
- `-lsetupapi` - Link against Setup API library (required for USB device enumeration)

**Example with multiple source files:**
```bash
gcc -Wall -O2 -c BRIDGEplate.c -o BRIDGEplate.o
gcc -Wall -O2 -c myprogram.c -o myprogram.o
gcc -o myprogram.exe myprogram.o BRIDGEplate.o -lsetupapi
```

### Linux

**Requirements:**
- GCC compiler
- Standard C library

**Basic compilation:**
```bash
gcc -Wall -O2 -o myprogram myprogram.c BRIDGEplate.c
```

**No additional libraries needed** - Linux uses standard POSIX APIs for serial communication.

**USB device permissions:**
```bash
# Add user to dialout group for serial port access
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect

# Or set permissions for specific device (temporary)
sudo chmod 666 /dev/ttyACM0
```

### Using the Makefile

A Makefile is provided for convenience:

```bash
# Build example program
make

# Build with debug symbols
make DEBUG=1

# Clean build artifacts
make clean

# Install library system-wide (Linux only, requires sudo)
sudo make install
```

---

## Library Initialization

Before using any plate functions, you must initialize the library and establish connection to the BRIDGEplate.

### BRIDGEplate_init

```c
int BRIDGEplate_init(void);
```

**Description:**  
Initializes the library and opens connection to the BRIDGEplate. Automatically searches for the device by VID/PID and opens the serial port.

**Returns:**
- `0` on success
- `-1` on failure (device not found or port open failed)

**Example:**
```c
#include "BRIDGEplate.h"
#include <stdio.h>

int main(void) {
    if (BRIDGEplate_init() != 0) {
        fprintf(stderr, "Failed to initialize BRIDGEplate\n");
        return 1;
    }
    
    printf("BRIDGEplate initialized successfully\n");
    
    // Your code here...
    
    BRIDGEplate_close();
    return 0;
}
```

### BRIDGEplate_close

```c
void BRIDGEplate_close(void);
```

**Description:**  
Closes the serial port connection and cleans up resources. Should be called before program exit.

**Example:**
```c
BRIDGEplate_close();
```

### POLL

```c
void POLL(void);
```

**Description:**  
Scans all addresses (0-7) for all plate types and displays which plates are connected. Useful for detecting the hardware configuration.

**Example:**
```c
printf("Scanning for connected plates...\n");
POLL();
```

**Output:**
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

---

## Common Function Patterns

Most plate types share common functions for basic operations:

### Address Verification

```c
int PLATETYPE_getADDR(int addr);
```

Verifies if a plate of the specified type is present at the given address.

**Parameters:**
- `addr` - Plate address (0-7)

**Returns:**
- The address if plate is present
- 0 or error value if not present

### Identification

```c
char* PLATETYPE_getID(int addr);
double PLATETYPE_getHWrev(int addr);
double PLATETYPE_getFWrev(int addr);
```

Get plate identification, hardware revision, and firmware revision.

### LED Control

```c
void PLATETYPE_setLED(int addr);      // Turn on LED
void PLATETYPE_clrLED(int addr);      // Turn off LED
void PLATETYPE_toggleLED(int addr);   // Toggle LED state
```

**Note:** DAQC and DAQC2 have multiple LEDs and require an additional LED number parameter.

### Help

```c
void PLATETYPE_help(void);
```

Displays help information for the plate type (sent to stdout).

---

## ADC Functions

The ADC (Analog-to-Digital Converter) plate provides 8 channels of high-precision voltage measurement with configurable sample rates and trigger modes.

### Basic Information

```c
int ADC_getADDR(int addr);
char* ADC_getID(int addr);
double ADC_getHWrev(int addr);
double ADC_getFWrev(int addr);
```

**Example:**
```c
if (ADC_getADDR(0) == 0) {
    char* id = ADC_getID(0);
    printf("ADC Plate: %s\n", id);
    printf("HW Rev: %.1f\n", ADC_getHWrev(0));
    printf("FW Rev: %.1f\n", ADC_getFWrev(0));
}
```

### LED Control

```c
void ADC_setLED(int addr);
void ADC_clrLED(int addr);
void ADC_toggleLED(int addr);
```

### Basic ADC Reading

```c
double ADC_getADC(int addr, int channel);
```

**Description:**  
Reads a single ADC channel voltage.

**Parameters:**
- `addr` - Plate address (0-7)
- `channel` - ADC channel (0-7)

**Returns:**  
Voltage reading in volts (typically 0.0 to 10.0V range, depending on configuration)

**Example:**
```c
double voltage = ADC_getADC(0, 2);
printf("Channel 2: %.3f V\n", voltage);
```

### Multi-Channel Reading

```c
int ADC_getADCall(int addr, double* results, int max_results);
int ADC_getSall(int addr, double* results, int max_results);
int ADC_getDall(int addr, double* results, int max_results);
int ADC_getIall(int addr, double* results, int max_results);
```

**Description:**  
Read all channels at once. Different functions return different measurement types:
- `getADCall` - All ADC channels
- `getSall` - Single-ended measurements
- `getDall` - Differential measurements
- `getIall` - Current measurements

**Parameters:**
- `addr` - Plate address (0-7)
- `results` - Array to store readings
- `max_results` - Maximum array size

**Returns:**  
Number of values read

**Example:**
```c
double voltages[8];
int count = ADC_getADCall(0, voltages, 8);
for (int i = 0; i < count; i++) {
    printf("Channel %d: %.3f V\n", i, voltages[i]);
}
```

### Mode Configuration

```c
void ADC_setMODE(int addr, int mode);
int ADC_getMODE(int addr);
```

**Description:**  
Set/get ADC operating mode.

**Parameters:**
- `mode` - Operating mode code

### Input Configuration

```c
void ADC_configINPUT(int addr, int channel, int sample_rate);
void ADC_configINPUT_enable(int addr, int channel, int sample_rate, int enable);
void ADC_enableINPUT(int addr, int channel);
void ADC_disableINPUT(int addr, int channel);
```

**Description:**  
Configure ADC input channels. The `configINPUT` function sets the sample rate, while `configINPUT_enable` also sets the enable status in one call.

**Parameters:**
- `addr` - Plate address (0-7)
- `channel` - ADC channel (0-7)
- `sample_rate` - Sample rate code (0-18, where 0=slowest, 18=fastest)
- `enable` - Enable status (0=disabled, 1=enabled)

**Example:**
```c
// Configure channel 3 with sample rate 5
ADC_configINPUT(0, 3, 5);
ADC_enableINPUT(0, 3);

// Or do both in one call
ADC_configINPUT_enable(0, 3, 5, 1);
```

### Sample Rate Table

```c
void ADC_srTable(int addr);
```

**Description:**  
Displays a table showing available sample rates for the ADC plate.

**Example:**
```c
ADC_srTable(0);
```

### Single Reading with Rate Control

```c
double ADC_readSINGLE(int addr, int channel);
double ADC_readSINGLE_rate(int addr, int channel, int rate);
```

**Description:**  
Read a single channel. The `readSINGLE` version uses default rate, while `readSINGLE_rate` allows specifying the sample rate.

**Parameters:**
- `addr` - Plate address (0-7)
- `channel` - ADC channel (0-7)
- `rate` - Sample rate (0-18)

**Example:**
```c
// Default rate
double v1 = ADC_readSINGLE(0, 2);

// Fastest rate
double v2 = ADC_readSINGLE_rate(0, 2, 18);

// Slowest rate
double v3 = ADC_readSINGLE_rate(0, 2, 0);
```

### Scanning Operations

```c
void ADC_startSCAN(int addr);
void ADC_startSCAN_rate(int addr, int rate);
void ADC_stopSCAN(int addr);
```

**Description:**  
Start/stop continuous scanning of enabled channels.

**Example:**
```c
// Enable channels to scan
ADC_enableINPUT(0, 0);
ADC_enableINPUT(0, 1);
ADC_enableINPUT(0, 2);

// Start scanning with specific rate
ADC_startSCAN_rate(0, 10);

// ... read data ...

ADC_stopSCAN(0);
```

### Block Data Acquisition

```c
int ADC_getBLOCK(int addr, double* results, int max_results);
```

**Description:**  
Retrieve a block of data from continuous scanning.

**Parameters:**
- `addr` - Plate address (0-7)
- `results` - Array to store readings
- `max_results` - Maximum array size

**Returns:**  
Number of values read

### Streaming

```c
void ADC_startSTREAM(int addr);
void ADC_startSTREAM_rate(int addr, int rate);
void ADC_stopSTREAM(int addr);
```

**Description:**  
Start/stop high-speed data streaming mode.

### Digital Input

```c
int ADC_getDINbit(int addr, int bit);
int ADC_getDINall(int addr);
```

**Description:**  
Read digital input pins on the ADC plate.

**Parameters:**
- `bit` - Bit number (0-7)

**Returns:**  
Bit state (0 or 1) for `getDINbit`, or 8-bit value for `getDINall`

**Example:**
```c
int bit0 = ADC_getDINbit(0, 0);
int all_bits = ADC_getDINall(0);
printf("Bit 0: %d, All bits: 0x%02X\n", bit0, all_bits);
```

### Trigger Configuration

```c
void ADC_configTRIG(int addr, int channel, int edge, double level);
void ADC_configTRIG_mode(int addr, int channel, int edge, double level, int mode);
void ADC_startTRIG(int addr, int channel);
void ADC_stopTRIG(int addr, int channel);
int ADC_triggerFREQ(int addr, int channel);
```

**Description:**  
Configure and control trigger-based data acquisition.

**Parameters:**
- `channel` - ADC channel (0-7)
- `edge` - Trigger edge (0=falling, 1=rising)
- `level` - Trigger voltage level
- `mode` - Trigger mode

**Example:**
```c
// Trigger on channel 0 rising edge at 2.5V
ADC_configTRIG(0, 0, 1, 2.5);
ADC_startTRIG(0, 0);

// Check trigger frequency
int freq = ADC_triggerFREQ(0, 0);
printf("Trigger frequency: %d Hz\n", freq);
```

### Event Management

```c
void ADC_enableEVENTS(int addr);
void ADC_disableEVENTS(int addr);
int ADC_check4EVENTS(int addr);
int ADC_getEVENTS(int addr);
```

**Description:**  
Enable/disable event notifications and check for/retrieve events.

**Returns:**  
Event status code

### Initialization and Reset

```c
void ADC_initADC(int addr);
void ADC_resetADC(int addr);
```

**Description:**  
Initialize or reset the ADC plate to default settings.

### Help

```c
void ADC_help(void);
```

**Description:**  
Display ADC plate help information.

---

## BRIDGE Functions

The BRIDGE plate is the main controller that manages the entire stack of plates. It has reduced functionality in this version (7 functions instead of the original 13).

### Basic Information

```c
char* BRIDGE_getID(void);
double BRIDGE_getHWrev(void);
double BRIDGE_getFWrev(void);
```

**Description:**  
Get BRIDGE plate identification and version information. Note that BRIDGE functions don't require an address parameter since there's only one BRIDGE per stack.

**Example:**
```c
char* id = BRIDGE_getID();
double hw = BRIDGE_getHWrev();
double fw = BRIDGE_getFWrev();
printf("BRIDGE: %s (HW: %.1f, FW: %.1f)\n", id, hw, fw);
```

### Stack Management

```c
void BRIDGE_resetSTACK(void);
int BRIDGE_getSRQ(void);
void BRIDGE_resetBRIDGE(void);
```

**Description:**
- `resetSTACK()` - Reset all plates in the stack
- `getSRQ()` - Get Service Request status
- `resetBRIDGE()` - Reset the BRIDGE plate itself

**Example:**
```c
// Check for service request
int srq = BRIDGE_getSRQ();
if (srq) {
    printf("Service request pending: %d\n", srq);
}

// Reset entire stack
BRIDGE_resetSTACK();
```

### Help

```c
void BRIDGE_help(void);
```

**Description:**  
Display BRIDGE plate help information.

---

## CURRENT Functions

The CURRENT plate provides 8 channels of 4-20mA current loop input measurement.

### Basic Information

```c
int CURRENT_getADDR(int addr);
char* CURRENT_getID(int addr);
double CURRENT_getHWrev(int addr);
double CURRENT_getFWrev(int addr);
```

### LED Control

```c
void CURRENT_setLED(int addr);
void CURRENT_clrLED(int addr);
void CURRENT_toggleLED(int addr);
```

### Current Measurement

```c
double CURRENT_getI(int addr, int channel);
```

**Description:**  
Read current from a single channel.

**Parameters:**
- `addr` - Plate address (0-7)
- `channel` - Input channel (1-8)

**Returns:**  
Current in milliamps (mA), typically 4.0 to 20.0 mA

**Example:**
```c
double current = CURRENT_getI(0, 1);
printf("Channel 1: %.2f mA\n", current);

// Convert to engineering units (example: 4-20mA = 0-100%)
double percent = (current - 4.0) / 16.0 * 100.0;
printf("Process value: %.1f%%\n", percent);
```

### Multi-Channel Reading

```c
int CURRENT_getIall(int addr, double* results, int max_results);
```

**Description:**  
Read all current channels at once.

**Parameters:**
- `addr` - Plate address (0-7)
- `results` - Array to store current readings (in mA)
- `max_results` - Maximum array size (should be 8)

**Returns:**  
Number of channels read

**Example:**
```c
double currents[8];
int count = CURRENT_getIall(0, currents, 8);

for (int i = 0; i < count; i++) {
    printf("Channel %d: %.2f mA\n", i + 1, currents[i]);
}
```

### Reset

```c
void CURRENT_resetCURRENT(int addr);
```

**Description:**  
Reset the CURRENT plate to default settings.

### Help

```c
void CURRENT_help(void);
```

---

## DAQC Functions

The DAQC (Data Acquisition and Control) plate combines ADC inputs, digital I/O, PWM outputs, DAC outputs, and other features.

### Basic Information

```c
int DAQC_getADDR(int addr);
char* DAQC_getID(int addr);
double DAQC_getHWrev(int addr);
double DAQC_getFWrev(int addr);
```

### LED Control

DAQC has 2 LEDs (0 and 1), so LED functions require the LED number:

```c
void DAQC_setLED(int addr, int led);
void DAQC_clrLED(int addr, int led);
void DAQC_toggleLED(int addr, int led);
int DAQC_getLED(int addr, int led);
```

**Parameters:**
- `led` - LED number (0 or 1)

**Example:**
```c
DAQC_setLED(0, 0);      // Turn on LED 0
DAQC_clrLED(0, 1);      // Turn off LED 1
DAQC_toggleLED(0, 0);   // Toggle LED 0
int state = DAQC_getLED(0, 0);  // Get LED 0 state
```

### ADC Functions

```c
double DAQC_getADC(int addr, int channel);
int DAQC_getADCall(int addr, double* results, int max_results);
```

**Description:**  
Read analog inputs (0-4.096V range, 12-bit resolution).

**Parameters:**
- `channel` - ADC channel (0-7)

**Example:**
```c
double voltage = DAQC_getADC(0, 3);
printf("ADC Channel 3: %.3f V\n", voltage);

double all_voltages[8];
int count = DAQC_getADCall(0, all_voltages, 8);
```

### Digital Output

```c
void DAQC_setDOUTbit(int addr, int bit);
void DAQC_clrDOUTbit(int addr, int bit);
void DAQC_toggleDOUTbit(int addr, int bit);
void DAQC_setDOUTall(int addr, int value);
int DAQC_getDOUTbyte(int addr);
```

**Description:**  
Control digital output pins.

**Parameters:**
- `bit` - Bit number (0-6)
- `value` - 8-bit value for all outputs

**Example:**
```c
// Set individual bits
DAQC_setDOUTbit(0, 0);      // Set bit 0 high
DAQC_clrDOUTbit(0, 1);      // Set bit 1 low
DAQC_toggleDOUTbit(0, 2);   // Toggle bit 2

// Set all at once
DAQC_setDOUTall(0, 0b01010101);

// Read current state
int state = DAQC_getDOUTbyte(0);
printf("DOUT state: 0x%02X\n", state);
```

### Digital Input

```c
int DAQC_getDINbit(int addr, int bit);
int DAQC_getDINall(int addr);
void DAQC_enableDINevent(int addr, int bit);
void DAQC_disableDINevent(int addr, int bit);
```

**Description:**  
Read digital input pins and configure input events.

**Example:**
```c
// Read single bit
int bit0 = DAQC_getDINbit(0, 0);

// Read all bits
int all_bits = DAQC_getDINall(0);
printf("Digital inputs: 0x%02X\n", all_bits);

// Enable event notification on bit 3
DAQC_enableDINevent(0, 3);
```

### PWM Control

```c
void DAQC_setPWM(int addr, int channel, int value);
int DAQC_getPWM(int addr, int channel);
void DAQC_setINTfreq(int addr, int channel, double freq);
```

**Description:**  
Control PWM outputs (0-1023 range for duty cycle).

**Parameters:**
- `channel` - PWM channel (0-1)
- `value` - Duty cycle (0-1023, where 1023 = 100%)
- `freq` - PWM frequency in Hz

**Example:**
```c
// Set 50% duty cycle on PWM channel 0
DAQC_setPWM(0, 0, 512);

// Set 25% duty cycle
DAQC_setPWM(0, 0, 256);

// Read current value
int pwm = DAQC_getPWM(0, 0);

// Set PWM frequency to 1 kHz
DAQC_setINTfreq(0, 0, 1000.0);
```

### DAC Control

```c
void DAQC_setDAC(int addr, int channel, double voltage);
double DAQC_getDAC(int addr, int channel);
void DAQC_setPOWER(int addr, double voltage);
double DAQC_getPOWER(int addr);
```

**Description:**  
Control DAC outputs (0-4.095V range, 12-bit resolution).

**Parameters:**
- `channel` - DAC channel (0-1)
- `voltage` - Output voltage (0.0 to 4.095V)

**Example:**
```c
// Set DAC channel 0 to 2.5V
DAQC_setDAC(0, 0, 2.5);

// Set DAC channel 1 to 1.8V
DAQC_setDAC(0, 1, 1.8);

// Read current DAC value
double v = DAQC_getDAC(0, 0);

// Set/get power supply voltage reference
DAQC_setPOWER(0, 5.0);
double pwr = DAQC_getPOWER(0);
```

### Temperature Measurement

```c
double DAQC_getTEMP(int addr);
double DAQC_getTEMP_scale(int addr, char scale);
```

**Description:**  
Read onboard temperature sensor.

**Parameters:**
- `scale` - Temperature scale ('C', 'F', or 'K')

**Returns:**  
Temperature in specified scale

**Example:**
```c
// Default (Celsius)
double temp_c = DAQC_getTEMP(0);

// Fahrenheit
double temp_f = DAQC_getTEMP_scale(0, 'F');

// Kelvin
double temp_k = DAQC_getTEMP_scale(0, 'K');

printf("Temperature: %.1f°C / %.1f°F / %.1fK\n", temp_c, temp_f, temp_k);
```

### Range Finder

```c
double DAQC_getRANGE(int addr, char units);
```

**Description:**  
Read ultrasonic range finder distance.

**Parameters:**
- `units` - Unit of measurement ('c' for cm, 'i' for inches)

**Returns:**  
Distance in specified units

**Example:**
```c
double dist_cm = DAQC_getRANGE(0, 'c');
double dist_in = DAQC_getRANGE(0, 'i');
printf("Distance: %.1f cm (%.1f in)\n", dist_cm, dist_in);
```

### Interrupt Management

```c
void DAQC_intEnable(int addr);
void DAQC_intDisable(int addr);
int DAQC_getINTflags(int addr);
```

**Description:**  
Enable/disable interrupts and check interrupt flags.

### Calibration

```c
void DAQC_calDAC(int addr);
void DAQC_calADC(int addr);
```

**Description:**  
Calibrate DAC and ADC channels.

### Reset and Help

```c
void DAQC_resetDAQC(int addr);
void DAQC_help(void);
```

---

## DAQC2 Functions

The DAQC2 plate is an enhanced version of DAQC with additional features including stepper motor control, function generator, and oscilloscope capabilities.

### Basic Information

```c
int DAQC2_getADDR(int addr);
char* DAQC2_getID(int addr);
double DAQC2_getHWrev(int addr);
double DAQC2_getFWrev(int addr);
```

### Multi-Color LED

DAQC2 has an RGB LED with 8 color settings:

```c
void DAQC2_setLED(int addr, int color);
```

**Parameters:**
- `color` - Color code (0-7):
  - 0 = Off
  - 1 = Red
  - 2 = Green
  - 3 = Yellow
  - 4 = Blue
  - 5 = Magenta
  - 6 = Cyan
  - 7 = White

**Example:**
```c
DAQC2_setLED(0, 0);  // Off
DAQC2_setLED(0, 1);  // Red
DAQC2_setLED(0, 7);  // White
```

### ADC, DAC, Digital I/O, PWM

DAQC2 includes all DAQC functions plus enhanced features:

```c
// ADC
double DAQC2_getADC(int addr, int channel);
int DAQC2_getADCall(int addr, double* results, int max_results);

// Digital I/O
void DAQC2_setDOUTbit(int addr, int bit);
void DAQC2_clrDOUTbit(int addr, int bit);
void DAQC2_toggleDOUTbit(int addr, int bit);
void DAQC2_setDOUTall(int addr, int value);
int DAQC2_getDINbit(int addr, int bit);
int DAQC2_getDINall(int addr);

// PWM
void DAQC2_setPWM(int addr, int channel, int value);
int DAQC2_getPWM(int addr, int channel);

// DAC
void DAQC2_setDAC(int addr, int channel, double voltage);
double DAQC2_getDAC(int addr, int channel);
```

### Function Generator

```c
void DAQC2_fgON(int addr);
void DAQC2_fgOFF(int addr);
void DAQC2_fgFREQ(int addr, double freq);
void DAQC2_fgTYPE(int addr, int wave_type);
void DAQC2_fgLEVEL(int addr, double amplitude);
```

**Description:**  
Built-in function generator with multiple waveform types.

**Parameters:**
- `freq` - Frequency in Hz
- `wave_type` - Waveform type:
  - 0 = Sine wave
  - 1 = Triangle wave
  - 2 = Square wave
  - 3 = Sawtooth wave
- `amplitude` - Peak amplitude in volts (0-4.095V)

**Example:**
```c
// Generate 1 kHz sine wave at 2.5V amplitude
DAQC2_fgFREQ(0, 1000.0);
DAQC2_fgTYPE(0, 0);        // Sine
DAQC2_fgLEVEL(0, 2.5);
DAQC2_fgON(0);

// Switch to square wave
DAQC2_fgTYPE(0, 2);

// Turn off
DAQC2_fgOFF(0);
```

### Stepper Motor Control

```c
void DAQC2_motorENABLE(int addr, int motor);
void DAQC2_motorDISABLE(int addr, int motor);
void DAQC2_motorDIR(int addr, int motor, int direction);
void DAQC2_motorRATE(int addr, int motor, int steps_per_sec);
void DAQC2_motorMOVE(int addr, int motor, int steps);
void DAQC2_motorJOG(int addr, int motor);
void DAQC2_motorSTOP(int addr, int motor);
void DAQC2_motorOFF(int addr, int motor);
```

**Description:**  
Control up to 2 stepper motors with precise step control.

**Parameters:**
- `motor` - Motor number (1 or 2)
- `direction` - Direction (0=forward, 1=reverse)
- `steps_per_sec` - Step rate (steps per second)
- `steps` - Number of steps to move

**Example:**
```c
// Configure motor 1
DAQC2_motorENABLE(0, 1);
DAQC2_motorDIR(0, 1, 0);        // Forward
DAQC2_motorRATE(0, 1, 200);     // 200 steps/sec

// Move 400 steps
DAQC2_motorMOVE(0, 1, 400);

// Continuous movement (jogging)
DAQC2_motorJOG(0, 1);

// Stop movement
DAQC2_motorSTOP(0, 1);

// Disable motor (removes holding torque)
DAQC2_motorDISABLE(0, 1);
```

### Oscilloscope Functions

```c
void DAQC2_setOSCchannel(int addr, int channel);
void DAQC2_setOSCsweep(int addr, int sweep_time);
void DAQC2_startOSC(int addr);
void DAQC2_stopOSC(int addr);
void DAQC2_runOSC(int addr);
int DAQC2_getOSCtraces(int addr, double* results, int max_results);
```

**Description:**  
Built-in oscilloscope for capturing waveforms.

**Parameters:**
- `channel` - ADC channel to capture (0-7)
- `sweep_time` - Time per division in microseconds
- `results` - Array to store trace data
- `max_results` - Maximum array size (typically 1024)

**Returns:**  
Number of trace points captured

**Example:**
```c
// Configure oscilloscope
DAQC2_setOSCchannel(0, 0);     // Channel 0
DAQC2_setOSCsweep(0, 100);     // 100 µs/div

// Capture trace
DAQC2_startOSC(0);

// Get trace data
double trace[1024];
int count = DAQC2_getOSCtraces(0, trace, 1024);

// Process trace
for (int i = 0; i < count; i++) {
    printf("%d: %.3f\n", i, trace[i]);
}

DAQC2_stopOSC(0);
```

### Reset and Help

```c
void DAQC2_resetDAQC2(int addr);
void DAQC2_help(void);
```

---

## DIGI Functions

The DIGI (Digital I/O) plate provides basic identification and LED control functions.

**Note:** This implementation does not include digital input/output control functions. Only basic information and LED control are available.

### Basic Information

```c
int DIGI_getADDR(int addr);
char* DIGI_getID(int addr);
double DIGI_getHWrev(int addr);
double DIGI_getFWrev(int addr);
```

**Example:**
```c
if (DIGI_getADDR(0) == 0) {
    char* id = DIGI_getID(0);
    printf("DIGI Plate: %s\n", id);
    printf("HW Rev: %.1f\n", DIGI_getHWrev(0));
    printf("FW Rev: %.1f\n", DIGI_getFWrev(0));
}
```

### LED Control

```c
void DIGI_setLED(int addr);
void DIGI_clrLED(int addr);
void DIGI_toggleLED(int addr);
```

**Example:**
```c
DIGI_setLED(0);       // Turn on LED
DIGI_clrLED(0);       // Turn off LED
DIGI_toggleLED(0);    // Toggle LED state
```

---

## RELAY Functions

The RELAY plate provides 7 electromechanical relays with SPST-NO (Single Pole Single Throw - Normally Open) contacts.

### Basic Information

```c
int RELAY_getADDR(int addr);
char* RELAY_getID(int addr);
double RELAY_getHWrev(int addr);
double RELAY_getFWrev(int addr);
```

### LED Control

```c
void RELAY_setLED(int addr);
void RELAY_clrLED(int addr);
void RELAY_toggleLED(int addr);
```

### Individual Relay Control

```c
void RELAY_relayON(int addr, int relay);
void RELAY_relayOFF(int addr, int relay);
void RELAY_relayTOGGLE(int addr, int relay);
```

**Description:**  
Control individual relays.

**Parameters:**
- `relay` - Relay number (1-7)

**Example:**
```c
// Turn on relay 1
RELAY_relayON(0, 1);

// Turn off relay 2
RELAY_relayOFF(0, 2);

// Toggle relay 3
RELAY_relayTOGGLE(0, 3);
```

### All Relays Control

```c
void RELAY_relayALL(int addr, int state);
int RELAY_relaySTATE(int addr);
```

**Description:**  
Control all relays at once or read current state.

**Parameters:**
- `state` - 7-bit value (0-127) representing relay states

**Example:**
```c
// Turn on relays 1, 3, and 5 (binary: 0010101)
RELAY_relayALL(0, 0b0010101);

// Turn on all relays
RELAY_relayALL(0, 0x7F);  // 0x7F = 0b1111111

// Turn off all relays
RELAY_relayALL(0, 0);

// Read current state
int state = RELAY_relaySTATE(0);
printf("Relay state: 0x%02X\n", state);
```

### Reset and Help

```c
void RELAY_resetRELAY(int addr);
void RELAY_help(void);
```

---

## RELAY2 Functions

The RELAY2 plate provides 8 electromechanical relays (one more than the RELAY plate).

### Basic Information

```c
int RELAY2_getADDR(int addr);
char* RELAY2_getID(int addr);
double RELAY2_getHWrev(int addr);
double RELAY2_getFWrev(int addr);
```

### LED Control

```c
void RELAY2_setLED(int addr);
void RELAY2_clrLED(int addr);
void RELAY2_toggleLED(int addr);
```

### Individual Relay Control

```c
void RELAY2_relayON(int addr, int relay);
void RELAY2_relayOFF(int addr, int relay);
void RELAY2_relayTOGGLE(int addr, int relay);
```

**Description:**  
Control individual relays.

**Parameters:**
- `relay` - Relay number (1-8)

**Example:**
```c
// Turn on relay 1
RELAY2_relayON(0, 1);

// Turn off relay 2
RELAY2_relayOFF(0, 2);

// Toggle relay 8
RELAY2_relayTOGGLE(0, 8);
```

### All Relays Control

```c
void RELAY2_relayALL(int addr, int state);
int RELAY2_relaySTATE(int addr);
```

**Description:**  
Control all relays at once or read current state.

**Parameters:**
- `state` - 8-bit value (0-255) representing relay states

**Example:**
```c
// Turn on relays 1, 4, and 8 (binary: 10001001)
RELAY2_relayALL(0, 0b10001001);

// Turn on all relays
RELAY2_relayALL(0, 0xFF);  // 0xFF = 0b11111111

// Turn off all relays
RELAY2_relayALL(0, 0);

// Read current state
int state = RELAY2_relaySTATE(0);
printf("Relay state: 0x%02X\n", state);
```

### Reset and Help

```c
void RELAY2_resetRELAY2(int addr);
void RELAY2_help(void);
```

---

## THERMO Functions

The THERMO (Thermocouple) plate provides 8 channels of thermocouple temperature measurement with cold junction compensation.

### Basic Information

```c
int THERMO_getADDR(int addr);
char* THERMO_getID(int addr);
double THERMO_getHWrev(int addr);
double THERMO_getFWrev(int addr);
```

### LED Control

```c
void THERMO_setLED(int addr);
void THERMO_clrLED(int addr);
void THERMO_toggleLED(int addr);
```

### Temperature Measurement

```c
double THERMO_getTEMP(int addr, int channel);
double THERMO_getTEMP_scale(int addr, int channel, char scale);
```

**Description:**  
Read thermocouple temperature.

**Parameters:**
- `channel` - Thermocouple channel (1-8)
- `scale` - Temperature scale ('C', 'F', or 'K')

**Returns:**  
Temperature in specified scale

**Example:**
```c
// Read channel 1 in Celsius (default)
double temp_c = THERMO_getTEMP(0, 1);

// Read in Fahrenheit
double temp_f = THERMO_getTEMP_scale(0, 1, 'F');

// Read in Kelvin
double temp_k = THERMO_getTEMP_scale(0, 1, 'K');

printf("Thermocouple 1: %.1f°C / %.1f°F / %.1fK\n", temp_c, temp_f, temp_k);
```

### Cold Junction Temperature

```c
double THERMO_getCOLD(int addr);
double THERMO_getCOLD_scale(int addr, char scale);
```

**Description:**  
Read cold junction (reference) temperature used for thermocouple compensation.

**Example:**
```c
// Celsius
double cj_temp = THERMO_getCOLD(0);

// Fahrenheit
double cj_temp_f = THERMO_getCOLD_scale(0, 'F');

printf("Cold junction: %.1f°C\n", cj_temp);
```

### Thermocouple Type Configuration

```c
void THERMO_setTYPE(int addr, int channel, char tc_type);
char THERMO_getTYPE(int addr, int channel);
```

**Description:**  
Set/get thermocouple type for a channel.

**Parameters:**
- `tc_type` - Thermocouple type character:
  - 'K' - Type K (Chromel-Alumel)
  - 'J' - Type J (Iron-Constantan)
  - 'T' - Type T (Copper-Constantan)
  - 'E' - Type E (Chromel-Constantan)
  - 'R' - Type R (Platinum-Rhodium)
  - 'S' - Type S (Platinum-Rhodium)
  - 'B' - Type B (Platinum-Rhodium)
  - 'N' - Type N (Nicrosil-Nisil)

**Example:**
```c
// Set channel 1 to Type K thermocouple
THERMO_setTYPE(0, 1, 'K');

// Set channel 2 to Type J thermocouple
THERMO_setTYPE(0, 2, 'J');

// Check what type is configured
char type = THERMO_getTYPE(0, 1);
printf("Channel 1 thermocouple type: %c\n", type);
```

### Reset and Help

```c
void THERMO_resetTHERMO(int addr);
void THERMO_help(void);
```

---

## Example Programs

### Example 1: Basic Initialization and Plate Discovery

```c
#include "BRIDGEplate.h"
#include <stdio.h>

int main(void) {
    // Initialize connection
    printf("Initializing BRIDGEplate...\n");
    if (BRIDGEplate_init() != 0) {
        fprintf(stderr, "Failed to connect to BRIDGEplate\n");
        return 1;
    }
    
    // Get BRIDGE information
    printf("\nBRIDGE Information:\n");
    printf("  ID: %s\n", BRIDGE_getID());
    printf("  HW Rev: %.1f\n", BRIDGE_getHWrev());
    printf("  FW Rev: %.1f\n", BRIDGE_getFWrev());
    
    // Scan for connected plates
    printf("\nScanning for connected plates...\n");
    POLL();
    
    // Clean up
    BRIDGEplate_close();
    return 0;
}
```

**Compile (Windows):**
```bash
gcc -Wall -O2 -o example1.exe example1.c BRIDGEplate.c -lsetupapi
```

**Compile (Linux):**
```bash
gcc -Wall -O2 -o example1 example1.c BRIDGEplate.c
```

### Example 2: ADC Data Acquisition

```c
#include "BRIDGEplate.h"
#include <stdio.h>
#include <unistd.h>  // For usleep

int main(void) {
    if (BRIDGEplate_init() != 0) {
        fprintf(stderr, "Failed to initialize\n");
        return 1;
    }
    
    // Check if ADC plate is present at address 0
    if (ADC_getADDR(0) == 0) {
        printf("ADC plate found at address 0\n");
        printf("ADC ID: %s\n", ADC_getID(0));
        
        // Read all channels 10 times
        printf("\nReading ADC channels:\n");
        for (int i = 0; i < 10; i++) {
            double voltages[8];
            int count = ADC_getADCall(0, voltages, 8);
            
            printf("Sample %d: ", i + 1);
            for (int ch = 0; ch < count; ch++) {
                printf("CH%d=%.3fV ", ch, voltages[ch]);
            }
            printf("\n");
            
            usleep(500000);  // 500ms delay
        }
    } else {
        printf("No ADC plate found at address 0\n");
    }
    
    BRIDGEplate_close();
    return 0;
}
```

### Example 3: Relay Control

```c
#include "BRIDGEplate.h"
#include <stdio.h>
#include <unistd.h>

int main(void) {
    if (BRIDGEplate_init() != 0) {
        fprintf(stderr, "Failed to initialize\n");
        return 1;
    }
    
    // Check for RELAY plate
    if (RELAY_getADDR(0) == 0) {
        printf("RELAY plate found at address 0\n");
        
        // Turn on LED
        RELAY_setLED(0);
        
        // Sequential relay test
        printf("Testing relays sequentially...\n");
        for (int relay = 1; relay <= 7; relay++) {
            printf("Turning on relay %d\n", relay);
            RELAY_relayON(0, relay);
            usleep(500000);  // 500ms
            
            printf("Turning off relay %d\n", relay);
            RELAY_relayOFF(0, relay);
            usleep(500000);
        }
        
        // Turn on all relays
        printf("\nTurning on all relays\n");
        RELAY_relayALL(0, 0x7F);
        sleep(2);
        
        // Turn off all relays
        printf("Turning off all relays\n");
        RELAY_relayALL(0, 0);
        
        // Turn off LED
        RELAY_clrLED(0);
    } else {
        printf("No RELAY plate found at address 0\n");
    }
    
    BRIDGEplate_close();
    return 0;
}
```

### Example 4: DAQC Digital I/O and DAC

```c
#include "BRIDGEplate.h"
#include <stdio.h>
#include <unistd.h>

int main(void) {
    if (BRIDGEplate_init() != 0) {
        fprintf(stderr, "Failed to initialize\n");
        return 1;
    }
    
    if (DAQC_getADDR(0) == 0) {
        printf("DAQC plate found at address 0\n");
        
        // Set LEDs
        DAQC_setLED(0, 0);
        DAQC_setLED(0, 1);
        
        // Test digital outputs
        printf("Testing digital outputs...\n");
        for (int bit = 0; bit < 7; bit++) {
            DAQC_setDOUTbit(0, bit);
            printf("Set DOUT bit %d\n", bit);
            usleep(200000);
        }
        
        // Clear all outputs
        DAQC_setDOUTall(0, 0);
        
        // Test DAC with ramp
        printf("\nGenerating DAC ramp on channel 0...\n");
        for (double v = 0.0; v <= 4.0; v += 0.5) {
            DAQC_setDAC(0, 0, v);
            printf("DAC = %.1f V\n", v);
            usleep(500000);
        }
        
        // Set to 0V
        DAQC_setDAC(0, 0, 0.0);
        
        // Read temperature
        double temp = DAQC_getTEMP(0);
        printf("\nOnboard temperature: %.1f°C\n", temp);
        
        // Clear LEDs
        DAQC_clrLED(0, 0);
        DAQC_clrLED(0, 1);
    } else {
        printf("No DAQC plate found at address 0\n");
    }
    
    BRIDGEplate_close();
    return 0;
}
```

### Example 5: Current Loop Monitoring

```c
#include "BRIDGEplate.h"
#include <stdio.h>
#include <unistd.h>

int main(void) {
    if (BRIDGEplate_init() != 0) {
        fprintf(stderr, "Failed to initialize\n");
        return 1;
    }
    
    if (CURRENT_getADDR(0) == 0) {
        printf("CURRENT plate found at address 0\n");
        printf("Monitoring 4-20mA inputs...\n\n");
        
        CURRENT_setLED(0);
        
        // Continuous monitoring
        for (int i = 0; i < 20; i++) {
            double currents[8];
            int count = CURRENT_getIall(0, currents, 8);
            
            printf("Sample %2d: ", i + 1);
            for (int ch = 0; ch < count; ch++) {
                // Convert 4-20mA to 0-100%
                double percent = (currents[ch] - 4.0) / 16.0 * 100.0;
                printf("CH%d=%5.2fmA (%5.1f%%) ", ch + 1, currents[ch], percent);
            }
            printf("\n");
            
            usleep(1000000);  // 1 second
        }
        
        CURRENT_clrLED(0);
    } else {
        printf("No CURRENT plate found at address 0\n");
    }
    
    BRIDGEplate_close();
    return 0;
}
```

### Example 6: DAQC2 Function Generator

```c
#include "BRIDGEplate.h"
#include <stdio.h>
#include <unistd.h>

int main(void) {
    if (BRIDGEplate_init() != 0) {
        fprintf(stderr, "Failed to initialize\n");
        return 1;
    }
    
    if (DAQC2_getADDR(0) == 0) {
        printf("DAQC2 plate found at address 0\n");
        
        // Set LED to green
        DAQC2_setLED(0, 2);
        
        const char* wave_names[] = {"Sine", "Triangle", "Square", "Sawtooth"};
        
        // Test each waveform type
        for (int type = 0; type < 4; type++) {
            printf("\nGenerating %s wave at 1 kHz, 2.5V amplitude\n", wave_names[type]);
            
            DAQC2_fgFREQ(0, 1000.0);
            DAQC2_fgTYPE(0, type);
            DAQC2_fgLEVEL(0, 2.5);
            DAQC2_fgON(0);
            
            sleep(3);  // Run for 3 seconds
            
            DAQC2_fgOFF(0);
            sleep(1);
        }
        
        // Frequency sweep with sine wave
        printf("\nFrequency sweep: 100 Hz to 10 kHz\n");
        DAQC2_fgTYPE(0, 0);  // Sine
        DAQC2_fgLEVEL(0, 2.0);
        
        for (double freq = 100.0; freq <= 10000.0; freq *= 1.5) {
            printf("Frequency: %.0f Hz\n", freq);
            DAQC2_fgFREQ(0, freq);
            DAQC2_fgON(0);
            usleep(500000);
        }
        
        DAQC2_fgOFF(0);
        DAQC2_setLED(0, 0);  // Turn off LED
    } else {
        printf("No DAQC2 plate found at address 0\n");
    }
    
    BRIDGEplate_close();
    return 0;
}
```

### Example 7: Thermocouple Monitoring

```c
#include "BRIDGEplate.h"
#include <stdio.h>
#include <unistd.h>

int main(void) {
    if (BRIDGEplate_init() != 0) {
        fprintf(stderr, "Failed to initialize\n");
        return 1;
    }
    
    if (THERMO_getADDR(0) == 0) {
        printf("THERMO plate found at address 0\n");
        
        // Configure thermocouple types
        THERMO_setTYPE(0, 1, 'K');  // Channel 1: Type K
        THERMO_setTYPE(0, 2, 'K');  // Channel 2: Type K
        
        THERMO_setLED(0);
        
        printf("\nMonitoring thermocouples...\n");
        printf("Channel 1 and 2 configured for Type K thermocouples\n\n");
        
        // Monitor for 30 seconds
        for (int i = 0; i < 30; i++) {
            // Read cold junction
            double cj = THERMO_getCOLD(0);
            
            // Read thermocouples
            double tc1_c = THERMO_getTEMP(0, 1);
            double tc1_f = THERMO_getTEMP_scale(0, 1, 'F');
            
            double tc2_c = THERMO_getTEMP(0, 2);
            double tc2_f = THERMO_getTEMP_scale(0, 2, 'F');
            
            printf("CJ: %6.2f°C  |  ", cj);
            printf("TC1: %6.2f°C (%6.2f°F)  |  ", tc1_c, tc1_f);
            printf("TC2: %6.2f°C (%6.2f°F)\n", tc2_c, tc2_f);
            
            sleep(1);
        }
        
        THERMO_clrLED(0);
    } else {
        printf("No THERMO plate found at address 0\n");
    }
    
    BRIDGEplate_close();
    return 0;
}
```

### Compiling All Examples

**Windows batch file (compile_all.bat):**
```batch
@echo off
gcc -Wall -O2 -o example1.exe example1.c BRIDGEplate.c -lsetupapi
gcc -Wall -O2 -o example2.exe example2.c BRIDGEplate.c -lsetupapi
gcc -Wall -O2 -o example3.exe example3.c BRIDGEplate.c -lsetupapi
gcc -Wall -O2 -o example4.exe example4.c BRIDGEplate.c -lsetupapi
gcc -Wall -O2 -o example5.exe example5.c BRIDGEplate.c -lsetupapi
gcc -Wall -O2 -o example6.exe example6.c BRIDGEplate.c -lsetupapi
gcc -Wall -O2 -o example7.exe example7.c BRIDGEplate.c -lsetupapi
echo All examples compiled successfully!
```

**Linux shell script (compile_all.sh):**
```bash
#!/bin/bash
gcc -Wall -O2 -o example1 example1.c BRIDGEplate.c
gcc -Wall -O2 -o example2 example2.c BRIDGEplate.c
gcc -Wall -O2 -o example3 example3.c BRIDGEplate.c
gcc -Wall -O2 -o example4 example4.c BRIDGEplate.c
gcc -Wall -O2 -o example5 example5.c BRIDGEplate.c
gcc -Wall -O2 -o example6 example6.c BRIDGEplate.c
gcc -Wall -O2 -o example7 example7.c BRIDGEplate.c
echo "All examples compiled successfully!"
```

Make executable on Linux:
```bash
chmod +x compile_all.sh
./compile_all.sh
```

---

## Appendix A: Error Handling

Most functions return values that indicate success or failure:

- **Measurement functions** return `double` - check for 0.0 or unreasonable values
- **Status functions** return `int` - check return value
- **String functions** return `char*` - check for NULL or empty string
- **Command functions** return `void` - no direct error indication

**Best practices:**
```c
// Check initialization
if (BRIDGEplate_init() != 0) {
    fprintf(stderr, "Initialization failed\n");
    return 1;
}

// Verify plate presence
if (ADC_getADDR(0) == 0) {
    // Plate is present
} else {
    fprintf(stderr, "No ADC plate at address 0\n");
}

// Check measurement validity
double voltage = ADC_getADC(0, 0);
if (voltage >= 0.0 && voltage <= 10.0) {
    // Valid reading
} else {
    fprintf(stderr, "Invalid voltage reading\n");
}
```

---

## Appendix B: Function Parameter Conventions

### Common Parameters

- **addr** - Plate address (0-7)
- **channel** - Input/output channel number (varies by plate type)
- **bit** - Bit number (0-7 typically)
- **value** - Numeric value (range depends on function)
- **scale** - Temperature scale ('C', 'F', 'K')
- **results** - Pointer to array for multi-value returns
- **max_results** - Size of results array

### Variable Argument Functions

The `parseIt` function accepts variable numbers of arguments:

```c
double parseIt(const char* cmd, int arg_count, ...);
```

The `arg_count` tells parseIt how many arguments follow. Each wrapper function specifies this:

```c
// 0 arguments after command name
void BRIDGE_resetSTACK(void) { 
    parseIt("BRIDGE.resetSTACK", 0); 
}

// 1 argument
void ADC_setLED(int addr) { 
    parseIt("ADC.setLED", 1, addr); 
}

// 2 arguments
void ADC_setMODE(int addr, int mode) { 
    parseIt("ADC.setMODE", 2, addr, mode); 
}
```

---

## Appendix C: Serial Communication Details

### Protocol

- **Baud Rate:** 115200
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1
- **Flow Control:** None (DTR/RTS asserted for device detection)

### Command Format

Commands sent to the hardware follow this format:
```
PLATETYPE.function(arg1,arg2,...)
```

Example:
```
ADC.getADC(0,2)
```

### Response Protocol

The BRIDGEplate firmware echoes each command followed by the response:

1. **Echo:** Command line is echoed back
2. **Response:** Actual data response on next line

The library automatically discards the echo and extracts the response.

### Timeouts

- **Initial wait:** 2 seconds for device to process command
- **Inter-byte:** 100ms between bytes in response
- **Overall:** 20 second maximum per transaction

---

## Appendix D: Platform-Specific Notes

### Windows

- Uses Windows API for serial communication (`CreateFile`, `ReadFile`, `WriteFile`)
- Uses Setup API for USB device enumeration
- COM port names use `\\.\COMx` format for ports >= 10
- Requires linking with `setupapi.lib`

### Linux

- Uses POSIX termios for serial communication
- Uses sysfs for USB device enumeration
- Serial ports appear as `/dev/ttyACM0`, `/dev/ttyACM1`, etc.
- Requires user in `dialout` group for serial port access

### macOS

Not currently supported. Would require implementation of USB device enumeration using IOKit.

---

## Appendix E: Troubleshooting

### Device Not Found

```
No COM port found with attached BRIDGEplate
```

**Solutions:**
- Check USB cable connection
- Verify BRIDGEplate is powered on
- Windows: Check Device Manager for COM port
- Linux: Check `lsusb` for device (2E8A:10E3)
- Linux: Check `/dev/ttyACM*` exists

### Permission Denied (Linux)

```
Failed to open port /dev/ttyACM0
```

**Solutions:**
```bash
# Temporary fix
sudo chmod 666 /dev/ttyACM0

# Permanent fix
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Timeouts or No Response

**Solutions:**
- Check that only one program is accessing the port
- Verify baud rate is 115200
- Try unplugging and reconnecting USB
- Check for interference on long USB cables

### Compilation Errors

**Windows:**
```
undefined reference to `SetupDiGetClassDevs'
```
**Solution:** Add `-lsetupapi` to link command

**Linux:**
```
undefined reference to `usleep'
```
**Solution:** Add `#include <unistd.h>` to source file

---

**End of Function Reference**
