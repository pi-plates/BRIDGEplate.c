# BRIDGEplate C Library Documentation

## Overview

This is a C library for communicating with Pi-Plates BRIDGEplate and its stackable plate ecosystem via USB CDC serial interface. The library supports multiple plate types including ADC, BRIDGE, CURRENT, DAQC, DAQC2, DIGI, RELAY, RELAY2, and THERMO plates.

## Features

- **Cross-platform**: Works on Windows and Linux
- **Automatic device detection**: Finds BRIDGEplate by VID/PID
- **String arguments**: Uses descriptive string constants for better code readability
- **Full plate support**: All supported Pi-Plate functions implemented
- **Type-safe**: Proper error handling and return types

## Compilation

### Windows
```bash
gcc -o myprogram myprogram.c BRIDGEplate.c -lsetupapi
```

### Linux
```bash
gcc -o myprogram myprogram.c BRIDGEplate.c
```

## Basic Usage

```c
#include "BRIDGEplate.h"

int main() {
    if (BRIDGEplate_init() != 0) {
        printf("Failed to initialize\n");
        return 1;
    }

    char* id = BRIDGE_getID();
    printf("Device: %s\n", id);

    BRIDGEplate_close();
    return 0;
}
```

---

## Core Functions

```c
int  BRIDGEplate_init(void);   // Initialize USB connection. Returns 0 on success, -1 on failure.
void BRIDGEplate_close(void);  // Close USB connection and cleanup.
void POLL(void);               // Poll all addresses for connected plates.
```

---

## ADC Plate

### Information
```c
int    ADC_getADDR(int addr);
char*  ADC_getID(int addr);
double ADC_getHWrev(int addr);
double ADC_getFWrev(int addr);
```

### LED Control
```c
void ADC_setLED(int addr);
void ADC_clrLED(int addr);
void ADC_toggleLED(int addr);
```

### Initialization
```c
void ADC_srTable(int addr);    // Print sample rate table
void ADC_initADC(int addr);    // Initialize ADC hardware
```

### Events
```c
void ADC_enableEVENTS(int addr);
void ADC_disableEVENTS(int addr);
int  ADC_check4EVENTS(int addr);
int  ADC_getEVENTS(int addr);
```

### Mode Configuration
```c
void ADC_setMODE(int addr, const char* mode);  // "SLOW", "MED", "FAST", "ADV" or "0"-"3"
int  ADC_getMODE(int addr);                    // Returns current mode as integer
```

### Input Configuration
```c
void ADC_configINPUT(int addr, int channel, const char* config);  // "SE" or "DIFF"
void ADC_enableINPUT(int addr, int channel);
void ADC_disableINPUT(int addr, int channel);
```

### Single-Channel ADC
```c
double ADC_getADC(int addr, int channel);                    // Read single channel
double ADC_readSINGLE(int addr, int channel);                // Read using default rate
double ADC_readSINGLE_rate(int addr, int channel, int rate); // Read at specified rate
void   ADC_startSINGLE(int addr, int channel);               // Start async single read
double ADC_getSINGLE(int addr);                              // Retrieve async single result
```

### Multi-Channel ADC
```c
int ADC_getADCall(int addr, double* results, int max_results); // All channels, raw
int ADC_getSall(int addr, double* results, int max_results);   // All single-ended channels
int ADC_getDall(int addr, double* results, int max_results);   // All differential channels
int ADC_getIall(int addr, double* results, int max_results);   // All current channels
```

### Scan Mode
```c
int  ADC_readSCAN(int addr, double* results, int max_results); // Read scan immediately
void ADC_startSCAN(int addr);                                  // Start async scan
int  ADC_getSCAN(int addr, double* results, int max_results);  // Retrieve async scan
```

### Block Mode
```c
int  ADC_getBLOCK(int addr, double* results, int max_results); // Read block immediately
void ADC_startBLOCK(int addr);                                 // Start async block read
```

### Stream Mode
```c
void ADC_startSTREAM(int addr);
int  ADC_getSTREAM(int addr, double* results, int max_results);
void ADC_stopSTREAM(int addr);
```

### Digital Inputs
```c
int  ADC_getDINbit(int addr, int bit);
int  ADC_getDINall(int addr);
void ADC_enableDINevent(int addr, int bit);
void ADC_disableDINevent(int addr, int bit);
```

### Trigger Control
```c
void ADC_configTRIG(int addr, const char* config);  // "RISING" or "FALLING"
void ADC_startTRIG(int addr);
void ADC_stopTRIG(int addr);
void ADC_triggerFREQ(int addr, int freq);
void ADC_swTRIGGER(int addr);
int  ADC_maxTRIGfreq(int addr);
```

### Help
```c
void ADC_help(void);  // Print available commands to stdout
```

---

## BRIDGE Plate

### Information
```c
char*  BRIDGE_getID(void);
double BRIDGE_getHWrev(void);
double BRIDGE_getFWrev(void);
```

### Control
```c
void BRIDGE_resetSTACK(void);
void BRIDGE_resetBRIDGE(void);
int  BRIDGE_getSRQ(void);
void BRIDGE_setMODE(const char* mode);  // Mode strings (hardware dependent)
```

### Help
```c
void BRIDGE_help(void);
```

---

## CURRENT Plate

### Information
```c
int    CURRENT_getADDR(int addr);
char*  CURRENT_getID(int addr);
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
int    CURRENT_getIall(int addr, double* results, int max_results);
```

### Help
```c
void CURRENT_help(void);
```

---

## DAQC Plate

### Information
```c
int    DAQC_getADDR(int addr);
char*  DAQC_getID(int addr);
double DAQC_getHWrev(int addr);
double DAQC_getFWrev(int addr);
```

### LED Control
```c
void DAQC_setLED(int addr, int led);     // led = numeric LED number (0, 1, ...)
void DAQC_clrLED(int addr, int led);
void DAQC_toggleLED(int addr, int led);
int  DAQC_getLED(int addr, int led);
```

### ADC
```c
double DAQC_getADC(int addr, int channel);
int    DAQC_getADCall(int addr, double* results, int max_results);
```

### Digital Inputs
```c
int  DAQC_getDINbit(int addr, int bit);
int  DAQC_getDINall(int addr);
void DAQC_enableDINint(int addr, int bit);
void DAQC_disableDINint(int addr, int bit);
```

### Digital Outputs
```c
void DAQC_setDOUTbit(int addr, int bit);
void DAQC_clrDOUTbit(int addr, int bit);
void DAQC_toggleDOUTbit(int addr, int bit);
void DAQC_setDOUTall(int addr, int value);
int  DAQC_getDOUTbyte(int addr);
```

### DAC
```c
void   DAQC_setDAC(int addr, int channel, double value);
double DAQC_getDAC(int addr, int channel);
```

### PWM
```c
void DAQC_setPWM(int addr, int channel, int value);
int  DAQC_getPWM(int addr, int channel);
```

### Range
```c
double DAQC_getRANGE(int addr);
```

### Temperature
```c
double DAQC_getTEMP(int addr, char scale);  // scale = 'C' or 'F'
```

### Interrupts
```c
void DAQC_intENABLE(int addr);
void DAQC_intDISABLE(int addr);
int  DAQC_getINTflags(int addr);
```

### Help
```c
void DAQC_help(void);
```

---

## DAQC2 Plate

### Information
```c
int    DAQC2_getADDR(int addr);
char*  DAQC2_getID(int addr);
double DAQC2_getHWrev(int addr);
double DAQC2_getFWrev(int addr);
void   DAQC2_RESET(int addr);
```

### Interrupts
```c
void DAQC2_intEnable(int addr);
void DAQC2_intDisable(int addr);
int  DAQC2_getINTflags(int addr);
```

### LED Control
```c
// RGB LED - accepts color name strings
void DAQC2_setLED(int addr, const char* led);
int  DAQC2_getLED(int addr);
```

**Valid color strings**: `"off"`, `"red"`, `"green"`, `"yellow"`, `"blue"`, `"magenta"`, `"cyan"`, `"white"`

```c
DAQC2_setLED(0, "red");      // Red only
DAQC2_setLED(0, "yellow");   // Red + Green
DAQC2_setLED(0, "magenta");  // Red + Blue
DAQC2_setLED(0, "cyan");     // Green + Blue
DAQC2_setLED(0, "white");    // All colors on
DAQC2_setLED(0, "off");      // Off
```

### ADC
```c
double DAQC2_getADC(int addr, int channel);
int    DAQC2_getADCall(int addr, double* results, int max_results);
```

### DAC
```c
void   DAQC2_setDAC(int addr, int channel, double value);
double DAQC2_getDAC(int addr, int channel);
```

### Digital Inputs
```c
int  DAQC2_getDINbit(int addr, int bit);
void DAQC2_enableDINint(int addr, int bit);
void DAQC2_disableDINint(int addr, int bit);
int  DAQC2_getDINall(int addr);
```

### Digital Outputs
```c
void DAQC2_setDOUTbit(int addr, int bit);
void DAQC2_clrDOUTbit(int addr, int bit);
void DAQC2_toggleDOUTbit(int addr, int bit);
void DAQC2_setDOUTall(int addr, int value);
int  DAQC2_getDOUTbyte(int addr);
```

### PWM
```c
void DAQC2_setPWM(int addr, int channel, int value);
int  DAQC2_getPWM(int addr, int channel);
```

### Frequency / SRQ
```c
int    DAQC2_getSRQ(int addr);
double DAQC2_getFREQ(int addr);
void   DAQC2_setSRQ(int addr, int value);
void   DAQC2_clrSRQ(int addr, int value);
```

### Function Generator
```c
void DAQC2_fgON(int addr);
void DAQC2_fgOFF(int addr);
void DAQC2_fgFREQ(int addr, double freq);
void DAQC2_fgTYPE(int addr, const char* type);  // "sine","square","triangle","sawtooth" or "0"-"3"
void DAQC2_fgLEVEL(int addr, double level);
```

### Motor Control
```c
void DAQC2_motorENABLE(int addr, int motor);
void DAQC2_motorDISABLE(int addr, int motor);
void DAQC2_motorMOVE(int addr, int motor, int steps);
void DAQC2_motorJOG(int addr, int motor);
void DAQC2_motorSTOP(int addr, int motor);
void DAQC2_motorDIR(int addr, int motor, const char* dir);  // "cw" or "ccw"
void DAQC2_motorRATE(int addr, int motor, int rate);
void DAQC2_motorOFF(int addr, int motor);
void DAQC2_motorINTenable(int addr, int motor);
void DAQC2_motorINTdisable(int addr, int motor);
```

### Oscilloscope
```c
void DAQC2_startOSC(int addr);
void DAQC2_stopOSC(int addr);
void DAQC2_runOSC(int addr);
void DAQC2_setOSCchannel(int addr, int channel);
void DAQC2_setOSCsweep(int addr, int sweep);
int  DAQC2_getOSCtraces(int addr, double* results, int max_results);
void DAQC2_setOSCtrigger(int addr, int trigger);
void DAQC2_trigOSCnow(int addr);
```

### Help
```c
void DAQC2_help(void);
```

---

## DIGI Plate

### Information
```c
int    DIGI_getADDR(int addr);
char*  DIGI_getID(int addr);
double DIGI_getHWrev(int addr);
double DIGI_getFWrev(int addr);
```

### LED Control
```c
void DIGI_setLED(int addr);
void DIGI_clrLED(int addr);
void DIGI_toggleLED(int addr);
```

---

## RELAY Plate

### Information
```c
int    RELAY_getADDR(int addr);
char*  RELAY_getID(int addr);
double RELAY_getHWrev(int addr);
double RELAY_getFWrev(int addr);
```

### LED Control
```c
void RELAY_setLED(int addr);
void RELAY_clrLED(int addr);
void RELAY_toggleLED(int addr);
```

### Relay Control
```c
void RELAY_relayON(int addr, int relay);
void RELAY_relayOFF(int addr, int relay);
void RELAY_relayTOGGLE(int addr, int relay);
void RELAY_relayALL(int addr, int value);
int  RELAY_relaySTATE(int addr);
```

---

## RELAY2 Plate

### Information
```c
int    RELAY2_getADDR(int addr);
char*  RELAY2_getID(int addr);
double RELAY2_getHWrev(int addr);
double RELAY2_getFWrev(int addr);
```

### LED Control
```c
void RELAY2_setLED(int addr);
void RELAY2_clrLED(int addr);
void RELAY2_toggleLED(int addr);
```

### Relay Control
```c
void RELAY2_relayON(int addr, int relay);
void RELAY2_relayOFF(int addr, int relay);
void RELAY2_relayTOGGLE(int addr, int relay);
void RELAY2_relayALL(int addr, int value);
int  RELAY2_relaySTATE(int addr);
```

---

## THERMO Plate

### Information
```c
int    THERMO_getADDR(int addr);
char*  THERMO_getID(int addr);
double THERMO_getHWrev(int addr);
double THERMO_getFWrev(int addr);
```

### LED Control
```c
void THERMO_setLED(int addr);
void THERMO_clrLED(int addr);
void THERMO_toggleLED(int addr);
```

### Thermocouple Configuration
```c
void  THERMO_setTYPE(int addr, int channel, const char* tc_type);
char* THERMO_getTYPE(int addr, int channel);
```

**Valid thermocouple type strings**:

| String | Type | Temperature Range |
|--------|------|-------------------|
| `"K"` | Chromel-Alumel | -200°C to 1372°C |
| `"J"` | Iron-Constantan | -210°C to 1200°C |
| `"T"` | Copper-Constantan | -200°C to 400°C |
| `"E"` | Chromel-Constantan | -200°C to 1000°C |
| `"R"` | Platinum-Rhodium | 0°C to 1768°C |
| `"S"` | Platinum-Rhodium | 0°C to 1768°C |
| `"B"` | Platinum-Rhodium | 200°C to 1820°C |
| `"N"` | Nicrosil-Nisil | -200°C to 1300°C |

### Temperature Reading
```c
double THERMO_getTEMP(int addr, int channel, char scale);  // scale = 'C' or 'F'
double THERMO_getCOLD(int addr, char scale);               // Cold junction temperature
```

---

## String Argument Reference

| Function | Argument | Valid Values |
|----------|----------|--------------|
| `ADC_setMODE` | mode | `"SLOW"` `"MED"` `"FAST"` `"ADV"` `"0"`-`"3"` |
| `ADC_configINPUT` | config | `"SE"` `"DIFF"` |
| `ADC_configTRIG` | config | `"RISING"` `"FALLING"` |
| `BRIDGE_setMODE` | mode | Hardware dependent |
| `DAQC2_setLED` | led | `"off"` `"red"` `"green"` `"yellow"` `"blue"` `"magenta"` `"cyan"` `"white"` |
| `DAQC2_fgTYPE` | type | `"sine"` `"square"` `"triangle"` `"sawtooth"` `"0"`-`"3"` |
| `DAQC2_motorDIR` | dir | `"cw"` `"ccw"` `"0"` `"1"` |
| `THERMO_setTYPE` | tc_type | `"K"` `"J"` `"T"` `"E"` `"R"` `"S"` `"B"` `"N"` |

---

## Example Programs

### Example 1: Basic BRIDGE I/O
```c
#include <stdio.h>
#include "BRIDGEplate.h"

int main() {
    if (BRIDGEplate_init() != 0) return 1;

    printf("Device: %s\n", BRIDGE_getID());
    printf("HW Rev: %.1f\n", BRIDGE_getHWrev());
    printf("FW Rev: %.1f\n", BRIDGE_getFWrev());
    printf("Digital inputs: 0x%02X\n", BRIDGE_getDINall());

    BRIDGE_setDOUTall(0xFF);   // All outputs on
    BRIDGE_setDOUTall(0x00);   // All outputs off

    BRIDGEplate_close();
    return 0;
}
```

### Example 2: ADC Reading
```c
#include <stdio.h>
#include "BRIDGEplate.h"

int main() {
    if (BRIDGEplate_init() != 0) return 1;

    ADC_setMODE(0, "FAST");
    ADC_configINPUT(0, 1, "SE");

    double voltage = ADC_getADC(0, 1);
    printf("Channel 1: %.3f V\n", voltage);

    double all[8];
    int n = ADC_getADCall(0, all, 8);
    for (int i = 0; i < n; i++)
        printf("Ch%d: %.3f V\n", i, all[i]);

    BRIDGEplate_close();
    return 0;
}
```

### Example 3: DAQC2 Function Generator and LED
```c
#include <stdio.h>
#include "BRIDGEplate.h"

int main() {
    if (BRIDGEplate_init() != 0) return 1;

    DAQC2_fgTYPE(0, "sine");
    DAQC2_fgFREQ(0, 1000.0);    // 1 kHz
    DAQC2_fgLEVEL(0, 3.3);      // 3.3V amplitude
    DAQC2_fgON(0);

    DAQC2_setLED(0, "green");   // Indicate running

    BRIDGEplate_close();
    return 0;
}
```

### Example 4: Motor Control
```c
#include <stdio.h>
#include "BRIDGEplate.h"

int main() {
    if (BRIDGEplate_init() != 0) return 1;

    DAQC2_motorENABLE(0, 1);
    DAQC2_motorDIR(0, 1, "cw");
    DAQC2_motorRATE(0, 1, 100);
    DAQC2_motorMOVE(0, 1, 200);   // Move 200 steps

    DAQC2_motorDIR(0, 1, "ccw");
    DAQC2_motorJOG(0, 1);         // Jog continuously
    DAQC2_motorSTOP(0, 1);
    DAQC2_motorOFF(0, 1);

    BRIDGEplate_close();
    return 0;
}
```

### Example 5: Oscilloscope
```c
#include <stdio.h>
#include "BRIDGEplate.h"

int main() {
    if (BRIDGEplate_init() != 0) return 1;

    DAQC2_setOSCchannel(0, 1);
    DAQC2_setOSCsweep(0, 10);
    DAQC2_setOSCtrigger(0, 0);
    DAQC2_startOSC(0);
    DAQC2_runOSC(0);

    double traces[1024];
    int n = DAQC2_getOSCtraces(0, traces, 1024);
    printf("Got %d trace samples\n", n);

    DAQC2_stopOSC(0);

    BRIDGEplate_close();
    return 0;
}
```

### Example 6: Temperature Measurement
```c
#include <stdio.h>
#include "BRIDGEplate.h"

int main() {
    if (BRIDGEplate_init() != 0) return 1;

    THERMO_setTYPE(0, 1, "K");
    THERMO_setTYPE(0, 2, "J");

    printf("TC type ch1: %s\n", THERMO_getTYPE(0, 1));
    printf("Ch1: %.2f C\n", THERMO_getTEMP(0, 1, 'C'));
    printf("Ch2: %.2f C\n", THERMO_getTEMP(0, 2, 'C'));
    printf("Cold junction: %.2f C\n", THERMO_getCOLD(0, 'C'));

    BRIDGEplate_close();
    return 0;
}
```

### Example 7: Relay Control
```c
#include <stdio.h>
#include "BRIDGEplate.h"

int main() {
    if (BRIDGEplate_init() != 0) return 1;

    RELAY_relayON(0, 1);
    RELAY_relayOFF(0, 1);
    RELAY_relayTOGGLE(0, 2);
    RELAY_relayALL(0, 0xFF);
    printf("State: 0x%02X\n", RELAY_relaySTATE(0));

    BRIDGEplate_close();
    return 0;
}
```

---

## Platform Notes

### Windows
- Automatic COM port detection via VID/PID
- Requires `setupapi.lib`
- Link with: `gcc ... -lsetupapi`

### Linux
- Automatic detection via `/dev/serial/by-id/`
- Requires `dialout` group membership:
  ```bash
  sudo usermod -a -G dialout $USER
  # Log out and back in
  ```

