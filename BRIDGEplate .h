/*
 * BRIDGEplate.h - Header file for BRIDGEplate interface
 * 
 * This module provides functions to communicate with PI-Plates BRIDGEplate
 * via USB CDC serial interface.
 */

#ifndef BRIDGEPLATE_H
#define BRIDGEPLATE_H

// ============================================================================
// Initialization and core functions
// ============================================================================

/**
 * Initialize connection to BRIDGEplate
 * Returns 0 on success, -1 on failure
 */
int BRIDGEplate_init(void);

/**
 * Close connection to BRIDGEplate
 */
void BRIDGEplate_close(void);

/**
 * Poll all addresses for connected plates
 */
void POLL(void);

// ============================================================================
// ADC Plate Functions
// ============================================================================

int ADC_getADDR(int addr);
char* ADC_getID(int addr);
double ADC_getHWrev(int addr);
double ADC_getFWrev(int addr);

void ADC_setLED(int addr);
void ADC_clrLED(int addr);
void ADC_toggleLED(int addr);

double ADC_getADC(int addr, int channel);
void ADC_srTable(int addr);
void ADC_initADC(int addr);

void ADC_enableEVENTS(int addr);
void ADC_disableEVENTS(int addr);
int ADC_check4EVENTS(int addr);
int ADC_getEVENTS(int addr);

int ADC_getADCall(int addr, double* results, int max_results);
int ADC_getSall(int addr, double* results, int max_results);
int ADC_getDall(int addr, double* results, int max_results);
int ADC_getIall(int addr, double* results, int max_results);

void ADC_setMODE(int addr, int mode);
int ADC_getMODE(int addr);

void ADC_configINPUT(int addr, int channel, int sample_rate);
void ADC_configINPUT_enable(int addr, int channel, int sample_rate, int enable);
void ADC_enableINPUT(int addr, int channel);
void ADC_disableINPUT(int addr, int channel);

double ADC_readSINGLE(int addr, int channel);
double ADC_readSINGLE_rate(int addr, int channel, int rate);
void ADC_startSINGLE(int addr, int channel);
void ADC_startSINGLE_rate(int addr, int channel, int rate);
double ADC_getSINGLE(int addr);

int ADC_readSCAN(int addr, double* results, int max_results);
int ADC_readSCAN_rate(int addr, double* results, int max_results, int rate);
void ADC_startSCAN(int addr);
void ADC_startSCAN_rate(int addr, int rate);
int ADC_getSCAN(int addr, double* results, int max_results);

int ADC_getBLOCK(int addr, double* results, int max_results);
int ADC_getBLOCK_rate(int addr, double* results, int max_results, int rate);
void ADC_startBLOCK(int addr);
void ADC_startBLOCK_rate(int addr, int rate);

void ADC_startSTREAM(int addr);
void ADC_startSTREAM_rate(int addr, int rate);
int ADC_getSTREAM(int addr, double* results, int max_results);
void ADC_stopSTREAM(int addr);

int ADC_getDINbit(int addr, int bit);
int ADC_getDINall(int addr);
void ADC_enableDINevent(int addr, int bit);
void ADC_disableDINevent(int addr, int bit);

void ADC_configTRIG(int addr, int config);
void ADC_startTRIG(int addr);
void ADC_stopTRIG(int addr);
void ADC_triggerFREQ(int addr, int freq);
void ADC_swTRIGGER(int addr);
int ADC_maxTRIGfreq(int addr);

void ADC_help(void);

// ============================================================================
// BRIDGE Plate Functions
// ============================================================================

char* BRIDGE_getID(void);
double BRIDGE_getHWrev(void);
double BRIDGE_getFWrev(void);

void BRIDGE_resetSTACK(void);
int BRIDGE_getSRQ(void);

void BRIDGE_resetBRIDGE(void);
void BRIDGE_help(void);

// ============================================================================
// CURRENT Plate Functions
// ============================================================================

int CURRENT_getADDR(int addr);
char* CURRENT_getID(int addr);
double CURRENT_getHWrev(int addr);
double CURRENT_getFWrev(int addr);

void CURRENT_setLED(int addr);
void CURRENT_clrLED(int addr);
void CURRENT_toggleLED(int addr);

double CURRENT_getI(int addr, int channel);
int CURRENT_getIall(int addr, double* results, int max_results);

void CURRENT_help(void);

// ============================================================================
// DAQC Plate Functions
// ============================================================================

int DAQC_getADDR(int addr);
char* DAQC_getID(int addr);
double DAQC_getHWrev(int addr);
double DAQC_getFWrev(int addr);

void DAQC_setLED(int addr, int led);
void DAQC_clrLED(int addr, int led);
void DAQC_toggleLED(int addr, int led);
int DAQC_getLED(int addr, int led);

double DAQC_getADC(int addr, int channel);
int DAQC_getADCall(int addr, double* results, int max_results);

int DAQC_getDINbit(int addr, int bit);
int DAQC_getDINall(int addr);
void DAQC_enableDINint(int addr, int bit);
void DAQC_disableDINint(int addr, int bit);

double DAQC_getTEMP(int addr);
double DAQC_getTEMP_scale(int addr, char scale);

void DAQC_setDOUTbit(int addr, int bit);
void DAQC_clrDOUTbit(int addr, int bit);
void DAQC_setDOUTall(int addr, int value);
int DAQC_getDOUTbyte(int addr);
void DAQC_toggleDOUTbit(int addr, int bit);

void DAQC_setPWM(int addr, int channel, int value);
int DAQC_getPWM(int addr, int channel);

void DAQC_setDAC(int addr, int channel, double value);
double DAQC_getDAC(int addr, int channel);

double DAQC_getRANGE(int addr);

void DAQC_intENABLE(int addr);
void DAQC_intDISABLE(int addr);
int DAQC_getINTflags(int addr);

void DAQC_help(void);

// ============================================================================
// DAQC2 Plate Functions
// ============================================================================

// Common Functions
int DAQC2_getADDR(int addr);
char* DAQC2_getID(int addr);
double DAQC2_getHWrev(int addr);
double DAQC2_getFWrev(int addr);
void DAQC2_RESET(int addr);

// Interrupt Functions
void DAQC2_intEnable(int addr);
void DAQC2_intDisable(int addr);
int DAQC2_getINTflags(int addr);

// Digital Output Functions
void DAQC2_setDOUTbit(int addr, int bit);
void DAQC2_clrDOUTbit(int addr, int bit);
void DAQC2_toggleDOUTbit(int addr, int bit);
void DAQC2_setDOUTall(int addr, int value);
int DAQC2_getDOUTbyte(int addr);

// Digital Input Functions
int DAQC2_getDINbit(int addr, int bit);
void DAQC2_enableDINint(int addr, int bit);
void DAQC2_disableDINint(int addr, int bit);
int DAQC2_getDINall(int addr);

// ADC Functions
double DAQC2_getADC(int addr, int channel);
int DAQC2_getADCall(int addr, double* results, int max_results);

// DAC Functions
void DAQC2_setDAC(int addr, int channel, double value);
double DAQC2_getDAC(int addr, int channel);

// LED Functions
void DAQC2_setLED(int addr, int led);
int DAQC2_getLED(int addr);

// Frequency Functions
int DAQC2_getSRQ(int addr);
double DAQC2_getFREQ(int addr);

// PWM Functions
void DAQC2_setPWM(int addr, int channel, int value);
int DAQC2_getPWM(int addr, int channel);

// Function Generator Functions
void DAQC2_fgON(int addr);
void DAQC2_fgOFF(int addr);
void DAQC2_fgFREQ(int addr, double freq);
void DAQC2_fgTYPE(int addr, int type);
void DAQC2_fgLEVEL(int addr, double level);

// SRQ Functions
void DAQC2_setSRQ(int addr, int value);
void DAQC2_clrSRQ(int addr, int value);

// Motor Control Functions
void DAQC2_motorENABLE(int addr, int motor);
void DAQC2_motorDISABLE(int addr, int motor);
void DAQC2_motorMOVE(int addr, int motor, int steps);
void DAQC2_motorJOG(int addr, int motor);
void DAQC2_motorSTOP(int addr, int motor);
void DAQC2_motorDIR(int addr, int motor, int dir);
void DAQC2_motorRATE(int addr, int motor, int rate);
void DAQC2_motorOFF(int addr, int motor);
void DAQC2_motorINTenable(int addr, int motor);
void DAQC2_motorINTdisable(int addr, int motor);

// Oscilloscope Functions
void DAQC2_startOSC(int addr);
void DAQC2_stopOSC(int addr);
void DAQC2_runOSC(int addr);
void DAQC2_setOSCchannel(int addr, int channel);
void DAQC2_setOSCsweep(int addr, int sweep);
int DAQC2_getOSCtraces(int addr, double* results, int max_results);
void DAQC2_setOSCtrigger(int addr, int trigger);
void DAQC2_trigOSCnow(int addr);

// Help Function
void DAQC2_help(void);

// ============================================================================
// DIGI Plate Functions
// ============================================================================

int DIGI_getADDR(int addr);
char* DIGI_getID(int addr);
double DIGI_getHWrev(int addr);
double DIGI_getFWrev(int addr);

void DIGI_setLED(int addr);
void DIGI_clrLED(int addr);
void DIGI_toggleLED(int addr);

// ============================================================================
// RELAY Plate Functions
// ============================================================================

int RELAY_getADDR(int addr);
char* RELAY_getID(int addr);
double RELAY_getHWrev(int addr);
double RELAY_getFWrev(int addr);

void RELAY_setLED(int addr);
void RELAY_clrLED(int addr);
void RELAY_toggleLED(int addr);

void RELAY_relayON(int addr, int relay);
void RELAY_relayOFF(int addr, int relay);
void RELAY_relayTOGGLE(int addr, int relay);
void RELAY_relayALL(int addr, int value);
int RELAY_relaySTATE(int addr);

// ============================================================================
// RELAY2 Plate Functions
// ============================================================================

int RELAY2_getADDR(int addr);
char* RELAY2_getID(int addr);
double RELAY2_getHWrev(int addr);
double RELAY2_getFWrev(int addr);

void RELAY2_setLED(int addr);
void RELAY2_clrLED(int addr);
void RELAY2_toggleLED(int addr);

void RELAY2_relayON(int addr, int relay);
void RELAY2_relayOFF(int addr, int relay);
void RELAY2_relayTOGGLE(int addr, int relay);
void RELAY2_relayALL(int addr, int value);
int RELAY2_relaySTATE(int addr);

// ============================================================================
// THERMO Plate Functions
// ============================================================================

int THERMO_getADDR(int addr);
char* THERMO_getID(int addr);
double THERMO_getHWrev(int addr);
double THERMO_getFWrev(int addr);

void THERMO_setLED(int addr);
void THERMO_clrLED(int addr);
void THERMO_toggleLED(int addr);

double THERMO_getTEMP(int addr, int channel);
double THERMO_getTEMP_scale(int addr, int channel, char scale);
double THERMO_getCOLD(int addr);
double THERMO_getCOLD_scale(int addr, char scale);

#endif // BRIDGEPLATE_H
