/*
 * BRIDGEplate.c - C implementation of BRIDGEplate interface
 * 
 * This module provides functions to communicate with PI-Plates BRIDGEplate
 * via USB CDC serial interface. It supports multiple plate types:
 * ADC, BRIDGE, CURRENT, DAQC, DAQC2, DIGI, RELAY, RELAY2, and THERMO
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <locale.h>
#include "BRIDGEplate.h"

#ifdef _WIN32
    #include <windows.h>
    #include <setupapi.h>
    #include <initguid.h>
    #include <devguid.h>
    #pragma comment(lib, "setupapi.lib")
    
    typedef HANDLE SerialPort;
    #define INVALID_PORT INVALID_HANDLE_VALUE
#else
    #include <fcntl.h>
    #include <unistd.h>
    #include <termios.h>
    #include <sys/ioctl.h>
    #include <dirent.h>
    #include <limits.h>
    
    typedef int SerialPort;
    #define INVALID_PORT -1
#endif

// Configuration
#define TARGET_VID "2E8A"
#define TARGET_PID "10E3"
#define BAUD_RATE 115200
#define TIMEOUT_MS 20000
#define MAX_RESPONSE 4096
#define MAX_CMD_LEN 256

// Global serial port handle
static SerialPort g_serial_port = INVALID_PORT;

// ============================================================================
// Platform-specific serial port functions
// ============================================================================

#ifdef _WIN32

// Find COM port by VID/PID on Windows
char* find_port_by_vid_pid(const char* vid, const char* pid) {
    HDEVINFO device_info_set;
    SP_DEVINFO_DATA device_info_data;
    DWORD i;
    static char port_name[32];
    
    // Create a device information set for all COM ports
    device_info_set = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, NULL, NULL, 
                                          DIGCF_PRESENT);
    if (device_info_set == INVALID_HANDLE_VALUE) {
        return NULL;
    }
    
    device_info_data.cbSize = sizeof(SP_DEVINFO_DATA);
    
    // Enumerate through all devices
    for (i = 0; SetupDiEnumDeviceInfo(device_info_set, i, &device_info_data); i++) {
        char hardware_id[256];
        DWORD data_type, buffer_size;
        
        // Get hardware ID
        buffer_size = sizeof(hardware_id);
        if (SetupDiGetDeviceRegistryPropertyA(device_info_set, &device_info_data,
                                             SPDRP_HARDWAREID, &data_type,
                                             (PBYTE)hardware_id, buffer_size,
                                             &buffer_size)) {
            // Check if VID and PID match
            if (strstr(hardware_id, vid) && strstr(hardware_id, pid)) {
                // Get the COM port name
                HKEY key;
                if (SetupDiOpenDevRegKey(device_info_set, &device_info_data,
                                        DICS_FLAG_GLOBAL, 0, DIREG_DEV,
                                        KEY_READ) != INVALID_HANDLE_VALUE) {
                    key = SetupDiOpenDevRegKey(device_info_set, &device_info_data,
                                               DICS_FLAG_GLOBAL, 0, DIREG_DEV,
                                               KEY_READ);
                    buffer_size = sizeof(port_name);
                    if (RegQueryValueExA(key, "PortName", NULL, &data_type,
                                        (LPBYTE)port_name, &buffer_size) == ERROR_SUCCESS) {
                        RegCloseKey(key);
                        SetupDiDestroyDeviceInfoList(device_info_set);
                        return port_name;
                    }
                    RegCloseKey(key);
                }
            }
        }
    }
    
    SetupDiDestroyDeviceInfoList(device_info_set);
    return NULL;
}

SerialPort open_serial_port(const char* port_name, int baud_rate) {
    char full_name[32];
    HANDLE handle;
    DCB dcb = {0};
    COMMTIMEOUTS timeouts = {0};
    
    // Add \\.\ prefix for COM ports >= 10
    snprintf(full_name, sizeof(full_name), "\\\\.\\%s", port_name);
    
    handle = CreateFileA(full_name, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                        OPEN_EXISTING, 0, NULL);
    if (handle == INVALID_HANDLE_VALUE) {
        return INVALID_PORT;
    }
    
    // Flush any existing data
    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
    
    // Configure serial port
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(handle, &dcb)) {
        CloseHandle(handle);
        return INVALID_PORT;
    }
    
    // Set basic parameters
    dcb.BaudRate = baud_rate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    
    // Disable flow control
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    
    // Other settings
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fNull = FALSE;
    dcb.fAbortOnError = FALSE;
    
    if (!SetCommState(handle, &dcb)) {
        CloseHandle(handle);
        return INVALID_PORT;
    }
    
    // Set timeouts
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = TIMEOUT_MS;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 500;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    
    if (!SetCommTimeouts(handle, &timeouts)) {
        CloseHandle(handle);
        return INVALID_PORT;
    }
    
    // Give the port a moment to stabilize
    Sleep(100);
    
    // Clear buffers again
    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    
    return handle;
}

void close_serial_port(SerialPort port) {
    if (port != INVALID_PORT) {
        CloseHandle(port);
    }
}

int write_serial(SerialPort port, const char* data, int len) {
    DWORD written;
    if (!WriteFile(port, data, len, &written, NULL)) {
        return -1;
    }
    return (int)written;
}

int read_serial(SerialPort port, char* buffer, int max_len) {
    DWORD read;
    if (!ReadFile(port, buffer, max_len, &read, NULL)) {
        return -1;
    }
    return (int)read;
}

#else  // Linux/POSIX

// Read a single line from a sysfs attribute file, strip trailing newline.
// Returns 1 on success, 0 if the file could not be opened or read.
static int read_sysfs_attr(const char* path, char* buf, size_t buflen) {
    FILE *fp = fopen(path, "r");
    if (!fp) return 0;
    if (!fgets(buf, (int)buflen, fp)) { fclose(fp); return 0; }
    buf[strcspn(buf, "\n")] = '\0';
    fclose(fp);
    return 1;
}

// Find device by VID/PID on Linux.
//
// Sysfs layout varies by kernel and driver.  The entry
//   /sys/class/tty/ttyACM0/device
// is a symlink whose target depth relative to the USB device node differs
// across systems.  The idVendor/idProduct files live in the USB device node.
// Rather than guessing the depth with hardcoded "../..", we resolve the
// symlink with realpath() and walk up until we find idVendor (or leave /sys).
char* find_port_by_vid_pid(const char* vid, const char* pid) {
    static char port_name[512];
    DIR *dir;
    struct dirent *entry;
    char device_link[512];
    char resolved[PATH_MAX];
    char candidate[PATH_MAX];
    char vid_buf[16], pid_buf[16];
    char vid_path[PATH_MAX + 16], pid_path[PATH_MAX + 16];
    char* slash;

    dir = opendir("/sys/class/tty");
    if (!dir) return NULL;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_name[0] == '.') continue;

        snprintf(device_link, sizeof(device_link),
                 "/sys/class/tty/%s/device", entry->d_name);

        if (!realpath(device_link, resolved)) continue;

        // Walk up from the resolved path looking for idVendor
        strncpy(candidate, resolved, PATH_MAX - 1);
        candidate[PATH_MAX - 1] = '\0';

        while (strncmp(candidate, "/sys", 4) == 0) {
            snprintf(vid_path, sizeof(vid_path), "%s/idVendor", candidate);
            if (!read_sysfs_attr(vid_path, vid_buf, sizeof(vid_buf))) {
                // No idVendor at this level — move up
                slash = strrchr(candidate, '/');
                if (!slash || slash == candidate) break;
                *slash = '\0';
                continue;
            }

            // Found idVendor; check it
            if (strcasecmp(vid_buf, vid) != 0) break;  // wrong VID, stop

            snprintf(pid_path, sizeof(pid_path), "%s/idProduct", candidate);
            if (!read_sysfs_attr(pid_path, pid_buf, sizeof(pid_buf))) break;

            if (strcasecmp(pid_buf, pid) == 0) {
                snprintf(port_name, sizeof(port_name), "/dev/%s", entry->d_name);
                closedir(dir);
                return port_name;
            }
            break;  // VID matched but PID didn't, stop climbing
        }
    }

    closedir(dir);
    return NULL;
}

SerialPort open_serial_port(const char* port_name, int baud_rate) {
    int fd;
    struct termios tty;
    
    fd = open(port_name, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        return INVALID_PORT;
    }
    
    // Flush any stale data left over from a previous session or reset banner
    tcflush(fd, TCIOFLUSH);
    
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return INVALID_PORT;
    }
    
    // Set baud rate
    speed_t speed;
    switch (baud_rate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        default:     speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 8N1 mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    tty.c_iflag &= ~IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 20;   // 2 second initial timeout (units of 0.1 s)
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return INVALID_PORT;
    }
    
    // Assert DTR and RTS.  The RP2040 USB CDC firmware watches DTR to detect
    // a connected host; without it the firmware may never enter its command
    // loop.  This matches the DTR_CONTROL_ENABLE / RTS_CONTROL_ENABLE that
    // the Windows path sets in the DCB.
    ioctl(fd, TIOCMBIS, &(int){TIOCM_DTR});
    ioctl(fd, TIOCMBIS, &(int){TIOCM_RTS});
    
    // Give the device time to finish booting.  open() on a USB CDC port
    // toggles DTR, which can reset the RP2040.  The firmware needs a moment
    // to come back up before we send commands.
    usleep(100000);   // 100 ms
    
    // Flush again — the device may have printed a reset/startup banner
    // during that 100 ms window.
    tcflush(fd, TCIOFLUSH);
    
    return fd;
}

void close_serial_port(SerialPort port) {
    if (port != INVALID_PORT) {
        close(port);
    }
}

int write_serial(SerialPort port, const char* data, int len) {
    return write(port, data, len);
}

int read_serial(SerialPort port, char* buffer, int max_len) {
    return read(port, buffer, max_len);
}

#endif

// ============================================================================
// Core communication functions
// ============================================================================

// Send command and receive response.
// Reads until newline or timeout.
char* CMD(const char* cmd) {
    static char response[MAX_RESPONSE];
    char cmd_buf[MAX_CMD_LEN];
    int len, total_read = 0;
    int bytes_read;
    char c;
    int debug = getenv("BRIDGEPLATE_DEBUG") != NULL;
    
    if (g_serial_port == INVALID_PORT) {
        response[0] = '\0';
        return response;
    }
    
    // Add newline to command
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\n", cmd);
    len = strlen(cmd_buf);
    
    if (debug) {
        fprintf(stderr, "[DEBUG] Sending command: %s", cmd_buf);
    }
    
    // Send command
    if (write_serial(g_serial_port, cmd_buf, len) != len) {
        if (debug) fprintf(stderr, "[DEBUG] Write failed\n");
        response[0] = '\0';
        return response;
    }
    
    // Read response until newline
    if (debug) fprintf(stderr, "[DEBUG] Reading response...\n");
    while (total_read < MAX_RESPONSE - 1) {
        bytes_read = read_serial(g_serial_port, &c, 1);
        if (bytes_read <= 0) break;
        
        if (c == '\n') break;
        if (c != '\r') {  // Skip CR
            response[total_read++] = c;
        }
    }
    
    response[total_read] = '\0';
    
    if (debug) {
        fprintf(stderr, "[DEBUG] Response: '%s' (length=%d)\n", response, total_read);
    }
    
    return response;
}

// Display block output (for help functions)
void dispBlock(const char* cmd) {
    char cmd_buf[MAX_CMD_LEN];
    char buffer[1024];
    int bytes_read;
    char *end_marker;
    
    if (g_serial_port == INVALID_PORT) {
        return;
    }
    
    snprintf(cmd_buf, sizeof(cmd_buf), "%s()\n", cmd);
    write_serial(g_serial_port, cmd_buf, strlen(cmd_buf));
    
    // Read until <<<END>>> marker
    while (1) {
        bytes_read = read_serial(g_serial_port, buffer, sizeof(buffer) - 1);
        if (bytes_read <= 0) break;
        
        buffer[bytes_read] = '\0';
        
        // Check for end marker
        end_marker = strstr(buffer, "<<<END>>>");
        if (end_marker) {
            *end_marker = '\0';
            printf("%s\n", buffer);
            break;
        }
        
        printf("%s", buffer);
        fflush(stdout);
    }
}

// Parse response - convert to number if possible
double convert_to_number(const char* str) {
    char *endptr;
    double value;
    
    // Force C locale for strtod to ensure '.' is always the decimal separator.
    // Without this, on Windows/Linux systems with non-US locales (German, French,
    // etc. which use ',' as decimal separator), strtod("2.483") would stop at
    // the '.' and fail to parse the full number.
    char *old_locale = setlocale(LC_NUMERIC, NULL);
    setlocale(LC_NUMERIC, "C");
    
    // Try to convert to number
    value = strtod(str, &endptr);
    
    // Restore original locale
    setlocale(LC_NUMERIC, old_locale);
    
    if (endptr != str && *endptr == '\0') {
        return value;
    }
    
    // If conversion fails, return 0 (caller should check the string directly)
    return 0.0;
}

// Parse command with arguments and return numeric response
double parseIt(const char* class_method, int argc, ...) {
    char cmd[MAX_CMD_LEN];
    char args[32] = "";
    char temp[16];
    va_list ap;
    int i;
    char* response;
    
    // Build argument string
    va_start(ap, argc);
    for (i = 0; i < argc; i++) {
        int arg = va_arg(ap, int);
        snprintf(temp, sizeof(temp), "%s%d", (i > 0 ? ", " : ""), arg);
        strncat(args, temp, sizeof(args) - strlen(args) - 1);
    }
    va_end(ap);
    
    // Build command string
    snprintf(cmd, sizeof(cmd), "%s(%s)", class_method, args);
    
    // Send command and get response
    response = CMD(cmd);
    
    return convert_to_number(response);
}

// Parse command and return string response
char* parseItStr(const char* class_method, int argc, ...) {
    char cmd[MAX_CMD_LEN];
    char args[32] = "";
    char temp[16];
    va_list ap;
    int i;
    
    // Build argument string
    va_start(ap, argc);
    for (i = 0; i < argc; i++) {
        int arg = va_arg(ap, int);
        snprintf(temp, sizeof(temp), "%s%d", (i > 0 ? ", " : ""), arg);
        strncat(args, temp, sizeof(args) - strlen(args) - 1);
    }
    va_end(ap);
    
    // Build command string
    snprintf(cmd, sizeof(cmd), "%s(%s)", class_method, args);
    
    // Send command and get response
    return CMD(cmd);
}

// Parse command and return array of doubles
int parseItArray(const char* class_method, double* result, int max_results, int argc, ...) {
    char cmd[MAX_CMD_LEN];
    char args[32] = "";
    char temp[16];
    va_list ap;
    int i;
    char* response;
    char* token;
    int count = 0;
    
    // Build argument string
    va_start(ap, argc);
    for (i = 0; i < argc; i++) {
        int arg = va_arg(ap, int);
        snprintf(temp, sizeof(temp), "%s%d", (i > 0 ? ", " : ""), arg);
        strncat(args, temp, sizeof(args) - strlen(args) - 1);
    }
    va_end(ap);
    
    // Build command string
    snprintf(cmd, sizeof(cmd), "%s(%s)", class_method, args);
    
    // Send command and get response
    response = CMD(cmd);
    
    // Check if response contains commas
    if (strchr(response, ',') == NULL) {
        // Single value
        if (count < max_results) {
            result[count++] = convert_to_number(response);
        }
        return count;
    }
    
    // Parse comma-separated values
    token = strtok(response, ",");
    while (token != NULL && count < max_results) {
        // Trim whitespace
        while (isspace(*token)) token++;
        result[count++] = convert_to_number(token);
        token = strtok(NULL, ",");
    }
    
    return count;
}

// ============================================================================
// ADC Plate Functions
// ============================================================================

int ADC_getADDR(int addr) { return (int)parseIt("ADC.getADDR", 1, addr); }
char* ADC_getID(int addr) { return parseItStr("ADC.getID", 1, addr); }
double ADC_getHWrev(int addr) { return parseIt("ADC.getHWrev", 1, addr); }
double ADC_getFWrev(int addr) { return parseIt("ADC.getFWrev", 1, addr); }
void ADC_setLED(int addr) { parseIt("ADC.setLED", 1, addr); }
void ADC_clrLED(int addr) { parseIt("ADC.clrLED", 1, addr); }
void ADC_toggleLED(int addr) { parseIt("ADC.toggleLED", 1, addr); }
double ADC_getADC(int addr, int channel) { return parseIt("ADC.getADC", 2, addr, channel); }
void ADC_srTable(int addr) { parseIt("ADC.srTable", 1, addr); }
void ADC_initADC(int addr) { parseIt("ADC.initADC", 1, addr); }
void ADC_enableEVENTS(int addr) { parseIt("ADC.enableEVENTS", 1, addr); }
void ADC_disableEVENTS(int addr) { parseIt("ADC.disableEVENTS", 1, addr); }
int ADC_check4EVENTS(int addr) { return (int)parseIt("ADC.check4EVENTS", 1, addr); }
int ADC_getEVENTS(int addr) { return (int)parseIt("ADC.getEVENTS", 1, addr); }

int ADC_getADCall(int addr, double* results, int max_results) {
    return parseItArray("ADC.getADCall", results, max_results, 1, addr);
}

int ADC_getSall(int addr, double* results, int max_results) {
    return parseItArray("ADC.getSall", results, max_results, 1, addr);
}

int ADC_getDall(int addr, double* results, int max_results) {
    return parseItArray("ADC.getDall", results, max_results, 1, addr);
}

int ADC_getIall(int addr, double* results, int max_results) {
    return parseItArray("ADC.getIall", results, max_results, 1, addr);
}

void ADC_setMODE(int addr, int mode) { parseIt("ADC.setMODE", 2, addr, mode); }
int ADC_getMODE(int addr) { return (int)parseIt("ADC.getMODE", 1, addr); }
// ADC_configINPUT without enable parameter (doesn't change enable status)
void ADC_configINPUT(int addr, int channel, int sample_rate) { 
    parseIt("ADC.configINPUT", 3, addr, channel, sample_rate); 
}

// ADC_configINPUT with enable parameter
void ADC_configINPUT_enable(int addr, int channel, int sample_rate, int enable) { 
    parseIt("ADC.configINPUT", 4, addr, channel, sample_rate, enable); 
}
void ADC_enableINPUT(int addr, int channel) { parseIt("ADC.enableINPUT", 2, addr, channel); }
void ADC_disableINPUT(int addr, int channel) { parseIt("ADC.disableINPUT", 2, addr, channel); }

// ADC_readSINGLE with 2 parameters (rate defaults to device setting)
double ADC_readSINGLE(int addr, int channel) { 
    return parseIt("ADC.readSINGLE", 2, addr, channel); 
}

// ADC_readSINGLE with 3 parameters (specify sample rate)
double ADC_readSINGLE_rate(int addr, int channel, int rate) { 
    return parseIt("ADC.readSINGLE", 3, addr, channel, rate); 
}

// ADC_startSINGLE with 2 parameters (rate defaults to device setting)
void ADC_startSINGLE(int addr, int channel) { 
    parseIt("ADC.startSINGLE", 2, addr, channel); 
}

// ADC_startSINGLE with 3 parameters (specify sample rate)
void ADC_startSINGLE_rate(int addr, int channel, int rate) { 
    parseIt("ADC.startSINGLE", 3, addr, channel, rate); 
}

double ADC_getSINGLE(int addr) { return parseIt("ADC.getSINGLE", 1, addr); }

// ADC_readSCAN without rate parameter (uses default)
int ADC_readSCAN(int addr, double* results, int max_results) {
    return parseItArray("ADC.readSCAN", results, max_results, 1, addr);
}

// ADC_readSCAN with rate parameter
int ADC_readSCAN_rate(int addr, double* results, int max_results, int rate) {
    char cmd[MAX_CMD_LEN];
    char args[32];
    snprintf(args, sizeof(args), "%d, %d", addr, rate);
    snprintf(cmd, sizeof(cmd), "ADC.readSCAN(%s)", args);
    
    char* response = CMD(cmd);
    int count = 0;
    char* token;
    
    if (strchr(response, ',') == NULL) {
        if (count < max_results) {
            results[count++] = convert_to_number(response);
        }
        return count;
    }
    
    token = strtok(response, ",");
    while (token != NULL && count < max_results) {
        while (isspace(*token)) token++;
        results[count++] = convert_to_number(token);
        token = strtok(NULL, ",");
    }
    return count;
}

// ADC_startSCAN without rate parameter (uses default)
void ADC_startSCAN(int addr) { 
    parseIt("ADC.startSCAN", 1, addr); 
}

// ADC_startSCAN with rate parameter
void ADC_startSCAN_rate(int addr, int rate) { 
    parseIt("ADC.startSCAN", 2, addr, rate); 
}

int ADC_getSCAN(int addr, double* results, int max_results) {
    return parseItArray("ADC.getSCAN", results, max_results, 1, addr);
}

// ADC_getBLOCK without rate parameter (uses default)
int ADC_getBLOCK(int addr, double* results, int max_results) {
    return parseItArray("ADC.getBLOCK", results, max_results, 1, addr);
}

// ADC_getBLOCK with rate parameter
int ADC_getBLOCK_rate(int addr, double* results, int max_results, int rate) {
    char cmd[MAX_CMD_LEN];
    char args[32];
    snprintf(args, sizeof(args), "%d, %d", addr, rate);
    snprintf(cmd, sizeof(cmd), "ADC.getBLOCK(%s)", args);
    
    char* response = CMD(cmd);
    int count = 0;
    char* token;
    
    if (strchr(response, ',') == NULL) {
        if (count < max_results) {
            results[count++] = convert_to_number(response);
        }
        return count;
    }
    
    token = strtok(response, ",");
    while (token != NULL && count < max_results) {
        while (isspace(*token)) token++;
        results[count++] = convert_to_number(token);
        token = strtok(NULL, ",");
    }
    return count;
}

// ADC_startBLOCK without rate parameter (uses default)
void ADC_startBLOCK(int addr) { 
    parseIt("ADC.startBLOCK", 1, addr); 
}

// ADC_startBLOCK with rate parameter
void ADC_startBLOCK_rate(int addr, int rate) { 
    parseIt("ADC.startBLOCK", 2, addr, rate); 
}

// ADC_startSTREAM without rate parameter (uses default)
void ADC_startSTREAM(int addr) { 
    parseIt("ADC.startSTREAM", 1, addr); 
}

// ADC_startSTREAM with rate parameter
void ADC_startSTREAM_rate(int addr, int rate) { 
    parseIt("ADC.startSTREAM", 2, addr, rate); 
}

int ADC_getSTREAM(int addr, double* results, int max_results) {
    return parseItArray("ADC.getSTREAM", results, max_results, 1, addr);
}

void ADC_stopSTREAM(int addr) { parseIt("ADC.stopSTREAM", 1, addr); }
int ADC_getDINbit(int addr, int bit) { return (int)parseIt("ADC.getDINbit", 2, addr, bit); }
int ADC_getDINall(int addr) { return (int)parseIt("ADC.getDINall", 1, addr); }
void ADC_enableDINevent(int addr, int bit) { parseIt("ADC.enableDINevent", 2, addr, bit); }
void ADC_disableDINevent(int addr, int bit) { parseIt("ADC.disableDINevent", 2, addr, bit); }
void ADC_configTRIG(int addr, int config) { parseIt("ADC.configTRIG", 2, addr, config); }
void ADC_startTRIG(int addr) { parseIt("ADC.startTRIG", 1, addr); }
void ADC_stopTRIG(int addr) { parseIt("ADC.stopTRIG", 1, addr); }
void ADC_triggerFREQ(int addr, int freq) { parseIt("ADC.triggerFREQ", 2, addr, freq); }
void ADC_swTRIGGER(int addr) { parseIt("ADC.swTRIGGER", 1, addr); }
int ADC_maxTRIGfreq(int addr) { return (int)parseIt("ADC.maxTRIGfreq", 1, addr); }
void ADC_help(void) { dispBlock("ADC.help"); }

// ============================================================================
// BRIDGE Plate Functions
// ============================================================================

char* BRIDGE_getID(void) { return parseItStr("BRIDGE.getID", 0); }
double BRIDGE_getHWrev(void) { return parseIt("BRIDGE.getHWrev", 0); }
double BRIDGE_getFWrev(void) { return parseIt("BRIDGE.getFWrev", 0); }
void BRIDGE_resetSTACK(void) { parseIt("BRIDGE.resetSTACK", 0); }
int BRIDGE_getSRQ(void) { return (int)parseIt("BRIDGE.getSRQ", 0); }
void BRIDGE_resetBRIDGE(void) { parseIt("BRIDGE.resetBRIDGE", 0); }
void BRIDGE_help(void) { dispBlock("BRIDGE.help"); }

// ============================================================================
// CURRENT Plate Functions
// ============================================================================

int CURRENT_getADDR(int addr) { return (int)parseIt("CURRENT.getADDR", 1, addr); }
char* CURRENT_getID(int addr) { return parseItStr("CURRENT.getID", 1, addr); }
double CURRENT_getHWrev(int addr) { return parseIt("CURRENT.getHWrev", 1, addr); }
double CURRENT_getFWrev(int addr) { return parseIt("CURRENT.getFWrev", 1, addr); }
void CURRENT_setLED(int addr) { parseIt("CURRENT.setLED", 1, addr); }
void CURRENT_clrLED(int addr) { parseIt("CURRENT.clrLED", 1, addr); }
void CURRENT_toggleLED(int addr) { parseIt("CURRENT.toggleLED", 1, addr); }
double CURRENT_getI(int addr, int channel) { return parseIt("CURRENT.getI", 2, addr, channel); }

int CURRENT_getIall(int addr, double* results, int max_results) {
    return parseItArray("CURRENT.getIall", results, max_results, 1, addr);
}

void CURRENT_help(void) { dispBlock("CURRENT.help"); }

// ============================================================================
// DAQC Plate Functions
// ============================================================================

int DAQC_getADDR(int addr) { return (int)parseIt("DAQC.getADDR", 1, addr); }
char* DAQC_getID(int addr) { return parseItStr("DAQC.getID", 1, addr); }
double DAQC_getHWrev(int addr) { return parseIt("DAQC.getHWrev", 1, addr); }
double DAQC_getFWrev(int addr) { return parseIt("DAQC.getFWrev", 1, addr); }
void DAQC_setLED(int addr, int led) { parseIt("DAQC.setLED", 2, addr, led); }
void DAQC_clrLED(int addr, int led) { parseIt("DAQC.clrLED", 2, addr, led); }
void DAQC_toggleLED(int addr, int led) { parseIt("DAQC.toggleLED", 2, addr, led); }
int DAQC_getLED(int addr, int led) { return (int)parseIt("DAQC.getLED", 2, addr, led); }
double DAQC_getADC(int addr, int channel) { return parseIt("DAQC.getADC", 2, addr, channel); }

int DAQC_getADCall(int addr, double* results, int max_results) {
    return parseItArray("DAQC.getADCall", results, max_results, 1, addr);
}

int DAQC_getDINbit(int addr, int bit) { return (int)parseIt("DAQC.getDINbit", 2, addr, bit); }
int DAQC_getDINall(int addr) { return (int)parseIt("DAQC.getDINall", 1, addr); }
void DAQC_enableDINint(int addr, int bit) { parseIt("DAQC.enableDINint", 2, addr, bit); }
void DAQC_disableDINint(int addr, int bit) { parseIt("DAQC.disableDINint", 2, addr, bit); }

// DAQC_getTEMP with default scale (Celsius)
double DAQC_getTEMP(int addr) { 
    return parseIt("DAQC.getTEMP", 2, addr, (int)'c'); 
}

// DAQC_getTEMP with specified scale
double DAQC_getTEMP_scale(int addr, char scale) { 
    return parseIt("DAQC.getTEMP", 2, addr, (int)scale); 
}

void DAQC_setDOUTbit(int addr, int bit) { parseIt("DAQC.setDOUTbit", 2, addr, bit); }
void DAQC_clrDOUTbit(int addr, int bit) { parseIt("DAQC.clrDOUTbit", 2, addr, bit); }
void DAQC_setDOUTall(int addr, int value) { parseIt("DAQC.setDOUTall", 2, addr, value); }
int DAQC_getDOUTbyte(int addr) { return (int)parseIt("DAQC.getDOUTbyte", 1, addr); }
void DAQC_toggleDOUTbit(int addr, int bit) { parseIt("DAQC.toggleDOUTbit", 2, addr, bit); }
void DAQC_setPWM(int addr, int channel, int value) { parseIt("DAQC.setPWM", 3, addr, channel, value); }
int DAQC_getPWM(int addr, int channel) { return (int)parseIt("DAQC.getPWM", 2, addr, channel); }
void DAQC_setDAC(int addr, int channel, double value) { 
    char cmd[MAX_CMD_LEN];
    snprintf(cmd, sizeof(cmd), "DAQC.setDAC(%d, %d, %.2f)", addr, channel, value);
    CMD(cmd);
}
double DAQC_getDAC(int addr, int channel) { return parseIt("DAQC.getDAC", 2, addr, channel); }
double DAQC_getRANGE(int addr) { return parseIt("DAQC.getRANGE", 1, addr); }
void DAQC_intENABLE(int addr) { parseIt("DAQC.intENABLE", 1, addr); }
void DAQC_intDISABLE(int addr) { parseIt("DAQC.intDISABLE", 1, addr); }
int DAQC_getINTflags(int addr) { return (int)parseIt("DAQC.getINTflags", 1, addr); }
void DAQC_help(void) { dispBlock("DAQC.help"); }

// ============================================================================
// DAQC2 Plate Functions
// ============================================================================

// Common Functions
int DAQC2_getADDR(int addr) { return (int)parseIt("DAQC2.getADDR", 1, addr); }
char* DAQC2_getID(int addr) { return parseItStr("DAQC2.getID", 1, addr); }
double DAQC2_getHWrev(int addr) { return parseIt("DAQC2.getHWrev", 1, addr); }
double DAQC2_getFWrev(int addr) { return parseIt("DAQC2.getFWrev", 1, addr); }
void DAQC2_RESET(int addr) { parseIt("DAQC2.RESET", 1, addr); }

// Interrupt Functions
void DAQC2_intEnable(int addr) { parseIt("DAQC2.intEnable", 1, addr); }
void DAQC2_intDisable(int addr) { parseIt("DAQC2.intDisable", 1, addr); }
int DAQC2_getINTflags(int addr) { return (int)parseIt("DAQC2.getINTflags", 1, addr); }

// Digital Output Functions
void DAQC2_setDOUTbit(int addr, int bit) { parseIt("DAQC2.setDOUTbit", 2, addr, bit); }
void DAQC2_clrDOUTbit(int addr, int bit) { parseIt("DAQC2.clrDOUTbit", 2, addr, bit); }
void DAQC2_toggleDOUTbit(int addr, int bit) { parseIt("DAQC2.toggleDOUTbit", 2, addr, bit); }
void DAQC2_setDOUTall(int addr, int value) { parseIt("DAQC2.setDOUTall", 2, addr, value); }
int DAQC2_getDOUTbyte(int addr) { return (int)parseIt("DAQC2.getDOUTbyte", 1, addr); }

// Digital Input Functions
int DAQC2_getDINbit(int addr, int bit) { return (int)parseIt("DAQC2.getDINbit", 2, addr, bit); }
void DAQC2_enableDINint(int addr, int bit) { parseIt("DAQC2.enableDINint", 2, addr, bit); }
void DAQC2_disableDINint(int addr, int bit) { parseIt("DAQC2.disableDINint", 2, addr, bit); }
int DAQC2_getDINall(int addr) { return (int)parseIt("DAQC2.getDINall", 1, addr); }

// ADC Functions
double DAQC2_getADC(int addr, int channel) { return parseIt("DAQC2.getADC", 2, addr, channel); }

int DAQC2_getADCall(int addr, double* results, int max_results) {
    return parseItArray("DAQC2.getADCall", results, max_results, 1, addr);
}

// DAC Functions
void DAQC2_setDAC(int addr, int channel, double value) {
    char cmd[MAX_CMD_LEN];
    snprintf(cmd, sizeof(cmd), "DAQC2.setDAC(%d, %d, %.2f)", addr, channel, value);
    CMD(cmd);
}
double DAQC2_getDAC(int addr, int channel) { return parseIt("DAQC2.getDAC", 2, addr, channel); }

// LED Functions
void DAQC2_setLED(int addr, int led) { parseIt("DAQC2.setLED", 2, addr, led); }
int DAQC2_getLED(int addr) { return (int)parseIt("DAQC2.getLED", 1, addr); }

// Frequency Functions
int DAQC2_getSRQ(int addr) { return (int)parseIt("DAQC2.getSRQ", 1, addr); }
double DAQC2_getFREQ(int addr) { return parseIt("DAQC2.getFREQ", 1, addr); }

// PWM Functions
void DAQC2_setPWM(int addr, int channel, int value) { parseIt("DAQC2.setPWM", 3, addr, channel, value); }
int DAQC2_getPWM(int addr, int channel) { return (int)parseIt("DAQC2.getPWM", 2, addr, channel); }

// Function Generator Functions
void DAQC2_fgON(int addr) { parseIt("DAQC2.fgON", 1, addr); }
void DAQC2_fgOFF(int addr) { parseIt("DAQC2.fgOFF", 1, addr); }
void DAQC2_fgFREQ(int addr, double freq) {
    char cmd[MAX_CMD_LEN];
    snprintf(cmd, sizeof(cmd), "DAQC2.fgFREQ(%d, %.2f)", addr, freq);
    CMD(cmd);
}
void DAQC2_fgTYPE(int addr, int type) { parseIt("DAQC2.fgTYPE", 2, addr, type); }
void DAQC2_fgLEVEL(int addr, double level) {
    char cmd[MAX_CMD_LEN];
    snprintf(cmd, sizeof(cmd), "DAQC2.fgLEVEL(%d, %.2f)", addr, level);
    CMD(cmd);
}

// SRQ Functions
void DAQC2_setSRQ(int addr, int value) { parseIt("DAQC2.setSRQ", 2, addr, value); }
void DAQC2_clrSRQ(int addr, int value) { parseIt("DAQC2.clrSRQ", 2, addr, value); }

// Motor Control Functions
void DAQC2_motorENABLE(int addr, int motor) { parseIt("DAQC2.motorENABLE", 2, addr, motor); }
void DAQC2_motorDISABLE(int addr, int motor) { parseIt("DAQC2.motorDISABLE", 2, addr, motor); }
void DAQC2_motorMOVE(int addr, int motor, int steps) { parseIt("DAQC2.motorMOVE", 3, addr, motor, steps); }
void DAQC2_motorJOG(int addr, int motor) { parseIt("DAQC2.motorJOG", 2, addr, motor); }
void DAQC2_motorSTOP(int addr, int motor) { parseIt("DAQC2.motorSTOP", 2, addr, motor); }
void DAQC2_motorDIR(int addr, int motor, int dir) { parseIt("DAQC2.motorDIR", 3, addr, motor, dir); }
void DAQC2_motorRATE(int addr, int motor, int rate) { parseIt("DAQC2.motorRATE", 3, addr, motor, rate); }
void DAQC2_motorOFF(int addr, int motor) { parseIt("DAQC2.motorOFF", 2, addr, motor); }
void DAQC2_motorINTenable(int addr, int motor) { parseIt("DAQC2.motorINTenable", 2, addr, motor); }
void DAQC2_motorINTdisable(int addr, int motor) { parseIt("DAQC2.motorINTdisable", 2, addr, motor); }

// Oscilloscope Functions
void DAQC2_startOSC(int addr) { parseIt("DAQC2.startOSC", 1, addr); }
void DAQC2_stopOSC(int addr) { parseIt("DAQC2.stopOSC", 1, addr); }
void DAQC2_runOSC(int addr) { parseIt("DAQC2.runOSC", 1, addr); }
void DAQC2_setOSCchannel(int addr, int channel) { parseIt("DAQC2.setOSCchannel", 2, addr, channel); }
void DAQC2_setOSCsweep(int addr, int sweep) { parseIt("DAQC2.setOSCsweep", 2, addr, sweep); }

int DAQC2_getOSCtraces(int addr, double* results, int max_results) {
    return parseItArray("DAQC2.getOSCtraces", results, max_results, 1, addr);
}

void DAQC2_setOSCtrigger(int addr, int trigger) { parseIt("DAQC2.setOSCtrigger", 2, addr, trigger); }
void DAQC2_trigOSCnow(int addr) { parseIt("DAQC2.trigOSCnow", 1, addr); }

// Help Function
void DAQC2_help(void) { dispBlock("DAQC2.help"); }

// ============================================================================
// DIGI Plate Functions
// ============================================================================

int DIGI_getADDR(int addr) { return (int)parseIt("DIGI.getADDR", 1, addr); }
char* DIGI_getID(int addr) { return parseItStr("DIGI.getID", 1, addr); }
double DIGI_getHWrev(int addr) { return parseIt("DIGI.getHWrev", 1, addr); }
double DIGI_getFWrev(int addr) { return parseIt("DIGI.getFWrev", 1, addr); }
void DIGI_setLED(int addr) { parseIt("DIGI.setLED", 1, addr); }
void DIGI_clrLED(int addr) { parseIt("DIGI.clrLED", 1, addr); }
void DIGI_toggleLED(int addr) { parseIt("DIGI.toggleLED", 1, addr); }

// ============================================================================
// RELAY Plate Functions
// ============================================================================

int RELAY_getADDR(int addr) { return (int)parseIt("RELAY.getADDR", 1, addr); }
char* RELAY_getID(int addr) { return parseItStr("RELAY.getID", 1, addr); }
double RELAY_getHWrev(int addr) { return parseIt("RELAY.getHWrev", 1, addr); }
double RELAY_getFWrev(int addr) { return parseIt("RELAY.getFWrev", 1, addr); }
void RELAY_setLED(int addr) { parseIt("RELAY.setLED", 1, addr); }
void RELAY_clrLED(int addr) { parseIt("RELAY.clrLED", 1, addr); }
void RELAY_toggleLED(int addr) { parseIt("RELAY.toggleLED", 1, addr); }
void RELAY_relayON(int addr, int relay) { parseIt("RELAY.relayON", 2, addr, relay); }
void RELAY_relayOFF(int addr, int relay) { parseIt("RELAY.relayOFF", 2, addr, relay); }
void RELAY_relayTOGGLE(int addr, int relay) { parseIt("RELAY.relayTOGGLE", 2, addr, relay); }
void RELAY_relayALL(int addr, int value) { parseIt("RELAY.relayALL", 2, addr, value); }
int RELAY_relaySTATE(int addr) { return (int)parseIt("RELAY.relaySTATE", 1, addr); }

// ============================================================================
// RELAY2 Plate Functions
// ============================================================================

int RELAY2_getADDR(int addr) { return (int)parseIt("RELAY2.getADDR", 1, addr); }
char* RELAY2_getID(int addr) { return parseItStr("RELAY2.getID", 1, addr); }
double RELAY2_getHWrev(int addr) { return parseIt("RELAY2.getHWrev", 1, addr); }
double RELAY2_getFWrev(int addr) { return parseIt("RELAY2.getFWrev", 1, addr); }
void RELAY2_setLED(int addr) { parseIt("RELAY2.setLED", 1, addr); }
void RELAY2_clrLED(int addr) { parseIt("RELAY2.clrLED", 1, addr); }
void RELAY2_toggleLED(int addr) { parseIt("RELAY2.toggleLED", 1, addr); }
void RELAY2_relayON(int addr, int relay) { parseIt("RELAY2.relayON", 2, addr, relay); }
void RELAY2_relayOFF(int addr, int relay) { parseIt("RELAY2.relayOFF", 2, addr, relay); }
void RELAY2_relayTOGGLE(int addr, int relay) { parseIt("RELAY2.relayTOGGLE", 2, addr, relay); }
void RELAY2_relayALL(int addr, int value) { parseIt("RELAY2.relayALL", 2, addr, value); }
int RELAY2_relaySTATE(int addr) { return (int)parseIt("RELAY2.relaySTATE", 1, addr); }

// ============================================================================
// THERMO Plate Functions
// ============================================================================

int THERMO_getADDR(int addr) { return (int)parseIt("THERMO.getADDR", 1, addr); }
char* THERMO_getID(int addr) { return parseItStr("THERMO.getID", 1, addr); }
double THERMO_getHWrev(int addr) { return parseIt("THERMO.getHWrev", 1, addr); }
double THERMO_getFWrev(int addr) { return parseIt("THERMO.getFWrev", 1, addr); }
void THERMO_setLED(int addr) { parseIt("THERMO.setLED", 1, addr); }
void THERMO_clrLED(int addr) { parseIt("THERMO.clrLED", 1, addr); }
void THERMO_toggleLED(int addr) { parseIt("THERMO.toggleLED", 1, addr); }

// THERMO_getTEMP with default scale (Celsius)
double THERMO_getTEMP(int addr, int channel) { 
    return parseIt("THERMO.getTEMP", 3, addr, channel, (int)'c'); 
}

// THERMO_getTEMP with specified scale
double THERMO_getTEMP_scale(int addr, int channel, char scale) { 
    return parseIt("THERMO.getTEMP", 3, addr, channel, (int)scale); 
}

// THERMO_getCOLD with default scale (Celsius)
double THERMO_getCOLD(int addr) { 
    return parseIt("THERMO.getCOLD", 2, addr, (int)'c'); 
}

// THERMO_getCOLD with specified scale
double THERMO_getCOLD_scale(int addr, char scale) { 
    return parseIt("THERMO.getCOLD", 2, addr, (int)scale); 
}

// ============================================================================
// POLL Function - Enumerate all connected plates
// ============================================================================

void POLL(void) {
    char ADCplates[8];
    char CURRENTplates[8];
    char DAQCplates[8];
    char DAQC2plates[8];
    char DIGIplates[8];
    char RELAYplates[8];
    char RELAY2plates[8];
    char THERMOplates[8];
    int i;
    char cmd[64];
    char* resp;
    
    // Initialize arrays
    for (i = 0; i < 8; i++) {
        ADCplates[i] = '-';
        CURRENTplates[i] = '-';
        DAQCplates[i] = '-';
        DAQC2plates[i] = '-';
        DIGIplates[i] = '-';
        RELAYplates[i] = '-';
        RELAY2plates[i] = '-';
        THERMOplates[i] = '-';
    }
    
    // Poll each address for each plate type
    for (i = 0; i < 8; i++) {
        snprintf(cmd, sizeof(cmd), "ADC.getADDR(%d)", i);
        resp = CMD(cmd);
        if (resp && resp[0] == '0' + i) ADCplates[i] = '0' + i;
        
        snprintf(cmd, sizeof(cmd), "CURRENT.getADDR(%d)", i);
        resp = CMD(cmd);
        if (resp && resp[0] == '0' + i) CURRENTplates[i] = '0' + i;
        
        snprintf(cmd, sizeof(cmd), "DAQC.getADDR(%d)", i);
        resp = CMD(cmd);
        if (resp && resp[0] == '0' + i) DAQCplates[i] = '0' + i;
        
        snprintf(cmd, sizeof(cmd), "DAQC2.getADDR(%d)", i);
        resp = CMD(cmd);
        if (resp && resp[0] == '0' + i) DAQC2plates[i] = '0' + i;
        
        snprintf(cmd, sizeof(cmd), "DIGI.getADDR(%d)", i);
        resp = CMD(cmd);
        if (resp && resp[0] == '0' + i) DIGIplates[i] = '0' + i;
        
        snprintf(cmd, sizeof(cmd), "RELAY.getADDR(%d)", i);
        resp = CMD(cmd);
        if (resp && resp[0] == '0' + i) RELAYplates[i] = '0' + i;
        
        snprintf(cmd, sizeof(cmd), "RELAY2.getADDR(%d)", i);
        resp = CMD(cmd);
        if (resp && resp[0] == '0' + i) RELAY2plates[i] = '0' + i;
        
        snprintf(cmd, sizeof(cmd), "THERMO.getADDR(%d)", i);
        resp = CMD(cmd);
        if (resp && resp[0] == '0' + i) THERMOplates[i] = '0' + i;
    }
    
    // Print results
    printf("ADCplates:     ");
    for (i = 0; i < 8; i++) printf("%c", ADCplates[i]);
    printf("\n");
    
    printf("CURRENTplates: ");
    for (i = 0; i < 8; i++) printf("%c", CURRENTplates[i]);
    printf("\n");
    
    printf("DAQCplates:    ");
    for (i = 0; i < 8; i++) printf("%c", DAQCplates[i]);
    printf("\n");
    
    printf("DAQC2plates:   ");
    for (i = 0; i < 8; i++) printf("%c", DAQC2plates[i]);
    printf("\n");
    
    printf("DIGIplates:    ");
    for (i = 0; i < 8; i++) printf("%c", DIGIplates[i]);
    printf("\n");
    
    printf("RELAYplates:   ");
    for (i = 0; i < 8; i++) printf("%c", RELAYplates[i]);
    printf("\n");
    
    printf("RELAY2plates:  ");
    for (i = 0; i < 8; i++) printf("%c", RELAY2plates[i]);
    printf("\n");
    
    printf("THERMOplates:  ");
    for (i = 0; i < 8; i++) printf("%c", THERMOplates[i]);
    printf("\n");
}

// ============================================================================
// Initialization and cleanup
// ============================================================================

int BRIDGEplate_init(void) {
    char* port_name;
    
    // Find the port
    port_name = find_port_by_vid_pid(TARGET_VID, TARGET_PID);
    if (!port_name) {
        fprintf(stderr, "No COM port found with attached BRIDGEplate.\n");
        return -1;
    }
    
    // printf("BRIDGEplate found on %s\n", port_name);
    
    // Open the serial port
    g_serial_port = open_serial_port(port_name, BAUD_RATE);
    if (g_serial_port == INVALID_PORT) {
        fprintf(stderr, "Failed to open port %s\n", port_name);
        return -1;
    }
    
    return 0;
}

void BRIDGEplate_close(void) {
    if (g_serial_port != INVALID_PORT) {
        close_serial_port(g_serial_port);
        g_serial_port = INVALID_PORT;
    }
}
