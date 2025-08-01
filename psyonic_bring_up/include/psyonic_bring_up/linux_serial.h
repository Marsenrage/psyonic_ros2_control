#ifndef LINUX_SERIAL_H
#define LINUX_SERIAL_H

#include <stdint.h>

// Function to connect to a specific serial device
int connect_serial(const char* device_path, const uint32_t &BAUD_RATE);

// Original autoconnect function
int autoconnect_serial(const uint32_t &BAUD_RATE);

// Serial communication functions
int serial_write(uint8_t *data, uint16_t &size);
int read_serial(uint8_t *readbuf, uint16_t &bufsize);
void close_serial(void);

#endif // LINUX_SERIAL_H