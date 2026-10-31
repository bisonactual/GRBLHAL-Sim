/*
  usb_host.h - host side of the simulated USB CDC port

  Part of grblHAL FlexiHAL Simulator

  The real Flexi-HAL shows up as a USB serial port. The simulator gives the
  sender a serial device on the host instead, a pseudo-terminal on Linux and
  macOS (through a symlink that stays put) or an existing COM port on Windows,
  usually one end of a com0com pair. The sender having the port open stands in
  for USB DTR.
*/

#ifndef _USB_HOST_H_
#define _USB_HOST_H_

#include <stdint.h>
#include <stdbool.h>

// Opens the host port. On Linux and macOS path is the symlink to create for the
// pseudo-terminal, on Windows it is the COM port to open ("COM7", say).
bool usb_host_init(const char *path);

// Called from the main thread once per frame. Notices the sender opening or closing
// the port, calls serial_set_dtr() when it does, and sends any pending output.
void usb_host_service(void);

// The next byte from the sender, or 0 if there isn't one. Doesn't block, same as ws_getchar().
uint8_t usb_host_getchar(void);

// Queues a byte for the sender. It is dropped if nobody has the port open.
void usb_host_putchar(uint8_t data);

void usb_host_shutdown(void);

#endif
