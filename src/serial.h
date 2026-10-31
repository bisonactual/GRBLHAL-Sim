/*
  serial.h - USB serial stream interface for simulator

  Part of grblHAL FlexiHAL Simulator
*/

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "grbl/hal.h"

const io_stream_t *serialInit(void);

// Free space in the stream's receive buffer, whether the stream is active or not.
uint16_t serialRxFree(void);

// The sender has opened (true) or closed (false) the host port.
void serial_set_dtr(bool on);

#endif
