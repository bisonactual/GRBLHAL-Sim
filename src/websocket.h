/*
  websocket.h - Minimal WebSocket server for grblHAL simulator

  Part of grblHAL FlexiHAL Simulator

  Implements RFC 6455 WebSocket protocol over a raw TCP socket.
  No external dependencies — just BSD sockets.
*/

#ifndef _WEBSOCKET_H_
#define _WEBSOCKET_H_

#include <stdint.h>
#include <stdbool.h>

// Initialize WebSocket server on given port. Returns 0 on success.
int ws_init(uint16_t port);

// Non-blocking: accept new connections, read incoming data.
// Returns next available byte from client, or 0 if none.
uint8_t ws_getchar(void);

// Send a byte to the connected WebSocket client.
// Buffers internally and flushes on newline.
void ws_putchar(uint8_t data);

// Drop outbound grblHAL data while keeping the client socket connected.
// Returns the new mute state.
bool ws_toggle_tx_muted(void);

// Close the active client connection, if present. Returns true if a client was
// connected and kicked.
bool ws_kick_client(void);

// Cleanup
void ws_shutdown(void);

#endif
