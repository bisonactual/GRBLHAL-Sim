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

// Called from the main thread once per frame. Accepts a client, reads its frames into
// the stream's receive buffer and sends the queued output, a frame per line.
void ws_service(void);

// Called on the grblHAL thread, from on_execute_realtime. Connects or disconnects the
// WebSocket stream to follow the client, like the networking plugin does.
void ws_stream_poll(void);

// Drop outbound grblHAL data while keeping the client socket connected.
// Returns the new mute state.
bool ws_toggle_tx_muted(void);

// Close the active client connection, if present. Returns true if a client was
// connected and kicked.
bool ws_kick_client(void);

// Cleanup
void ws_shutdown(void);

#endif
