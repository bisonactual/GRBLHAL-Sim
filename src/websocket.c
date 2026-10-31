/*
  websocket.c - Minimal RFC 6455 WebSocket server and grblHAL WebSocket stream

  Part of grblHAL FlexiHAL Simulator

  Hand-rolled WebSocket implementation using only BSD sockets.
  Supports a single client connection at a time (sufficient for sender use).
  Handles text frames, ping/pong, and close frames.

  The client is a grblHAL stream of its own (StreamType_WebSocket), connected
  with stream_connect() when the handshake completes and dropped with
  stream_disconnect() when the client goes, as the networking plugin does on
  the real board. The socket is serviced from the simulator's main thread; the
  stream functions run on the grblHAL thread; the two meet in ring buffers.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// grblHAL's headers have to come first. vfs.h has a field called st_mtime, and
// <sys/stat.h>, which the socket headers pull in, defines that as a macro.
#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "shared.h"

#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET socket_t;
#define SOCK_INVALID INVALID_SOCKET
#define sock_close closesocket
#define sock_errno WSAGetLastError()
#define SOCK_WOULDBLOCK WSAEWOULDBLOCK
#else
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <errno.h>
typedef int socket_t;
#define SOCK_INVALID (-1)
#define sock_close close
#define sock_errno errno
#define SOCK_WOULDBLOCK EWOULDBLOCK
#endif

#include "websocket.h"

// --- SHA-1 (minimal, for WebSocket handshake only) ---

static void sha1(const uint8_t *msg, size_t len, uint8_t digest[20])
{
    uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE,
             h3 = 0x10325476, h4 = 0xC3D2E1F0;

    size_t new_len = len + 1;
    while (new_len % 64 != 56) new_len++;

    uint8_t *buf = calloc(new_len + 8, 1);
    memcpy(buf, msg, len);
    buf[len] = 0x80;

    uint64_t bits = (uint64_t)len * 8;
    for (int i = 0; i < 8; i++)
        buf[new_len + i] = (uint8_t)(bits >> (56 - 8 * i));

    for (size_t offset = 0; offset < new_len + 8; offset += 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; i++)
            w[i] = (buf[offset + 4*i] << 24) | (buf[offset + 4*i+1] << 16) |
                    (buf[offset + 4*i+2] << 8) | buf[offset + 4*i+3];
        for (int i = 16; i < 80; i++) {
            uint32_t t = w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16];
            w[i] = (t << 1) | (t >> 31);
        }

        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int i = 0; i < 80; i++) {
            uint32_t f, k;
            if (i < 20)      { f = (b & c) | (~b & d); k = 0x5A827999; }
            else if (i < 40) { f = b ^ c ^ d;           k = 0x6ED9EBA1; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
            else              { f = b ^ c ^ d;           k = 0xCA62C1D6; }

            uint32_t temp = ((a << 5) | (a >> 27)) + f + e + k + w[i];
            e = d; d = c; c = (b << 30) | (b >> 2); b = a; a = temp;
        }
        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }
    free(buf);

    uint32_t h[] = {h0, h1, h2, h3, h4};
    for (int i = 0; i < 5; i++) {
        digest[4*i]   = (h[i] >> 24) & 0xFF;
        digest[4*i+1] = (h[i] >> 16) & 0xFF;
        digest[4*i+2] = (h[i] >> 8) & 0xFF;
        digest[4*i+3] = h[i] & 0xFF;
    }
}

// --- Base64 encode ---

static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void base64_encode(const uint8_t *in, size_t len, char *out)
{
    size_t i, j = 0;
    for (i = 0; i + 2 < len; i += 3) {
        out[j++] = b64[in[i] >> 2];
        out[j++] = b64[((in[i] & 3) << 4) | (in[i+1] >> 4)];
        out[j++] = b64[((in[i+1] & 0xF) << 2) | (in[i+2] >> 6)];
        out[j++] = b64[in[i+2] & 0x3F];
    }
    if (i < len) {
        out[j++] = b64[in[i] >> 2];
        if (i + 1 < len) {
            out[j++] = b64[((in[i] & 3) << 4) | (in[i+1] >> 4)];
            out[j++] = b64[(in[i+1] & 0xF) << 2];
        } else {
            out[j++] = b64[(in[i] & 3) << 4];
            out[j++] = '=';
        }
        out[j++] = '=';
    }
    out[j] = '\0';
}

// --- WebSocket state (main thread) ---

static socket_t listen_fd = SOCK_INVALID;
static socket_t client_fd = SOCK_INVALID;
static bool ws_tx_muted = false;

// Client state shared with the grblHAL thread. client_gen counts handshakes,
// so a reconnect between two polls still shows up as a disconnect.
static volatile bool client_up = false;
static volatile uint32_t client_gen = 0;

// Transmit line buffer
#define WS_TX_BUF_SIZE 512
static uint8_t tx_buf[WS_TX_BUF_SIZE];
static uint16_t tx_len = 0;

// Raw socket read buffer for accumulating partial frames
#define RAW_BUF_SIZE 4096
static uint8_t raw_buf[RAW_BUF_SIZE];
static int raw_len = 0;

// --- Stream buffers (main thread <-> grblHAL thread) ---

#define WS_TX_RING_SIZE 8192 // must be a power of 2

static stream_rx_buffer_t rxbuffer = {0};
static uint8_t tx_ring[WS_TX_RING_SIZE];
static volatile uint_fast16_t tx_head = 0, tx_tail = 0;
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static void set_nonblocking(socket_t fd)
{
#ifdef WIN32
    u_long mode = 1;
    ioctlsocket(fd, FIONBIO, &mode);
#else
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
}

// Real-time commands go straight to grblHAL and everything else into the stream's
// receive buffer, the way a UART interrupt would handle it.
static void rx_push(uint8_t c)
{
    if (enqueue_realtime_command(c))
        return;

    uint_fast16_t next = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);
    if (next == shared_load(rxbuffer.tail))
        rxbuffer.overflow = 1;
    else {
        rxbuffer.data[rxbuffer.head] = c;
        shared_store(rxbuffer.head, next);
    }
}

static void client_closed(const char *reason)
{
    sock_close(client_fd);
    client_fd = SOCK_INVALID;
    shared_store(client_up, false);
    raw_len = 0;
    tx_len = 0;
    shared_store(tx_tail, shared_load(tx_head));
    printf("[WS] %s\n", reason);
}

// --- Blocking recv helper (with timeout) for handshake only ---

static int recv_blocking(socket_t fd, char *buf, int len, int timeout_ms)
{
    // Use select to wait for data with a timeout
    fd_set fds;
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    int ret = select((int)fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0) return ret;

    return recv(fd, buf, len, 0);
}

// --- WebSocket handshake (done BEFORE setting non-blocking) ---

static const char *WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

static bool do_handshake(socket_t fd)
{
    // Read the HTTP upgrade request — socket is still blocking at this point
    // Read byte-by-byte until we see \r\n\r\n
    char buf[2048] = {0};
    int hlen = 0;

    while (hlen < (int)sizeof(buf) - 1) {
        int n = recv_blocking(fd, buf + hlen, 1, 3000); // 3 second timeout
        if (n <= 0) {
            printf("[WS] Handshake: recv failed (got %d bytes so far)\n", hlen);
            return false;
        }
        hlen++;
        if (hlen >= 4 && memcmp(buf + hlen - 4, "\r\n\r\n", 4) == 0)
            break;
    }

    if (hlen < 10) {
        printf("[WS] Handshake: request too short (%d bytes)\n", hlen);
        return false;
    }

    // Find Sec-WebSocket-Key
    char *key_start = strstr(buf, "Sec-WebSocket-Key: ");
    if (!key_start) {
        printf("[WS] Handshake: no Sec-WebSocket-Key found\n");
        return false;
    }
    key_start += 19;
    char *key_end = strstr(key_start, "\r\n");
    if (!key_end) return false;

    size_t key_len = key_end - key_start;
    char accept_input[128];
    memcpy(accept_input, key_start, key_len);
    strcpy(accept_input + key_len, WS_MAGIC);

    uint8_t sha[20];
    sha1((uint8_t *)accept_input, strlen(accept_input), sha);

    char accept_b64[64];
    base64_encode(sha, 20, accept_b64);

    char response[512];
    snprintf(response, sizeof(response),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n"
        "\r\n", accept_b64);

    send(fd, response, (int)strlen(response), 0);
    printf("[WS] Handshake completed successfully\n");
    return true;
}

// --- WebSocket frame parsing (non-blocking, handles partial reads) ---

static void process_raw_buffer(void)
{
    // Try to parse complete WebSocket frames from raw_buf
    while (raw_len >= 2) {
        uint8_t *p = raw_buf;
        int needed = 2;

        uint8_t opcode = p[0] & 0x0F;
        bool masked = (p[1] & 0x80) != 0;
        uint64_t payload_len = p[1] & 0x7F;
        int header_len = 2;

        if (payload_len == 126) {
            needed += 2;
            if (raw_len < needed) return; // need more data
            payload_len = (p[2] << 8) | p[3];
            header_len = 4;
        } else if (payload_len == 127) {
            needed += 8;
            if (raw_len < needed) return;
            payload_len = 0;
            for (int i = 0; i < 8; i++)
                payload_len = (payload_len << 8) | p[2 + i];
            header_len = 10;
        }

        if (masked) needed = header_len + 4 + (int)payload_len;
        else        needed = header_len + (int)payload_len;

        if (raw_len < needed) return; // need more data

        uint8_t *mask_key = NULL;
        uint8_t *payload = NULL;

        if (masked) {
            mask_key = p + header_len;
            payload = p + header_len + 4;
        } else {
            payload = p + header_len;
        }

        // Unmask
        if (masked) {
            for (uint64_t i = 0; i < payload_len; i++)
                payload[i] ^= mask_key[i % 4];
        }

        // Handle frame
        switch (opcode) {
            case 0x01: // Text
            case 0x02: // Binary
                for (uint64_t i = 0; i < payload_len; i++)
                    rx_push(payload[i]);
                break;
            case 0x08: // Close
                client_closed("Client sent close frame");
                return;
            case 0x09: { // Ping -> Pong
                uint8_t pong_header[2] = {0x8A, 0x00};
                send(client_fd, (char *)pong_header, 2, 0);
                break;
            }
            case 0x0A: // Pong — ignore
                break;
        }

        // Consume this frame from raw_buf
        int consumed = needed;
        raw_len -= consumed;
        if (raw_len > 0)
            memmove(raw_buf, raw_buf + consumed, raw_len);
    }
}

static void process_incoming(void)
{
    // Read whatever is available into raw_buf
    int space = RAW_BUF_SIZE - raw_len;
    if (space <= 0) return;

    int n = recv(client_fd, (char *)raw_buf + raw_len, space, 0);
    if (n > 0) {
        raw_len += n;
        process_raw_buffer();
    } else if (n == 0) {
        client_closed("Client disconnected");
    } else {
        // n < 0
#ifdef WIN32
        if (sock_errno != SOCK_WOULDBLOCK)
#else
        if (errno != EAGAIN && errno != EWOULDBLOCK)
#endif
            client_closed("Client connection error");
        // EAGAIN/WOULDBLOCK is fine — just no data right now
    }
}

// --- Send a WebSocket text frame ---

static void ws_send_frame(const uint8_t *data, size_t len)
{
    if (client_fd == SOCK_INVALID) return;

    uint8_t header[10];
    size_t hlen = 0;

    header[0] = 0x81; // FIN + text opcode
    if (len < 126) {
        header[1] = (uint8_t)len;
        hlen = 2;
    } else if (len < 65536) {
        header[1] = 126;
        header[2] = (len >> 8) & 0xFF;
        header[3] = len & 0xFF;
        hlen = 4;
    } else {
        header[1] = 127;
        for (int i = 0; i < 8; i++)
            header[2 + i] = (uint8_t)(len >> (56 - 8 * i));
        hlen = 10;
    }

    send(client_fd, (char *)header, (int)hlen, 0);
    send(client_fd, (char *)data, (int)len, 0);
}

// Empties the stream's transmit ring into frames, one per line as before.
static void process_outgoing(void)
{
    while (tx_tail != shared_load(tx_head)) {
        uint8_t data = tx_ring[tx_tail];
        shared_store(tx_tail, (tx_tail + 1) & (WS_TX_RING_SIZE - 1));

        if (ws_tx_muted)
            continue;

        tx_buf[tx_len++] = data;
        if (data == '\n' || data == '\r' || tx_len >= WS_TX_BUF_SIZE - 1) {
            ws_send_frame(tx_buf, tx_len);
            tx_len = 0;
        }
    }
}

// --- grblHAL stream (grblHAL thread) ---

static int32_t wsGetC(void)
{
    int32_t data;
    uint_fast16_t bptr = rxbuffer.tail;

    if (bptr == shared_load(rxbuffer.head))
        return -1;

    data = (int32_t)rxbuffer.data[bptr++];
    shared_store(rxbuffer.tail, bptr & (RX_BUFFER_SIZE - 1));
    return data;
}

static uint16_t wsRxCount(void)
{
    uint_fast16_t head = shared_load(rxbuffer.head), tail = shared_load(rxbuffer.tail);
    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t wsRxFree(void)
{
    return (RX_BUFFER_SIZE - 1) - wsRxCount();
}

static void wsRxFlush(void)
{
    shared_store(rxbuffer.tail, shared_load(rxbuffer.head));
    rxbuffer.overflow = false;
}

static void wsRxCancel(void)
{
    wsRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    shared_store(rxbuffer.head, (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1));
}

static bool wsPutC(const uint8_t c)
{
    uint_fast16_t next_head = (tx_head + 1) & (WS_TX_RING_SIZE - 1);

    while (next_head == shared_load(tx_tail)) {
        if (!shared_load(client_up))
            return true;
        if (!hal.stream_blocking_callback())
            return false;
    }

    if (shared_load(client_up)) {
        tx_ring[tx_head] = c;
        shared_store(tx_head, next_head);
    }

    return true;
}

static void wsWriteS(const char *data)
{
    uint8_t c, *ptr = (uint8_t *)data;
    while ((c = *ptr++) != '\0')
        wsPutC(c);
}

static bool wsSuspendInput(bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

static bool wsIsConnected(void)
{
    return client_up;
}

static enqueue_realtime_command_ptr wsSetRtHandler(enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;
    if (handler)
        enqueue_realtime_command = handler;
    return prev;
}

static const io_stream_t ws_stream = {
    .type = StreamType_WebSocket,
    .is_connected = wsIsConnected,
    .read = wsGetC,
    .write = wsWriteS,
    .write_char = wsPutC,
    .write_all = wsWriteS,
    .get_rx_buffer_free = wsRxFree,
    .get_rx_buffer_count = wsRxCount,
    .reset_read_buffer = wsRxFlush,
    .cancel_read_buffer = wsRxCancel,
    .suspend_read = wsSuspendInput,
    .set_enqueue_rt_handler = wsSetRtHandler
};

void ws_stream_poll(void)
{
    static bool connected = false;
    static uint32_t connected_gen = 0;

    bool up = shared_load(client_up);
    uint32_t gen = shared_load(client_gen);

    if (connected && (!up || gen != connected_gen)) {
        connected = false;
        stream_disconnect(&ws_stream);
        wsRxFlush();
    }

    if (!connected && up) {
        connected = true;
        connected_gen = gen;
        stream_connect(&ws_stream);
    }
}

// --- Public API (main thread) ---

int ws_init(uint16_t port)
{
#ifdef WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
        return -1;
#endif

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd == SOCK_INVALID) return -1;

    int opt = 1;
#ifdef WIN32
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));
#else
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("[WS] Failed to bind port %d\n", port);
        return -1;
    }

    listen(listen_fd, 1);
    set_nonblocking(listen_fd);

    printf("[WS] Listening on ws://0.0.0.0:%d\n", port);
    return 0;
}

void ws_service(void)
{
    if (listen_fd == SOCK_INVALID)
        return;

    // Try to accept new connection
    if (client_fd == SOCK_INVALID) {
        struct sockaddr_in caddr;
        socklen_t clen = sizeof(caddr);
        // listen_fd is non-blocking, so accept returns immediately if no connection
        socket_t fd = accept(listen_fd, (struct sockaddr *)&caddr, &clen);
        if (fd != SOCK_INVALID) {
            // Do handshake while socket is STILL BLOCKING
            if (do_handshake(fd)) {
                // Now set non-blocking for normal operation
                set_nonblocking(fd);
                // Disable Nagle for low-latency responses
                int flag = 1;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag));
                client_fd = fd;
                raw_len = 0;
                tx_len = 0;
                shared_store(client_gen, client_gen + 1);
                shared_store(client_up, true);
                printf("[WS] Client connected and ready\n");
            } else {
                printf("[WS] Handshake failed, closing\n");
                sock_close(fd);
            }
        }
    }

    // Read from connected client
    if (client_fd != SOCK_INVALID)
        process_incoming();

    if (client_fd != SOCK_INVALID)
        process_outgoing();
}

bool ws_toggle_tx_muted(void)
{
    ws_tx_muted = !ws_tx_muted;
    if (ws_tx_muted)
        tx_len = 0;
    return ws_tx_muted;
}

bool ws_kick_client(void)
{
    if (client_fd == SOCK_INVALID)
        return false;

    client_closed("Client kicked");
    return true;
}

void ws_shutdown(void)
{
    if (client_fd != SOCK_INVALID)
        client_closed("Shutting down");
    if (listen_fd != SOCK_INVALID) {
        sock_close(listen_fd);
        listen_fd = SOCK_INVALID;
    }
#ifdef WIN32
    WSACleanup();
#endif
}
