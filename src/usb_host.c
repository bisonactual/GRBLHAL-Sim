/*
  usb_host.c - host side of the simulated USB CDC port

  Part of grblHAL FlexiHAL Simulator

  On Linux and macOS this is a pseudo-terminal. The sender opens the slave
  side through a symlink that stays put, and the master side notices it being
  opened and closed (POLLHUP) and passes that on as DTR, like the Flexi-HAL's
  USB stream does.

  On Windows it is an existing COM port, usually one end of a com0com pair,
  and the sender's DTR shows up as our DSR.
*/

#ifndef WIN32
#define _DEFAULT_SOURCE      // cfmakeraw, symlink (built as strict C11)
#define _XOPEN_SOURCE 600    // posix_openpt, grantpt, unlockpt, ptsname
#define _DARWIN_C_SOURCE
#endif

#include <stdio.h>
#include <string.h>

#include "usb_host.h"
#include "serial.h"

static bool dtr = false;

static uint8_t rx_buf[256];
static int rx_len = 0, rx_pos = 0;

static uint8_t tx_buf[512];
static int tx_len = 0;

static void port_changed(bool open)
{
    dtr = open;
    rx_len = rx_pos = 0;
    tx_len = 0;
    printf("[USB] Sender %s the port\n", open ? "opened" : "closed");
    serial_set_dtr(open);
}

#ifdef WIN32

#include <windows.h>

static HANDLE port = INVALID_HANDLE_VALUE;

bool usb_host_init(const char *path)
{
    char name[64];
    snprintf(name, sizeof(name), "\\\\.\\%s", path);

    port = CreateFileA(name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (port == INVALID_HANDLE_VALUE) {
        printf("[USB] Could not open %s (error %lu)\n", path, GetLastError());
        return false;
    }

    DCB dcb = { .DCBlength = sizeof(DCB) };
    GetCommState(port, &dcb);
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    SetCommState(port, &dcb);

    // Reads return straight away with whatever is there, and writes give up after
    // 10 ms so a sender that stops reading can't hold up the simulator.
    COMMTIMEOUTS timeouts = { MAXDWORD, 0, 0, 0, 10 };
    SetCommTimeouts(port, &timeouts);

    printf("[USB] Serial port %s\n", path);
    return true;
}

static bool port_is_open(void)
{
    DWORD status = 0;
    return GetCommModemStatus(port, &status) && (status & MS_DSR_ON);
}

static int port_read(uint8_t *buf, int size)
{
    DWORD n = 0;
    return ReadFile(port, buf, (DWORD)size, &n, NULL) ? (int)n : 0;
}

static void port_write(const uint8_t *buf, int size)
{
    DWORD n;
    WriteFile(port, buf, (DWORD)size, &n, NULL);
}

void usb_host_shutdown(void)
{
    if (port != INVALID_HANDLE_VALUE) {
        CloseHandle(port);
        port = INVALID_HANDLE_VALUE;
    }
}

#else

#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <termios.h>

static int master_fd = -1;
static char link_path[256] = "";

bool usb_host_init(const char *path)
{
    const char *slave;

    if ((master_fd = posix_openpt(O_RDWR | O_NOCTTY)) < 0 ||
         grantpt(master_fd) || unlockpt(master_fd) || !(slave = ptsname(master_fd))) {
        printf("[USB] Could not create a pseudo-terminal\n");
        return false;
    }

    // Raw mode, so the line discipline doesn't echo or translate anything.
    int fd = open(slave, O_RDWR | O_NOCTTY);
    if (fd >= 0) {
        struct termios t;
        if (tcgetattr(fd, &t) == 0) {
            cfmakeraw(&t);
            tcsetattr(fd, TCSANOW, &t);
        }
        close(fd);
    }

    fcntl(master_fd, F_SETFL, fcntl(master_fd, F_GETFL) | O_NONBLOCK);

    strncpy(link_path, path, sizeof(link_path) - 1);
    unlink(link_path);
    if (symlink(slave, link_path) != 0) {
        printf("[USB] Could not create %s; use %s directly\n", link_path, slave);
        link_path[0] = '\0';
    } else
        printf("[USB] Serial port %s -> %s\n", link_path, slave);

    return true;
}

static bool port_is_open(void)
{
    struct pollfd p = { .fd = master_fd, .events = POLLIN };
    poll(&p, 1, 0);
    return !(p.revents & POLLHUP);
}

static int port_read(uint8_t *buf, int size)
{
    ssize_t n = read(master_fd, buf, (size_t)size);
    return n > 0 ? (int)n : 0;
}

static void port_write(const uint8_t *buf, int size)
{
    // Non-blocking, so if the sender stops reading the output just gets dropped.
    if (write(master_fd, buf, (size_t)size) < 0) {}
}

void usb_host_shutdown(void)
{
    if (master_fd >= 0) {
        close(master_fd);
        master_fd = -1;
    }
    if (*link_path)
        unlink(link_path);
}

#endif

static void flush_tx(void)
{
    if (tx_len) {
        port_write(tx_buf, tx_len);
        tx_len = 0;
    }
}

void usb_host_service(void)
{
    bool open = port_is_open();

    if (open != dtr)
        port_changed(open);

    if (dtr)
        flush_tx();
}

uint8_t usb_host_getchar(void)
{
    if (!dtr)
        return 0;

    if (rx_pos >= rx_len) {
        rx_len = port_read(rx_buf, sizeof(rx_buf));
        rx_pos = 0;
        if (rx_len == 0)
            return 0;
    }

    return rx_buf[rx_pos++];
}

void usb_host_putchar(uint8_t data)
{
    if (!dtr)
        return;

    tx_buf[tx_len++] = data;
    if (data == '\n' || tx_len == sizeof(tx_buf))
        flush_tx();
}
