/*
  serial.c - USB serial stream for simulator

  Part of grblHAL FlexiHAL Simulator

  The emulated UART plays the Flexi-HAL's USB CDC stream: it is the base
  stream, flagged as USB, and connected while the sender holds DTR (see
  usb_host.c), like the real board.

  Based on grblHAL Simulator by Terje Io
*/

#include <string.h>

#include "mcu.h"
#include "serial.h"
#include "simulator.h"
#include "shared.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;
static volatile serial_linestate_t linestate = {0};

// The "interrupt" runs on the simulator's main thread and grblHAL runs on its own,
// so the buffer indices go through shared.h.

static void uart_interrupt_handler(void);

static int32_t serialGetC(void)
{
    int32_t data;
    uint_fast16_t bptr = rxbuffer.tail;

    if (bptr == shared_load(rxbuffer.head))
        return -1;

    data = (int32_t)rxbuffer.data[bptr++];
    shared_store(rxbuffer.tail, bptr & (RX_BUFFER_SIZE - 1));
    return data;
}

static inline uint16_t serialRxCount(void)
{
    uint_fast16_t head = shared_load(rxbuffer.head), tail = shared_load(rxbuffer.tail);
    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t serialRxFree(void)
{
    return (RX_BUFFER_SIZE - 1) - serialRxCount();
}

static bool serialIsConnected(void)
{
    return linestate.dtr;
}

// Called on the main thread, which is interrupt context as far as grblHAL is
// concerned, like the USB stack's line state callback on the real board.
void serial_set_dtr(bool on)
{
    serial_linestate_t state = { .value = linestate.value };

    state.dtr = on;
    linestate.value = state.value;

    if (hal.stream.state.is_usb && hal.stream.on_linestate_changed)
        hal.stream.on_linestate_changed(state);
}

static void serialRxFlush(void)
{
    shared_store(rxbuffer.tail, shared_load(rxbuffer.head));
    rxbuffer.overflow = false;
}

static void serialRxCancel(void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    shared_store(rxbuffer.head, (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1));
}

static bool serialPutC(const uint8_t c)
{
    uint_fast16_t next_head;
    next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);

    while (shared_load(txbuffer.tail) == next_head) {
        if (!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;
    shared_store(txbuffer.head, next_head);
    uart.tx_irq_enable = 1;
    return true;
}

static void serialWriteS(const char *data)
{
    uint8_t c, *ptr = (uint8_t *)data;
    while ((c = *ptr++) != '\0')
        serialPutC(c);
}

static bool serialSuspendInput(bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

static uint16_t serialTxCount(void)
{
    uint_fast16_t head = shared_load(txbuffer.head), tail = shared_load(txbuffer.tail);
    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static enqueue_realtime_command_ptr serialSetRtHandler(enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;
    if (handler)
        enqueue_realtime_command = handler;
    return prev;
}

const io_stream_t *serialInit(void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.is_usb = On,
        .state.linestate_event = On,
        .is_connected = serialIsConnected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    mcu_register_irq_handler(uart_interrupt_handler, UART_IRQ);
    uart.rx_irq_enable = 1;

    return &stream;
}

static void uart_interrupt_handler(void)
{
    uint_fast16_t bptr;
    int32_t data;

    if (uart.tx_irq) {
        bptr = txbuffer.tail;
        if (shared_load(txbuffer.head) != bptr) {
            uart.tx_data = txbuffer.data[bptr++];
            bptr &= (TX_BUFFER_SIZE - 1);
            uart.tx_irq = 0;
            uart.tx_flag = 1;
            shared_store(txbuffer.tail, bptr);
            if (bptr == shared_load(txbuffer.head))
                uart.tx_irq_enable = 0;
        }
    }

    if (uart.rx_irq) {
        bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);
        if (bptr == shared_load(rxbuffer.tail)) {
            rxbuffer.overflow = 1;
            uart.rx_irq = 0;
        } else {
            data = uart.rx_data;
            uart.rx_irq = 0;
            if (data == 0x06)
                sim.exit = exit_REQ;
            if (!enqueue_realtime_command((uint8_t)data)) {
                rxbuffer.data[rxbuffer.head] = (uint8_t)data;
                shared_store(rxbuffer.head, bptr);
            }
        }
    }
}
