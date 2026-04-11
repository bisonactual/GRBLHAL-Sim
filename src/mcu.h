/*
  mcu.h - peripherals emulator for simulator MCU

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Terje Io, Jens Geisler, Adam Shelly
*/

#ifndef _MCU_H_
#define _MCU_H_

#include <stdint.h>
#include <stdbool.h>

#define MCU_N_TIMERS 3
#define MCU_N_GPIO 12

typedef enum {
    Systick_IRQ = 0,
    UART_IRQ,
    Timer0_IRQ,
    Timer1_IRQ,
    Timer2_IRQ,
    GPIO0_IRQ,
    GPIO1_IRQ,
    GPIO2_IRQ,
    GPIO3_IRQ,
    GPIO4_IRQ,
    GPIO5_IRQ,
    GPIO6_IRQ,
    GPIO7_IRQ,
    GPIO8_IRQ,
    GPIO9_IRQ,
    GPIO10_IRQ,
    GPIO11_IRQ,
    IRQ_N_HANDLERS
} irq_num_t;

typedef struct {
    volatile bool enable;
    bool irq_enable;
    uint32_t value;
    uint32_t load;
    uint32_t prescale;
    uint32_t prescaler;
    uint32_t compare;
} mcu_timer_t;

typedef struct {
    bool rx_irq;
    bool tx_irq;
    bool tx_flag;
    bool rx_irq_enable;
    bool tx_irq_enable;
    uint8_t rx_data;
    uint8_t tx_data;
    uint32_t cdiv;
} mcu_uart_t;

typedef union {
    uint16_t value;
    uint16_t mask;
    struct {
        uint16_t pin0  :1, pin1  :1, pin2  :1, pin3  :1,
                 pin4  :1, pin5  :1, pin6  :1, pin7  :1,
                 pin8  :1, pin9  :1, pin10 :1, pin11 :1,
                 pin12 :1, pin13 :1, pin14 :1, pin15 :1;
    };
} gpio_pins_t;

typedef struct {
    gpio_pins_t dir;
    gpio_pins_t state;
    gpio_pins_t irq_mask;
    gpio_pins_t irq_state;
    gpio_pins_t rising;
    gpio_pins_t falling;
    gpio_pins_t pullup;
    gpio_pins_t pulldown;
} gpio_port_t;

extern mcu_uart_t uart;
extern mcu_timer_t timer[MCU_N_TIMERS];
extern gpio_port_t gpio[MCU_N_GPIO];
extern mcu_timer_t systick_timer;

typedef void (*interrupt_handler)(void);

void mcu_reset(void);
void mcu_enable_interrupts(void);
void mcu_disable_interrupts(void);
void mcu_master_clock(void);
void mcu_register_irq_handler(interrupt_handler handler, irq_num_t irq_num);
void mcu_gpio_set(gpio_port_t *port, uint16_t pins, uint16_t mask);
uint8_t mcu_gpio_get(gpio_port_t *port, uint16_t mask);
void mcu_gpio_in(gpio_port_t *port, uint16_t pins, uint16_t mask);
void mcu_gpio_toggle_in(gpio_port_t *port, uint16_t pins);
void simulate_serial(void);

#endif
