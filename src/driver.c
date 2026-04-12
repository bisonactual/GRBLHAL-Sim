/*
  driver.c - FlexiHAL simulator HAL driver

  Part of grblHAL FlexiHAL Simulator

  Based on grblHAL Simulator by Terje Io
*/

#include <string.h>

#include "mcu.h"
#include "driver.h"
#include "serial.h"
#include "eeprom.h"
#include "grbl_eeprom_extensions.h"
#include "platform.h"
#include "fs_sim.h"

#include "grbl/hal.h"

#ifndef SQUARING_ENABLED
#define SQUARING_ENABLED 0
#endif

static spindle_id_t spindle_id;
static bool probe_invert;
static uint32_t ticks = 0;
static delay_t delay = { .ms = 1, .callback = NULL };
static on_execute_realtime_ptr on_execute_realtime;

void SysTick_Handler(void);
void Stepper_IRQHandler(void);
void Limits0_IRQHandler(void);
void Control_IRQHandler(void);

#if SQUARING_ENABLED
static axes_signals_t motors_0 = {AXES_BITMASK}, motors_1 = {AXES_BITMASK};
void Limits1_IRQHandler(void);
#endif

static void driver_delay_ms(uint32_t ms, void (*callback)(void))
{
    if ((delay.ms = ms) > 0) {
        systick_timer.enable = 1;
        if (!(delay.callback = callback))
            while (delay.ms);
    } else if (callback)
        callback();
}

inline static void set_step_outputs(axes_signals_t step_out)
{
    step_out.bits = step_out.bits ^ settings.steppers.step_invert.bits;
    mcu_gpio_set(&gpio[STEP_PORT0], step_out.bits, AXES_BITMASK);
}

inline static void set_dir_outputs(axes_signals_t dir_out)
{
    mcu_gpio_set(&gpio[DIR_PORT], dir_out.value ^ settings.steppers.dir_invert.mask, AXES_BITMASK);
}

static void stepperEnable(axes_signals_t enable, bool hold)
{
    mcu_gpio_set(&gpio[STEPPER_ENABLE_PORT], enable.value ^ settings.steppers.enable_invert.mask, AXES_BITMASK);
}

static void stepperWakeUp(void)
{
    timer[STEPPER_TIMER].load = 5000;
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].enable = 1;
}

static void stepperGoIdle(bool clear_signals)
{
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].load = 0;
    timer[STEPPER_TIMER].enable = 0;

    if (clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

static void stepperCyclesPerTick(uint32_t cycles_per_tick)
{
    timer[STEPPER_TIMER].load = cycles_per_tick;
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].enable = 1;
}

static void stepperPulseStart(stepper_t *stepper)
{
    if (stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }
    if (stepper->step_out.bits)
        set_step_outputs(stepper->step_out);
}

static limit_signals_t limitsGetState(void)
{
    limit_signals_t signals = {0};
    signals.min.value = gpio[LIMITS_PORT0].state.value;
    if (settings.limits.invert.mask)
        signals.min.mask ^= settings.limits.invert.mask;
    return signals;
}

static void limitsEnable(bool on, axes_signals_t homing_cycle)
{
    gpio[LIMITS_PORT0].irq_mask.mask = on ? AXES_BITMASK : 0;
    gpio[LIMITS_PORT0].irq_state.mask = 0;
}

static control_signals_t systemGetState(void)
{
    control_signals_t signals;
    signals.mask = gpio[CONTROL_PORT].state.value;
    signals.limits_override = settings.control_invert.limits_override;
    if (settings.control_invert.mask)
        signals.mask ^= settings.control_invert.mask;
    return signals;
}

static void probeConfigureInvertMask(bool is_probe_away, bool probing)
{
    probe_invert = settings.probe.invert_probe_pin;
    if (is_probe_away)
        probe_invert ^= is_probe_away;
}

probe_state_t probeGetState(void)
{
    probe_state_t state = {0};
    state.value = mcu_gpio_get(&gpio[PROBE_PORT], PROBE_MASK);
    state.triggered ^= probe_invert;
    return state;
}

static void spindleSetState(spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    mcu_gpio_set(&gpio[SPINDLE_PORT], state.value ^ settings.pwm_spindle.invert.mask, SPINDLE_MASK);
}

static void spindle_set_speed(spindle_ptrs_t *spindle, uint_fast16_t pwm_value) {}

static uint_fast16_t spindleGetPWM(spindle_ptrs_t *spindle, float rpm) { return 0; }

static spindle_state_t spindleGetState(spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};
    state.value = gpio[SPINDLE_PORT].state.value ^ settings.pwm_spindle.invert.mask;
    return state;
}

static bool spindleConfig(spindle_ptrs_t *spindle)
{
    if (spindle == NULL) return false;
    static spindle_pwm_t spindle_pwm;
    spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, 1000000);
    return true;
}

static void coolantSetState(coolant_state_t mode)
{
    mcu_gpio_set(&gpio[COOLANT_PORT], mode.value ^ settings.coolant.invert.mask, COOLANT_MASK);
}

static coolant_state_t coolantGetState(void)
{
    coolant_state_t state = {0};
    state.value = gpio[COOLANT_PORT].state.value ^ settings.coolant.invert.mask;
    return state;
}

static void bitsSetAtomic(volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    *ptr |= bits;
}

static uint_fast16_t bitsClearAtomic(volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    return prev;
}

static uint_fast16_t valueSetAtomic(volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    uint_fast16_t prev = *ptr;
    *ptr = value;
    return prev;
}

void settings_changed(settings_t *settings, settings_changed_flags_t changed)
{
    if (changed.spindle) {
        spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
        if (spindle_id == spindle_get_default())
            spindle_select(spindle_id);
    }
}

bool driver_setup(settings_t *settings)
{
    timer[STEPPER_TIMER].prescaler = 0;
    timer[STEPPER_TIMER].irq_enable = 1;
    mcu_register_irq_handler(Stepper_IRQHandler, Timer0_IRQ);

    gpio[STEPPER_ENABLE_PORT].dir.mask = AXES_BITMASK;
    gpio[STEP_PORT0].dir.mask = AXES_BITMASK;
    gpio[DIR_PORT].dir.mask = AXES_BITMASK;
    gpio[COOLANT_PORT].dir.mask = COOLANT_MASK;
    gpio[SPINDLE_PORT].dir.mask = SPINDLE_MASK;

    gpio[LIMITS_PORT0].dir.mask = AXES_BITMASK;
    gpio[LIMITS_PORT0].rising.mask = AXES_BITMASK;
    mcu_register_irq_handler(Limits0_IRQHandler, LIMITS_IRQ0);

    gpio[CONTROL_PORT].dir.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].rising.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].irq_mask.mask = CONTROL_MASK;
    mcu_register_irq_handler(Control_IRQHandler, CONTROL_IRQ);

    mcu_gpio_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT, PROBE_CONNECTED_BIT);

    settings_changed_flags_t changed_flags = {0};
    hal.settings_changed(settings, changed_flags);
    hal.stepper.go_idle(true);

    spindle_ptrs_t *spindle;
    if ((spindle = spindle_get(0)))
        spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);

    hal.coolant.set_state((coolant_state_t){0});

    return settings->version.id == 23;
}

void sim_process_realtime(uint_fast16_t state)
{
    on_execute_realtime(state);
}

uint32_t millis(void) { return ticks; }

bool driver_init(void)
{
    mcu_reset();

    mcu_register_irq_handler(SysTick_Handler, Systick_IRQ);

    systick_timer.load = F_CPU / 1000 - 1;
    systick_timer.irq_enable = 1;
    systick_timer.enable = 1;

    hal.info = "Flexi-HAL (Simulator)";
    hal.driver_version = "260410";
    hal.driver_setup = driver_setup;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.f_step_timer = F_CPU;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = sim_process_realtime;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigureInvertMask;

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.laser = On,
        .cap.direction = On,
        .config = spindleConfig,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindle_set_speed,
        .set_state = spindleSetState,
        .get_state = spindleGetState
    };

    spindle_register(&spindle, "FlexiHAL PWM spindle");

    hal.control.get_state = systemGetState;

    memcpy(&hal.stream, serialInit(), sizeof(io_stream_t));
    hal.nvs.type = NVS_EEPROM;
    hal.nvs.get_byte = eeprom_get_char;
    hal.nvs.put_byte = eeprom_put_char;
    hal.nvs.memcpy_to_nvs = memcpy_to_eeprom;
    hal.nvs.memcpy_from_nvs = memcpy_from_eeprom;

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = millis;

    hal.driver_cap.amass_level = 3;
    hal.coolant_cap.flood = On;
    hal.coolant_cap.mist = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.signals_cap.safety_door_ajar = On;
    hal.signals_cap.probe_disconnected = On;
    hal.signals_cap.e_stop = On;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

    // Initialize dynamically-loaded plugins.
    // NOTE: we call my_plugin_init() directly instead of #include "grbl/plugins_init.h"
    // because plugins_init.h has #if XXXX_ENABLE blocks that would double-init any
    // plugin whose enable macro is defined — our CMake auto-discovery sets those macros
    // for the compiler, so plugins_init.h would call e.g. atci_init() a second time
    // on top of the call already in my_plugin_init().
    extern void my_plugin_init(void);
    my_plugin_init();

    // Mount the simulated SD card AFTER plugins have initialized so that
    // the tooltable plugin's vfs.on_mount callback is already registered.
#if SDCARD_ENABLE
    fs_sim_init();
#endif

    return hal.version == 10;
}

void Stepper_IRQHandler(void) { hal.stepper.interrupt_callback(); }

void Control_IRQHandler(void)
{
    gpio[CONTROL_PORT].irq_state.value = ~CONTROL_MASK;
    hal.control.interrupt_callback(hal.control.get_state());
}

void Limits0_IRQHandler(void)
{
    gpio[LIMITS_PORT0].irq_state.value = (uint8_t)~AXES_BITMASK;
    hal.limits.interrupt_callback(hal.limits.get_state());
}

void SysTick_Handler(void)
{
    ticks++;
    if (delay.ms && --delay.ms == 0) {
        if (delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
