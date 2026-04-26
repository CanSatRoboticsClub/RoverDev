#include "PWMmotorDriver.h"

static bool configured_slices[8] = {false, false, false, false, false, false, false, false};

static uint16_t wrap_val = 99;

void addPins(struct motor* motor_pins, uint IN1, uint IN2) {
    if (!motor_pins) return;
    if (IN1 > 29 || IN2 > 29) return;

    uint IN1_slice = pwm_gpio_to_slice_num(IN1);
    uint IN2_slice = pwm_gpio_to_slice_num(IN2);

    gpio_set_function(IN1, GPIO_FUNC_PWM);
    gpio_set_function(IN2, GPIO_FUNC_PWM);

    motor_pins->IN1 = IN1;
    motor_pins->slice_IN1 = IN1_slice;
    motor_pins->chan_IN1 = pwm_gpio_to_channel(IN1);

    motor_pins->IN2 = IN2;
    motor_pins->slice_IN2 = IN2_slice;
    motor_pins->chan_IN2 = pwm_gpio_to_channel(IN2);

    motor_pins->wrap = wrap_val;

    if (!configured_slices[motor_pins->slice_IN1]) {
        pwm_set_wrap(motor_pins->slice_IN1, wrap_val);
        pwm_set_enabled(motor_pins->slice_IN1, true);
        configured_slices[motor_pins->slice_IN1] = true;
    }
    if (!configured_slices[motor_pins->slice_IN2]) {
        pwm_set_wrap(motor_pins->slice_IN2, wrap_val);
        pwm_set_enabled(motor_pins->slice_IN2, true);
        configured_slices[motor_pins->slice_IN2] = true;
    }
}

void setMotorPWM(struct motor* motor_pins, float duty_cycle, bool forward) {
    if (!motor_pins) return;
    if (duty_cycle < 0.0f || duty_cycle > 1.0f) return;

    uint16_t level = (uint16_t)(duty_cycle * motor_pins->wrap);

    if (forward) {
        pwm_set_chan_level(motor_pins->slice_IN1, motor_pins->chan_IN1, level);
        pwm_set_chan_level(motor_pins->slice_IN2, motor_pins->chan_IN2, 0);
    }
    else {
        pwm_set_chan_level(motor_pins->slice_IN1, motor_pins->chan_IN1, 0);
        pwm_set_chan_level(motor_pins->slice_IN2, motor_pins->chan_IN2, level);
    }
}

void pausePWM() {
    for (int i = 0; i < 8; i++) {
        if (configured_slices[i]) {
            pwm_set_enabled(i, false);
        }
    }
}

void unpausePWM() {
    for (int i = 0; i < 8; i++) {
        if (configured_slices[i]) {
            pwm_set_enabled(i, true);
        }
    }
}
