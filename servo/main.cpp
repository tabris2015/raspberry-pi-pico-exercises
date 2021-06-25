#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#define LED_PIN 25
#define UP_PIN 16
#define SERVO_PIN 22
#define DOWN_PIN 17
#define LEFT_PIN 18
#define RIGHT_PIN 19
#define A_PIN 20
#define B_PIN 21

void init_pwm(uint slice, uint channel)
{
    pwm_set_clkdiv(slice, 125.0);   // 1ms pwm resolution
    pwm_set_wrap(slice, 16667);        // ticks to get 60Hz frequency
    pwm_set_chan_level(slice, channel, 0);
    pwm_set_enabled(slice, true);
}

void set_pwm(uint slice, uint channel, uint16_t value)
{
    pwm_set_chan_level(slice, channel, value);
    pwm_set_enabled(slice, true);
}

void setup() {
    gpio_init(LED_PIN);
    gpio_init(UP_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_set_dir(UP_PIN, GPIO_IN);
    gpio_pull_up(UP_PIN);
}

void set_angle(uint slice, uint channel, uint8_t angle)
{
    if(angle > 180 || angle < 0) return;
    uint16_t level_us = (angle * 10) + 550;
    set_pwm(slice, channel, level_us);
}

int main()
{
    setup();
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    uint channel = pwm_gpio_to_channel(SERVO_PIN);
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    uint interval = 3000;
    init_pwm(slice_num, channel);

    while (true)
    {
        set_angle(slice_num, channel, 90);
        gpio_put(LED_PIN, true);
        sleep_ms(interval);
        set_angle(slice_num, channel, 0);
        gpio_put(LED_PIN, false);
        sleep_ms(interval);
    }
}
