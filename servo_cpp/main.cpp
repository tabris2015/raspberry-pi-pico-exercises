#include "pico/stdlib.h"
#include "pico/time.h"
#include "servo.h"

#define LED_PIN 25
#define UP_PIN 16
#define SERVO_PIN 22
#define DOWN_PIN 17
#define LEFT_PIN 18
#define RIGHT_PIN 19
#define A_PIN 20
#define B_PIN 21


Servo servo(SERVO_PIN, 550, 2350);


void setup() {
    gpio_init(LED_PIN);
    gpio_init(UP_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_set_dir(UP_PIN, GPIO_IN);
    gpio_pull_up(UP_PIN);
}

int main()
{
    setup();

    uint interval = 3000;

    while (true)
    {
        servo.write_angle(90);
        gpio_put(LED_PIN, true);
        sleep_ms(interval);
        servo.write_angle(0);
        gpio_put(LED_PIN, false);
        sleep_ms(500);
    }
}
