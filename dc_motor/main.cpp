#include "stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "dc_motor.h"
#include "pid_controller.h"

#define LED_PIN 17
#define M1_PWM_PIN 2
#define M1_ENA_PIN 3
#define M1_ENB_PIN 4

#define M2_PWM_PIN 7
#define M2_ENA_PIN 5
#define M2_ENB_PIN 6

#define POT_PIN 26

#define M1_ENC_A_PIN 10
#define M1_ENC_B_PIN 11

#define M2_ENC_A_PIN 13
#define M2_ENC_B_PIN 12

#define M1_ENC_INVERTED false
#define M2_ENC_INVERTED true

#define MOTOR_PPR 1496.0f

#define PREV_MASK 0x1
#define CURR_MASK 0x2
#define INVALID_MASK 0x3


const uint32_t encoder1_mask = (0x01 << M1_ENC_A_PIN) | (0x01 << M1_ENC_B_PIN);
const uint32_t encoder2_mask = (0x01 << M2_ENC_A_PIN) | (0x01 << M2_ENC_B_PIN);
volatile int32_t encoder1_ticks = 0;
volatile int32_t encoder2_ticks = 0;
volatile uint32_t encoder1_state;
volatile uint32_t encoder2_state;

float input1 = 0;
float input2 = 0;
float output1 = 0;
float output2 = 0;
float setpoint1 = 12.566f;      // rad/s -- 120 RPM
float setpoint2 = 12.566f;      // rad/s -- 120 RPM

float kp1 = 0.04;
float ki1 = 0.01;
float kd1 = 0;

float kp2 = 0.04;
float ki2 = 0.01;
float kd2 = 0;

uint32_t sample_time_ms = 20;

DCMotor motor1(M1_ENA_PIN, M1_ENB_PIN, M1_PWM_PIN);
DCMotor motor2(M2_ENA_PIN, M2_ENB_PIN, M2_PWM_PIN);

//Servo servo(SERVO_PIN, 550, 2350);

PID controller1(&input1, &output1, &setpoint1, kp1, ki1, kd1, sample_time_ms);
PID controller2(&input2, &output2, &setpoint2, kp2, ki2, kd2, sample_time_ms);


void gpio_callback(uint gpio, uint32_t events)
{
    int32_t change1 = 0;
    int32_t change2 = 0;
    uint32_t new_state1 = ((gpio_get_all() & encoder1_mask) >> (M1_ENC_B_PIN > M1_ENC_A_PIN ? M1_ENC_A_PIN : M1_ENC_B_PIN)) & 0x3;
    uint32_t new_state2 = ((gpio_get_all() & encoder2_mask) >> (M2_ENC_B_PIN > M2_ENC_A_PIN ? M2_ENC_A_PIN : M2_ENC_B_PIN)) & 0x3;

    if(((new_state1 ^ encoder1_state) != INVALID_MASK) && (new_state1 != encoder1_state))
    {
        change1 = (encoder1_state & PREV_MASK) ^ ((new_state1 & CURR_MASK) >> 1);
        if(change1 == 0)
        {
            change1 = -1;
        }
        if(M1_ENC_INVERTED) change1 = -1 * change1;
        encoder1_ticks -= change1;
    }

    if(((new_state2 ^ encoder2_state) != INVALID_MASK) && (new_state2 != encoder2_state))
    {
        change2 = (encoder2_state & PREV_MASK) ^ ((new_state2 & CURR_MASK) >> 1);
        if(change2 == 0)
        {
            change2 = -1;
        }
        if(M2_ENC_INVERTED) change2 = -1 * change2;
        encoder2_ticks -= change2;
    }
    encoder1_state = new_state1;
    encoder2_state = new_state2;
}


void setup() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // adc
    adc_init();
    adc_gpio_init(POT_PIN);
    adc_select_input(0);

    gpio_init(M1_ENC_A_PIN);
    gpio_pull_up(M1_ENC_A_PIN);
    gpio_init(M1_ENC_B_PIN);
    gpio_pull_up(M1_ENC_B_PIN);
    gpio_init(M2_ENC_A_PIN);
    gpio_pull_up(M2_ENC_A_PIN);
    gpio_init(M2_ENC_B_PIN);
    gpio_pull_up(M2_ENC_B_PIN);

    gpio_set_irq_enabled_with_callback(M1_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(M1_ENC_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_A_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(M2_ENC_B_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    encoder1_state = ((gpio_get_all() & encoder1_mask) >> M1_ENC_A_PIN) & 0x3;
    encoder2_state = ((gpio_get_all() & encoder2_mask) >> M2_ENC_A_PIN) & 0x3;

}

int main()
{
    int32_t past_ticks1 = encoder1_ticks;
    int32_t past_ticks2 = encoder2_ticks;
    setup();
    motor1.write(0.0f);
    uint32_t i = 0;
    const float k = 3.3f / (1 << 12);
    while (true)
    {
        gpio_put(LED_PIN, true);
        sleep_ms(5);
        gpio_put(LED_PIN, false);
        sleep_ms(15);
        uint16_t read = adc_read();

        setpoint1 = k * read * 10.0f;
        setpoint2 = setpoint1;

        int32_t delta_ticks1 = encoder1_ticks - past_ticks1;
        int32_t delta_ticks2 = encoder2_ticks - past_ticks2;

        float phi_l1 = 2.0f * M_PI * (delta_ticks1 / MOTOR_PPR);
        float phi_l2 = 2.0f * M_PI * (delta_ticks2 / MOTOR_PPR);

        input1 = phi_l1 / (sample_time_ms / 1000.0f);
        input2 = phi_l2 / (sample_time_ms / 1000.0f);

        controller1.compute();
        controller2.compute();

        motor1.write(output1);
        motor2.write(output2);

        i++;
        if(i % 10 == 0)
        {
            printf("setpoint: \t%.2f - %.2f \tinput: \t%.2f - %.2f \toutput: \t%.2f - %.2f \r", setpoint1, setpoint2, input1, input2, output1, output2);
        }
        past_ticks1 = encoder1_ticks;
        past_ticks2 = encoder2_ticks;
    }
}
