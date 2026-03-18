#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "driver/gpio.h"

// ===== SERVO PINS =====
#define SERVO1 13
#define SERVO2 12
#define SERVO3 14
#define SERVO4 27

// ===== JOYSTICK =====
#define JOY_X ADC1_CHANNEL_6   // GPIO34
#define JOY_Y ADC1_CHANNEL_7   // GPIO35
#define JOY_SW 32

// ===== PWM CONFIG =====
#define PWM_FREQ 50
#define PWM_RES LEDC_TIMER_16_BIT

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_PERIOD_US 20000

// ===== CONTROL =====
#define DEADZONE 150
#define SMOOTH_STEP 3

int mode = 0;

// ===== CONVERT =====
uint32_t us_to_duty(int us)
{
    uint32_t duty_max = (1 << 16) - 1;
    return (us * duty_max) / SERVO_PERIOD_US;
}

int angle_to_us(int angle)
{
    return SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
}

// ===== SERVO INIT =====
void servo_init()
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    int pins[4] = {SERVO1, SERVO2, SERVO3, SERVO4};

    for(int i = 0; i < 4; i++)
    {
        ledc_channel_config_t ch = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&ch);
    }
}

// ===== SMOOTH MOVE =====
int smooth(int current, int target)
{
    if(current < target) current += SMOOTH_STEP;
    else if(current > target) current -= SMOOTH_STEP;
    return current;
}

// ===== MAIN =====
void app_main(void)
{
    printf("=== SYSTEM START ===\n");

    servo_init();

    // ADC config
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(JOY_X, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(JOY_Y, ADC_ATTEN_DB_11);

    // Button config
    gpio_set_direction(JOY_SW, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JOY_SW, GPIO_PULLUP_ONLY);

    int angleX = 90, angleY = 90;
    int targetX = 90, targetY = 90;

    int lastButton = 1;

    while (1)
    {
        int x = adc1_get_raw(JOY_X);
        int y = adc1_get_raw(JOY_Y);

        // ===== DEADZONE =====
        if(abs(x - 2048) > DEADZONE)
            targetX = (x * 180) / 4095;

        if(abs(y - 2048) > DEADZONE)
            targetY = (y * 180) / 4095;

        // ===== SMOOTH =====
        angleX = smooth(angleX, targetX);
        angleY = smooth(angleY, targetY);

        int usX = angle_to_us(angleX);
        int usY = angle_to_us(angleY);

        uint32_t dutyX = us_to_duty(usX);
        uint32_t dutyY = us_to_duty(usY);

        // ===== BUTTON MODE SWITCH =====
        int btn = gpio_get_level(JOY_SW);
        if(btn == 0 && lastButton == 1)
        {
            mode = !mode;
            printf("Switch mode: %d\n", mode);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        lastButton = btn;

        // ===== CONTROL =====
        if(mode == 0)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, 0, dutyX);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, 0);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, dutyY);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, 1);
        }
        else
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, 2, dutyX);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, 2);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, 3, dutyY);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, 3);
        }

        // ===== DEBUG =====
        printf("Mode:%d | X:%d Y:%d | AngleX:%d AngleY:%d\n",
               mode, x, y, angleX, angleY);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}