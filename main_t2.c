#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <pigpiod_if2.h>

#define S1 5
#define S2 6
#define S3 13
#define S4 19
#define S5 26

#define SENSOR_COUNT 5

#define PWMI2CADR 0x40
#define PWMI2CCH 1

#define PWM_MODE1    0x00
#define PWM_MODE2    0x01
#define PWM_PRESCALE 0xFE
#define PWM_0_ON_L   0x06
#define PWM_0_ON_H   0x07
#define PWM_0_OFF_L  0x08
#define PWM_0_OFF_H  0x09

#define ENA_PWM 8
#define IN1_PWM 9
#define IN2_PWM 10
#define ENB_PWM 13
#define IN3_PWM 11
#define IN4_PWM 12

#define PWM_MAX_COUNT 4095
#define MOTOR_MAX  16
#define MOTOR_MIN -16

/*
 * ライントレース制御パラメータ
 * KP/KD はスケール済みエラー（実値×10）に対して作用する
 * turn = KP * error_scaled / 10 + KD * filtered_d / 10 で最終値を戻す
 */
#define BASE_SPEED 8
#define KP_NUM 4    /* KP = KP_NUM / KP_DEN */
#define KP_DEN 10
#define KD_NUM 3
#define KD_DEN 10

/*
 * Dローパスフィルタ係数 (0〜255, 大きいほど平滑)
 * filtered_d = (LPF_ALPHA * filtered_d + (256 - LPF_ALPHA) * raw_d) / 256
 */
#define LPF_ALPHA 180

/*
 * ライン消失タイムアウト（ループ回数）
 * 10ms × 100 = 1秒で停止
 */
#define LOST_TIMEOUT_LOOPS 100

#define LINE_DETECTED_VALUE 1

/* ------------------------------------------------------------------ */

typedef struct {
    int error_scaled;    /* 現在エラー ×10 */
    int prev_error_scaled;
    int filtered_d;      /* LPFかけたderivative ×10 */
    int last_error_sign; /* -1, 0, 1 */
    int lost_loops;      /* ライン消失継続カウント */
} PdState;

static volatile sig_atomic_t running = 1;
static PdState g_state = {0, 0, 0, 0, 0};

/* ------------------------------------------------------------------ */

void sigHandler(int sig);
void initHardware(int *pd, int *fd);
void stopHardware(int pd, int fd);

uint8_t readSensorsAsBits(int pd);
void controlLineTracePD(int pd, int fd, uint8_t sensors);

int  clamp(int value, int min, int max);
void set_pwm_output(int pd, int fd, int channel, int on_count, int off_count);
void set_pwm_full_on(int pd, int fd, int channel);
void set_pwm_full_off(int pd, int fd, int channel);
int  motor_drive(int pd, int fd, int left_motor, int right_motor);

/* ------------------------------------------------------------------ */

int main(void)
{
    int pd, fd;

    signal(SIGINT,  sigHandler);
    signal(SIGTERM, sigHandler);

    initHardware(&pd, &fd);
    motor_drive(pd, fd, 0, 0);

    while (running)
    {
        uint8_t sensors = readSensorsAsBits(pd);
        printf("sensors = 0x%02X\n", sensors);
        controlLineTracePD(pd, fd, sensors);
        time_sleep(0.01);
    }

    printf("Stopping...\n");
    stopHardware(pd, fd);
    return 0;
}

/* ------------------------------------------------------------------ */

void sigHandler(int sig) { (void)sig; running = 0; }

void initHardware(int *pd, int *fd)
{
    *pd = pigpio_start(NULL, NULL);
    if (*pd < 0) { fprintf(stderr, "pigpio connection failed.\n"); exit(EXIT_FAILURE); }

    *fd = i2c_open(*pd, PWMI2CCH, PWMI2CADR, 0);
    if (*fd < 0) { fprintf(stderr, "Failed to open I2C.\n"); pigpio_stop(*pd); exit(EXIT_FAILURE); }

    i2c_write_byte_data(*pd, *fd, PWM_MODE1,    0x10);
    i2c_write_byte_data(*pd, *fd, PWM_PRESCALE, 61);
    i2c_write_byte_data(*pd, *fd, PWM_MODE1,    0x00);
    time_sleep(0.001);
    i2c_write_byte_data(*pd, *fd, PWM_MODE1,    0xA0);
    i2c_write_byte_data(*pd, *fd, PWM_MODE2,    0x04);

    set_mode(*pd, S1, PI_INPUT);
    set_mode(*pd, S2, PI_INPUT);
    set_mode(*pd, S3, PI_INPUT);
    set_mode(*pd, S4, PI_INPUT);
    set_mode(*pd, S5, PI_INPUT);

    printf("Init success.\n");
}

void stopHardware(int pd, int fd)
{
    motor_drive(pd, fd, 0, 0);
    i2c_close(pd, fd);
    pigpio_stop(pd);
}

/* ------------------------------------------------------------------ */

uint8_t readSensorsAsBits(int pd)
{
    const int gpios[SENSOR_COUNT] = {S1, S2, S3, S4, S5};
    uint8_t sensors = 0x00;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (gpio_read(pd, gpios[i]) == LINE_DETECTED_VALUE)
            sensors |= (uint8_t)(1U << i);
    }
    return sensors;
}

/* ------------------------------------------------------------------ */

void controlLineTracePD(int pd, int fd, uint8_t sensors)
{
    /*
     * 重みを ×10 にスケールアップ: -20, -10, 0, 10, 20
     * → エラーの分解能が整数除算でも細かくなる
     */
    const int weights[SENSOR_COUNT] = {-20, -10, 0, 10, 20};

    int left_speed, right_speed;

    /*
     * 全センサーON = 黒べた上 → 直進扱い
     */
    if (sensors == 0x1F)
    {
        g_state.error_scaled = 0;
        g_state.prev_error_scaled = 0;
        g_state.filtered_d = 0;
        g_state.lost_loops = 0;
        motor_drive(pd, fd, BASE_SPEED, BASE_SPEED);
        return;
    }

    int sum = 0, count = 0;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if ((sensors & (uint8_t)(1U << i)) != 0)
        {
            sum += weights[i];
            count++;
        }
    }

    if (count > 0)
    {
        g_state.lost_loops = 0;

        int error_scaled = sum / count; /* 単位は元の重み×10 */

        /*
         * Derivative（生値）にLPFをかけてノイズ低減
         * filtered_d は ×10 スケール
         */
        int raw_d = error_scaled - g_state.prev_error_scaled;
        g_state.filtered_d =
            (LPF_ALPHA * g_state.filtered_d + (256 - LPF_ALPHA) * raw_d) / 256;

        g_state.prev_error_scaled = error_scaled;
        g_state.error_scaled = error_scaled;
        g_state.last_error_sign = (error_scaled > 0) ? 1 : (error_scaled < 0) ? -1 : 0;

        /*
         * turn = KP * error + KD * filtered_d
         * スケール ×10 を /10 で戻す
         */
        int turn = KP_NUM * error_scaled / KP_DEN
                 + KD_NUM * g_state.filtered_d / KD_DEN;

        left_speed  = BASE_SPEED + turn;
        right_speed = BASE_SPEED - turn;
    }
    else
    {
        /*
         * ライン消失
         */
        g_state.lost_loops++;

        if (g_state.lost_loops >= LOST_TIMEOUT_LOOPS)
        {
            /* タイムアウト：安全停止 */
            printf("Line lost timeout. Stopping.\n");
            running = 0;
            motor_drive(pd, fd, 0, 0);
            return;
        }

        /* 消失直後：最後に見えた方向へ旋回 */
        if (g_state.last_error_sign < 0)
        {
            left_speed  = -5;
            right_speed =  8;
        }
        else if (g_state.last_error_sign > 0)
        {
            left_speed  =  8;
            right_speed = -5;
        }
        else
        {
            left_speed  = 4;
            right_speed = 4;
        }
    }

    left_speed  = clamp(left_speed,  MOTOR_MIN, MOTOR_MAX);
    right_speed = clamp(right_speed, MOTOR_MIN, MOTOR_MAX);
    motor_drive(pd, fd, left_speed, right_speed);
}

/* ------------------------------------------------------------------ */

int clamp(int value, int min, int max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void set_pwm_output(int pd, int fd, int channel, int on_count, int off_count)
{
    int reg = PWM_0_ON_L + 4 * channel;
    on_count  = clamp(on_count,  0, PWM_MAX_COUNT);
    off_count = clamp(off_count, 0, PWM_MAX_COUNT);
    i2c_write_byte_data(pd, fd, reg + 0, on_count  & 0xFF);
    i2c_write_byte_data(pd, fd, reg + 1, (on_count  >> 8) & 0x0F);
    i2c_write_byte_data(pd, fd, reg + 2, off_count & 0xFF);
    i2c_write_byte_data(pd, fd, reg + 3, (off_count >> 8) & 0x0F);
}

void set_pwm_full_on(int pd, int fd, int channel)
{
    int reg = PWM_0_ON_L + 4 * channel;
    i2c_write_byte_data(pd, fd, reg + 0, 0x00);
    i2c_write_byte_data(pd, fd, reg + 1, 0x10);
    i2c_write_byte_data(pd, fd, reg + 2, 0x00);
    i2c_write_byte_data(pd, fd, reg + 3, 0x00);
}

void set_pwm_full_off(int pd, int fd, int channel)
{
    int reg = PWM_0_ON_L + 4 * channel;
    i2c_write_byte_data(pd, fd, reg + 0, 0x00);
    i2c_write_byte_data(pd, fd, reg + 1, 0x00);
    i2c_write_byte_data(pd, fd, reg + 2, 0x00);
    i2c_write_byte_data(pd, fd, reg + 3, 0x10);
}

int motor_drive(int pd, int fd, int left_motor, int right_motor)
{
    left_motor  = clamp(left_motor,  MOTOR_MIN, MOTOR_MAX);
    right_motor = clamp(right_motor, MOTOR_MIN, MOTOR_MAX);

    int left_pwm  = abs(left_motor)  * PWM_MAX_COUNT / MOTOR_MAX;
    int right_pwm = abs(right_motor) * PWM_MAX_COUNT / MOTOR_MAX;

    if (left_motor > 0)       { set_pwm_full_on(pd, fd, IN3_PWM); set_pwm_full_off(pd, fd, IN4_PWM); }
    else if (left_motor < 0)  { set_pwm_full_off(pd, fd, IN3_PWM); set_pwm_full_on(pd, fd, IN4_PWM); }
    else                      { set_pwm_full_off(pd, fd, IN3_PWM); set_pwm_full_off(pd, fd, IN4_PWM); }

    if (right_motor > 0)      { set_pwm_full_on(pd, fd, IN1_PWM); set_pwm_full_off(pd, fd, IN2_PWM); }
    else if (right_motor < 0) { set_pwm_full_off(pd, fd, IN1_PWM); set_pwm_full_on(pd, fd, IN2_PWM); }
    else                      { set_pwm_full_off(pd, fd, IN1_PWM); set_pwm_full_off(pd, fd, IN2_PWM); }

    if (left_motor == 0)  set_pwm_full_off(pd, fd, ENB_PWM);
    else                  set_pwm_output(pd, fd, ENB_PWM, 0, left_pwm);

    if (right_motor == 0) set_pwm_full_off(pd, fd, ENA_PWM);
    else                  set_pwm_output(pd, fd, ENA_PWM, 0, right_pwm);

    return 0;
}