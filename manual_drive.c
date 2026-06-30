#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <pigpiod_if2.h>

/* ---- ハードウェア定義（main_t.cと同じ） ---- */
#define PWMI2CADR 0x40
#define PWMI2CCH  1

#define PWM_MODE1    0x00
#define PWM_MODE2    0x01
#define PWM_PRESCALE 0xFE
#define PWM_0_ON_L   0x06

#define ENA_PWM 8
#define IN1_PWM 9
#define IN2_PWM 10
#define ENB_PWM 13
#define IN3_PWM 11
#define IN4_PWM 12

#define PWM_MAX_COUNT 4095
#define MOTOR_MAX  16
#define MOTOR_MIN -16

/* ---- 操作設定 ---- */
#define DRIVE_SPEED  10   /* w: 前進速度 */
#define TURN_SPEED   8    /* a/d: 旋回速度（片側逆転） */

/* ---- 操作設定 ---- */
#define DRIVE_SPEED   10
#define TURN_SPEED     8
#define KEY_TIMEOUT_SEC 0.10   /* これより長くキーが来なければ停止 */

/* ------------------------------------------------------------------ */

static volatile sig_atomic_t running = 1;
static struct termios g_old_termios;

/* ------------------------------------------------------------------ */

int  clamp(int value, int min, int max);
void set_pwm_output(int pd, int fd, int channel, int on_count, int off_count);
void set_pwm_full_on(int pd, int fd, int channel);
void set_pwm_full_off(int pd, int fd, int channel);
int  motor_drive(int pd, int fd, int left_motor, int right_motor);

void initHardware(int *pd, int *fd);
void stopHardware(int pd, int fd);

void termios_raw(void);
void termios_restore(void);
int  kbhit(void);       /* キー入力が来ているか（ノンブロッキング） */
int  getch_nb(void);    /* 1文字読む（来ていなければ -1） */

void sigHandler(int sig);

/* ------------------------------------------------------------------ */

int main(void)
{
    int pd, fd;

    signal(SIGINT,  sigHandler);
    signal(SIGTERM, sigHandler);

    initHardware(&pd, &fd);
    termios_raw();

    motor_drive(pd, fd, 0, 0);

    printf("=== Manual drive mode ===\n");
    printf("  a : left spin\n");
    printf("  d : right spin\n");
    printf("  q : quit\n");
    printf("Key held = moving, released = stop\n\n");

    /*
     * 最後に押されたキーを記憶しておき、
     * 新しいキーが来なければ同じ動作を継続する。
     * q が来たら停止して終了。
     */
    double last_key_time = 0.0;
    char   last_cmd      = 0;

    while (running)
    {
        int c = getch_nb();

        if (c != -1)
        {
            if (c == 'q' || c == 'Q') { running = 0; break; }
            last_cmd      = (char)c;
            last_key_time = time_time();   /* 受信時刻を更新 */
        }

        /* タイムアウトで強制停止 */
        double idle = time_time() - last_key_time;
        if (idle > KEY_TIMEOUT_SEC)
            last_cmd = 0;

        int left = 0, right = 0;
        switch (last_cmd)
        {
        case 'a': case 'A': left = -TURN_SPEED; right =  TURN_SPEED; break;
        case 'd': case 'D': left =  TURN_SPEED; right = -TURN_SPEED; break;
        case 'w': case 'W': left =  DRIVE_SPEED; right = DRIVE_SPEED; break;
        case 's': case 'S': left = -DRIVE_SPEED; right = -DRIVE_SPEED; break;
        default: break;
        }

        motor_drive(pd, fd, left, right);
        printf("\rcmd=%c  idle=%.2fs  L=%+3d R=%+3d   ",
               last_cmd ? last_cmd : '-', idle, left, right);
        fflush(stdout);

        time_sleep(0.02);
    }

    printf("\nStopping...\n");
    termios_restore();
    stopHardware(pd, fd);
    return 0;
}

/* ------------------------------------------------------------------ */

void sigHandler(int sig)
{
    (void)sig;
    running = 0;
    termios_restore();
}

/* ------------------------------------------------------------------ */

void termios_raw(void)
{
    struct termios t;
    tcgetattr(STDIN_FILENO, &g_old_termios);
    t = g_old_termios;

    /*
     * ICANON : 行バッファリング無効（Enterなしで1文字ずつ読める）
     * ECHO   : 入力文字をターミナルに表示しない
     */
    t.c_lflag &= ~((tcflag_t)(ICANON | ECHO));
    t.c_cc[VMIN]  = 0;   /* read() は即座にリターン */
    t.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSANOW, &t);

    /* stdin をノンブロッキングに設定 */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void termios_restore(void)
{
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_termios);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

int getch_nb(void)
{
    unsigned char c;
    int n = (int)read(STDIN_FILENO, &c, 1);
    return (n == 1) ? (int)c : -1;
}

/* ------------------------------------------------------------------ */

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

    printf("Init success.\n");
}

void stopHardware(int pd, int fd)
{
    motor_drive(pd, fd, 0, 0);
    i2c_close(pd, fd);
    pigpio_stop(pd);
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

    if (left_motor > 0)       { set_pwm_full_on(pd, fd, IN3_PWM);  set_pwm_full_off(pd, fd, IN4_PWM); }
    else if (left_motor < 0)  { set_pwm_full_off(pd, fd, IN3_PWM); set_pwm_full_on(pd, fd, IN4_PWM); }
    else                      { set_pwm_full_off(pd, fd, IN3_PWM); set_pwm_full_off(pd, fd, IN4_PWM); }

    if (right_motor > 0)      { set_pwm_full_on(pd, fd, IN1_PWM);  set_pwm_full_off(pd, fd, IN2_PWM); }
    else if (right_motor < 0) { set_pwm_full_off(pd, fd, IN1_PWM); set_pwm_full_on(pd, fd, IN2_PWM); }
    else                      { set_pwm_full_off(pd, fd, IN1_PWM); set_pwm_full_off(pd, fd, IN2_PWM); }

    if (left_motor == 0)  set_pwm_full_off(pd, fd, ENB_PWM);
    else                  set_pwm_output(pd, fd, ENB_PWM, 0, left_pwm);

    if (right_motor == 0) set_pwm_full_off(pd, fd, ENA_PWM);
    else                  set_pwm_output(pd, fd, ENA_PWM, 0, right_pwm);

    return 0;
}