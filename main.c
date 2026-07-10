//
// Created by stellarcielo on 2026/05/12.
//

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pigpiod_if2.h>
#include <stdint.h>

#define S1 5
#define S2 6
#define S3 13
#define S4 19
#define S5 26

// PWM ユニットの I2C アドレス
// i2cdetect で確認可能、違っていたら修正して下さい
#define PWMI2CADR 0x40
// PWM ユニットが接続されている I2C のチャネル番号
#define PWMI2CCH 1
// モータードライバの各入力が接続されている PWM ユニットのチャネル番号
// 右側のモーター：パワーユニットの K1 または K2 に接続（説明書は誤り）
// ENA は PWM 駆動に使う（1 でブリッジ動作、0 はブリッジオフ）
// IN1 と IN2 は右車輪の回転方向を決める（後進：0,1、前進：1,0）（0,0 と 1,1 はブレーキ）
#define ENA_PWM 8
#define IN1_PWM 9
#define IN2_PWM 10
// 左側のモーター：パワーユニットの K3 または K4 に接続（説明書は誤り）
// ENB は PWM 駆動に使う（1 でブリッジ動作、0 はブリッジオフ）
// IN3 と IN4 は左車輪の回転方向を決める（後進：0,1、前進：1,0）（0,0 と 1,1 はブレーキ）
#define ENB_PWM 13
#define IN3_PWM 11
#define IN4_PWM 12
// PWM モジュールのレジスタ番号
#define PWM_MODE1 0
#define PWM_MODE2 1
#define PWM_SUBADR1 2
#define PWM_SUBADR2 3
#define PWM_SUBADR3 4
#define PWM_ALLCALL 5
// PWM 番号×4+PWM_0_??_? でレジスタ番号は求まる
#define PWM_0_ON_L 6
#define PWM_0_ON_H 7
#define PWM_0_OFF_L 8
#define PWM_0_OFF_H 9
// PWM 出力定数
#define PWMFULLON 16
#define PWMFULLOFF 0
// プリスケーラのレジスタ番号
// PWM 周波数を決めるレジスタ番号、100Hz なら 61 をセット
#define PWM_PRESCALE 254

#define BASE_SPEED 8
#define TURN_WAIT 30

void initHard(int *pd, int *fd);
void sigHandler(int sig);
int motor_drive(int pd, int fd, int lm, int rm);
void readAllSensors(int pd, int gpios[], uint8_t sensors[]);

volatile sig_atomic_t running = 1;

int main(void){
    int pd, fd;
    int gpios[] = {S1, S2, S3, S4, S5};
    uint8_t sensors = 0x00; //1バイトの変数のためuint8_tを使っています。5つのセンサーをまとめてビット列で管理するためです.
    int checkpoint = 0;

    /* checkpoint
     *    s0→*******←1
     *          *←2
     *    g4→*******←3
     */

    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    initHard(&pd, &fd);
    motor_drive(pd, fd, 0, 0);
    //printf("reset");

    while (running){
        readAllSensors(pd, gpios, &sensors);
        printf("sensors:%x checkpoint:%d\n", sensors, checkpoint);

        if ((sensors & 0x1F) == 0x1F){
            motor_drive(pd, fd, 0, 0);
        }else if ((sensors & 0x1F) == 0b00000100){
            // ↑.
            motor_drive(pd, fd, BASE_SPEED, BASE_SPEED);
        }else if ((sensors & 0x1F) == 0b00000010){
            // ←↑.
            motor_drive(pd, fd, BASE_SPEED/2, BASE_SPEED);
        }else if ((sensors & 0x1F) == 0b00001000){
            // ↑→.
            motor_drive(pd, fd, BASE_SPEED, BASE_SPEED/2);
        }else if (checkpoint == 1 && (sensors & 0x1F) == 0b00000111){
            for (int i = 0; i < TURN_WAIT; i++){
                motor_drive(pd, fd, BASE_SPEED, -BASE_SPEED);
                time_sleep(0.1);
            }
        }else if (checkpoint == 2 && (sensors & 0x1F) == 0b00000000){
            for (int i = 0; i < TURN_WAIT; i++){
                motor_drive(pd, fd, BASE_SPEED, -BASE_SPEED);
                time_sleep(0.1);
            }
        }else if (checkpoint == 4 && (sensors & 0x1F) == 0b00000000){
            motor_drive(pd, fd, 0, 0);
            break;
        }else if ((sensors & 0x1F) == 0b00000000){
            motor_drive(pd, fd, BASE_SPEED, -BASE_SPEED);
            checkpoint += 1;
        }

        time_sleep(0.01);
    }

    printf("Stopping...\n\n");
    motor_drive(pd, fd, 0, 0);
    pigpio_stop(pd);
    return 0;
}

void sigHandler(int sig){
    running = 0;
}

void initHard(int *pd, int *fd){
    if ((*pd = pigpio_start(NULL, NULL)) < 0)
    {
        fprintf(stderr, "pigpio connection failed.\n");
        fprintf(stderr, "pigpio check start.\n");
        exit(EXIT_FAILURE);
    }
    // I2C 接続と PWM ユニットの初期化
    *fd = i2c_open(*pd,PWMI2CCH,PWMI2CADR, 0);
    if (*fd < 0)
    {
        fprintf(stderr, "Failed to init I2C.\n");
        exit(EXIT_FAILURE);
    }
    i2c_write_byte_data(*pd, *fd,PWM_PRESCALE, 61); //PWM 周期 10ms に設定
    printf("PWM 周期は 10ms です。値の更新間隔に注意して下さい。\n");
    i2c_write_byte_data(*pd, *fd,PWM_MODE1, 0x10); //SLEEP mode
    i2c_write_byte_data(*pd, *fd,PWM_MODE1, 0); //normal mode
    time_sleep(0.001); // wait for stabilizing internal oscillator
    i2c_write_byte_data(*pd, *fd,PWM_MODE1, 0xA0); //Restart all PWM ch

    set_mode(*pd, S1, PI_INPUT);
    set_mode(*pd, S2, PI_INPUT);
    set_mode(*pd, S3, PI_INPUT);
    set_mode(*pd, S4, PI_INPUT);
    set_mode(*pd, S5, PI_INPUT);

    printf("Init success.\n");
}

void readAllSensors(int pd, int gpios[], uint8_t *sensors){
    //printf("r");
    *sensors = 0x00;
    for (int i = 0; i < 5; i++)
    {
        *sensors += ((char)gpio_read(pd, gpios[i]) & 0x01) << i;
    }
}