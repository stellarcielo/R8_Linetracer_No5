//
// Created by stellarcielo on 2026/04/14.
//

// インクルード指定：<stdio.h> <stdlib.h> <pigpiod_if2.h>
// 以下、定数宣言です
#include <stdio.h>
#include <stdlib.h>
#include <pigpiod_if2.h>
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
// 以下、制御プログラムの例
// PWM ユニット自体は 12 ビット精度だが、上位 4 ビット分を制御（つまり、16 段階で制御）
// あまり細かく制御しても、モーター特性やタイヤの摩擦等の影響が大きいと考えられるためです
// もし必要と思うなら、自分で PWM ユニットのマニュアルを見てプログラムを書いて下さい
// そのためには以下の 2 つの関数を対応できるように書き換える必要があります
int set_pwm_output(int pd, int fd, int pwmch, int outval)
// motor_drive( )から呼ばれる関数、PWM ユニットへの書き込みをやっています
// 直接、他から呼び出す必要はないと思われますが、必要ならどうぞ
{
    int ef = 0;
    int regno;

    if ((pwmch < 0) || (pwmch > 15)) ef = 1; // チャネルの指定違反チェック
    if ((outval < 0) || (outval > 16)) ef = ef + 2; // 出力値の指定違反チェック
    if (ef == 0){
        regno = PWM_0_ON_L + pwmch * 4; // 1ch あたり 4 レジスタで 16ch 分あるので
        if (outval == 16){
            i2c_write_byte_data(pd, fd, regno + 3, 0);
            i2c_write_byte_data(pd, fd, regno + 1, 0x10);
        }
        else{
            i2c_write_byte_data(pd, fd, regno + 1, 0);
            i2c_write_byte_data(pd, fd, regno + 3, outval);
        }
    }
    return ef; // エラーがなければ 0 が返る
}

int motor_drive(int pd, int fd, int lm, int rm)
// モーターを制御するための関数
// pd は pigpod に接続時の接続番号
// fd は I2C 初期化時のファイルディスクプリタ（デバイス番号のようなもの）
// lm は左モーター、rm は右モーターの駆動数値で、-16～+16 の範囲で指定
// 負の場合は後方向に回転、正の場合は前方向に回転
// 絶対値が大きいほど、パワーが大きくなる
{
    set_pwm_output(pd, fd, ENA_PWM, 0); // Right motor disable
    set_pwm_output(pd, fd, ENB_PWM, 0); // Left motor disable
    // Right motor PWM control
    if (rm < 0){
        ;
        set_pwm_output(pd, fd, IN1_PWM, 0); // OUT1->GND;
        set_pwm_output(pd, fd, IN2_PWM, 16); // OUT2->+Vs
        rm = abs(rm);
    }
    else{
        set_pwm_output(pd, fd, IN1_PWM, 16); // OUT1->+Vs
        set_pwm_output(pd, fd, IN2_PWM, 0); // OUT2->GND
    }
    // Left motor PWM control
    if (lm < 0){
        set_pwm_output(pd, fd, IN3_PWM, 0); // OUT3->GND
        set_pwm_output(pd, fd, IN4_PWM, 16); // OUT4->+Vs
        lm = abs(lm);
    }
    else{
        set_pwm_output(pd, fd, IN3_PWM, 16); // OUT3->+Vs
        set_pwm_output(pd, fd, IN4_PWM, 0); // OUT4->GND
    }
    if (lm > 16) lm = 16;
    if (rm > 16) rm = 16;
    set_pwm_output(pd, fd, ENA_PWM, rm); // Right motor PWM start
    set_pwm_output(pd, fd, ENB_PWM, lm); // Left motor PWM start
    return 0; // 戻り値は常に 0
}

//この他に、プログラムの最初の方で以下の pigpiod への接続と PWM ユニットの初期化が必要。
//（どこかの関数の中身の一部を抜き出した形で記述されているので注意！）
int main(){
    int pd, fd;
    // pigpiod に接続
    // pd に接続番号が入る ← pigpiod の関数を呼び出すときに必須
    pd = pigpio_start(NULL, NULL);
    if (pd < 0)
    {
        fprintf(stderr, "pigpiod の接続に失敗しました。\n");
        fprintf(stderr, "pigpiod が起動しているか確認してください。\n");
        exit(EXIT_FAILURE);
    }
    // I2C 接続と PWM ユニットの初期化
    fd = i2c_open(pd,PWMI2CCH,PWMI2CADR, 0);
    if (fd < 0)
    {
        fprintf(stderr, "I2C の初期化に失敗しました。終了します。\n");
        exit(EXIT_FAILURE);
    }
    i2c_write_byte_data(pd, fd,PWM_PRESCALE, 61); //PWM 周期 10ms に設定
    printf("PWM 周期は 10ms です。値の更新間隔に注意して下さい。¥n");
    i2c_write_byte_data(pd, fd,PWM_MODE1, 0x10); //SLEEP mode
    i2c_write_byte_data(pd, fd,PWM_MODE1, 0); //normal mode
    time_sleep(0.001); // wait for stabilizing internal oscillator
    i2c_write_byte_data(pd, fd,PWM_MODE1, 0xA0); //Restart all PWM ch
    motor_drive(pd, fd, 0, 0); //Stop left and right motor

    //sample
    motor_drive(pd, fd, 16, 16);
    time_sleep(2.0);
    motor_drive(pd, fd, 0, 0);
}
