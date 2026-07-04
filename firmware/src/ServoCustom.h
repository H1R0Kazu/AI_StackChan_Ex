#ifndef _SERVO_CUSTOM_H
#define _SERVO_CUSTOM_H

#include <Stackchan_servo.h>

class ServoCustom : public StackchanSERVO {
public:
    ServoCustom(){};

    // StackchanSERVO::begin() を隠蔽(shadow)する。ServoCustom型のポインタ/参照経由でのみ有効
    // (Robot.cppはServoCustom*経由の非仮想呼び出しのため、これで置き換わる)。
    // RT_DYN_XL330の場合のみ独自の初期化(実測キャリブレーションオフセット・双方向オフセット正規化・
    // HW_ERRORクリア)を行い、それ以外のservo_typeは基底クラスの実装にそのまま委譲する。
    void begin(int servo_pin_x, int16_t start_degree_x, int16_t offset_x,
               int servo_pin_y, int16_t start_degree_y, int16_t offset_y,
               ServoType servo_type = PWM, m5::I2C_Class* i2c = nullptr);

    void moveToOrigin();
    void moveTo(int degX, int degY);
    void moveTo(int degX, int degY, uint32_t millis_for_move);

    // サーボ診断ユーティリティ（トルク/エラー状態の確認・復旧、状態ダンプ）
    int32_t readTorqueEnable(uint8_t id);
    int32_t readHwErrorStatus(uint8_t id);
    bool ensureTorqueOn(uint8_t id);
    void releaseTorque(uint8_t id);
    void dumpState(const char* tag);
};

#endif  //_SERVO_CUSTOM_H
