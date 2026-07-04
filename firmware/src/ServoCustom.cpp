#include "ServoCustom.h"

// stackchan-arduino Stackchan_servo.cpp の convertDYNIXELXL330_RT() と同じ変換式。
// libdeps側はfile-local staticのため直接呼べず、ここに複製している。
// (degree -360..720 の domain を tick -4095..8191 にマップする、EXTENDED_POSITION用の変換)
static long convertDynamixelRT(int16_t degree) {
  return map(degree, -360, 720, -4095, 8191);
}

// convertDynamixelRT() の逆関数。生tick値から実測角度(度)を求める。
static float ticksToDegree(float ticks) {
  return (ticks + 4095.0f) * 1080.0f / 12286.0f - 360.0f;
}

// 可動域ソフトクランプ（2026-07-04実測キャリブレーション値）
// 正面比 Y: -11.6°〜+12.7°、X: 実測上は無制限だが安全マージンとして±45°に制限

static const int TILT_MIN_DEG = -11;
static const int TILT_MAX_DEG = 12;
static const int PAN_MIN_DEG = -45;
static const int PAN_MAX_DEG = 45;

static int clampAxis(int deg, int lo, int hi, const char* axisName) {
  if (deg < lo || deg > hi) {
    Serial.printf("[ServoClamp] %s deg=%d はクランプ範囲[%d,%d]外のため制限します\n", axisName, deg, lo, hi);
    return deg < lo ? lo : hi;
  }
  return deg;
}

void ServoCustom::begin(int servo_pin_x, int16_t start_degree_x, int16_t offset_x,
                         int servo_pin_y, int16_t start_degree_y, int16_t offset_y,
                         ServoType servo_type, m5::I2C_Class* i2c) {
  if (servo_type != RT_DYN_XL330) {
    // RT_DYN_XL330以外は既存の基底クラス実装をそのまま使う
    StackchanSERVO::begin(servo_pin_x, start_degree_x, offset_x,
                           servo_pin_y, start_degree_y, offset_y,
                           servo_type, i2c);
    return;
  }

  _init_param.servo[AXIS_X].pin          = servo_pin_x;
  _init_param.servo[AXIS_X].start_degree = start_degree_x;
  _init_param.servo[AXIS_X].offset       = offset_x;
  _init_param.servo[AXIS_Y].pin          = servo_pin_y;
  _init_param.servo[AXIS_Y].start_degree = start_degree_y;
  _init_param.servo[AXIS_Y].offset       = offset_y;
  _servo_type = servo_type;
  _i2c = i2c;

  Serial2.begin(1000000, SERIAL_8N1, _init_param.servo[AXIS_X].pin, _init_param.servo[AXIS_Y].pin);
  _dxl = Dynamixel2Arduino(Serial2);
  _dxl.begin(1000000);
  _dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (uint8_t id = AXIS_X + 1; id <= AXIS_Y + 1; id++) {
    bool pingOk = false;
    for (int retry = 0; retry < 3 && !pingOk; retry++) {
      if (retry > 0) delay(50);
      pingOk = _dxl.ping(id);
    }
    Serial.printf("[ServoInit] ping(%d) -> %s\n", id, pingOk ? "OK" : "FAIL");

    int32_t hwErr = readHwErrorStatus(id);
    if (hwErr != 0) {
      Serial.printf("[ServoInit] id=%d HARDWARE_ERROR_STATUS=0x%02X -> reboot\n", id, (unsigned int)hwErr);
      _dxl.reboot(id);
      delay(500);
    }

    _dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
    _dxl.writeControlTableItem(DRIVE_MODE, id, 4);  // Velocityのパラメータを移動時間(msec)で指定するモードに変更
    _dxl.writeControlTableItem(PROFILE_VELOCITY, id, 1000);
  }
  delay(100);

  // オフセット正規化: 電源投入時の巻き数(ラウンド)状態に関わらず、
  // 起動時ゴールが常に現在位置から±180°以内に収まるよう offset を選び直す。
  // (旧ロジックの「乖離していれば常に+360を足す」一方向ヒューリスティックを、
  //  最も近い360°の倍数を選ぶ双方向補正に置き換える)
  float presentDeg[2];
  for (uint8_t axis = AXIS_X; axis <= AXIS_Y; axis++) {
    uint8_t id = axis + 1;
    float presentTicks = _dxl.getPresentPosition(id);
    presentDeg[axis] = ticksToDegree(presentTicks);
    int16_t startDeg = _init_param.servo[axis].start_degree;
    int normalizedOffset = (int)roundf((presentDeg[axis] - startDeg) / 360.0f) * 360;
    _init_param.servo[axis].offset += normalizedOffset;
    Serial.printf("[ServoInit] id=%d presentTicks=%.1f presentDeg=%.1f startDeg=%d -> offset=%d\n",
                  id, presentTicks, presentDeg[axis], startDeg, _init_param.servo[axis].offset);
  }

  _dxl.torqueOn(AXIS_X + 1);
  delay(100);
  _dxl.torqueOn(AXIS_Y + 1);
  delay(100);

  // 安全ガード: 設定ミス等で原点が現在位置から大きく離れている場合は動かさない
  // (機械限界への激突防止。トルクはONにするが、原点への移動指令は出さない)
  for (uint8_t axis = AXIS_X; axis <= AXIS_Y; axis++) {
    uint8_t id = axis + 1;
    int16_t goalDeg = _init_param.servo[axis].start_degree + _init_param.servo[axis].offset;
    int32_t goalTicks = convertDynamixelRT(goalDeg);
    float diffDeg = fabsf(ticksToDegree(goalTicks) - presentDeg[axis]);
    if (diffDeg > 45.0f) {
      Serial.printf("[ServoInit] ERROR: id=%d origin is %.1f deg away from present. Skip homing. Check yaml center settings.\n", id, diffDeg);
      continue;
    }
    _dxl.setGoalPosition(id, goalTicks);
    Serial.printf("[ServoInit] goalPosition id=%d -> %d\n", id, goalTicks);
  }

  _last_degree_x = _init_param.servo[AXIS_X].start_degree;
  _last_degree_y = _init_param.servo[AXIS_Y].start_degree;
}

void ServoCustom::moveToOrigin(){
  moveXY(_init_param.servo[AXIS_X].start_degree, _init_param.servo[AXIS_Y].start_degree, 1000);
}

void ServoCustom::moveTo(int degX, int degY){
  degX = clampAxis(degX, PAN_MIN_DEG, PAN_MAX_DEG, "X");
  degY = clampAxis(degY, TILT_MIN_DEG, TILT_MAX_DEG, "Y");
  moveXY(_init_param.servo[AXIS_X].start_degree + degX, _init_param.servo[AXIS_Y].start_degree + degY, 1000);
}

void ServoCustom::moveTo(int degX, int degY, uint32_t millis_for_move){
  degX = clampAxis(degX, PAN_MIN_DEG, PAN_MAX_DEG, "X");
  degY = clampAxis(degY, TILT_MIN_DEG, TILT_MAX_DEG, "Y");
  moveXY(_init_param.servo[AXIS_X].start_degree + degX, _init_param.servo[AXIS_Y].start_degree + degY, millis_for_move);
}

// サーボ診断ユーティリティ（トルク/エラー状態の確認・復旧、状態ダンプ）
int32_t ServoCustom::readTorqueEnable(uint8_t id){
  return _dxl.readControlTableItem(TORQUE_ENABLE, id);
}

int32_t ServoCustom::readHwErrorStatus(uint8_t id){
  return _dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id);
}

bool ServoCustom::ensureTorqueOn(uint8_t id){
  int32_t hwErr = readHwErrorStatus(id);
  if (hwErr != 0) {
    Serial.printf("[ServoDiag] id=%d HARDWARE_ERROR_STATUS=0x%02X -> reboot\n", id, (unsigned int)hwErr);
    _dxl.reboot(id);
    delay(500);
    // reboot でRAM上のレジスタは初期化される(PROFILE_VELOCITY=0=最高速になる)ため再設定する。
    // DRIVE_MODEはEEPROM保持のため再設定不要。
    _dxl.writeControlTableItem(PROFILE_VELOCITY, id, 1000);
  }
  if (readTorqueEnable(id) != 0) {
    return true;
  }
  return _dxl.torqueOn(id);
}

void ServoCustom::releaseTorque(uint8_t id){
  _dxl.torqueOff(id);
}

void ServoCustom::dumpState(const char* tag){
  for (uint8_t axis = AXIS_X; axis <= AXIS_Y; axis++) {
    uint8_t id = axis + 1;
    float pos = getPosition(id);
    int32_t torque = readTorqueEnable(id);
    int32_t hwErr = readHwErrorStatus(id);
    Serial.printf("[ServoDump:%s] id=%d presentPos=%.1f TORQUE_ENABLE=%ld HARDWARE_ERROR_STATUS=0x%02lX start_degree=%d offset=%d\n",
                  tag, id, pos, (long)torque, (unsigned long)hwErr,
                  _init_param.servo[axis].start_degree, _init_param.servo[axis].offset);
  }
}
