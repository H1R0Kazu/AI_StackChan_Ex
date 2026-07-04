#if defined(REALTIME_API)

#include <Arduino.h>
#include <deque>
#include <SD.h>
#include <SPIFFS.h>
#include "mod/ModManager.h"
#include "RealtimeAiMod.h"
#include <Avatar.h>
#include "Robot.h"
#include "llm/ChatGPT/FunctionCall.h"
#include <WiFiClientSecure.h>
#include "Scheduler.h"
#include "MySchedule.h"
#include "share/SDUtil.h"
#include "driver/HeadTouchSensor.h"

using namespace m5avatar;


/// 外部参照 ///
extern Avatar avatar;
extern bool servo_home;
extern volatile bool servo_random_active;
extern volatile int servo_rand_x;
extern volatile int servo_rand_y;
extern void sw_tone();
extern void alarm_tone();
///////////////



RealtimeAiMod::RealtimeAiMod(bool _isOffline)
  : isOffline{_isOffline}
{
  box_servo.setupBox(80, 120, 80, 80);
  box_stt.setupBox(0, 0, M5.Display.width(), 60);
  box_BtnA.setupBox(0, 100, 40, 60);
  box_BtnC.setupBox(280, 100, 40, 60);

  pRtLLM = (RealtimeLLMBase*)robot->llm;
  pRtLLM->invokeWebSocketLoopTask();

  //servo_home = false;

#if 0
  if(!isOffline){
    //スケジューラ設定
    init_schedule();
  }
#endif
}


void RealtimeAiMod::init(void)
{
  //avatar.setSpeechText("Realtime AI");
  avatar.set_isSubWindowEnable(true);
  pRtLLM->resumeWebSocketLoopTask();
}

void RealtimeAiMod::pause(void)
{
  avatar.set_isSubWindowEnable(false);
  pRtLLM->suspendWebSocketLoopTask();
}


void RealtimeAiMod::update(int page_no)
{

}

void RealtimeAiMod::btnA_pressed(void)
{
#if defined(ARDUINO_M5STACK_ATOMS3R)
  Serial.println("Btn A pressed");
  sw_tone();
  toggleRealtimeRecord();
#endif
}

void RealtimeAiMod::btnB_longPressed(void)
{

}

void RealtimeAiMod::btnC_pressed(void)
{
  static bool isQrDrawing = false;
  if(!isQrDrawing){
    avatar.setSpeechText("");
    String url = String("http://") + WiFi.localIP().toString();
    avatar.updateSubWindowQrcode(url);
    avatar.set_isSubWindowEnable(true);
    isQrDrawing = true;
  }else{
    avatar.set_isSubWindowEnable(false);
    isQrDrawing = false;
  }
}

void RealtimeAiMod::display_touched(int16_t x, int16_t y)
{
  if (box_stt.contain(x, y))
  {
    sw_tone();
    toggleRealtimeRecord();
  }
#ifdef USE_SERVO
  if (box_servo.contain(x, y))
  {
    sw_tone();
    servo_home = !servo_home;
  }
#endif
  if (box_BtnA.contain(x, y))
  {
    //sw_tone();
  }
  if (box_BtnC.contain(x, y))
  {
    btnC_pressed();
  }

}

void RealtimeAiMod::doubleTapped(float ax, float ay, float az)
{
  Serial.printf("Mod double tapped. ax=%.3f ay=%.3f az=%.3f\n", ax, ay, az);
#if defined(ARDUINO_M5STACK_ATOMS3R)
  sw_tone();
  toggleRealtimeRecord();
#endif
}


void RealtimeAiMod::idle(void)
{
#ifdef REALTIME_API_WITH_TTS

  if(robot->asyncPlaying || (pRtLLM->getOutputTextQueueSize() != 0)){
    // 発話中
    pRtLLM->setSpeaking(true);
    servo_random_active = false;
  }
  else{
    // 発話停止中かつキューにテキストがない場合はLLM機能に発話終了を通知
    pRtLLM->setSpeaking(false);
  }

#endif  //REALTIME_API_WITH_TTS

  // Alarm (Function Calling)
  alarmEventHandler();
  updateHeadTouchExpression();

#ifdef USE_SERVO
  // アイドル時のランダムサーボ動作
  {
    static uint32_t lastMoveTime = 0;
    static uint32_t nextMoveInterval = 3000;
    static bool isInRandomMove = false;
    static uint32_t randomMoveEndTime = 0;

    uint32_t now = millis();
    if (!pRtLLM->isSpeaking()) {
      if (isInRandomMove) {
        if (now > randomMoveEndTime) {
          isInRandomMove = false;
          servo_random_active = false;
          lastMoveTime = now;
          nextMoveInterval = random(3000, 6001);
        }
      } else if (now - lastMoveTime > nextMoveInterval) {
        servo_rand_x = random(-20, 21);
        servo_rand_y = random(-10, 11);
        servo_random_active = true;
        isInRandomMove = true;
        randomMoveEndTime = now + random(1000, 2001);
      }
    } else {
      if (isInRandomMove) {
        isInRandomMove = false;
        servo_random_active = false;
      }
    }
  }
#endif

#if 0 
  //スケジューラ処理
  if(!isOffline){
    run_schedule();
  }
#endif

}

void RealtimeAiMod::alarmEventHandler()
{
  if(xAlarmTimer != NULL){
    TickType_t xRemainingTime;

    /* Query the period of the timer that expires. */
    xRemainingTime = xTimerGetExpiryTime( xAlarmTimer ) - xTaskGetTickCount();
    avatarText = "Alarm countdown: " + String(xRemainingTime / 1000);
    avatar.set_isSubWindowEnable(true);
    avatar.updateSubWindowTxt(avatarText, 0, 0, 200, 50);
  }

  if (alarmTimerCallbacked) {
    alarmTimerCallbacked = false;
    avatar.set_isSubWindowEnable(false);
    alarm_tone();
  }

  if (alarmTimerCanceled) {
    alarmTimerCanceled = false;
    avatar.set_isSubWindowEnable(false);
  }

}

void RealtimeAiMod::updateHeadTouchExpression(void)
{
  HeadTouchSensor::Gesture gesture = HeadTouchSensor::update();
  if (HeadTouchSensor::isPetGesture(gesture)) {
    headTouchHappyUntilMs = millis() + 3000;
    headTouchHappyActive = true;
    avatar.setExpression(Expression::Happy);
    Serial.printf("[HeadTouch] pet gesture=%s\n", HeadTouchSensor::gestureName(gesture));
  }

  if (headTouchHappyActive && millis() < headTouchHappyUntilMs) {
    avatar.setExpression(Expression::Happy);
    return;
  }

  if (headTouchHappyActive) {
    headTouchHappyActive = false;
    avatar.setExpression(Expression::Neutral);
  }
}

bool RealtimeAiMod::isBusy(void)
{
  if(pRtLLM->isRealtimeRecording() || pRtLLM->isSpeaking()){
    return true;
  }else{
    return false;
  }
}

void RealtimeAiMod::toggleRealtimeRecord(void)
{
  if(pRtLLM->isRealtimeRecording()){
    pRtLLM->stopRealtimeRecord();
  }else{
    pRtLLM->startRealtimeRecord();
  }
}

#endif //REALTIME_API
