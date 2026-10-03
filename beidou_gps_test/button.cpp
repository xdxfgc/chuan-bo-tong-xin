/* ============================================================
 *  button.cpp —— 板载按键模块（实现）
 *
 *  状态机很简单：记录真实电平，变化后等 BTN_DEBOUNCE_MS 再确认；
 *  抬手时按住不足 BTN_LONG_MS 算短按；按住到 BTN_LONG_MS 就地触发长按
 *  （不等抬手，这样长按的手感是"到点就有反馈"）。
 *  全程不阻塞、不用 delay，标定那 15 秒里采样照常跑。
 * ============================================================ */

#include "button.h"
#include "config.h"
#include "mag.h"

/* 按键动作：短按把当前朝向设为出发点，长按开始标定 */
static void buttonShortAction() {
  String msg;
  magZeroHere(msg);
  Serial.printf("[按键] 短按：%s\n", msg.c_str());
}

static void buttonLongAction() {
  String msg;
  magCalAutoStart(BTN_CAL_SECONDS, msg);
  Serial.printf("[按键] 长按：%s\n", msg.c_str());
}

#if BTN_ENABLE

static bool          sStable   = false;   // 消抖后的状态，true=按着
static bool          sRawLast  = false;   // 上一次读到的原始状态
static bool          sLongDone = false;   // 这一轮按住是否已经报过长按
static unsigned long sChangeMs = 0;       // 原始电平最近一次变化的时刻
static unsigned long sPressMs  = 0;       // 确认按下的时刻

static inline bool readRaw() {
#if BTN_ACTIVE_LOW
  return digitalRead(BTN_PIN) == LOW;
#else
  return digitalRead(BTN_PIN) == HIGH;
#endif
}

void buttonBegin() {
#if BTN_ACTIVE_LOW
  pinMode(BTN_PIN, INPUT_PULLUP);
#else
  pinMode(BTN_PIN, INPUT);
#endif
  sRawLast  = readRaw();
  sStable   = sRawLast;
  sChangeMs = millis();
}

void buttonUpdate() {
  bool raw = readRaw();
  unsigned long now = millis();

  if (raw != sRawLast) {                 // 电平抖了，先记时间，等稳定
    sRawLast  = raw;
    sChangeMs = now;
  }
  if (now - sChangeMs < BTN_DEBOUNCE_MS) return;   // 还在消抖窗口里

  if (raw != sStable) {                  // 确认状态真的变了
    sStable = raw;
    if (sStable) {                       // 按下
      sPressMs  = now;
      sLongDone = false;
    } else if (!sLongDone) {             // 抬手，且这轮没触发过长按 → 短按
      buttonShortAction();
    }
  }

  /* 长按：按住到点就地触发一次，不用等抬手 */
  if (sStable && !sLongDone && (now - sPressMs >= BTN_LONG_MS)) {
    sLongDone = true;
    buttonLongAction();
  }
}

bool buttonPressed() { return sStable; }

#else   /* BTN_ENABLE == 0：全部变成空实现，不占代码也不占引脚 */

void buttonBegin()  {}
void buttonUpdate() {}
bool buttonPressed() { return false; }

#endif
