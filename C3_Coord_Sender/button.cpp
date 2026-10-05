/* =====================================================================
   button.cpp   板载 BOOT 键实现（非阻塞消抖 + 长按判定）
   ===================================================================== */

#include "button.h"

#define BTN_DEBOUNCE_MS 30

static bool          s_stable      = false;   // 消抖之后的按键状态
static bool          s_lastRaw     = false;
static unsigned long s_rawChangeMs = 0;
static unsigned long s_pressMs     = 0;
static bool          s_fired       = false;   // 这一次长按已经触发过了
static bool          s_resetEvent  = false;

// 读原始电平：BTN_ACTIVE_LOW = 1 表示按下是低电平
static bool readRaw() {
#if BTN_ACTIVE_LOW
  return digitalRead(BTN_PIN) == LOW;
#else
  return digitalRead(BTN_PIN) == HIGH;
#endif
}

void buttonBegin() {
  pinMode(BTN_PIN, BTN_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  s_stable      = readRaw();
  s_lastRaw     = s_stable;
  s_rawChangeMs = millis();
  s_fired       = false;
  s_resetEvent  = false;

  DBG.printf("复位按键：BOOT 键（GPIO%d），长按 %lu 秒退出激活状态\n",
             BTN_PIN, (unsigned long)(BTN_RESET_MS / 1000));
  if (s_stable) DBG.println("          ⚠ 上电时按键是按住状态，松开才会开始工作");
}

void buttonUpdate() {
  bool          raw = readRaw();
  unsigned long now = millis();

  /* 消抖：原始电平变了先记时间，稳定 BTN_DEBOUNCE_MS 之后才算数 */
  if (raw != s_lastRaw) {
    s_lastRaw     = raw;
    s_rawChangeMs = now;
  }

  if ((now - s_rawChangeMs) >= BTN_DEBOUNCE_MS && raw != s_stable) {
    s_stable = raw;
    if (s_stable) {                 // 刚按下
      s_pressMs = now;
      s_fired   = false;
    }
  }

  /* 长按到时间就触发一次（按住不放也只触发一次，松开后重新计时） */
  if (s_stable && !s_fired && (now - s_pressMs >= BTN_RESET_MS)) {
    s_fired      = true;
    s_resetEvent = true;
    DBG.printf("[按键] BOOT 键已按满 %lu 秒\n", (unsigned long)(BTN_RESET_MS / 1000));
  }
}

bool buttonPressed()  { return s_stable; }

unsigned long buttonHoldMs() {
  if (!s_stable) return 0;
  return millis() - s_pressMs;
}

bool buttonResetEvent() {
  bool e = s_resetEvent;
  s_resetEvent = false;          // 读一次就清
  return e;
}
