/* =====================================================================
   trigger.cpp   三重确认状态机实现
   ---------------------------------------------------------------------
   状态：
     IDLE     平时
     WAIT     水感已确认，等姿态条件成立（最多等 TRIGGER_WAIT_MS）
     ACTIVE   三重确认通过，一直保持到人工复位

   为什么要有 WAIT 这一段：
     水感确认（连续湿 2 秒）之后，姿态那三条判据还要各自成立。
     三条判据都能在 1 秒内算出来，所以整体激活时间在 3 秒以内，
     符合文档"20 次入水试验激活时间不超过 3 秒"。
     但也不能无限等 —— 等太久说明是"沾了点水但人还在船上"，要放弃。
   ===================================================================== */

#include "trigger.h"
#include "water.h"
#include "posture.h"

static bool          s_active     = false;
static unsigned long s_waitSinceMs = 0;     // WAIT 状态开始计时（0 = 不在 WAIT）
static unsigned      s_count      = 0;
static unsigned long s_lastActiveMs = 0;

void triggerBegin() {
  s_active       = false;
  s_waitSinceMs  = 0;
  s_count        = 0;
  s_lastActiveMs = 0;
}

void triggerUpdate() {
  if (s_active) return;                     // 已激活：保持，等人工复位

  /* 第一重：水感。要有水，而且要连续湿够 WATER_CONFIRM_MS（持续时间）。 */
  bool waterOK = waterIsWet() && (waterWetMs() >= WATER_CONFIRM_MS);
  if (!waterOK) {
    if (s_waitSinceMs) {
      s_waitSinceMs = 0;                    // 还没等到姿态条件水就干了
      DBG.println("[激活] 水感中断，本次判定取消");
    }
    return;
  }

  if (s_waitSinceMs == 0) {
    s_waitSinceMs = millis();
    DBG.println("[激活] 水感已确认，正在等姿态条件…");
  }

  /* 第二重：姿态。三条子判据都满足才算"漂在水面"。 */
  if (!postureLooksLikeFloating()) {
    if (millis() - s_waitSinceMs > TRIGGER_WAIT_MS) {
      s_waitSinceMs = 0;
      DBG.println("[激活] 等姿态条件超时，判定为误触发，放弃本次");
    }
    return;
  }

  /* 三重都过了 */
  s_active       = true;
  s_waitSinceMs  = 0;
  s_lastActiveMs = millis();
  s_count++;
  DBG.printf("[激活] ★三重确认通过，落水已确认（第 %u 次）—— 开始上报\n", s_count);
  DBG.println("[激活] 要退出请长按板载 BOOT 键 5 秒，或串口敲 reset");
}

bool triggerActive() { return s_active; }

void triggerReset() {
  if (!s_active) {
    DBG.println("[激活] 当前未处于激活状态，无需复位");
    return;
  }
  s_active      = false;
  s_waitSinceMs = 0;
  DBG.println("[激活] 已复位，退出激活状态（要再触发需要重新满足三重确认）");
}

const char* triggerStateText() {
  if (s_active)          return "已激活（落水确认）";
  if (s_waitSinceMs)     return "等待姿态条件";
  return "待机";
}

unsigned triggerCount() { return s_count; }

void triggerPrintReport() {
  char buf[48];
  if (s_active) {
    snprintf(buf, sizeof(buf), "已激活 %lu 秒",
             (millis() - s_lastActiveMs) / 1000UL);
  } else {
    snprintf(buf, sizeof(buf), "待机");
  }
  DBG.printf("[激活] 状态 %-14s  累计激活 %u 次\n", buf, s_count);
}
