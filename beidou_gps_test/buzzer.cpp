/* =====================================================================
   buzzer.cpp   蜂鸣器实现
   ---------------------------------------------------------------------
   用非阻塞状态机按“节奏表”驱动引脚，全程不 delay，不影响采样和网页。

   自动恢复的逻辑：
     用户关掉蜂鸣器时记下当时的告警等级当基准。之后
       · 告警全部解除（等级回到 0）→ 自动恢复，下次有新告警能正常响
       · 出现比基准更高的告警等级 → 自动恢复并响（情况恶化必须提醒）
   这样“随时关掉”和“遇到情况自己启动”两件事就不冲突了。
   ===================================================================== */

#include "buzzer.h"
#include "beacon.h"
#include "berth.h"
#include "gnss.h"
#include "anchor.h"

#if BUZZER_ENABLE

struct Step { uint16_t ms; uint8_t on; };

// 三短声急促循环（人员落水）
static const Step PAT3[] = {{100,1},{120,0},{100,1},{120,0},{100,1},{900,0}};
// 两短声循环（靠泊严重 / 信标失联）
static const Step PAT2[] = {{200,1},{150,0},{200,1},{1200,0}};
// 单短声，每 2 秒（提醒级）
static const Step PAT1[] = {{120,1},{1880,0}};

static const Step* s_pat    = nullptr;
static uint8_t     s_patLen = 0;
static uint8_t     s_step   = 0;
static unsigned long s_stepMs = 0;

static int  s_level   = 0;
static bool s_userOn  = true;    // 用户开关，默认开
static int  s_baseLv  = 0;       // 关掉时的告警等级基准

/* ---------------- 汇总当前告警等级 ---------------- */

static int currentLevel() {
  // 3 级：人员落水（需要人工确认才解除）
  if (beaconAlarmActive()) return 3;

  // 2 级：靠泊严重告警、信标失联
  uint8_t bc = berthAlarmCode();
  if (bc == 0x02 || bc == 0x03) return 2;
  if (anchorAlarmCode() == 0x22) return 2;      // 走锚（严重级）
  if (beaconEverLinked() && !beaconLinkUp()) return 2;

  // 1 级：靠泊提醒、疑似走锚、定位失锁
  if (bc == 0x01) return 1;
  if (anchorAlarmCode() == 0x21) return 1;      // 疑似走锚（提醒级）
  if (gpsAlarmCode() == 0x06) return 1;

  return 0;
}

// 按模块的触发极性输出：响 / 停
static inline void buzzerWrite(bool on) {
#if BUZZER_ACTIVE_LOW
  digitalWrite(BUZZER_PIN, on ? LOW : HIGH);
#else
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
#endif
}

static void applyStep() { buzzerWrite(s_pat[s_step].on != 0); }

static void setPattern(int level) {
  switch (level) {
    case 3:  s_pat = PAT3; s_patLen = sizeof(PAT3) / sizeof(Step); break;
    case 2:  s_pat = PAT2; s_patLen = sizeof(PAT2) / sizeof(Step); break;
    default: s_pat = PAT1; s_patLen = sizeof(PAT1) / sizeof(Step); break;
  }
  s_step   = 0;
  s_stepMs = millis();
  applyStep();
}

static void silence() {
  s_pat   = nullptr;
  s_level = 0;
  buzzerWrite(false);
}

/* ---------------- 对外接口 ---------------- */

void buzzerBegin() {
  // 先把输出寄存器设成“不响”的电平，再切成输出，
  // 否则低电平触发的模块（MH-FMD）上电瞬间会“滴”一下
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? HIGH : LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerWrite(false);
  s_userOn = true;
  s_baseLv = 0;
  Serial.printf("蜂鸣器已就绪：GPIO%d（%s触发）\n", BUZZER_PIN,
                BUZZER_ACTIVE_LOW ? "低电平" : "高电平");
}

void buzzerUpdate() {
  int lv = currentLevel();

  // 用户关掉了蜂鸣器：告警解除或等级升高时自动恢复
  if (!s_userOn) {
    if (lv == 0 || lv > s_baseLv) {
      s_userOn = true;
      Serial.println("[蜂鸣器] 已自动恢复（告警解除或等级升高）");
    } else {
      silence();
      return;
    }
  }

  if (lv == 0) { silence(); return; }

  if (s_level != lv || s_pat == nullptr) {
    s_level = lv;
    setPattern(lv);
    return;
  }

  unsigned long now = millis();
  if (now - s_stepMs >= s_pat[s_step].ms) {
    s_stepMs = now;
    s_step = (s_step + 1) % s_patLen;
    applyStep();
  }
}

void buzzerSetUserOn(bool on) {
  if (on) {
    s_userOn = true;
    Serial.println("[蜂鸣器] 已打开");
  } else {
    s_userOn = false;
    s_baseLv = currentLevel();       // 记下关掉时的等级当基准
    silence();
    Serial.printf("[蜂鸣器] 已关闭（当前告警等级 %d，告警解除或升级后会自动恢复）\n", s_baseLv);
  }
}

bool buzzerUserOn()  { return s_userOn; }
bool buzzerSounding(){ return s_userOn && s_pat != nullptr; }
int  buzzerLevel()   { return currentLevel(); }

String buzzerLevelText() {
  switch (currentLevel()) {
    case 3:  return "人员落水";
    case 2:  return "严重告警";
    case 1:  return "提醒级告警";
    default: return "无告警";
  }
}

String buzzerStateText() {
  if (!s_userOn)  return "已关闭";
  if (currentLevel() == 0) return "待机";
  return buzzerLevelText();
}

void buzzerAcknowledge() {
  beaconAcknowledge();
  Serial.println("[蜂鸣器] 已确认落水告警，蜂鸣器停止");
}

#else   /* BUZZER_ENABLE == 0：空实现 */

void buzzerBegin() {}
void buzzerUpdate() {}
void buzzerSetUserOn(bool) {}
bool buzzerUserOn() { return false; }
bool buzzerSounding() { return false; }
int  buzzerLevel() { return 0; }
String buzzerLevelText() { return "未启用"; }
String buzzerStateText() { return "未启用"; }
void buzzerAcknowledge() {}

#endif
