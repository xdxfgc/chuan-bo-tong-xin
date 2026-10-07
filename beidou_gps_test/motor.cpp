/* =====================================================================
   motor.cpp   TB6612 电机驱动实现
   ---------------------------------------------------------------------
   AIN1/AIN2 的真值表（TB6612 数据手册）：
     AIN1=H AIN2=L  → 正转（前进）
     AIN1=L AIN2=H  → 反转（倒车）
     AIN1=L AIN2=L  → 滑行（电机自由转，慢慢停）
     AIN1=H AIN2=H  → 刹车（电机绕组被短接，停得快）
   PWMA 给 0~255 的占空比调速（LEDC，5kHz）。
   ===================================================================== */

#include "motor.h"
#include <math.h>

#if MOTOR_ENABLE

static bool          s_ready  = false;
static float         s_target = 0.0f;    // 目标油门 -1 ~ +1
static float         s_mag    = 0.0f;    // 当前输出大小 0 ~ 1
static int8_t        s_dir    = 0;       // 当前方向：0 停 / +1 前进 / -1 倒车
static bool          s_brake  = false;   // 刹车状态（与滑行互斥）
static unsigned long s_lastMs = 0;       // 上一次斜坡计算时刻
static unsigned long s_zeroMs = 0;       // 刚降到 0 的时刻（换向等待用）

/* 把方向和转速真正写到引脚下 */
static void applyOutput() {
  if (s_brake) {
    digitalWrite(MOTOR_AIN1_PIN, HIGH);
    digitalWrite(MOTOR_AIN2_PIN, HIGH);
    ledcWrite(MOTOR_PWM_PIN, 255);
    return;
  }
  if (s_dir > 0) {
    digitalWrite(MOTOR_AIN1_PIN, HIGH);
    digitalWrite(MOTOR_AIN2_PIN, LOW);
  } else if (s_dir < 0) {
    digitalWrite(MOTOR_AIN1_PIN, LOW);
    digitalWrite(MOTOR_AIN2_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_AIN1_PIN, LOW);      // 滑行
    digitalWrite(MOTOR_AIN2_PIN, LOW);
  }

  int duty = (int)(s_mag * 255.0f + 0.5f);
  if (duty < 0)   duty = 0;
  if (duty > 255) duty = 255;
  ledcWrite(MOTOR_PWM_PIN, duty);
}

void motorBegin() {
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN,  OUTPUT);

  /* ESP32 core 3.x 的 LEDC 写法：按引脚绑定 */
  ledcAttach(MOTOR_PWM_PIN, MOTOR_PWM_HZ, MOTOR_PWM_BITS);

  s_target = 0.0f; s_mag = 0.0f; s_dir = 0; s_brake = false;
  s_lastMs = millis();
  applyOutput();
  s_ready = true;

  Serial.printf("电机驱动已就绪：TB6612 AIN1=GPIO%d AIN2=GPIO%d PWMA=GPIO%d（%lu Hz）\n",
                MOTOR_AIN1_PIN, MOTOR_AIN2_PIN, MOTOR_PWM_PIN,
                (unsigned long)MOTOR_PWM_HZ);
  Serial.println("电机已停止。串口输入 motor 0.5 前进、motor -0.3 倒车、motor stop 停车");
}

void motorUpdate() {
  if (!s_ready) return;

  unsigned long now = millis();
  float dt = (float)(now - s_lastMs) / 1000.0f;
  if (dt < 0.01f) return;                   // 10ms 以内不重复算，省 CPU
  s_lastMs = now;

  /* 目标油门 → 方向 + 大小（死区内一律当停车） */
  float tgt = s_target;
  if (fabsf(tgt) < MOTOR_DEADBAND) tgt = 0.0f;
  int8_t tdir = (tgt > 0.0f) ? 1 : (tgt < 0.0f ? -1 : 0);
  float  tmag = fabsf(tgt);

  /* 一旦给了有效油门就取消刹车 */
  if (tdir != 0) s_brake = false;
  if (s_brake) { applyOutput(); return; }

  /* 换向保护：原来在转、现在要反方向 → 先当停车处理，降速、停稳、再反向 */
  bool reversing = (tdir != 0 && s_dir != 0 && tdir != s_dir);
  if (reversing) { tdir = 0; tmag = 0.0f; }

  /* 斜坡：软启动 / 软停 */
  float step = MOTOR_RAMP_PS * dt;
  if      (tmag > s_mag + step) s_mag += step;
  else if (tmag < s_mag - step) s_mag -= step;
  else                          s_mag  = tmag;

  /* 方向切换 */
  if (tdir != s_dir) {
    if (s_dir != 0) {
      /* 原来在转：等转速降到 0，再等 MOTOR_REV_GAP_MS 才允许换向 */
      if (s_mag <= 0.0f) {
        if (s_zeroMs == 0) s_zeroMs = now;
        if (now - s_zeroMs >= MOTOR_REV_GAP_MS) {
          s_dir    = tdir;
          s_zeroMs = 0;
        }
      }
    } else {
      s_dir    = tdir;              // 本来是停着的，直接起步
      s_zeroMs = 0;
    }
  } else {
    s_zeroMs = 0;
  }

  applyOutput();
}

void motorSetThrottle(float t) {
  if (t >  1.0f) t =  1.0f;
  if (t < -1.0f) t = -1.0f;
  s_target = t;
}

void motorStop() {
  s_target = 0.0f;
  s_brake  = false;
}

void motorBrake() {
  s_target = 0.0f;
  s_mag    = 0.0f;
  s_dir    = 0;
  s_zeroMs = 0;
  s_brake  = true;
  applyOutput();
}

void motorEmergencyStop() {
  s_target = 0.0f;
  motorBrake();
  Serial.println("[电机] 急停");
}

bool  motorReady()  { return s_ready; }
float motorTarget() { return s_target; }
float motorOutput() { return s_brake ? 0.0f : s_mag; }

const char* motorStateText() {
  static char b[28];
  if (s_brake) return "刹车";
  if (s_dir > 0) {
    snprintf(b, sizeof(b), "前进 %.0f%%", s_mag * 100.0f);
    return b;
  }
  if (s_dir < 0) {
    snprintf(b, sizeof(b), "倒车 %.0f%%", s_mag * 100.0f);
    return b;
  }
  return (s_mag > 0.0f) ? "降速中" : "滑行";
}

/* 串口命令：
     motor            看状态
     motor 0.5        前进 50%（-1 ~ +1）
     motor -0.3       倒车 30%
     motor stop       滑行停
     motor brake      刹车（停得快）
     motor estop      急停（清零 + 刹车） */
String motorCmd(const String& arg) {
  String a = arg;
  a.trim();
  a.toLowerCase();

  if (a.length() == 0) {
    char b[128];
    snprintf(b, sizeof(b), "[电机] %s   目标 %.2f   输出 %.2f   引脚 AIN1=%d AIN2=%d PWM=%d",
             motorStateText(), s_target, motorOutput(),
             MOTOR_AIN1_PIN, MOTOR_AIN2_PIN, MOTOR_PWM_PIN);
    return String(b) + "\n用法：motor 0.5 前进 / motor -0.3 倒车 / motor stop / motor brake";
  }

  if (a == "stop" || a == "stop1") {
    motorStop();
    return String("[电机] 滑行停");
  }
  if (a == "brake") {
    motorBrake();
    return String("[电机] 刹车");
  }
  if (a == "estop" || a == "x") {
    motorEmergencyStop();
    return String("[电机] 急停（油门清零 + 刹车）");
  }

  float t = a.toFloat();
  if (t == 0.0f && a != "0" && !a.startsWith("0.") && !a.startsWith("-0."))
    return String("[电机] 看不懂这个参数。用法：motor 0.5 / motor -0.3 / motor stop");

  motorSetThrottle(t);
  char b[64];
  snprintf(b, sizeof(b), "[电机] 目标油门 %.2f（软启动，约 %.1f 秒到全速）",
           t, 1.0f / MOTOR_RAMP_PS);
  return String(b);
}

#else   /* MOTOR_ENABLE == 0：全部空实现，不占代码 */

void  motorBegin() {}
void  motorUpdate() {}
void  motorSetThrottle(float) {}
void  motorStop() {}
void  motorBrake() {}
void  motorEmergencyStop() {}
bool  motorReady()  { return false; }
float motorTarget() { return 0.0f; }
float motorOutput() { return 0.0f; }
const char* motorStateText() { return "未启用"; }
String motorCmd(const String&) { return String("[电机] 本功能未启用"); }

#endif
