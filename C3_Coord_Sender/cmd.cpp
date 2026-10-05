/* =====================================================================
   cmd.cpp   串口命令实现
   ---------------------------------------------------------------------
   命令一览（串口监视器 115200，行尾选"换行"）：
     help            打印这份清单
     wet             看当前水感状态
     wet on / off    强制"入水" / 解除（演示用，不用碰电极）
     post            看姿态三判据的详情
     sim post on/off 强制姿态条件通过 / 解除（按键模拟不出"失重→冲击"）
     reset           复位激活状态（等同长按 BOOT 键 5 秒）
     trig            看激活状态
   ===================================================================== */

#include "cmd.h"
#include "water.h"
#include "posture.h"
#include "trigger.h"

#if SRC_MODE == 3
#error "SRC_MODE 3（串口输入坐标）和串口命令都要读串口，不能同时开：把 SRC_MODE 改成 0 或 2"
#endif

static char   s_buf[32];
static size_t s_len = 0;

void cmdBegin() {
  DBG.println("串口命令：help 看全部；wet on/off 模拟入水；sim post on/off 模拟姿态；reset 复位");
}

static void printHelp() {
  DBG.println("---------- 串口命令 ----------");
  DBG.println("  help            打印这份清单");
  DBG.println("  wet             看当前水感状态");
  DBG.println("  wet on / off    强制“入水” / 解除（演示用，不用碰电极）");
  DBG.println("  post            看姿态三条判据的详情");
  DBG.println("  sim post on/off 强制姿态条件通过 / 解除");
  DBG.println("  reset           复位激活状态（等同长按 BOOT 键 5 秒）");
  DBG.println("  trig            看激活状态");
  DBG.println("------------------------------");
}

static void handle(const char* c) {
  if (!strcmp(c, "help")) { printHelp(); return; }

  /* ---------------- 水感 ---------------- */
  if (!strcmp(c, "wet"))        { waterPrintStatus(); return; }
#if WATER_SIM_CMD
  if (!strcmp(c, "wet on")) {
    waterSetSim(true);
    DBG.println("[水感] 已强制置为“入水”（模拟）");
    return;
  }
  if (!strcmp(c, "wet off")) {
    waterSetSim(false);
    DBG.println("[水感] 已解除强制，恢复真实采样");
    return;
  }
#endif

  /* ---------------- 姿态 ---------------- */
  if (!strcmp(c, "post")) {
    DBG.printf("[姿态] 就绪=%d  合加速度 %.3f g  俯仰 %.1f°  横滚 %.1f°\n",
               postureReady() ? 1 : 0, postureAccelG(),
               posturePitchDeg(), postureRollDeg());
    DBG.printf("[姿态] 相对基准 %.1f°  (阈值 %.0f°)  运动 %.3f g  (阈值 %.2f g)\n",
               postureTiltFromBase(), POSTURE_TILT_MIN_DEG,
               postureMotionLevel(), POSTURE_CALM_MAX_G);
    float hz = postureSwayHz();
    if (hz < 0.0f) DBG.println("[姿态] 摇摆频率 --（8 秒窗口还没填满）");
    else           DBG.printf("[姿态] 摇摆频率 %.2f Hz（走路 1~2Hz，波浪 0.1~0.5Hz）\n", hz);
    DBG.printf("[姿态] ①入水冲击 %s   ②已翻转 %s   ③平静 %s   综合 %s\n",
               postureImpactRecent() ? "有" : "无",
               postureFlipped()      ? "是" : "否",
               postureCalm()         ? "是" : "否",
               postureLooksLikeFloating() ? "符合漂浮" : "不符合");
    return;
  }
#if TRIGGER_SIM_ENABLE
  if (!strcmp(c, "sim post on"))  { postureSetSim(true);  DBG.println("[姿态] 已强制姿态条件通过（模拟）"); return; }
  if (!strcmp(c, "sim post off")) { postureSetSim(false); DBG.println("[姿态] 已解除姿态强制"); return; }
#endif

  /* ---------------- 激活状态 ---------------- */
  if (!strcmp(c, "reset")) { triggerReset(); return; }
  if (!strcmp(c, "trig"))  { triggerPrintReport(); return; }

  DBG.printf("不认识这条命令：%s（敲 help 看清单）\n", c);
}

void cmdPoll() {
  while (DBG.available()) {
    char ch = (char)DBG.read();
    if (ch == '\n' || ch == '\r') {
      if (s_len) {
        s_buf[s_len] = '\0';
        handle(s_buf);
        s_len = 0;
      }
    } else if (s_len < sizeof(s_buf) - 1) {
      s_buf[s_len++] = (char)tolower((unsigned char)ch);   // 统一转小写
    } else {
      s_len = 0;                                           // 太长就丢弃
    }
  }
}
