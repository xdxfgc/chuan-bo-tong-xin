/* =====================================================================
   water.cpp   水感检测实现
   ---------------------------------------------------------------------
   原理：
     平时   两个脚都是高阻输入 —— 电极上一点电都没有，零功耗、不腐蚀
     采样   每 WATER_SAMPLE_MS 毫秒做一次，每次两个方向各测一遍：
               方向 1：A 输出低，B 开上拉读
               方向 2：B 输出低，A 开上拉读
            电流方向交替，这就是防止电极电解腐蚀的关键。

   为什么不用一直通电：
     电极长期带着直流电泡在水里，会电解腐蚀，几天就烂了。
     文档表 18 专门写了"间歇通电采样，减少电极电化学腐蚀"，说的就是这件事。
   ===================================================================== */

#include "water.h"

#if WATER_SIM_CMD && SRC_MODE == 3
#error "SRC_MODE 3（串口输入坐标）和 WATER_SIM_CMD 都要读串口，不能同时开：把 WATER_SIM_CMD 改成 0"
#endif

static bool          s_wet         = false;   // 本次采样结果
static unsigned long s_wetSinceMs  = 0;       // 从什么时候开始连续湿
static unsigned long s_lastSampleMs = 0;
static unsigned long s_lastStatusMs = 0;      // 定期状态打印的节流
static uint32_t      s_sampleCount  = 0;      // 累计采样次数（看程序还活着）
static int           s_dirCount     = 0;      // 本次两个方向里导通了几个（0/1/2）
static bool          s_simForced   = false;   // 被串口命令强制成"入水"
static bool          s_firstSample = true;

/* 采一次：drivePin 输出低，sensePin 开内部上拉去读。
   两片电极被水连通时，sensePin 会被 drivePin 拉低。
   内部上拉约 45kΩ，自来水（两片 1cm² 间距 3mm）的电阻是几百欧，
   分压之后远低于阈值，所以不需要外接电阻。                       */
static bool sampleOnce(int drivePin, int sensePin) {
  pinMode(drivePin, OUTPUT);
  digitalWrite(drivePin, LOW);
  pinMode(sensePin, INPUT_PULLUP);
  delayMicroseconds(300);                       // 等电平建立
  bool conductive = (digitalRead(sensePin) == LOW);
  pinMode(sensePin, INPUT);                     // 立刻恢复高阻
  pinMode(drivePin, INPUT);                     // 减少通电时间 = 减少腐蚀
  return conductive;
}

void waterBegin() {
  pinMode(WATER_PIN_A, INPUT);                  // 平时高阻，零功耗
  pinMode(WATER_PIN_B, INPUT);

  s_wet         = false;
  s_wetSinceMs  = millis();
  s_lastSampleMs = 0;
  s_lastStatusMs = 0;
  s_sampleCount  = 0;
  s_dirCount     = 0;
  s_simForced   = false;
  s_firstSample = true;

  DBG.printf("水感检测：电极接 GPIO%d / GPIO%d，每 %lu ms 采样一次\n",
             WATER_PIN_A, WATER_PIN_B, (unsigned long)WATER_SAMPLE_MS);
  DBG.println("          两片不锈钢间距 3~5 毫米，中间不能短路");
  DBG.println("          临时测试：两根杜邦线插这两个脚，线头碰一起就是入水");
#if WATER_SIM_CMD
  DBG.println("          也可以串口敲 wet on / wet off 强制模拟");
#endif
#if SEND_ONLY_WET
  DBG.println("          SEND_ONLY_WET = 1：没入水时信标保持静默，入水才开始上报");
#else
  DBG.println("          SEND_ONLY_WET = 0：不管有没有入水都照常上报（联调用）");
#endif
#if WATER_STATUS_MS > 0
  DBG.printf("          每 %lu ms 打一行状态；不想看就把 WATER_STATUS_MS 设成 0\n",
             (unsigned long)WATER_STATUS_MS);
#endif
}

void waterUpdate() {
  if (!s_firstSample && millis() - s_lastSampleMs < WATER_SAMPLE_MS) return;
  bool first     = s_firstSample;
  s_firstSample  = false;
  s_lastSampleMs = millis();
  s_sampleCount++;

  bool wet;
  int  dirs;
  if (s_simForced) {
    wet = true;                                 // 串口命令强制
    dirs = 2;
  } else {
    /* 两个方向各测一次，任一次导通就算有水。
       不要求两次都导通：淡水的电阻比自来水大，宽松一点更不容易漏报。 */
    bool dir1 = sampleOnce(WATER_PIN_A, WATER_PIN_B);
    bool dir2 = sampleOnce(WATER_PIN_B, WATER_PIN_A);
    dirs = (dir1 ? 1 : 0) + (dir2 ? 1 : 0);
    wet  = (dirs > 0);
  }
  s_dirCount = dirs;

  if (first) {
    /* 开机第一次采样单独报一下。
       不然"没入水"时串口一片安静，分不清是没入水还是程序压根没在采样。 */
    s_wet = wet;
    if (wet) s_wetSinceMs = millis();
    DBG.printf("[水感] 开机初次检测：%s\n", wet ? "导通（有水）" : "干燥（未导通）");
  } else if (wet != s_wet) {
    s_wet = wet;
    if (wet) {
      s_wetSinceMs = millis();
      DBG.println("[水感] 检测到入水");
    } else {
      DBG.println("[水感] 已离水");
    }
  }

#if WATER_STATUS_MS > 0
  /* 定期报状态。没入水的时候串口本来就安静，加这一行才分得清
     "没入水"和"程序没在采样"。                                     */
  if (millis() - s_lastStatusMs >= WATER_STATUS_MS) {
    s_lastStatusMs = millis();
    waterPrintStatus();
  }
#endif
}

bool waterIsWet() { return s_wet; }

unsigned long waterWetMs() {
  if (!s_wet) return 0;
  return millis() - s_wetSinceMs;
}

void waterPrintStatus() {
  /* dirs 是这一轮两个采样方向里导通了几个：
       0 = 干燥
       2 = 两个方向都导通（水/短接线，信号好）
       1 = 只有一个方向导通（水膜残留或信号偏弱，注意一下） */
  const char* quality = (s_dirCount >= 2) ? "两方向都导通"
                      : (s_dirCount == 1) ? "只有单方向导通，信号偏弱"
                                          : "两方向都不导通";
  if (s_simForced) {
    DBG.printf("[水感] 有水（命令强制）  累计采样 %lu 次\n",
               (unsigned long)s_sampleCount);
  } else if (s_wet) {
    DBG.printf("[水感] 有水  %s  已持续 %.1f 秒  累计采样 %lu 次\n",
               quality, waterWetMs() / 1000.0, (unsigned long)s_sampleCount);
  } else {
    DBG.printf("[水感] 没水（干燥）  %s  累计采样 %lu 次\n",
               quality, (unsigned long)s_sampleCount);
  }
}

#if WATER_SIM_CMD
/* ---------------- 串口模拟命令 ----------------
   wet on    强制置为"入水"
   wet off   解除强制，恢复真实采样
   wet       打印当前状态
   演示时信标装进壳子里了，不用去短接引脚。                             */

static char   s_cmd[24];
static size_t s_cmdLen = 0;

void waterPollCommand() {
  while (DBG.available()) {
    char c = (char)DBG.read();
    if (c == '\n' || c == '\r') {
      if (s_cmdLen) {
        s_cmd[s_cmdLen] = '\0';
        if (!strcmp(s_cmd, "wet on")) {
          s_simForced  = true;
          s_wet        = true;
          s_wetSinceMs = millis();
          DBG.println("[水感] 已强制置为“入水”（模拟）");
        } else if (!strcmp(s_cmd, "wet off")) {
          s_simForced = false;
          DBG.println("[水感] 已解除强制，恢复真实采样");
        } else if (!strcmp(s_cmd, "wet")) {
          waterPrintStatus();
        }
        s_cmdLen = 0;
      }
    } else if (s_cmdLen < sizeof(s_cmd) - 1) {
      s_cmd[s_cmdLen++] = (char)tolower((unsigned char)c);   // 统一转小写
    } else {
      s_cmdLen = 0;                                          // 太长就丢弃
    }
  }
}
#else
void waterPollCommand() {}      // 没打开命令就什么都不做
#endif
