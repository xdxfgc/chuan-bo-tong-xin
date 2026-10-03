/* =====================================================================
   cmd.cpp   串口命令实现
   ===================================================================== */

#include "cmd.h"
#include "voice.h"
#include "mag.h"

static char   s_buf[32];
static size_t s_n = 0;

static void printHelp() {
  Serial.println("---------------- 串口命令 ----------------");
  Serial.println("  v0~v16      语音音量（会播一句“你好”试听）");
  Serial.println("  cal [秒]    开始磁力计标定，默认 15 秒（拿起来绕「8」字慢转）");
  Serial.println("  cal stop    提前结束标定");
  Serial.println("  cal reset   清除已保存的标定");
  Serial.println("  zero        把当前船头朝向设为出发点（0°）");
  Serial.println("  decl -5.2   设置磁偏角（真北修正）");
  Serial.println("  mag         立刻打印一行磁力计数据");
  Serial.println("  scan        扫描磁力计所在的 I2C 总线");
  Serial.println("------------------------------------------");
}

void cmdPrintMagLine() {
  Serial.printf("[磁力计] 航向 %.1f 度  相对出发点 %.1f 度  偏差 %+.1f 度  强度 %.1f uT  %s\n",
                magHeadingDeg(), magHeadingRel(), magDeviation(), magFieldUT(),
                magCalibrated() ? "已标定" : "未标定");
}

static void setVolumeFromText(const char* p) {
  int v = atoi(p);
  if (v > 16) v = 16;
  if (v < 0)  v = 0;
  voiceSetVolume((uint8_t)v);
  voiceSpeakTest();
}

static void runCmd(char* cmd) {
  String s = String(cmd);
  s.trim();
  if (s.length() == 0) return;

  String low = s;
  low.toLowerCase();
  String msg;

  if (low == "help" || low == "?") { printHelp(); return; }

  if (low == "scan") { Serial.println(magScanI2C(msg)); return; }

  if (low == "mag" || low == "data") { cmdPrintMagLine(); return; }

  if (low == "zero") {
    magZeroHere(msg);
    Serial.printf("[磁力计] %s\n", msg.c_str());
    return;
  }

  if (low.startsWith("decl")) {
    String arg = low.substring(4); arg.trim();
    magSetDeclination(arg.toFloat(), msg);
    Serial.printf("[磁力计] %s\n", msg.c_str());
    return;
  }

  if (low.startsWith("cal")) {
    String arg = low.substring(3); arg.trim();
    if      (arg == "reset")  magCalReset(msg);
    else if (arg == "stop")   magCalStop(msg);
    else if (arg == "cancel") magCalCancel(msg);
    else {
      int sec = arg.length() ? arg.toInt() : MAG_CAL_DEFAULT_SEC;
      if (sec < 5)   sec = 5;
      if (sec > 120) sec = 120;
      magCalAutoStart((uint32_t)sec, msg);
    }
    Serial.printf("[磁力计] %s\n", msg.c_str());
    return;
  }

  // 音量：v10 / 10 两种写法都认
  if ((low[0] == 'v' || low[0] == 'V') && low.length() > 1 &&
      low[1] >= '0' && low[1] <= '9') {
    setVolumeFromText(low.c_str() + 1);
    return;
  }
  if (low[0] >= '0' && low[0] <= '9') { setVolumeFromText(low.c_str()); return; }

  Serial.printf("未知命令：%s（输入 help 看说明）\n", s.c_str());
}

void cmdBegin() {
  Serial.println("串口命令已就绪，输入 help 查看。");
}

void cmdPoll() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (s_n) { s_buf[s_n] = '\0'; runCmd(s_buf); s_n = 0; }
    } else if (s_n < sizeof(s_buf) - 1) {
      s_buf[s_n++] = c;
    } else {
      s_n = 0;                                  // 太长就丢掉重来
    }
  }
}

void cmdPollEvents() {
  // 标定进度：每秒提示一次剩余时间
  static uint32_t lastRemain = 0xFFFFFFFF;
  if (magCalibrating()) {
    uint32_t r = magCalRemainingSec();
    if (r != lastRemain) {
      lastRemain = r;
      Serial.printf("[磁力计] 标定中… 还剩 %u 秒\n", (unsigned)r);
    }
  } else {
    lastRemain = 0xFFFFFFFF;
  }

  // 标定结束提示
  if (magCalJustFinished()) {
    Serial.printf("[磁力计] %s\n", magCalLastMessage().c_str());
  }
}
