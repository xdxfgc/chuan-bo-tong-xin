/* =====================================================================
   cmd.cpp   串口命令实现
   ===================================================================== */

#include "cmd.h"
#include "voice.h"
#include "mag.h"
#include "berth.h"
#include "buzzer.h"
#include "anchor.h"
#include "logbook.h"

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
  Serial.println("  dock        打印靠泊状态（距离、接近速度、告警）");
  Serial.println("  ack         确认落水告警（停止蜂鸣器）");
  Serial.println("  buzz on/off 打开或关闭蜂鸣器");
  Serial.println("  moor set    把当前位置设为锚泊基准，开始走锚监测");
  Serial.println("  moor clear  清除锚泊基准，停止监测");
  Serial.println("  moor        打印锚泊状态（位移、漂移方向与速率）");
  Serial.println("  log on/off  开始或暂停数据记录（网页上也有按钮）");
  Serial.println("  log dump    把已记录的数据以 CSV 打印到串口");
  Serial.println("  log clear   清空记录");
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

  if (low == "dock") { berthPrintReport(); return; }

  if (low == "ack") { buzzerAcknowledge(); return; }

  if (low.startsWith("moor")) {
    String arg = low.substring(4); arg.trim();
    if      (arg == "set")   anchorSetReference();
    else if (arg == "clear") anchorClearReference();
    else                     anchorPrintReport();
    return;
  }

  if (low.startsWith("log")) {
    String arg = low.substring(3); arg.trim();
    if (arg == "on")         logbookSetOn(true);
    else if (arg == "off")   logbookSetOn(false);
    else if (arg == "clear") logbookClear();
    else if (arg == "dump") {
      Serial.println(logbookHeader());
      for (uint16_t k = 0; k < logbookCount(); k++) Serial.print(logbookLine(k));
      Serial.printf("# 共 %u 条\n", (unsigned)logbookCount());
    } else {
      logbookPrintStatus();
    }
    return;
  }

  if (low.startsWith("buzz")) {
    String arg = low.substring(4); arg.trim();
    if      (arg == "on")  buzzerSetUserOn(true);
    else if (arg == "off") buzzerSetUserOn(false);
    else Serial.printf("蜂鸣器：%s（buzz on / buzz off 可切换）\n", buzzerStateText().c_str());
    return;
  }

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
