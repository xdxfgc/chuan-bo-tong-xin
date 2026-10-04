/* =====================================================================
   link.cpp   一问一答实现
   ===================================================================== */

#include "link.h"
#include "radio.h"

static uint32_t s_okRounds   = 0;
static uint32_t s_retries    = 0;
static uint32_t s_failRounds = 0;

/* 应答有两种格式，都认得：
     新：A,<船ID>,<序号>,C,<船端定位有效>,<纬度>,<经度>
     老：A,<序号>,C,<船端定位有效>,<纬度>,<经度>
   统一从 ",C," 往后取字段，两种格式都能解析。
   顺便把船端自己的坐标打出来——联调时能看出"船在哪"。 */
static void printAckInfo(const String& ack) {
  if (!ack.startsWith("A,")) return;

  int pc = ack.indexOf(",C,");
  if (pc < 0) return;

  int    fix = 0;
  double cla = 0.0, clo = 0.0;
  if (sscanf(ack.c_str() + pc + 1, "C,%d,%lf,%lf", &fix, &cla, &clo) == 3) {
    if (fix) DBG.printf("           船端中心坐标: %.6f, %.6f\n", cla, clo);
    else     DBG.println("           船端中心：尚未定位");
  }
}

bool linkSendWithAck(const char* frame, String& ackOut, int& attemptsUsed) {
  attemptsUsed = 0;

  for (int attempt = 1; attempt <= MAX_RETRY; attempt++) {
    attemptsUsed = attempt;

    if (attempt > 1) {                 // 第一次由主程序打印“第 N 轮 发出 …”
      s_retries++;
      DBG.printf("           重发 %s (第 %d 次)\n", frame, attempt);
    }

    if (!radioSend(frame)) {
      DBG.println("           发送失败：超时没有 TxDone");
      break;
    }

    if (radioReceive(ackOut, ACK_TIMEOUT_MS)) {
      DBG.printf("           收到应答 <- %s   RSSI=%d dBm  SNR=%.1f dB\n",
                 ackOut.c_str(), radioLastRssi(), radioLastSnr());
      printAckInfo(ackOut);
      s_okRounds++;
      return true;
    }

    DBG.println("           超时：没等到应答");
  }

  s_failRounds++;
  return false;
}

void linkPrintStats() {
  DBG.printf("统计: 成功 %lu 轮 | 重发 %lu 次 | 失败 %lu 轮\n\n",
             (unsigned long)s_okRounds, (unsigned long)s_retries, (unsigned long)s_failRounds);
}
