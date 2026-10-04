/* =====================================================================
   selfcast.cpp   本船位置定期广播实现
   ===================================================================== */

#include "selfcast.h"
#include "lora_link.h"
#include "gnss.h"

static uint32_t      s_seq      = 0;
static unsigned long s_lastMs   = 0;
static bool          s_sentOnce = false;

void selfcastBegin() {
  s_seq      = 0;
  s_lastMs   = 0;
  s_sentOnce = false;
}

void selfcastUpdate() {
  /* 第一个判断用 s_sentOnce：上电后 s_lastMs 是 0，直接相减会立刻放行，
     那样相当于一通电就先发一帧，跟"每隔 2 秒"对不上。 */
  if (s_sentOnce && (millis() - s_lastMs < BOAT_BCAST_MS)) return;
  s_lastMs   = millis();
  s_sentOnce = true;

  if (!loraIsReady()) return;        // 射频还没起来就跳过这一轮，下一轮再试

  const GpsStatus& g = gpsGet();

  char tx[64];
  snprintf(tx, sizeof(tx), "S,%u,%lu,P,%d,%.6f,%.6f",
           (unsigned)DEV_ID, (unsigned long)s_seq, g.valid ? 1 : 0, g.lat, g.lon);

  if (!loraSendText(tx))            Serial.println("[广播] 发送失败：800ms 内没有 TxDone");
  else if (!g.valid)                Serial.printf("[广播] %s（本船还没定位）\n", tx);
  else                              Serial.printf("[广播] %s\n", tx);

  s_seq++;
}
