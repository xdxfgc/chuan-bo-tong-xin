/* =====================================================================
   coord.cpp   坐标来源实现
   ---------------------------------------------------------------------
   四种模式的代码都用 SRC_MODE 隔开，只会编译用到的那一段，
   所以不用的模式不会占程序空间。
   ===================================================================== */

#include "coord.h"
#include <math.h>

#if SRC_MODE == 2
#include <HardwareSerial.h>

static HardwareSerial GnssSerial(1);
static char   gLine[128];
static int    gLen   = 0;
static bool   gValid = false;
static double gLat = 0.0, gLon = 0.0;

/* 调试用的状态。只有 SRC_MODE == 2 会用到，所以不会占别的模式的空间。 */
static int           gSats       = 0;      // 参与定位的卫星数（GGA 第 8 字段）
static int           gFixQuality = 0;      // GGA 第 7 字段：0=没定位 1=单点 2=差分
static unsigned long gLastFixMs  = 0;      // 最近一次「定位有效」的时刻
static unsigned long gLastGgaMs  = 0;      // 最近一次收到 GGA 的时刻（不管定没定上）
static unsigned long gLastMsgMs  = 0;      // 状态打印节流

/* 超过这么久没有再收到有效定位，就当定位丢了。
   光靠 gValid 是不行的：模块被拔掉之后 gValid 会一直停在 true，
   信标就会拿着一份过期的坐标不停往外发。                          */
#define GNSS_STALE_MS 10000UL

// 度分（ddmm.mmmm）转十进制度
static double nmeaDeg(double raw, char hemi) {
  int    deg = (int)(raw / 100.0);
  double dec = deg + (raw - deg * 100.0) / 60.0;
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return dec;
}

// 取 NMEA 第 index 个字段
static bool nmeaField(const char* line, int index, char* out, size_t outSize) {
  const char* p = line + 1;
  int cur = 0;
  while (true) {
    const char* st = p;
    while (*p && *p != ',' && *p != '*') p++;
    if (cur == index) {
      size_t n = (size_t)(p - st);
      if (n >= outSize) n = outSize - 1;
      memcpy(out, st, n);
      out[n] = '\0';
      return true;
    }
    if (*p != ',') return false;
    p++;
    cur++;
  }
}

static void gnssHandleLine(char* line) {
  const char* star = strchr(line, '*');
  if (star) {                                   // 校验和
    uint8_t sum = 0;
    for (const char* p = line + 1; p < star; p++) sum ^= (uint8_t)(*p);
    if (sum != (uint8_t)strtol(star + 1, nullptr, 16)) return;
  }
  if (line[0] != '$' || strncmp(line + 3, "GGA", 3)) return;

  char a[24], b[24];
  nmeaField(line, 2, a, sizeof(a)); nmeaField(line, 3, b, sizeof(b));
  if (a[0]) gLat = nmeaDeg(atof(a), b[0]);
  nmeaField(line, 4, a, sizeof(a)); nmeaField(line, 5, b, sizeof(b));
  if (a[0]) gLon = nmeaDeg(atof(a), b[0]);

  // 第 7 字段是定位质量，第 8 字段是参与定位的卫星数
  nmeaField(line, 6, a, sizeof(a));
  gFixQuality = atoi(a);
  nmeaField(line, 7, a, sizeof(a));
  gSats = atoi(a);

  gLastGgaMs = millis();

  /* 只有「质量 > 0」而且坐标不是 (0,0) 才算真的定上位。
     没定位的 GGA 每个字段都是空的，atof 会得到 0，所以要一起判断。 */
  if (gFixQuality > 0 && !(fabs(gLat) < 1e-9 && fabs(gLon) < 1e-9)) {
    gValid     = true;
    gLastFixMs = millis();
  }
  /* 定位质量变 0 时这里不立刻置无效，交给 gnssCheckStale 按超时判，
     免得卫星偶尔掉一下坐标就闪断。 */
}

static void gnssPollOnce() {
  while (GnssSerial.available()) {
    char c = (char)GnssSerial.read();
    if (c == '\n' || c == '\r') {
      if (gLen > 0) { gLine[gLen] = '\0'; gnssHandleLine(gLine); gLen = 0; }
    } else if (gLen < (int)sizeof(gLine) - 1) {
      gLine[gLen++] = c;
    } else {
      gLen = 0;
    }
  }
}

// 定位超时检查：太久没有有效定位就当丢了
static void gnssCheckStale() {
  if (gValid && millis() - gLastFixMs > GNSS_STALE_MS) {
    gValid = false;
  }
}

/* 每 5 秒打一行状态。
   这一行的用处是把三种情况分开，不然调试时只能干瞪眼：
     完全没数据   -> 接线/波特率/供电不对（TX、RX 接反是最常见的）
     有数据没定位 -> 在正常搜星，把天线挪到窗边或室外
     已定位       -> 正常，下面接着打印卫星数和坐标                     */
static void gnssPrintStatus() {
  if (gLastMsgMs != 0 && millis() - gLastMsgMs < 5000) return;
  gLastMsgMs = millis();

  if (gLastGgaMs == 0) {
    DBG.println("[北斗] 一句数据都没收到 —— 检查 TXD/RXD 有没有接反、"
                "波特率是不是 9600、模块供电和共地");
  } else if (millis() - gLastGgaMs > 3000) {
    DBG.println("[北斗] 串口断了，之前有数据现在没有 —— 检查接线是不是松了");
  } else if (gValid) {
    DBG.printf("[北斗] 已定位  卫星 %d  %.6f, %.6f\n", gSats, gLat, gLon);
  } else {
    DBG.printf("[北斗] 正在搜星…（卫星 %d，还没定上位）天线贴窗边或拿到室外会快很多\n",
               gSats);
  }
}
#endif   // SRC_MODE == 2

#if SRC_MODE == 3
/* ---------------- 串口手动输入目标坐标 ----------------
   串口监视器里敲 “26.210000,111.600000” 回车（逗号或空格分隔都行），
   立刻生效，从下一轮开始按这个坐标发出去，不用重新烧录。            */
static bool   s_inValid = false;
static double s_inLat   = 0.0;
static double s_inLon   = 0.0;
static char   s_inBuf[32];
static size_t s_inLen   = 0;

static void pollSerialCoord() {
  while (DBG.available()) {
    char c = (char)DBG.read();
    if (c == '\n' || c == '\r') {
      if (s_inLen) {
        s_inBuf[s_inLen] = '\0';
        double la = 0.0, lo = 0.0;
        if (sscanf(s_inBuf, "%lf%*[, ]%lf", &la, &lo) == 2 &&
            fabs(la) <= 90.0 && fabs(lo) <= 180.0 &&
            !(fabs(la) < 1e-9 && fabs(lo) < 1e-9)) {
          s_inLat   = la;
          s_inLon   = lo;
          s_inValid = true;
          DBG.printf("已设定目标坐标：%.6f, %.6f（从下一轮开始发送）\n", la, lo);
        } else {
          DBG.println("格式不对，示例：26.210000,111.600000");
        }
        s_inLen = 0;
      }
    } else if (s_inLen < sizeof(s_inBuf) - 1) {
      s_inBuf[s_inLen++] = c;
    } else {
      s_inLen = 0;
    }
  }
}
#endif   // SRC_MODE == 3

/* ---------------- 对外接口 ---------------- */

void coordBegin() {
#if SRC_MODE == 2
  GnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  DBG.printf("本端北斗：UART1 RX=GPIO%d TX=GPIO%d  %d bps\n",
             GNSS_RX_PIN, GNSS_TX_PIN, GNSS_BAUD);
  DBG.println("          模块 TXD -> 上面的 RX 脚，模块 RXD -> 上面的 TX 脚（要交叉）");
  DBG.println("          冷启动第一次定位要 30~60 秒，室内基本定不上，把天线贴窗边或拿出去");
#elif SRC_MODE == 3
  DBG.println("坐标来源：串口手动输入（还没输入过，当前按“未定位”发送）");
  DBG.println("用法：在串口监视器里敲 纬度,经度 再回车，例如 26.210000,111.600000");
#endif
}

void coordPoll() {
#if SRC_MODE == 2
  gnssPollOnce();
  gnssCheckStale();
  gnssPrintStatus();
#elif SRC_MODE == 3
  pollSerialCoord();
#endif
}

void coordGet(bool* valid, double* lat, double* lon) {
#if SRC_MODE == 0
  *valid = true;
  *lat = BASE_LAT;
  *lon = BASE_LON;

#elif SRC_MODE == 1
  double t = (millis() / 1000.0) / DRIFT_PERIOD_S * (2.0 * M_PI);
  *valid = true;
  *lat = BASE_LAT + (DRIFT_RADIUS_M / 111320.0) * cos(t);
  *lon = BASE_LON + (DRIFT_RADIUS_M / (111320.0 * cos(BASE_LAT * M_PI / 180.0))) * sin(t);

#elif SRC_MODE == 2
  *valid = gValid;
  *lat = gLat;
  *lon = gLon;

#else
  *valid = s_inValid;
  *lat   = s_inValid ? s_inLat : 0.0;
  *lon   = s_inValid ? s_inLon : 0.0;
#endif
}

const char* coordSourceText() {
#if SRC_MODE == 0
  return "固定坐标";
#elif SRC_MODE == 1
  return "绕点漂移（演示）";
#elif SRC_MODE == 2
  return "本端北斗（真实定位）";
#else
  return "串口手动输入";
#endif
}

void coordBuildPayload(char* buf, size_t n) {
  bool   v;
  double la, lo;
  coordGet(&v, &la, &lo);
  snprintf(buf, n, "P,%d,%.6f,%.6f", v ? 1 : 0, la, lo);
}
