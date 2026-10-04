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
  nmeaField(line, 6, a, sizeof(a));
  gValid = (atoi(a) > 0);
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
  DBG.printf("本端北斗：UART1 RX=GPIO%d TX=GPIO%d %d bps\n",
             GNSS_RX_PIN, GNSS_TX_PIN, GNSS_BAUD);
#elif SRC_MODE == 3
  DBG.println("坐标来源：串口手动输入（还没输入过，当前按“未定位”发送）");
  DBG.println("用法：在串口监视器里敲 纬度,经度 再回车，例如 26.210000,111.600000");
#endif
}

void coordPoll() {
#if SRC_MODE == 2
  gnssPollOnce();
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
