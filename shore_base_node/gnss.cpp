/* =====================================================================
   gnss.cpp   北斗定位实现
   ===================================================================== */

#include "gnss.h"

static HardwareSerial GPSSerial(1);      // UART0 留给 USB 调试

static GpsStatus st;

static int  visibleGps = 0, visibleBds = 0, visibleOther = 0;

static uint8_t alarmCode = 0x00;
static String  alarmText = "尚未定位";
static bool    everFixed = false;

static String   lastGgaLine = "--";
static String   poseFrameHex = "--";
static uint16_t frameSeq = 0;

static char lineBuf[128];
static int  lineLen = 0;

/* ---------------- 工具 ---------------- */

// 取 NMEA 语句中第 index 个字段（index 0 为语句名，如 GNGGA）
static bool getField(const char* line, int index, char* out, size_t outSize) {
  if (!line || line[0] != '$') return false;
  const char* p = line + 1;
  int cur = 0;
  while (true) {
    const char* start = p;
    while (*p && *p != ',' && *p != '*') p++;
    if (cur == index) {
      size_t n = (size_t)(p - start);
      if (n >= outSize) n = outSize - 1;
      memcpy(out, start, n);
      out[n] = '\0';
      return true;
    }
    if (*p != ',') return false;
    p++;
    cur++;
  }
}

// 度分格式（ddmm.mmmm）转十进制度
static double nmeaToDeg(double raw, char hemi) {
  int    deg = (int)(raw / 100.0);
  double min = raw - deg * 100.0;
  double dec = deg + min / 60.0;
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return dec;
}

static void parseTime(const char* s) {
  if (strlen(s) < 6) return;
  st.hour   = (s[0] - '0') * 10 + (s[1] - '0');
  st.minute = (s[2] - '0') * 10 + (s[3] - '0');
  st.second = (s[4] - '0') * 10 + (s[5] - '0');
}

static String twoDigit(int v) {
  char b[4];
  snprintf(b, sizeof(b), "%02d", v);
  return String(b);
}

// CRC16（MODBUS），对应文档表26 的 2 字节校验
static uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

static void putFloat(uint8_t* buf, size_t& p, float v) {
  uint8_t b[4];
  memcpy(b, &v, 4);
  buf[p++] = b[0]; buf[p++] = b[1]; buf[p++] = b[2]; buf[p++] = b[3];
}

/* ---------------- NMEA 解析 ---------------- */

static void handleNmea(char* line) {
  // 有 "*hh" 就校验，不通过直接丢
  const char* star = strchr(line, '*');
  if (star) {
    uint8_t sum = 0;
    for (const char* p = line + 1; p < star; p++) sum ^= (uint8_t)(*p);
    uint8_t given = (uint8_t)strtol(star + 1, nullptr, 16);
    if (sum != given) return;
  }
  if (line[0] != '$') return;

  const char* type = line + 3;      // 跳过 '$' 和 2 位系统标识（GP/GN/BD）
  char a[40], b[40];

  /* --- GGA：定位质量、经纬度、海拔、卫星数 --- */
  if (!strncmp(type, "GGA", 3)) {
    getField(line, 1, a, sizeof(a)); parseTime(a);

    getField(line, 2, a, sizeof(a));
    getField(line, 3, b, sizeof(b));
    if (a[0]) st.lat = nmeaToDeg(atof(a), b[0]);

    getField(line, 4, a, sizeof(a));
    getField(line, 5, b, sizeof(b));
    if (a[0]) st.lon = nmeaToDeg(atof(a), b[0]);

    getField(line, 6, a, sizeof(a)); st.fixQuality = atoi(a);
    getField(line, 7, a, sizeof(a)); st.satsUsed   = atoi(a);
    getField(line, 8, a, sizeof(a)); if (a[0]) st.hdop     = atof(a);
    getField(line, 9, a, sizeof(a)); if (a[0]) st.altitude = atof(a);

    st.valid = (st.fixQuality > 0);
    if (st.valid) { st.lastFixMs = millis(); everFixed = true; }
    lastGgaLine = String(line);
    return;
  }

  /* --- RMC：定位状态、速度、航向、日期 --- */
  if (!strncmp(type, "RMC", 3)) {
    getField(line, 1, a, sizeof(a)); parseTime(a);

    getField(line, 2, a, sizeof(a));
    if (a[0] == 'A') { st.valid = true; st.lastFixMs = millis(); everFixed = true; }

    getField(line, 3, a, sizeof(a));
    getField(line, 4, b, sizeof(b));
    if (a[0]) st.lat = nmeaToDeg(atof(a), b[0]);

    getField(line, 5, a, sizeof(a));
    getField(line, 6, b, sizeof(b));
    if (a[0]) st.lon = nmeaToDeg(atof(a), b[0]);

    getField(line, 7, a, sizeof(a));
    if (a[0]) st.speedKmh = atof(a) * 1.852f;      // 节 -> 千米每小时

    getField(line, 8, a, sizeof(a));
    if (a[0]) { st.course = atof(a); st.heading = st.course; }

    getField(line, 9, a, sizeof(a));
    if (strlen(a) >= 6) {
      st.day   = (a[0] - '0') * 10 + (a[1] - '0');
      st.month = (a[2] - '0') * 10 + (a[3] - '0');
      st.year  = 2000 + (a[4] - '0') * 10 + (a[5] - '0');
    }
    return;
  }

  /* --- GSA：定位类型 --- */
  if (!strncmp(type, "GSA", 3)) {
    getField(line, 2, a, sizeof(a));
    if (a[0]) st.fixType = atoi(a);
    return;
  }

  /* --- GSV：可见卫星数 --- */
  if (!strncmp(type, "GSV", 3)) {
    getField(line, 2, a, sizeof(a));
    if (atoi(a) == 1) {
      getField(line, 3, b, sizeof(b));
      int  n  = atoi(b);
      char t1 = line[1], t2 = line[2];
      if (t1 == 'G' && t2 == 'P')      visibleGps = n;
      else if (t1 == 'B' && t2 == 'D') visibleBds = n;
      else                             visibleOther = n;
      st.satsView = visibleGps + visibleBds + visibleOther;
    }
    return;
  }

  /* --- TXT：天线状态 --- */
  if (!strncmp(type, "TXT", 3)) {
    getField(line, 4, a, sizeof(a));
    if (a[0]) {
      strncpy(st.antenna, a, sizeof(st.antenna) - 1);
      st.antenna[sizeof(st.antenna) - 1] = '\0';
    }
    return;
  }

  /* --- ZDA：日期与时间 --- */
  if (!strncmp(type, "ZDA", 3)) {
    getField(line, 1, a, sizeof(a)); parseTime(a);
    getField(line, 2, a, sizeof(a)); if (a[0]) st.day   = atoi(a);
    getField(line, 3, a, sizeof(a)); if (a[0]) st.month = atoi(a);
    getField(line, 4, a, sizeof(a)); if (a[0]) st.year  = atoi(a);
    return;
  }
}

static void readSerial() {
  while (GPSSerial.available()) {
    char c = (char)GPSSerial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleNmea(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                                  // 溢出整行丢弃
    }
  }
}

/* ---------------- 组帧与告警 ---------------- */

// 0x01 位姿帧：同步字 + 类型 + 序号 + 时间戳 + 长度 + 载荷 + 校验
static String buildPoseFrame() {
  uint8_t buf[64];
  size_t  p = 0;

  buf[p++] = 0xA5; buf[p++] = 0x5A;
  buf[p++] = 0x01;
  buf[p++] = (frameSeq >> 8) & 0xFF;
  buf[p++] = frameSeq & 0xFF;

  uint32_t ts = millis();
  buf[p++] = ts & 0xFF;
  buf[p++] = (ts >> 8) & 0xFF;
  buf[p++] = (ts >> 16) & 0xFF;
  buf[p++] = (ts >> 24) & 0xFF;

  size_t lenPos = p;
  buf[p++] = 0;
  size_t payloadStart = p;

  putFloat(buf, p, (float)st.lat);
  putFloat(buf, p, (float)st.lon);
  putFloat(buf, p, st.speedKmh / 3.6f);
  putFloat(buf, p, st.course);
  putFloat(buf, p, st.heading);
  buf[p++] = st.valid ? 1 : 0;
  buf[p++] = (uint8_t)(st.satsUsed > 255 ? 255 : st.satsUsed);

  buf[lenPos] = (uint8_t)(p - payloadStart);

  uint16_t crc = crc16(buf, p);
  buf[p++] = (crc >> 8) & 0xFF;
  buf[p++] = crc & 0xFF;

  frameSeq++;

  String s;
  char t[4];
  for (size_t i = 0; i < p; i++) {
    snprintf(t, sizeof(t), "%02X ", buf[i]);
    s += t;
  }
  return s;
}

// 定位失锁告警 0x06：定位有效后连续 5 秒无定位即触发（文档附录B）
static void updateAlarm() {
  if (st.valid) {
    alarmCode = 0x00;
    alarmText = "无";
  } else if (everFixed && (millis() - st.lastFixMs > 5000)) {
    alarmCode = 0x06;
    alarmText = "定位失锁（0x06 提醒级）";
  } else {
    alarmCode = 0x00;
    alarmText = "尚未定位";
  }
}

/* ---------------- 对外接口 ---------------- */

void gpsBegin() {
  st.valid      = false;
  st.fixQuality = 0;
  st.fixType    = 0;
  st.lat = 0.0;  st.lon = 0.0;
  st.altitude = 0.0f;
  st.speedKmh = 0.0f;
  st.course = 0.0f;  st.heading = 0.0f;
  st.hdop = 99.9f;
  st.satsUsed = 0;   st.satsView = 0;
  st.hour = st.minute = st.second = 0;
  st.day = st.month = st.year = 0;
  st.lastFixMs = 0;
  strncpy(st.antenna, "未知", sizeof(st.antenna) - 1);
  st.antenna[sizeof(st.antenna) - 1] = '\0';

  GPSSerial.setRxBufferSize(1024);                  // 默认 256，加大防丢数据
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("定位串口: UART1  RX=GPIO%d  TX=GPIO%d  %d bps\n",
                GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
  Serial.println("提示: 北斗天线请置于露天、平面朝天，冷启动约 30 秒到 1 分钟。");
}

void gpsUpdate() {
  readSerial();

  // 每秒刷新一次告警和位姿帧，保证串口与网页看到的是同一份数据
  static unsigned long lastTick = 0;
  if (millis() - lastTick >= 1000) {
    lastTick = millis();
    updateAlarm();
    poseFrameHex = buildPoseFrame();
  }
}

const GpsStatus& gpsGet() { return st; }
uint8_t gpsAlarmCode()    { return alarmCode; }
String  gpsAlarmText()    { return alarmText; }
String  gpsAntenna()      { return String(st.antenna); }
String  gpsPoseFrameHex() { return poseFrameHex; }
String  gpsLastGga()      { return lastGgaLine; }

String gpsUtcTime() {
  if (st.year == 0) return "--:--:--";
  return twoDigit(st.hour) + ":" + twoDigit(st.minute) + ":" + twoDigit(st.second);
}

String gpsBeijingTime() {
  if (st.year == 0) return "--:--:--";
  int h = st.hour + 8;
  if (h >= 24) h -= 24;
  return twoDigit(h) + ":" + twoDigit(st.minute) + ":" + twoDigit(st.second);
}

String gpsDateText() {
  if (st.year == 0) return "--";
  char b[16];
  snprintf(b, sizeof(b), "%04d-%02d-%02d", st.year, st.month, st.day);
  return String(b);
}

void gpsPrintReport() {
  Serial.println();
  Serial.println("=============== 北斗定位 ===============");
  Serial.printf("定位状态 : %s\n", st.valid ? "定位成功" : "未定位");
  Serial.printf("定位类型 : %s\n", st.fixType == 3 ? "三维定位(3D)" :
                                    (st.fixType == 2 ? "二维定位(2D)" : "未定位"));
  Serial.printf("纬度     : %.6f N\n", st.lat);
  Serial.printf("经度     : %.6f E\n", st.lon);
  Serial.printf("海拔     : %.1f m\n", st.altitude);
  Serial.printf("卫星     : 参与 %d / 可见 %d\n", st.satsUsed, st.satsView);
  Serial.printf("HDOP     : %.1f\n", st.hdop);
  Serial.printf("对地速度 : %.2f km/h\n", st.speedKmh);
  Serial.printf("航向     : %.1f 度\n", st.course);
  Serial.printf("UTC 时间 : %s\n", gpsUtcTime().c_str());
  Serial.printf("北京时间 : %s\n", gpsBeijingTime().c_str());
  Serial.printf("日期     : %s\n", gpsDateText().c_str());
  Serial.printf("天线     : %s\n", gpsAntenna().c_str());
  Serial.printf("告警     : %s\n", alarmText.c_str());
  Serial.printf("位姿帧   : %s\n", poseFrameHex.c_str());
  Serial.printf("原始GGA  : %s\n", lastGgaLine.c_str());
  Serial.println("========================================");
}
