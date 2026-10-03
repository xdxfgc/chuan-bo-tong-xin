/* =====================================================================
   北斗定位模块测试程序
   通导一体化水上安全终端 · 移动终端定位子系统
   ---------------------------------------------------------------------
   硬件：ESP32-S3-DevKitC-1  +  ATGM336H-5N 北斗/GPS 双模定位模块

   接线：模块 TX  -> ESP32 GPIO17   (ESP32 的接收脚)
         模块 RX  -> ESP32 GPIO18   (ESP32 的发送脚)
         模块 5V  -> ESP32 5V
         模块 GND -> ESP32 GND

   功能：
     1. 读取并解析定位模块输出的 NMEA 语句（GGA / RMC / GSA / GSV / TXT / ZDA）
     2. 在 Arduino IDE 串口监视器（波特率 115200）打印定位结果
     3. 自动连接 WiFi 并启动网页服务，打开网页即可实时查看测试结果
     4. 按文档表27 组装 0x01 位姿帧，按文档附录B 实现 0x06 定位失锁告警

   注意：ESP32 只支持 2.4GHz 频段。若 WiFi 名称是路由器的 5GHz 信号，
         将无法连接，请改用同一路由器的 2.4GHz 信号（密码相同）。
   ===================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <VL53L1X.h>       // Pololu 的 VL53L1X 库（库管理器里装 VL53L1X）

/* ---------------- 接线与串口配置（需要改动时改这里） ---------------- */
#define GPS_RX_PIN   17      // ESP32 接收 <- 模块 TX
#define GPS_TX_PIN   18      // ESP32 发送 -> 模块 RX
#define GPS_BAUD     9600    // ATGM336H-5N 默认波特率

/* ---------------- 定位数据（字段对应文档表27 的 0x01 位姿帧） ---------------- */
struct GpsStatus {
  bool   valid;        // 定位是否有效
  int    fixQuality;   // GGA 定位质量：0 未定位 / 1 单点 / 2 差分
  int    fixType;      // GSA 定位类型：1 未定位 / 2 二维 / 3 三维
  double lat;          // 纬度（度）
  double lon;          // 经度（度）
  float  altitude;     // 海拔（米）
  float  speedKmh;     // 对地速度（千米每小时）
  float  course;       // 航向（度）
  float  heading;      // 艏向（度，未接磁力计时与航向一致）
  float  hdop;         // 水平精度因子
  int    satsUsed;     // 参与定位卫星数
  int    satsView;     // 可见卫星数
  int    hour, minute, second;
  int    day, month, year;
  char   antenna[20];  // 天线状态
  unsigned long lastFixMs;
};

/* ---------------- 网络配置（需要改动时改这里） ---------------- */
// 注意：ESP32 只支持 2.4GHz 频段，连不上 5GHz 的 WiFi。
// 小米路由器有两个名字：不带 _5G 的是 2.4GHz（ESP32 必需），带 _5G 的是 5GHz（连不上）。
// 这里用的和「温湿度检测 / DHT11_ESP32」工程相同，那套是验证过能连上的。
#define WIFI_SSID   "Xiaomi_AE4D"
#define WIFI_PASS   "123456780"

#define WEB_PORT    80

/* ---------------- 固定 IP（需要改动时改这里） ----------------
   打开下面的开关后，ESP32 每次都使用同一个地址，网页网址就固定了。
   把 1 改成 0，则恢复成由路由器自动分配。
   注意：前三段必须和路由器在同一网段。小米路由器是 192.168.31.x，
   网关是 192.168.31.1。若固定 IP 连不上，程序会自动回退成自动获取，不会卡死。 */
#define USE_STATIC_IP 1

IPAddress STATIC_IP     (192, 168, 31, 200);   // 想固定的地址，最后一段建议 100~240
IPAddress STATIC_GATEWAY(192, 168, 31, 1);     // 网关，一般就是路由器地址
IPAddress STATIC_SUBNET (255, 255, 255, 0);    // 子网掩码
IPAddress STATIC_DNS    (192, 168, 31, 1);     // DNS，填网关即可

/* ---------------- 热点兜底（连不上路由器时用） ----------------
   如果路由器连不上（密码不对、频段不支持等），ESP32 会自己开一个热点，
   保证网页任何时候都能打开。手机连上这个热点，打开 http://192.168.4.1 即可。 */
#define AP_FALLBACK   1
#define AP_SSID      "Beidou-GPS"
#define AP_PASS      "12345678"       // 热点密码，至少 8 位

/* ---------------- 激光测距 VL53L1X（I2C） ----------------
   接线：VIN -> 3.3V（切勿接 5V，会烧坏）  GND -> GND
         SCL -> GPIO22    SDA -> GPIO21
   最简四根线即可工作，GPIO1 和 XSHUT 可以空着。 */
#define TOF_SDA_PIN    21        // SDA 接 GPIO21
#define TOF_SCL_PIN    22        // SCL 接 GPIO22
#define TOF_LONG_RANGE 1         // 1 = 远距模式（最远约 4 米），0 = 短距模式
#define TOF_READ_MS    50        // 读取间隔（毫秒）

/* =====================================================================
   激光测距模块 VL53L1X
   ===================================================================== */

static VL53L1X tof;
static bool     tofReady = false;       // 模块是否初始化成功
static bool     tofValid = false;       // 本次读数是否有效
static uint16_t tofMm    = 0;           // 距离（毫米）
static unsigned long tofLastReadMs = 0;

void tofBegin() {
  Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN);
  Wire.setClock(400000);
  tof.setTimeout(500);

  if (!tof.init()) {
    tofReady = false;
    Serial.println("激光测距 VL53L1X 初始化失败：检查 VIN 是否接 3.3V、GND 是否共地、SDA/SCL 是否接对。");
    return;
  }

  tof.setDistanceMode(TOF_LONG_RANGE ? VL53L1X::Long : VL53L1X::Short);
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(50);
  tofReady = true;
  Serial.printf("激光测距 VL53L1X 已启动：SDA=GPIO%d  SCL=GPIO%d  %s模式\n",
                TOF_SDA_PIN, TOF_SCL_PIN, TOF_LONG_RANGE ? "远距" : "短距");
}

void tofUpdate() {
  if (!tofReady) return;
  if (millis() - tofLastReadMs < TOF_READ_MS) return;
  tofLastReadMs = millis();

  uint16_t d = tof.read(false);
  if (tof.timeoutOccurred()) {
    tofValid = false;
    return;
  }
  if (tof.ranging_data.range_status == VL53L1X::RangeValid) {
    tofMm    = d;
    tofValid = true;
  } else {
    tofValid = false;
  }
}

bool     tofIsReady()    { return tofReady; }
bool     tofIsValid()    { return tofValid; }
uint16_t tofDistanceMm() { return tofMm; }

// 供串口和网页显示的文字
String tofText() {
  if (!tofReady) return "模块未连接";
  if (!tofValid) return "无效（超出量程或信号弱）";
  char b[32];
  snprintf(b, sizeof(b), "%u mm (%.2f m)", (unsigned)tofMm, tofMm / 1000.0);
  return String(b);
}

/* =====================================================================
   北斗定位模块
   ===================================================================== */


/* ---------------- 内部状态 ---------------- */
static HardwareSerial GPSSerial(1);     // 用 UART1，避开 USB 调试口

static GpsStatus st;

static int visibleGps = 0, visibleBds = 0, visibleOther = 0;

static uint8_t alarmCode = 0x00;        // 对应文档附录B 告警码
static String  alarmText = "尚未定位";
static bool    everFixed = false;

static String  lastGgaLine = "--";
static String  poseFrameHex = "--";
static uint16_t frameSeq = 0;

static char lineBuf[128];
static int  lineLen = 0;

/* ---------------- 工具函数 ---------------- */

// 取 NMEA 语句中第 index 个字段（index 0 为语句名，如 GNGGA）
bool getField(const char* line, int index, char* out, size_t outSize) {
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
double nmeaToDeg(double raw, char hemi) {
  int    deg = (int)(raw / 100.0);
  double min = raw - deg * 100.0;
  double dec = deg + min / 60.0;
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return dec;
}

// 解析 hhmmss.ss 形式的时间
void parseTime(const char* s) {
  if (strlen(s) < 6) return;
  st.hour   = (s[0] - '0') * 10 + (s[1] - '0');
  st.minute = (s[2] - '0') * 10 + (s[3] - '0');
  st.second = (s[4] - '0') * 10 + (s[5] - '0');
}

String twoDigit(int v) {
  char b[4];
  snprintf(b, sizeof(b), "%02d", v);
  return String(b);
}

// CRC16（MODBUS），对应文档表26 的 2 字节校验
uint16_t crc16(const uint8_t* data, size_t len) {
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

void putFloat(uint8_t* buf, size_t& p, float v) {
  uint8_t b[4];
  memcpy(b, &v, 4);
  buf[p++] = b[0]; buf[p++] = b[1]; buf[p++] = b[2]; buf[p++] = b[3];
}

/* ---------------- NMEA 解析 ---------------- */

void handleNmea(char* line) {
  // 校验和（存在 "*hh" 时校验，失败则丢弃）
  const char* star = strchr(line, '*');
  if (star) {
    uint8_t sum = 0;
    for (const char* p = line + 1; p < star; p++) sum ^= (uint8_t)(*p);
    uint8_t given = (uint8_t)strtol(star + 1, nullptr, 16);
    if (sum != given) return;
  }
  if (line[0] != '$') return;

  const char* type = line + 3;   // 跳过 '$' 与 2 位系统标识（GP/GN/BD）
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

    // 以 GGA 的定位质量作为定位是否有效的判据
    st.valid = (st.fixQuality > 0);
    if (st.valid) {
      st.lastFixMs = millis();
      everFixed    = true;
    }
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

  /* --- GSA：定位类型（1 未定位 / 2 二维 / 3 三维） --- */
  if (!strncmp(type, "GSA", 3)) {
    getField(line, 2, a, sizeof(a));
    if (a[0]) st.fixType = atoi(a);
    return;
  }

  /* --- GSV：可见卫星数 --- */
  if (!strncmp(type, "GSV", 3)) {
    getField(line, 2, a, sizeof(a));          // 分句序号
    if (atoi(a) == 1) {
      getField(line, 3, b, sizeof(b));        // 本系统可见卫星数
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

  /* --- ZDA：日期与时间（含四位年份） --- */
  if (!strncmp(type, "ZDA", 3)) {
    getField(line, 1, a, sizeof(a)); parseTime(a);
    getField(line, 2, a, sizeof(a)); if (a[0]) st.day   = atoi(a);
    getField(line, 3, a, sizeof(a)); if (a[0]) st.month = atoi(a);
    getField(line, 4, a, sizeof(a)); if (a[0]) st.year  = atoi(a);
    return;
  }
}

void readSerial() {
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
      lineLen = 0;   // 溢出则丢弃整行
    }
  }
}

/* ---------------- 组帧与告警 ---------------- */

// 组装 0x01 位姿帧：同步字 + 类型 + 序号 + 时间戳 + 长度 + 载荷 + 校验
String buildPoseFrame() {
  uint8_t buf[64];
  size_t  p = 0;

  buf[p++] = 0xA5; buf[p++] = 0x5A;              // 同步字
  buf[p++] = 0x01;                               // 类型：位姿帧
  buf[p++] = (frameSeq >> 8) & 0xFF;             // 序号（高字节）
  buf[p++] = frameSeq & 0xFF;                    // 序号（低字节）

  uint32_t ts = millis();                        // 时间戳（毫秒）
  buf[p++] = ts & 0xFF;
  buf[p++] = (ts >> 8) & 0xFF;
  buf[p++] = (ts >> 16) & 0xFF;
  buf[p++] = (ts >> 24) & 0xFF;

  size_t lenPos = p;
  buf[p++] = 0;                                  // 长度占位
  size_t payloadStart = p;

  putFloat(buf, p, (float)st.lat);               // 纬度
  putFloat(buf, p, (float)st.lon);               // 经度
  putFloat(buf, p, st.speedKmh / 3.6f);          // 对地速度（米每秒）
  putFloat(buf, p, st.course);                   // 航向
  putFloat(buf, p, st.heading);                  // 艏向
  buf[p++] = st.valid ? 1 : 0;                   // 定位状态
  buf[p++] = (uint8_t)(st.satsUsed > 255 ? 255 : st.satsUsed);   // 卫星数

  buf[lenPos] = (uint8_t)(p - payloadStart);     // 回填长度

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

// 定位失锁告警 0x06：定位有效后连续 5 秒无定位即触发（对应文档附录B）
void updateAlarm() {
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

/* ---------------- 对外接口实现 ---------------- */

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

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("定位串口: UART1  RX=GPIO%d  TX=GPIO%d  %d bps\n",
                GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
  Serial.println("提示: 天线请置于露天、平面朝天，冷启动约 30 秒到 1 分钟。");
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
  Serial.println("============= 北斗定位 / 激光测距 =============");
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
  Serial.printf("激光距离 : %s\n", tofText().c_str());
  Serial.println("==================================================");
}
/* =====================================================================
   网络模块
   ===================================================================== */



static WebServer server(WEB_PORT);
static unsigned long lastWifiTry = 0;
static bool apMode = false;          // 是否正运行在热点兜底模式

/* ---------------- 网页 ---------------- */

static const char INDEX_HTML[] = R"HTML(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>北斗定位模块测试</title>
<style>
  :root{--bg:#0f1720;--card:#18222e;--line:#26333f;--txt:#e6edf3;--dim:#8b98a5;--accent:#38bdf8}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--txt);
       font-family:-apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
  .wrap{max-width:820px;margin:0 auto;padding:20px}
  h1{font-size:22px;margin:4px 0 2px}
  .sub{color:var(--dim);font-size:13px;margin:0 0 16px}
  .banner{padding:14px 16px;border-radius:12px;font-size:17px;font-weight:600;
          margin-bottom:16px;background:#22303c;border:1px solid var(--line)}
  .banner.ok{background:#0f2e1c;border-color:#1f6f3f;color:#7ee2a8}
  .banner.bad{background:#331717;border-color:#7f2d2d;color:#ff9b9b}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
  .card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px}
  .k{color:var(--dim);font-size:12px;margin-bottom:6px}
  .v{font-size:19px;font-weight:600;font-variant-numeric:tabular-nums}
  .v.small{font-size:15px}
  h2{font-size:15px;margin:22px 0 8px;color:var(--dim);font-weight:600}
  .hex,.raw{background:#0b1219;border:1px solid var(--line);border-radius:10px;
            padding:12px;font-family:Consolas,Menlo,monospace;font-size:12.5px;
            word-break:break-all;color:#9fe0b0}
  .raw{color:#9ec7e8}
  a{color:var(--accent)}
</style>
</head>
<body>
<div class="wrap">
  <h1>北斗定位模块测试</h1>
  <p class="sub">ESP32-S3 + ATGM336H-5N · 串口 9600 · 通导一体化水上安全终端</p>
  <div class="banner" id="banner">正在等待数据…</div>
  <div class="grid">
    <div class="card"><div class="k">纬度</div><div class="v" id="lat">--</div></div>
    <div class="card"><div class="k">经度</div><div class="v" id="lon">--</div></div>
    <div class="card"><div class="k">海拔</div><div class="v" id="alt">--</div></div>
    <div class="card"><div class="k">卫星（参与/可见）</div><div class="v" id="sat">--</div></div>
    <div class="card"><div class="k">HDOP</div><div class="v" id="hdop">--</div></div>
    <div class="card"><div class="k">对地速度</div><div class="v" id="spd">--</div></div>
    <div class="card"><div class="k">航向</div><div class="v" id="crs">--</div></div>
    <div class="card"><div class="k">定位类型</div><div class="v" id="ftype">--</div></div>
    <div class="card"><div class="k">北京时间</div><div class="v small" id="bj">--</div></div>
    <div class="card"><div class="k">UTC 时间</div><div class="v small" id="utc">--</div></div>
    <div class="card"><div class="k">日期</div><div class="v small" id="date">--</div></div>
    <div class="card"><div class="k">天线状态</div><div class="v small" id="ant">--</div></div>
    <div class="card"><div class="k">激光距离</div><div class="v" id="tof">--</div></div>
  </div>
  <h2>位姿帧（文档表27 · 0x01）</h2>
  <div class="hex" id="frame">--</div>
  <h2>原始语句（GGA）</h2>
  <div class="raw" id="raw">--</div>
  <h2>地图</h2>
  <div class="raw" id="maplink">待定位</div>
  <p class="sub" id="foot">运行时间 -- 秒</p>
</div>
<script>
async function tick(){
  try{
    const d = await (await fetch('/data',{cache:'no-store'})).json();
    const b = document.getElementById('banner');
    if(d.valid){ b.className='banner ok'; b.textContent = '定位成功 · ' + (d.fixType===3?'三维定位':'二维定位'); }
    else { b.className='banner bad'; b.textContent = d.alarm; }
    document.getElementById('lat').textContent   = d.valid ? d.lat.toFixed(6)+'° N' : '--';
    document.getElementById('lon').textContent   = d.valid ? d.lon.toFixed(6)+'° E' : '--';
    document.getElementById('alt').textContent   = d.valid ? d.alt.toFixed(1)+' m' : '--';
    document.getElementById('sat').textContent   = d.satsUsed + ' / ' + d.satsView;
    document.getElementById('hdop').textContent  = d.hdop.toFixed(1);
    document.getElementById('spd').textContent   = d.speedKmh.toFixed(2)+' km/h';
    document.getElementById('crs').textContent   = d.course.toFixed(1)+'°';
    document.getElementById('ftype').textContent = d.fixType===3?'三维定位':(d.fixType===2?'二维定位':'未定位');
    document.getElementById('bj').textContent    = d.bj;
    document.getElementById('utc').textContent   = d.utc;
    document.getElementById('date').textContent  = d.date;
    document.getElementById('ant').textContent   = d.antenna;
    document.getElementById('tof').textContent   = d.tofText;
    document.getElementById('frame').textContent = d.frame;
    document.getElementById('raw').textContent   = d.raw;
    const ml = document.getElementById('maplink');
    if(d.valid){
      ml.innerHTML = '<a target="_blank" href="https://www.openstreetmap.org/?mlat='+d.lat+
                     '&mlon='+d.lon+'#map=17/'+d.lat+'/'+d.lon+'">在 OpenStreetMap 上查看当前位置</a>';
    } else { ml.textContent = '待定位'; }
    document.getElementById('foot').textContent = '运行时间 ' + d.runSec + ' 秒';
  }catch(e){
    const b = document.getElementById('banner');
    b.className='banner bad';
    b.textContent='与 ESP32 的连接中断，请确认手机或电脑连的是同一个 WiFi';
  }
}
tick(); setInterval(tick, 1000);
</script>
</body>
</html>
)HTML";

/* ---------------- JSON 接口 ---------------- */

String escapeJson(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if (c == '\r') o += "\\r";
    else o += c;
  }
  return o;
}

String buildJson() {
  const GpsStatus& d = gpsGet();
  char num[32];

  String j = "{";
  j += "\"valid\":";       j += (d.valid ? "true" : "false");
  j += ",\"fixType\":";    j += d.fixType;
  j += ",\"fixQuality\":"; j += d.fixQuality;
  snprintf(num, sizeof(num), "%.6f", d.lat); j += ",\"lat\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", d.lon); j += ",\"lon\":"; j += num;
  snprintf(num, sizeof(num), "%.1f", d.altitude); j += ",\"alt\":"; j += num;
  j += ",\"satsUsed\":";   j += d.satsUsed;
  j += ",\"satsView\":";   j += d.satsView;
  snprintf(num, sizeof(num), "%.1f", d.hdop); j += ",\"hdop\":"; j += num;
  snprintf(num, sizeof(num), "%.2f", d.speedKmh); j += ",\"speedKmh\":"; j += num;
  snprintf(num, sizeof(num), "%.1f", d.course); j += ",\"course\":"; j += num;
  j += ",\"utc\":\"";      j += gpsUtcTime();      j += "\"";
  j += ",\"bj\":\"";       j += gpsBeijingTime();  j += "\"";
  j += ",\"date\":\"";     j += gpsDateText();     j += "\"";
  j += ",\"antenna\":\"";  j += escapeJson(gpsAntenna()); j += "\"";
  j += ",\"alarmCode\":";  j += gpsAlarmCode();
  j += ",\"alarm\":\"";    j += escapeJson(gpsAlarmText()); j += "\"";
  j += ",\"frame\":\"";    j += gpsPoseFrameHex(); j += "\"";
  j += ",\"raw\":\"";      j += escapeJson(gpsLastGga()); j += "\"";
  j += ",\"tofReady\":";   j += (tofIsReady() ? "true" : "false");
  j += ",\"tofValid\":";   j += (tofIsValid() ? "true" : "false");
  j += ",\"tofMm\":";      j += tofDistanceMm();
  j += ",\"tofText\":\"";  j += escapeJson(tofText()); j += "\"";
  j += ",\"runSec\":";     j += (millis() / 1000);
  j += "}";
  return j;
}

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleData() {
  server.send(200, "application/json; charset=utf-8", buildJson());
}

void handleNotFound() {
  server.send(404, "text/plain; charset=utf-8", "404 Not Found");
}

/* ---------------- WiFi ---------------- */

// 连不上路由器时的兜底：ESP32 自己开热点，网页照常可用
void startApMode() {
  WiFi.mode(WIFI_AP);
  delay(200);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println();
  Serial.println("========== 已切换到热点模式 ==========");
  if (!ok) {
    Serial.println("热点启动失败。");
    return;
  }
  apMode = true;
  Serial.print("热点名称: "); Serial.println(AP_SSID);
  Serial.print("热点密码: "); Serial.println(AP_PASS);
  Serial.print("网页地址: http://"); Serial.println(WiFi.softAPIP());
  Serial.println("用手机连上这个热点，再打开上面的地址即可查看页面。");
  Serial.println("（修好路由器密码后重新上电，会自动改回连路由器。）");
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  // 关闭 WiFi 省电模式：默认的省电模式会导致周期性掉线
  WiFi.setSleep(false);
  delay(100);

  // 开机先扫描一遍，把 ESP32 能看到的 WiFi 打印出来
  // （ESP32 只支持 2.4GHz，看不到 5GHz 的网络，用这个就能确认 SSID 该填哪个）
  Serial.println("正在扫描附近的 WiFi…");
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("没有扫描到任何网络，请确认路由器已开启 2.4GHz 频段。");
  } else {
    Serial.printf("扫描到 %d 个网络（ESP32 只能看到 2.4GHz 的）：\n", n);
    for (int i = 0; i < n; i++) {
      Serial.printf("  %2d) %s   信号 %d dBm   信道 %d\n",
                    i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
    }
    Serial.print("代码里 WIFI_SSID 必须是上面其中之一，当前填的是：");
    Serial.println(WIFI_SSID);
  }
  WiFi.scanDelete();
  WiFi.mode(WIFI_STA);
  delay(100);

#if USE_STATIC_IP
  if (WiFi.config(STATIC_IP, STATIC_GATEWAY, STATIC_SUBNET, STATIC_DNS)) {
    Serial.print("使用固定 IP: ");
    Serial.println(STATIC_IP);
  } else {
    Serial.println("固定 IP 配置失败，改用路由器自动分配。");
  }
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("正在连接 WiFi: %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

#if USE_STATIC_IP
  // 固定 IP 没连上（多半是网段不对或地址冲突），自动回退成自动获取
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("固定 IP 没有连上，改用路由器自动分配重试…");
    WiFi.disconnect(true, true);
    delay(200);
    WiFi.mode(WIFI_STA);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);   // 取消固定 IP
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("正在连接 WiFi: %s", WIFI_SSID);
    t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
  }
#endif

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi 连接成功，IP 地址: ");
    Serial.println(WiFi.localIP());
    Serial.print("请在浏览器打开测试页面: http://");
    Serial.println(WiFi.localIP());
    return;
  }

  // 连不上路由器：先说清原因，再开热点兜底
  int s = (int)WiFi.status();
  Serial.printf("WiFi 连接失败，状态码 %d。", s);
  if (s == WL_NO_SSID_AVAIL) {
    Serial.println("找不到这个网络名称，请对照上面的扫描结果核对 SSID。");
  } else if (s == WL_CONNECT_FAILED) {
    Serial.println("密码不对，请核对 WIFI_PASS。");
  } else {
    Serial.println("请确认 SSID 是 2.4GHz 频段且密码正确。");
  }

#if AP_FALLBACK
  startApMode();
#else
  Serial.println("后台会持续重试连接路由器。");
#endif
}

/* ---------------- 对外接口实现 ---------------- */

void netBegin() {
  connectWifi();
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("网页服务已启动。");
}

void netLoop() {
  server.handleClient();

  // 热点兜底模式下不再去重连路由器，避免把热点断掉
  if (apMode) return;

  // WiFi 掉线后自动重连：确认已经断开时才重试，
  // 避免在"正在连接"的状态下反复触发 wifi:sta is connecting 报错
  if (WiFi.status() == WL_DISCONNECTED && millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();
    Serial.println("WiFi 已断开，正在重连…");
    WiFi.reconnect();
  }
}

bool netConnected() { return WiFi.status() == WL_CONNECTED; }
String netIP()      { return WiFi.localIP().toString(); }
static const uint32_t DBG_BAUD = 115200;   // 串口监视器波特率

void setup() {
  Serial.begin(DBG_BAUD);
  delay(300);

  Serial.println();
  Serial.println("==================================================");
  Serial.println(" 北斗定位模块测试程序");
  Serial.println(" 通导一体化水上安全终端 · 移动终端定位子系统");
  Serial.println("==================================================");

  gpsBegin();   // 初始化北斗定位模块
  tofBegin();   // 初始化激光测距模块
  netBegin();   // 连接 WiFi 并启动网页服务

  Serial.println("初始化完成，开始接收定位数据。");
}

void loop() {
  gpsUpdate();   // 读取并解析定位数据（内部每秒刷新告警与位姿帧）
  tofUpdate();   // 读取激光测距数据
  netLoop();     // 处理网页请求与 WiFi 重连

  // 每秒打印一次定位结果到串口监视器
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    gpsPrintReport();
  }
}

