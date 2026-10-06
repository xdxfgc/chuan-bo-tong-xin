/* =====================================================================
   oled.cpp   岸基节点 OLED 画面实现
   ---------------------------------------------------------------------
   两种画面自动切换：
     收到过信标坐标 → 信标界面（坐标 / 方位 / 距离 / 指北箭头 / 本节点坐标）
     还没收到信标   → 岸基状态（卫星数 / 本节点坐标 / 信标与船端在线 / 岸侧测距）

   和船端用同一套 U8g2 写法，去掉了激光、姿态、磁力计、靠泊、走锚那些
   岸基节点没有的内容。
   ===================================================================== */

#include "oled.h"
#include "gnss.h"
#include "track.h"
#include "tof.h"
#include <U8g2lib.h>
#include <math.h>

/* 硬件 I2C 驱动：和船端同一套写法，接线 SDA=GPIO21 / SCL=GPIO22。
   构造参数顺序：旋转、RES（I2C 版没有，填 U8X8_PIN_NONE）、SCL、SDA。
   如果手上的屏其实是 SH1106（不是 SSD1306），把下面这行的型号换一下即可：
     U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN); */
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);
static unsigned long s_lastMs = 0;

static bool    s_begun = false;   // 是否已经初始化（网页上要显示）
static uint8_t s_addr  = 0;       // 开机扫描到的实际 I2C 地址（0 = 没扫到）

static void useCN() { u8g2.setFont(u8g2_font_wqy12_t_gb2312); }   // 中文（GB2312）
static void useSM() { u8g2.setFont(u8g2_font_6x12_tf); }          // 数字与字母
static void drawCN(int x, int y, const char* s) { useCN(); u8g2.drawUTF8(x, y, s); }
static void drawSM(int x, int y, const char* s) { useSM(); u8g2.drawStr(x, y, s); }
static int  smW(const char* s) { useSM(); return u8g2.getStrWidth(s); }

// 指北箭头：从 (cx,cy) 朝方位角 bearing 画一段线加两片箭头
static void drawArrow(int cx, int cy, int r, float bearing) {
  double rad = bearing * M_PI / 180.0;
  int tx = cx + (int)lround(r * sin(rad));
  int ty = cy - (int)lround(r * cos(rad));
  u8g2.drawLine(cx, cy, tx, ty);
  for (int k = -1; k <= 1; k += 2) {
    double a = rad + M_PI + k * 0.5;
    u8g2.drawLine(tx, ty,
                  tx + (int)lround(4 * sin(a)),
                  ty - (int)lround(4 * cos(a)));
  }
}

/* 扫一遍 I2C 总线：既能确认屏到底在不在、地址是 0x3C 还是 0x3D，
   也能顺便看到总线上还有没有别的设备（船端那条会多出 0x29 和 0x68）。 */
static uint8_t scanI2C() {
  uint8_t found = 0;
  Serial.printf("I2C 扫描（SDA=GPIO%d SCL=GPIO%d）：\n", OLED_SDA_PIN, OLED_SCL_PIN);
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  发现设备 0x%02X\n", a);
      if (!found && (a == 0x3C || a == 0x3D)) found = a;
    }
  }
  if (!found) Serial.println("  没扫到 0x3C/0x3D：查 VCC(3.3V)、GND、SDA->GPIO21、SCL->GPIO22 是否接牢");
  return found;
}

void oledBegin() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);   // 先起 I2C 总线
  Wire.setClock(400000);                    // 不提速的话一屏 1024 字节要阻塞近 100ms
  s_addr = scanI2C();

  /* 扫到哪个地址就用哪个，避免模块是 0x3D 时黑屏 */
  if (s_addr) u8g2.setI2CAddress((uint8_t)(s_addr << 1));   // U8g2 用 8 位地址
  u8g2.begin();
  s_begun = true;

  /* 先亮一屏，接对了就该立刻看到这三行 */
  u8g2.clearBuffer();
  useCN();
  u8g2.drawUTF8(0, 16, "岸基节点");
  u8g2.drawUTF8(0, 34, "屏幕就绪");
  useSM();
  u8g2.drawStr(0, 54, "OLED I2C OK");
  u8g2.sendBuffer();

  Serial.printf("OLED 已启动：I2C SDA=GPIO%d SCL=GPIO%d 地址=0x%02X\n",
                OLED_SDA_PIN, OLED_SCL_PIN, s_addr ? s_addr : OLED_ADDR);
}

const char* oledStatusText() {
  static char buf[48];
  if (!s_begun) return "未初始化";
  if (s_addr) snprintf(buf, sizeof(buf), "I2C 0x%02X SDA%d SCL%d", s_addr, OLED_SDA_PIN, OLED_SCL_PIN);
  else        snprintf(buf, sizeof(buf), "I2C 未扫到屏 SDA%d SCL%d", OLED_SDA_PIN, OLED_SCL_PIN);
  return buf;
}

/* ---------------- 画面 A：信标界面 ---------------- */

static void renderBeacon() {
  const TrackTarget& b = trackBeacon();
  const GpsStatus&   g = gpsGet();
  char buf[32];

  /* 第 1 行：标题 + 链路状态 */
  drawCN(0, 11, "信标坐标");
  if (b.linkUp) {
    snprintf(buf, sizeof(buf), "%ddBm", b.rssi);
    drawSM(128 - smW(buf), 11, buf);
  } else {
    drawCN(128 - 36, 11, "无信号");
  }

  /* 第 2、3 行：信标经纬度 */
  if (b.valid) {
    snprintf(buf, sizeof(buf), "%c%.6f", b.lat >= 0 ? 'N' : 'S', fabs(b.lat));
    drawSM(0, 23, buf);
    snprintf(buf, sizeof(buf), "%c%.6f", b.lon >= 0 ? 'E' : 'W', fabs(b.lon));
    drawSM(0, 35, buf);
  } else {
    drawCN(0, 23, "信标未定位");
  }

  /* 第 4 行：绝对方位 + 距离 + 指北箭头（岸基没有船头方向，一律绝对方位） */
  if (b.haveDir) {
    float d = b.distM;
    drawCN(0, 47, trackDirText(b));
    if (d < 1000.0f) snprintf(buf, sizeof(buf), "%d", (int)(d + 0.5f));
    else             snprintf(buf, sizeof(buf), "%.1f", d / 1000.0f);
    drawSM(28, 47, buf);
    drawCN(28 + smW(buf) + 2, 47, d < 1000.0f ? "米" : "公里");
    drawArrow(117, 43, 6, b.bearing);
  } else if (b.valid) {
    drawCN(0, 47, "本节点未定位");
  }

  /* 第 5 行：本节点坐标 */
  drawCN(0, 59, "本机");
  if (g.valid) {
    snprintf(buf, sizeof(buf), "%.4f %.4f", g.lat, g.lon);
    drawSM(26, 59, buf);
  } else {
    drawCN(26, 59, "未定位");
  }
}

/* ---------------- 画面 B：岸基状态 ---------------- */

static void renderOwn() {
  const GpsStatus&   g = gpsGet();
  const TrackTarget& b = trackBeacon();
  const TrackTarget& v = trackVessel();
  char buf[32];

  /* 第 1 行：标题 + 卫星数 */
  drawCN(0, 11, "岸基节点");
  if (g.valid) snprintf(buf, sizeof(buf), "%d星 %s", g.satsUsed, g.fixType == 3 ? "3D" : "2D");
  else         snprintf(buf, sizeof(buf), "%d星", g.satsUsed);
  drawSM(128 - smW(buf), 11, buf);

  /* 第 2、3 行：本节点经纬度 */
  if (g.valid) {
    snprintf(buf, sizeof(buf), "%c%.6f", g.lat >= 0 ? 'N' : 'S', fabs(g.lat));
    drawSM(0, 23, buf);
    snprintf(buf, sizeof(buf), "%c%.6f", g.lon >= 0 ? 'E' : 'W', fabs(g.lon));
    drawSM(0, 35, buf);
  } else {
    drawCN(0, 23, "等待定位…");
  }

  /* 第 4 行：信标与船端在线状态 */
  drawCN(0, 47, "信标");
  drawCN(30, 47, b.linkUp ? "在线" : "离线");
  drawCN(64, 47, "船端");
  drawCN(94, 47, v.linkUp ? "在线" : "离线");

  /* 第 5 行：岸侧激光测距 */
  drawCN(0, 59, "岸侧");
  if (!tofIsReady()) {
    drawCN(26, 59, "未接");
  } else if (!tofIsValid()) {
    drawCN(26, 59, "无效");
  } else {
    snprintf(buf, sizeof(buf), "%.2f m", tofDistanceMm() / 1000.0f);
    drawSM(26, 59, buf);
  }
}

/* ---------------- 对外接口 ---------------- */

void oledUpdate() {
  if (millis() - s_lastMs < OLED_REFRESH_MS) return;
  s_lastMs = millis();

  u8g2.clearBuffer();
  if (trackBeacon().has) renderBeacon();
  else                   renderOwn();
  u8g2.sendBuffer();
}