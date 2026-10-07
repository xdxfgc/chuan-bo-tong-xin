/* =====================================================================
   oled.cpp   OLED 画面实现
   ---------------------------------------------------------------------
   两种画面自动切换：
     收到过信标坐标 → 信标界面（坐标 / 方位 / 距离 / 指北箭头 / 本船坐标）
     还没收到信标   → 本船状态（卫星数 / 坐标 / 激光距离 / 时间与精度）
   ===================================================================== */

#include "oled.h"
#include "gnss.h"
#include "ownpos.h"     // 本船位置来源（北斗 / 手动坐标）
#include "tof.h"
#include "imu.h"
#include "mag.h"
#include "beacon.h"
#include "berth.h"
#include "anchor.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
static unsigned long s_lastMs = 0;

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

void oledBegin() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);   // 与激光共用总线，重复调用无副作用
  Wire.setClock(400000);
  u8g2.setI2CAddress(OLED_ADDR * 2);        // U8g2 的地址要左移一位
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  Serial.printf("OLED 已启动：SDA=GPIO%d  SCL=GPIO%d  地址 0x%02X\n",
                OLED_SDA_PIN, OLED_SCL_PIN, OLED_ADDR);
}

/* ---------------- 画面 A：信标界面 ---------------- */

static void renderBeacon() {
  char buf[32];

  /* 第 1 行：标题（带信标编号）+ 链路状态 */
  if (beaconId() <= 0) {
    drawCN(0, 11, "信标坐标");            // 老格式帧没带编号
  } else {
    char title[20];
    snprintf(title, sizeof(title), "信标%d", beaconId());
    drawCN(0, 11, title);
  }
  if (beaconLinkUp()) {
    snprintf(buf, sizeof(buf), "%ddBm", beaconRssi());
    drawSM(128 - smW(buf), 11, buf);
  } else {
    drawCN(128 - 36, 11, "无信号");
  }

  /* 第 2、3 行：信标经纬度 */
  if (beaconTargetValid()) {
    snprintf(buf, sizeof(buf), "%c%.6f", beaconLat() >= 0 ? 'N' : 'S', fabs(beaconLat()));
    drawSM(0, 23, buf);
    snprintf(buf, sizeof(buf), "%c%.6f", beaconLon() >= 0 ? 'E' : 'W', fabs(beaconLon()));
    drawSM(0, 35, buf);
  } else {
    drawCN(0, 23, "信标未定位");
  }

  /* 第 4 行：方位 + 距离 + 方向箭头
     能用相对船头方位时，箭头和文字都是「相对船头」，并在上方画一个小尖表示屏幕上方=船头；
     没有磁力计时退回绝对方位，箭头就是指南针的指北箭头。 */
  if (beaconHaveDir()) {
    float d = beaconDistM();
    bool  rel = beaconUseRel();
    int   dx  = rel ? 40 : 28;                 // 相对方位词是 3 个字，要留宽一点

    drawCN(0, 47, rel ? beaconRelDirText() : beaconDirText());
    if (d < 1000.0f) snprintf(buf, sizeof(buf), "%d", (int)(d + 0.5f));
    else             snprintf(buf, sizeof(buf), "%.1f", d / 1000.0f);
    drawSM(dx, 47, buf);
    drawCN(dx + smW(buf) + 2, 47, d < 1000.0f ? "米" : "公里");

    if (rel) {                                  // 船头参考：屏幕上方 = 船头
      u8g2.drawLine(114, 33, 117, 30);
      u8g2.drawLine(120, 33, 117, 30);
    }
    drawArrow(117, 43, 6, beaconArrowBearing());
  } else if (beaconTargetValid()) {
    drawCN(0, 47, "本船未定位");
  }

  /* 第 5 行：本船坐标 */
  drawCN(0, 59, "本船");
  /* 这里跟着"系统认为的本船位置"走：室内演示时可能是手动坐标 */
  if (ownPosValid()) {
    snprintf(buf, sizeof(buf), "%.4f %.4f", ownPosLat(), ownPosLon());
    drawSM(26, 59, buf);
  } else {
    drawCN(26, 59, "未定位");
  }
}

/* ---------------- 画面 B：本船状态 ---------------- */

static void renderOwn() {
  const GpsStatus& g = gpsGet();
  char buf[32];

  /* 第 1 行：标题 + 卫星数 */
  drawCN(0, 11, "本船状态");
  if (g.valid) {
    snprintf(buf, sizeof(buf), "%d星 %s", g.satsUsed, g.fixType == 3 ? "3D" : "2D");
  } else {
    snprintf(buf, sizeof(buf), "%d星", g.satsUsed);
  }
  drawSM(128 - smW(buf), 11, buf);

  /* 第 2、3 行：本船经纬度 */
  if (g.valid) {
    snprintf(buf, sizeof(buf), "%c%.6f", g.lat >= 0 ? 'N' : 'S', fabs(g.lat));
    drawSM(0, 23, buf);
    snprintf(buf, sizeof(buf), "%c%.6f", g.lon >= 0 ? 'E' : 'W', fabs(g.lon));
    drawSM(0, 35, buf);
  } else {
    drawCN(0, 23, "等待定位…");
  }

  /* 第 4 行：激光距离 + 航向 */
  drawCN(0, 47, "激光");
  if (!tofIsReady()) {
    drawCN(26, 47, "未连接");
  } else if (tofIsValid()) {
    snprintf(buf, sizeof(buf), "%umm", tofDistanceMm());
    drawSM(26, 47, buf);
  } else {
    drawCN(26, 47, "无效");
  }

  drawCN(64, 47, "航向");
  if (magPresent() && magCalibrated()) {
    snprintf(buf, sizeof(buf), "%.0f", magHeadingDeg());
    drawSM(90, 47, buf);
  } else {
    drawSM(90, 47, "--");
  }

  /* 第 5 行：俯仰与横滚 */
  const ImuData& im = imuGet();
  if (im.ready) {
    drawCN(0, 59, "俯仰");
    snprintf(buf, sizeof(buf), "%.1f", im.pitch);
    drawSM(26, 59, buf);
    drawCN(60, 59, "横滚");
    snprintf(buf, sizeof(buf), "%.1f", im.roll);
    drawSM(86, 59, buf);
  } else {
    drawCN(0, 59, "姿态未连接");
  }
}

/* ---------------- 对外接口 ---------------- */

/* ---------------- 画面 C：靠泊 ---------------- */

static void renderBerth() {
  char buf[32];
  uint8_t a = berthAlarmCode();

  /* 第 1 行：标题 + 等级 */
  drawCN(0, 11, berthUsingSide() ? "靠泊·右舷" : "靠泊·船头");
  if (berthDocked())               drawCN(96, 11, "已靠妥");
  else if (a == 0x02 || a == 0x03) drawCN(96, 11, "严重");
  else if (a == 0x01)              drawCN(96, 11, "提醒");
  else                             drawCN(96, 11, "正常");

  /* 第 2 行：大号距离数字 + 单位 */
  if (berthValid() && berthDistanceM() >= 0.0f) {
    float d = berthDistanceM();
    bool  meter = (d >= 1.0f);
    if (meter) snprintf(buf, sizeof(buf), "%.2f", d);
    else       snprintf(buf, sizeof(buf), "%.0f", d * 100.0f);

    u8g2.setFont(u8g2_font_10x20_tf);
    int w = u8g2.getStrWidth(buf);
    int x = (128 - w - 28) / 2;
    if (x < 0) x = 0;
    u8g2.drawStr(x, 38, buf);
    drawCN(x + w + 3, 36, meter ? "米" : "厘米");
  } else {
    drawCN(34, 34, "测距无效");
  }

  /* 第 3 行：接近速度 */
  drawCN(0, 52, "接近");
  snprintf(buf, sizeof(buf), "%.2f", berthSpeedMps());
  drawSM(26, 52, buf);
  drawCN(26 + smW(buf) + 2, 52, "米每秒");

  /* 第 4 行：状态或告警 */
  if (berthDocked()) {
    drawCN(0, 63, "靠泊完成");
  } else {
    switch (a) {
      case 0x01: drawCN(0, 63, "接近速度偏大"); break;
      case 0x02: drawCN(0, 63, "接近速度过大"); break;
      case 0x03: drawCN(0, 63, "距岸过近");     break;
      default:   drawCN(0, 63, "缓慢接近中");   break;
    }
  }
}

/* ---------------- 画面 D：锚泊监测 ---------------- */

static void renderAnchor() {
  char buf[32];
  uint8_t a = anchorAlarmCode();

  /* 第 1 行：标题 + 等级 */
  drawCN(0, 11, "锚泊监测");
  if      (a == 0x22) drawCN(96, 11, "严重");
  else if (a == 0x21) drawCN(96, 11, "提醒");
  else                drawCN(96, 11, "正常");

  /* 第 2 行：大号位移数字 */
  float d = anchorDriftM();
  bool  meter = (d >= 1.0f);
  if (meter) snprintf(buf, sizeof(buf), "%.2f", d);
  else       snprintf(buf, sizeof(buf), "%.0f", d * 100.0f);

  u8g2.setFont(u8g2_font_10x20_tf);
  int w = u8g2.getStrWidth(buf);
  int x = (128 - w - 28) / 2;
  if (x < 0) x = 0;
  u8g2.drawStr(x, 38, buf);
  drawCN(x + w + 3, 36, meter ? "米" : "厘米");

  /* 第 3 行：漂移方向与速率 */
  drawCN(0, 52, "漂移");
  snprintf(buf, sizeof(buf), "%03.0f", anchorDriftDir());
  drawSM(26, 52, buf);
  drawCN(50, 52, "度");
  drawCN(68, 52, "速率");
  snprintf(buf, sizeof(buf), "%.3f", anchorDriftSpeed());
  drawSM(94, 52, buf);

  /* 第 4 行：基准坐标 */
  drawCN(0, 63, "基准");
  snprintf(buf, sizeof(buf), "%.4f %.4f", anchorRefLat(), anchorRefLon());
  drawSM(26, 63, buf);
}

void oledUpdate() {
  if (millis() - s_lastMs < OLED_REFRESH_MS) return;
  s_lastMs = millis();

  u8g2.clearBuffer();
  /* 画面优先级：落水告警最高，其次是靠泊，然后是锚泊，最后是信标与本船状态 */
  if (beaconAlarmActive())    renderBeacon();
  else if (berthShowOnScreen()) renderBerth();
  else if (anchorActive())      renderAnchor();
  else if (beaconHasTarget())   renderBeacon();
  else                          renderOwn();
  u8g2.sendBuffer();
}
