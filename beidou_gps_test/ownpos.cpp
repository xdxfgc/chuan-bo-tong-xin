/* =====================================================================
   ownpos.cpp   本船位置来源实现
   ---------------------------------------------------------------------
   手动坐标存在 Preferences（flash）里：
     on   1 = 用手动坐标，0 = 用北斗
     lat/lon  手动填的坐标
   为什么存 flash：室内演示经常要断电挪地方，不存的话每次开机都得重填。
   ===================================================================== */

#include "ownpos.h"
#include "gnss.h"
#include <Preferences.h>
#include <math.h>

#if OWN_POS_ENABLE

static Preferences sPrefs;

static bool   sManual = false;      // 当前是不是手动模式
static double sLat    = 0.0;
static double sLon    = 0.0;

/* 坐标合法性：范围要对，而且不能是 (0,0) ——
   那个点是几内亚湾，明显是没填不是真位置。 */
static bool coordOk(double lat, double lon) {
  if (!(lat >= -90.0 && lat <= 90.0))   return false;
  if (!(lon >= -180.0 && lon <= 180.0)) return false;
  if (fabs(lat) < 1e-6 && fabs(lon) < 1e-6) return false;
  return true;
}

void ownPosBegin() {
  sPrefs.begin("ownpos", true);                 // 只读打开
  sManual = (sPrefs.getUChar("on", 0) != 0);
  sLat    = sPrefs.getDouble("lat", 0.0);
  sLon    = sPrefs.getDouble("lon", 0.0);
  sPrefs.end();

  if (sManual && !coordOk(sLat, sLon)) sManual = false;   // 存坏了就退回北斗

  Serial.printf("[位置] 来源：%s", ownPosSrcText().c_str());
  if (sManual) Serial.printf("（%.6f, %.6f）", sLat, sLon);
  Serial.println();
  if (sManual)
    Serial.println("[位置] 手动坐标期间走锚监测不可用，要用请串口敲 pos auto");
}

bool ownPosManual() { return sManual; }

bool ownPosValid() {
  if (sManual) return coordOk(sLat, sLon);
  return gpsGet().valid;
}

double ownPosLat() {
  if (sManual) return sLat;
  return gpsGet().lat;
}

double ownPosLon() {
  if (sManual) return sLon;
  return gpsGet().lon;
}

bool ownPosSet(double lat, double lon) {
  if (!coordOk(lat, lon)) return false;

  sLat    = lat;
  sLon    = lon;
  sManual = true;

  sPrefs.begin("ownpos", false);
  sPrefs.putUChar("on", 1);
  sPrefs.putDouble("lat", lat);
  sPrefs.putDouble("lon", lon);
  sPrefs.end();

  Serial.printf("[位置] 已切到手动坐标：%.6f, %.6f（已存 flash，重启不丢）\n", lat, lon);
  return true;
}

void ownPosClear() {
  sManual = false;

  sPrefs.begin("ownpos", false);
  sPrefs.putUChar("on", 0);
  sPrefs.end();

  Serial.println("[位置] 已切回北斗定位");
}

String ownPosSrcText() {
  return sManual ? String("手动（模拟）") : String("北斗");
}

/* 串口命令：
     pos                       看当前状态
     pos auto                  切回北斗
     pos 26.212676,111.599388  设定手动坐标（逗号或空格分隔都行） */
String ownPosCmd(const String& arg) {
  String a = arg;
  a.trim();
  String out;

  if (a.length() == 0) {
    out  = "[位置] 来源：" + ownPosSrcText();
    if (sManual) {
      char b[64];
      snprintf(b, sizeof(b), "  当前坐标 %.6f, %.6f", sLat, sLon);
      out += String(b);
    } else {
      out += ownPosValid() ? "  北斗已定位" : "  北斗还没定位";
    }
    out += "\n用法：pos 26.212676,111.599388   设定手动坐标\n      pos auto                    切回北斗";
    return out;
  }

  if (a.equalsIgnoreCase("auto") || a.equalsIgnoreCase("gps")) {
    ownPosClear();
    return String("[位置] 已切回北斗定位");
  }

  /* 逗号或空格分隔 */
  int sep = a.indexOf(',');
  if (sep < 0) sep = a.indexOf(' ');
  if (sep <= 0) return String("[位置] 格式不对。用：pos 26.212676,111.599388");

  double la = a.substring(0, sep).toDouble();
  double lo = a.substring(sep + 1).toDouble();
  if (!ownPosSet(la, lo))
    return String("[位置] 坐标不合法：纬度 -90~90，经度 -180~180，且不能是 0,0");

  char b[96];
  snprintf(b, sizeof(b), "[位置] 已切到手动坐标 %.6f, %.6f（已存 flash）", la, lo);
  return String(b);
}

#else   /* OWN_POS_ENABLE == 0：整个功能不编译，全部退回北斗 */

void   ownPosBegin() {}
bool   ownPosManual() { return false; }
bool   ownPosValid()  { return gpsGet().valid; }
double ownPosLat()    { return gpsGet().lat; }
double ownPosLon()    { return gpsGet().lon; }
bool   ownPosSet(double, double) { return false; }
void   ownPosClear() {}
String ownPosSrcText() { return String("北斗"); }
String ownPosCmd(const String&) { return String("[位置] 本功能未启用"); }

#endif
