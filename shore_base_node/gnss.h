/* =====================================================================
   gnss.h   北斗定位模块（ATGM336H-5N）
   ---------------------------------------------------------------------
   读取 NMEA、解析定位数据、按文档组 0x01 位姿帧、产生定位失锁告警。
   ===================================================================== */

#ifndef CB_GNSS_H
#define CB_GNSS_H

#include "config.h"

/* 定位数据（字段对应文档表27 的 0x01 位姿帧） */
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

void gpsBegin();                 // 初始化定位串口
void gpsUpdate();                // 周期调用：读串口、解析、刷新告警与位姿帧
void gpsPrintReport();           // 打印定位状态到串口监视器

const GpsStatus& gpsGet();       // 当前定位数据
uint8_t gpsAlarmCode();          // 告警码（文档附录B）
String  gpsAlarmText();          // 告警文字
String  gpsUtcTime();            // UTC 时间
String  gpsBeijingTime();        // 北京时间
String  gpsDateText();           // 日期
String  gpsAntenna();            // 天线状态文字
String  gpsPoseFrameHex();       // 0x01 位姿帧的十六进制字符串
String  gpsLastGga();            // 最近一条原始 GGA 语句

#endif
