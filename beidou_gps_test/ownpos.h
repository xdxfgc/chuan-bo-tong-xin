/* =====================================================================
   ownpos.h   本船位置来源（北斗 / 手动坐标）
   ---------------------------------------------------------------------
   全船只有一个地方回答"本船在哪"：就是这里。
   默认用北斗；室内演示时可以切到手动坐标（存在 flash 里，重启不丢）。

   用法：
     ownPosValid() / ownPosLat() / ownPosLon()   —— 算方位距离时一律用这一组
     gpsGet()                                    —— 只有"北斗自己的状态"
                                                     （卫星数、HDOP、原始语句、
                                                      锚泊监测）才直接用
   ===================================================================== */

#ifndef CB_OWNPOS_H
#define CB_OWNPOS_H

#include "config.h"

void   ownPosBegin();                    // 开机：从 flash 读回上次的手动坐标

bool   ownPosManual();                   // 现在用的是不是手动坐标
bool   ownPosValid();                    // 本船位置能不能用（手动有效 或 北斗有效）
double ownPosLat();
double ownPosLon();

bool   ownPosSet(double lat, double lon);   // 切到手动坐标并保存，坐标不合法返回 false
void   ownPosClear();                        // 切回北斗

String ownPosSrcText();                  // “北斗” / “手动（模拟）”
String ownPosCmd(const String& arg);     // 串口命令 pos 的处理，返回要打印的文字

#endif
