/* =====================================================================
   water.h   水感检测（落水触发）
   ---------------------------------------------------------------------
   硬件：两片不锈钢电极，分别接 WATER_PIN_A / WATER_PIN_B，间距 3~5 毫米。
         自制电路，不需要任何电阻电容。

   演示替代：
     - 两根杜邦线分别插在这两个脚上，把线头碰在一起 = 入水
     - 或者用串口命令 wet on / wet off 强制模拟（WATER_SIM_CMD 打开时）
   ===================================================================== */

#ifndef CB_WATER_H
#define CB_WATER_H

#include "config.h"
#include "debug.h"

void waterBegin();          // 配脚（平时高阻，零功耗）
void waterUpdate();         // 周期采样，建议每轮 loop 都调
void waterPollCommand();    // 读串口模拟命令（只有 WATER_SIM_CMD 打开时才有效）

bool          waterIsWet();     // 此刻有没有水
unsigned long waterWetMs();     // 已经连续湿了多少毫秒（给三重确认用）
void          waterPrintStatus();   // 打印一行当前状态（定期打印和 wet 命令都用它）

#endif
