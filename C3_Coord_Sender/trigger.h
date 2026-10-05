/* =====================================================================
   trigger.h   落水激活判定（三重确认的状态机）
   ---------------------------------------------------------------------
   文档的"三重确认"：
     水感检测  +  姿态判定  +  持续时间
         ↑            ↑            ↑
      主触发      抑制误报     滤掉瞬时干扰

   一旦确认激活就**一直保持**，直到人工复位（BOOT 键长按 5 秒）——
   这是文档 6.3 的行为：打捞上来擦干，长按复位键五秒才退出。
   中途被浪盖住又露出来、或者短暂离水，都不会退出。
   ===================================================================== */

#ifndef CB_TRIGGER_H
#define CB_TRIGGER_H

#include "config.h"
#include "debug.h"

void triggerBegin();
void triggerUpdate();          // 周期调用：推进状态机
bool triggerActive();          // 落水已确认（上报的条件就是它）
void triggerReset();           // 人工复位，退出激活状态

void        triggerPrintReport();
const char* triggerStateText();
unsigned    triggerCount();    // 上电以来激活过几次

#endif
