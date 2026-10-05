/* =====================================================================
   坐标发送端（信标 · 通信部分）· 主程序
   ---------------------------------------------------------------------
   硬件：ESP32-C3 SuperMini + SX1278 433MHz

   做三件事：
     1. 每 2 秒取一次自己的坐标（来源在 config.h 里选，共四种）
     2. 组帧后通过 LoRa 广播：M,<信标ID>,<序号>,P,<定位有效>,<纬度>,<经度>
     3. 等船端应答；收不到自动重传，最多 3 次

   代码结构（每加一项功能就再加一对 .h/.cpp，然后在 setup/loop 里各加一行）：

     config.h              引脚、射频参数、节奏、本机标识、坐标来源
     debug.h / debug.cpp   调试串口（兼容 C3 的两种 USB 模式）
     coord.h / coord.cpp   坐标来源（固定 / 绕点 / 本端北斗 / 串口输入）
     water.h / water.cpp   水感检测（落水触发）—— 电极 / 杜邦线 / 串口命令
     radio.h / radio.cpp   LoRa 收发底层（发一帧、收一帧）
     link.h / link.cpp     一问一答（发送 + 等应答 + 重传 + 统计）

   接线见 config.h 顶部；开发板选 ESP32C3 Dev Module，串口 115200。
   ===================================================================== */

#include "config.h"
#include "debug.h"
#include "coord.h"
#include "water.h"
#include "radio.h"
#include "link.h"

static uint32_t seq     = 0;  // 帧序号，每轮加一
static bool     saidIdle = false;   // "未入水，保持静默"只在开始时提示一次
#if SEND_ONLY_WET
static bool     s_reporting = false;   // 当前是不是处于"已确认落水、正在上报"的状态
#endif

void setup() {
  DBG.begin(115200);
  delay(1500);                // 等 USB 主机把串口打开，避免开头几个字丢掉
  DBG.println();
  DBG.println("===== 坐标发送端 ESP32-C3（信标） =====");
  DBG.printf("本机标识 DEV_ID = %d（每只信标要不一样）\n", DEV_ID);
  DBG.printf("坐标来源：%s\n", coordSourceText());
#if SRC_MODE == 0 || SRC_MODE == 1
  DBG.printf("基坐标：%.6f, %.6f\n", BASE_LAT, BASE_LON);
#endif

  coordBegin();               // 坐标来源（模式 2 会打开北斗串口）
  waterBegin();               // 水感检测（电极 / 杜邦线）
  radioBegin();               // 射频（内部会先按实际接线打开 SPI 总线）
}

void loop() {
  waterPollCommand();         // 串口模拟命令 wet on / wet off
  waterUpdate();              // 周期采样（每 WATER_SAMPLE_MS 毫秒一次）

  // 射频没起来就每秒重试一次，不会一直趴着
  if (!radioIsReady()) {
    delay(1000);
    radioBegin();
    return;
  }

#if SEND_ONLY_WET
  /* 三重确认里的"持续时间"：碰到水不算数，要连续湿够 WATER_CONFIRM_MS
     才认为真的落水了。这样线头抖一下、浪溅一下都不会触发。
     （采样间隔是 WATER_SAMPLE_MS，所以实际确认时间比设定值多半个采样周期） */
  bool wetConfirmed = waterIsWet() && (waterWetMs() >= WATER_CONFIRM_MS);

  if (!wetConfirmed) {
    if (s_reporting) {                 // 刚才还在上报，现在断了
      s_reporting = false;
      DBG.println("[信标] 已离水，恢复静默");
    }
    if (!waterIsWet() && !saidIdle) {  // 只是没入水，提示一次就够
      saidIdle = true;
      DBG.println("[信标] 未入水，保持静默（等入水才开始上报）");
    }
    /* 湿了但还没满确认时间 —— 水感模块自己的状态行会说明进展，这里不重复刷 */
    delay(100);                        // 别空转
    return;
  }

  if (!s_reporting) {
    s_reporting = true;
    saidIdle    = false;               // 下次离水后还要能重新提示一次
    DBG.printf("[信标] 确认落水（连续湿 %.1f 秒），开始上报\n", waterWetMs() / 1000.0);
  }
#endif

  coordPoll();                // 模式 2/3 需要读串口

  char payload[32];
  coordBuildPayload(payload, sizeof(payload));   // 组出 P,<有效>,<纬度>,<经度>

  char tx[64];
  snprintf(tx, sizeof(tx), "M,%d,%lu,%s", DEV_ID, (unsigned long)seq, payload);

  DBG.printf("[第 %lu 轮] 发出 %s\n", (unsigned long)seq, tx);

  String ack;
  int    attempts = 0;
  if (!linkSendWithAck(tx, ack, attempts)) {
    DBG.println("           本轮失败，下一轮继续");
  }

  seq++;
  linkPrintStats();
  delay(ROUND_PERIOD_MS);
}
