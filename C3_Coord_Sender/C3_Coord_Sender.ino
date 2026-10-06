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
     posture.h / posture.cpp  姿态判定（MPU6050）—— 三重确认的第二条
     trigger.h / trigger.cpp  落水激活状态机（三重确认合在一起）
     button.h / button.cpp    板载 BOOT 键：长按 5 秒复位
     cmd.h / cmd.cpp          串口命令（全工程唯一读串口的地方）
     radio.h / radio.cpp   LoRa 收发底层（发一帧、收一帧）
     link.h / link.cpp     一问一答（发送 + 等应答 + 重传 + 统计）

   接线见 config.h 顶部；开发板选 ESP32C3 Dev Module，串口 115200。
   ===================================================================== */

#include "config.h"
#include "debug.h"
#include "coord.h"
#include "water.h"
#include "posture.h"
#include "trigger.h"
#include "button.h"
#include "cmd.h"
#include "radio.h"
#include "link.h"
#include "power.h"

static uint32_t seq     = 0;  // 帧序号，每轮加一
static bool     saidIdle = false;   // "没在激活状态，保持静默"只在开始时提示一次

#if ENABLE_DEEP_SLEEP
/* 深睡模式下的两个状态：
     gActiveMode —— 本次上电是不是"确认入水"了（没确认就直接睡了，不会到这）
     gLastWetMs  —— 最近一次"水感导通"的时刻，用来判断"离水多久了" */
static bool          gActiveMode = false;
static unsigned long gLastWetMs  = 0;
#endif

/* 等水感确认：连续湿够 WATER_CONFIRM_MS 毫秒返回 true，超时返回 false。
   等待期间照常采样。                                              */
static bool waterConfirm(uint32_t timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    waterUpdate();
    if (waterIsWet() && waterWetMs() >= WATER_CONFIRM_MS) return true;
    delay(20);
  }
  return false;
}

/* 等待下一轮 —— 但不能死等。
   信标平时 8 秒一帧，如果这里直接 delay(8000)，那 8 秒里水感根本不采样：
   用户碰住电极，最长要等 8 秒才被发现，再加 2 秒确认，
   文档要求的"20 次入水试验激活时间不超过 3 秒"就达不到了。

   所以改成：等待期间照常采样，**水感状态一变就立刻返回**，马上发一帧。
   这样"碰住电极 → 报警"的总延迟只有 WATER_CONFIRM_MS（2 秒）左右。       */
static void waitNextRound(uint32_t period, bool wetNow) {
  unsigned long t0 = millis();
  while (millis() - t0 < period) {
    delay(20);
    waterUpdate();                              // 等待期间照常采样
    bool wet1 = waterIsWet() && (waterWetMs() >= WATER_CONFIRM_MS);
    if (wet1 != wetNow) return;                 // 状态变了，立刻开下一轮
  }
}

void setup() {
  powerBegin();               // 最先做：解除上次休眠留下的引脚保持

  DBG.begin(115200);

  /* 冷启动等久一点，让 USB 主机把串口打开（不然开头几个字丢掉）；
     被水感叫醒时抢时间，只等一小会儿。 */
#if ENABLE_DEEP_SLEEP
  delay(powerWokeByWater() ? WAKE_LOG_DELAY_MS : 1500);
#else
  delay(1500);
#endif
  DBG.println();
  DBG.println("===== 坐标发送端 ESP32-C3（信标） =====");
  DBG.printf("本机标识 DEV_ID = %d（每只信标要不一样）\n", DEV_ID);
  DBG.printf("坐标来源：%s\n", coordSourceText());
#if SRC_MODE == 0 || SRC_MODE == 1
  DBG.printf("基坐标：%.6f, %.6f\n", BASE_LAT, BASE_LON);
#endif

  /* 唤醒原因：区分"冷启动"和"被水感叫醒" */
  bool byWater = powerWokeByWater();
  DBG.printf("唤醒原因：%s\n", byWater ? "水感电极" : "冷启动 / 复位");

  /* 被水感叫醒的话，先把北斗的电加上，让它早点开始搜星 */
  if (byWater) powerGpsOn();

  coordBegin();               // 坐标来源（模式 2 会打开北斗串口）
  if (byWater) coordWake();   // 北斗只是待机过的话，叫它醒来
  waterBegin();               // 水感检测（电极 / 杜邦线）
  postureBegin();             // 姿态传感器（MPU6050，I2C GPIO0/1）
  triggerBegin();             // 落水激活状态机
  buttonBegin();              // 板载 BOOT 键（长按复位）
  radioBegin();               // 射频（内部会先按实际接线打开 SPI 总线）
  cmdBegin();                 // 串口命令提示

#if ENABLE_DEEP_SLEEP
  /* 上电之后先给水感一段确认时间：
       确认入水  → 进入上报模式（下面 loop 里一直发）
       没确认    → 说明是误触发，或者只是正常上电，直接睡回去

     唤醒 ≠ 落水：水把电极接通只是一瞬间的事，浪花、冷凝水都能做到。
     真正的判定还是"连续湿够 WATER_CONFIRM_MS"。                     */
  DBG.printf("[电源] 等待水感确认（要连续湿 %lu ms）…\n",
             (unsigned long)WATER_CONFIRM_MS);
  if (!waterConfirm(WATER_CONFIRM_MS + 800)) {
    DBG.println("[电源] 没检测到入水（误触发或者只是上电）");
    powerGoSleep();           // 睡下去，不返回
  }
  gActiveMode = true;
  gLastWetMs  = millis();
  DBG.println("[电源] 确认入水，进入上报模式");
#endif
}

void loop() {
  cmdPoll();                  // 串口命令
  buttonUpdate();             // 板载按键（长按 5 秒复位）
  waterUpdate();              // 水感采样（每 WATER_SAMPLE_MS 毫秒一次）
  postureUpdate();            // 姿态读取（每 POSTURE_READ_MS 毫秒一次）
  triggerUpdate();            // 三重确认状态机
  posturePrintReport();       // 姿态数据行（每秒一行）

  if (buttonResetEvent()) triggerReset();   // BOOT 键长按 5 秒 = 复位

#if ENABLE_DEEP_SLEEP
  /* 上报模式下的"回睡"判断：
       水感导通 → 刷新"最近湿的时刻"
       一直不导通、且超过 IDLE_TO_SLEEP_MS → 认为已经离开水面，回去睡
     用"持续一段时间"而不是"一离开就睡"，是为了不被浪盖住又露出来打断。 */
  if (gActiveMode) {
    if (waterIsWet()) {
      gLastWetMs = millis();
    } else if (millis() - gLastWetMs > IDLE_TO_SLEEP_MS) {
      DBG.printf("[电源] 已离水超过 %lu 秒，回去睡\n",
                 (unsigned long)(IDLE_TO_SLEEP_MS / 1000));
      powerGoSleep();           // 睡下去，不返回
    }
  }
#endif

  // 射频没起来就每秒重试一次，不会一直趴着
  if (!radioIsReady()) {
    delay(1000);
    radioBegin();
    return;
  }

#if SEND_ONLY_WET
  /* 上报的条件就是"三重确认通过"。
     三重确认 = 水感连续湿够 2 秒 + 姿态符合漂浮特征，全在 trigger 里判。
     一旦激活就一直上报，直到长按 BOOT 键 5 秒复位 —— 中途被浪盖住
     又露出来也不会停，这才是文档 6.3 要的行为。                     */
  if (!triggerActive()) {
    if (!saidIdle) {
      saidIdle = true;
      DBG.println("[信标] 未激活，保持静默（等三重确认通过才开始上报）");
    }
    delay(100);                        // 别空转
    return;
  }
  saidIdle = false;                    // 下次复位后还要能重新提示一次
#endif

  coordPoll();                // 模式 2/3 需要读串口

  /* 水感状态：**连续湿够 WATER_CONFIRM_MS 才算 1**，碰一下就抖开的不算。
     这个位跟着每一帧发出去，船端看它决定要不要报警 ——
     所以信标可以一直发（链路随时在线），但船端只在真的入水时才响。 */
  bool wet = waterIsWet() && (waterWetMs() >= WATER_CONFIRM_MS);

  char payload[32];
  coordBuildPayload(payload, sizeof(payload));   // 组出 P,<有效>,<纬度>,<经度>

  /* 帧格式：M,<信标ID>,<序号>,P,<定位有效>,<纬度>,<经度>,<水感 0/1>
     最后那个字段是新加的；老固件的船端读到多余字段会自动忽略，不受影响。 */
  char tx[72];
  snprintf(tx, sizeof(tx), "M,%d,%lu,%s,%d",
           DEV_ID, (unsigned long)seq, payload, wet ? 1 : 0);

  DBG.printf("[第 %lu 轮] 发出 %s\n", (unsigned long)seq, tx);

  String ack;
  int    attempts = 0;
  if (!linkSendWithAck(tx, ack, attempts)) {
    DBG.println("           本轮失败，下一轮继续");
  }

  seq++;
  linkPrintStats();

  /* 平时慢发、入水快发。
     正常值守 8 秒一帧，占空比约 6%；入水后 2 秒一帧，占空比约 21%。
     一眼就能在串口上看出现在是哪一档。                              */
  static bool lastWet = false;
  if (wet != lastWet) {
    lastWet = wet;
    DBG.printf("[节奏] 切到%s：每 %lu ms 发一帧\n",
               wet ? "入水快发" : "值守慢发",
               (unsigned long)(wet ? ROUND_PERIOD_MS : IDLE_HEARTBEAT_MS));
  }

  /* 用"可打断的等待"代替 delay：水感一变就立刻发下一帧，
     不然碰住电极要等最多 8 秒才被上报。 */
  waitNextRound(wet ? ROUND_PERIOD_MS : IDLE_HEARTBEAT_MS, wet);
}
