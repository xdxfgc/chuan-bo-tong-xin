/* =====================================================================
   船舶终端（主程序）
   通导一体化水上安全终端 · 移动终端
   ---------------------------------------------------------------------
   硬件：ESP32 + 北斗 ATGM336H-5N + 激光 VL53L1X + LoRa SX1278 + SYN6288

   代码结构（每加一个硬件，就再加一对 .h/.cpp，然后在 setup/loop 里各加一行）：

     config.h                全部引脚与参数，换接线只改这一个文件
     gb2312_voice.h          SYN6288 播报用的 GB2312 词表
     gnss.h / gnss.cpp       北斗定位：NMEA 解析、位姿帧、定位失锁告警
     tof.h / tof.cpp         激光测距 VL53L1X
     oled.h / oled.cpp       OLED 显示（SSD1306，与激光共用 I2C 总线）
     imu.h / imu.cpp         MPU6050 六轴姿态（同样挂在 I2C 总线上）
     mag.h / mag.cpp         磁力计 GY-282（HMC5983）：航向、标定，走独立的 Wire1
     berth.h / berth.cpp     靠泊辅助：距离滤波、接近速度、分级告警、靠妥判定
     buzzer.h / buzzer.cpp   蜂鸣器：汇总各级告警，用不同节奏响铃
     anchor.h / anchor.cpp   走锚监测：基准位置、位移、漂移趋势、分级告警
     logbook.h / logbook.cpp 数据记录与回放：每秒存一帧，网页下载与回放
     button.h / button.cpp   板载 BOOT 键：短按设出发点，长按开始标定
     cmd.h / cmd.cpp         串口命令：音量、标定、出发点、磁偏角、扫描
     lora_link.h / lora_link.cpp   LoRa 收发与 ACK 应答
     voice.h / voice.cpp     SYN6288 语音播报
     beacon.h / beacon.cpp   信标接收、方位解算、搜索引导
     net.h / net.cpp         WiFi 与网页服务（页面 + JSON 接口）

   工作流程：
     信标落水 → LoRa 发坐标 → 本板收到并回 ACK
     → 用本船北斗位置算出方向与距离 → 语音播报 + 屏幕显示 + 网页同步
   ===================================================================== */

#include "config.h"
#include "gnss.h"
#include "tof.h"
#include "oled.h"
#include "imu.h"
#include "mag.h"
#include "berth.h"
#include "buzzer.h"
#include "anchor.h"
#include "logbook.h"
#include "button.h"
#include "cmd.h"
#include "lora_link.h"
#include "voice.h"
#include "beacon.h"
#include "net.h"

void setup() {
  Serial.begin(DBG_BAUD);
  delay(300);

  Serial.println();
  Serial.println("==================================================");
  Serial.println(" 船舶终端：北斗定位 + 激光测距 + LoRa 信标接收");
  Serial.println(" 通导一体化水上安全终端 · 移动终端");
  Serial.println("==================================================");

  gpsBegin();     // 北斗定位
  tofBegin();     // 激光测距
  oledBegin();    // OLED 显示（与激光共用 I2C 总线）
  imuBegin();     // MPU6050 姿态（同一条 I2C 总线）
  magBegin();     // 磁力计（独立的 Wire1：GPIO25/26）
  berthBegin();   // 靠泊辅助（用激光测距）
  buzzerBegin();  // 蜂鸣器（GPIO33，高电平触发）
  anchorBegin();  // 走锚监测
  logbookBegin(); // 数据记录（黑匣子）
  buttonBegin();  // 板载 BOOT 按键
  loraBegin();    // LoRa（失败会在 beaconUpdate 里每 2 秒重试）
  voiceBegin();   // 语音串口
  beaconBegin();  // 信标状态
  netBegin();     // WiFi + 网页服务

  delay(1000);            // 等 SYN6288 上电稳定再念第一句
  voiceSpeakStartup();
  cmdBegin();             // 打印串口命令提示

  Serial.printf("语音：当前音量 %u/16，串口输入 v0~v16 回车可随时改\n",
                (unsigned)voiceVolume());
  Serial.println("初始化完成，开始接收定位与信标数据。");
}

void loop() {
  cmdPoll();           // 串口命令：音量 / 标定 / 出发点 / 磁偏角
  cmdPollEvents();     // 标定进度与结束提示
  buttonUpdate();      // 板载按键（短按设出发点、长按标定）

  gpsUpdate();         // 读定位并解析
  tofUpdate();         // 读激光测距
  oledUpdate();        // 刷屏（内部 250ms 限速）
  imuUpdate();         // 读姿态（50Hz）
  magUpdate();         // 读磁力计、算航向（内部按速率自己节流）
  berthUpdate();       // 靠泊判断（距离滤波、速度拟合、分级告警、播报）
  beaconUpdate();      // 收信标、算方位、按需播报、判断链路超时

  // 每 2 秒广播一次本船状态，岸基节点靠它掌握船的位置（ID 为 DEV_ID）
  static unsigned long lastBc = 0;
  if (millis() - lastBc >= BROADCAST_MS) {
    lastBc = millis();
    const GpsStatus& g = gpsGet();
    loraSendShipStatus(g.valid, g.lat, g.lon, g.speedKmh / 1.852f, g.course, g.satsUsed);
  }

  anchorUpdate();      // 锚泊位移监测（用网页“设基准”启动）
  buzzerUpdate();      // 汇总告警等级，驱动蜂鸣器（要放在各模块更新之后）
  logbookUpdate();     // 数据记录：每秒把当前状态存一帧
  netLoop();           // 处理网页请求与 WiFi 重连

  // 每秒打印一次状态到串口监视器
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    gpsPrintReport();
    tofPrintReport();
    imuPrintReport();
    cmdPrintMagLine();
    berthPrintReport();
    anchorPrintReport();
    logbookPrintStatus();
    beaconPrintReport();
  }
}
