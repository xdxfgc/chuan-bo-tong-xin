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
  loraBegin();    // LoRa（失败会在 beaconUpdate 里每 2 秒重试）
  voiceBegin();   // 语音串口
  beaconBegin();  // 信标状态
  netBegin();     // WiFi + 网页服务

  delay(1000);            // 等 SYN6288 上电稳定再念第一句
  voiceSpeakStartup();

  Serial.printf("语音：当前音量 %u/16，串口输入 v0~v16 回车可随时改\n",
                (unsigned)voiceVolume());
  Serial.println("初始化完成，开始接收定位与信标数据。");
}

void loop() {
  voicePollSerial();   // 串口命令：在线调音量

  gpsUpdate();         // 读定位并解析
  tofUpdate();         // 读激光测距
  oledUpdate();        // 刷屏（内部 250ms 限速）
  imuUpdate();         // 读姿态（50Hz）
  beaconUpdate();      // 收信标、算方位、按需播报、判断链路超时
  netLoop();           // 处理网页请求与 WiFi 重连

  // 每秒打印一次状态到串口监视器
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    gpsPrintReport();
    tofPrintReport();
    imuPrintReport();
    beaconPrintReport();
  }
}
