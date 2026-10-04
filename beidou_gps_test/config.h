/* =====================================================================
   config.h   全部可调参数
   ---------------------------------------------------------------------
   换接线、改参数只动这一个文件。
   所有用到的库也集中在这里 include：Arduino 的库发现机制有时扫不到
   「只被额外 .cpp 文件包含」的库，写在被 .ino 直接包含的头文件里最稳。
   ===================================================================== */

#ifndef CB_CONFIG_H
#define CB_CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <VL53L1X.h>
#include <LoRa.h>

/* ---------------- 北斗定位模块 ATGM336H-5N（UART1） ----------------
   模块 TX -> GPIO17   模块 RX -> GPIO18   5V/GND
   接 ESP32 时模块上的 USB 口不要同时插电脑，那是同一路串口。        */
static const int      GPS_RX_PIN = 17;
static const int      GPS_TX_PIN = 18;
static const uint32_t GPS_BAUD   = 9600;

/* ---------------- 激光测距 VL53L1X（I2C） ----------------
   VIN -> 3.3V（切勿接 5V）  GND -> GND  SCL -> GPIO22  SDA -> GPIO21 */
static const int TOF_SDA_PIN = 21;
static const int TOF_SCL_PIN = 22;
#define TOF_LONG_RANGE 1        // 1 = 远距模式（最远约 4 米），0 = 短距
#define TOF_READ_MS    50       // 读取间隔（毫秒）

/* ---------------- OLED 显示屏（SSD1306 128x64，I2C） ----------------
   和激光测距共用一条 I2C 总线：VL53L1X = 0x29，SSD1306 = 0x3C，
   地址不同，挂在一起互不冲突。
   VCC -> 3.3V   GND -> GND   SDA -> GPIO21   SCL -> GPIO22          */
static const int      OLED_SDA_PIN    = 21;
static const int      OLED_SCL_PIN    = 22;
static const uint8_t  OLED_ADDR       = 0x3C;   // 少数模块是 0x3D，读不出就改这里
static const uint32_t OLED_REFRESH_MS = 250;    // 两次刷屏的最短间隔

/* ---------------- MPU6050 六轴姿态（I2C） ----------------
   和激光、OLED 共用一条 I2C 总线：0x29 / 0x3C / 0x68 三个地址互不冲突。
   VCC -> 3.3V   GND -> GND   SDA -> GPIO21   SCL -> GPIO22
   AD0 -> GND 或不接（地址 0x68）；AD0 接高电平则变成 0x69。
   ⚠ MPU6050 没有磁力计：俯仰和横滚准，偏航会随时间漂。          */
static const uint8_t  IMU_ADDR    = 0x68;
static const uint32_t IMU_READ_MS = 20;         // 读取间隔（50Hz，对应文档采集频率）

/* ---------------- 磁力计 GY-282 / HMC5983（单独一路 I2C） ----------------
   单独走 Wire1（GPIO25/26），和激光/OLED/MPU6050 那条（Wire，21/22）分开：
   各用各的硬件控制器、时钟和总线复位，磁力计这边自动降速也不会拖累激光。
   VCC -> 3.3V   GND -> GND   SDA -> GPIO25   SCL -> GPIO26
   想省一对脚：把 I2C_BUS_INDEX 改成 0、SDA/SCL 改成 21/22，
   地址 0x1E 与 0x29 / 0x3C / 0x68 不冲突。                          */
#define I2C_BUS_INDEX   1        // 1 = Wire1（磁力计单独一路，默认）  0 = Wire（与激光共用）
#define I2C_SDA         25
#define I2C_SCL         26
#define I2C_HZ          50000UL  // 先跑 50kHz 求稳；代码会按上升时间自动降速
#define MAG_ADDR        0x1E     // HMC5983 固定地址（与 HMC5883L 相同）

/* HMC5983 参数（上电先从 flash 读，没有才用这里的默认值） */
#define MAG_AVG         1        // 平均次数：0=1次 1=2 2=4 3=8
#define MAG_RATE        6        // 输出速率：0=0.75Hz 1=1.5 2=3 3=7.5 4=15 5=30 6=75 7=220
#define MAG_GAIN        1        // 量程：0=±0.88Ga 1=±1.3 2=±1.9 3=±2.5 4=±4.0 5=±4.7 6=±5.6 7=±8.1
#define MAG_MODE        0        // 0=连续测量（实测失败率比单次低）
#define MAG_READ_MS     50UL     // 最快多久读一帧
#define MAG_EMA_ALPHA   0.30f    // 航向平滑系数：大=跟手但抖，小=稳但迟
#define MAG_DECL_DEG    0.0f     // 磁偏角（真北修正），长沙一带约 -5
#define MAG_TEMP_ENABLE 0        // 读片上温度（实验性）
#define MAG_TEMP_OFFSET_C 0.0f
#define MAG_CAL_DEFAULT_SEC  15  // 标定默认秒数
#define MAG_CAL_MIN_RANGE_UT 8.0f

/* ---------------- 板载 BOOT 按键（GPIO0） ----------------
   短按：把当前船头朝向设为出发点（0°）
   长按 3 秒：开始磁力计标定（拿起来绕「8」字慢慢转）
   不想用就把 BTN_ENABLE 改成 0。                                    */
#define BTN_ENABLE      1
#define BTN_PIN         0
#define BTN_ACTIVE_LOW  1
#define BTN_DEBOUNCE_MS 30
#define BTN_LONG_MS     3000
#define BTN_CAL_SECONDS 15

/* ---------------- 靠泊辅助（只用激光测距，不需要额外硬件） ----------------
   靠近到 BERTH_ENTER_M 以内自动开始监测，退出时用 BERTH_EXIT_M 做滞回。
   阈值按文档附录 B 的告警码取值，实测觉得太敏感或太迟钝就改这里。        */
#define BERTH_ENABLE 1

static const float BERTH_ENTER_M     = 3.5f;    // 进入监测的距离（激光量程 4 米，留余量）
static const float BERTH_EXIT_M      = 3.8f;    // 退出监测的距离（滞回 0.3 米）
static const uint32_t BERTH_EXIT_MS  = 3000;    // 超过退出距离要持续这么久才结束

static const float BERTH_SPEED_WARN  = 0.15f;   // 0x01 接近速度偏大（提醒级）
static const float BERTH_SPEED_ALARM = 0.30f;   // 0x02 接近速度过大（严重级）
static const float BERTH_NEAR_M      = 0.50f;   // 0x03 距岸过近的距离条件
static const float BERTH_NEAR_SPEED  = 0.10f;   // 0x03 距岸过近的速度条件

/* 告警的解除阈值（滞回）：触发用上面那组数字，解除用下面这组，两者留约 10% 的差。
   如果触发和解除用同一个数，速度正好在阈值上下浮动时，蜂鸣器会一秒响一次停一次
   地“哒哒”跳；留出差值之后就不会了。 */
static const float BERTH_SPEED_WARN_OFF  = 0.13f;   // 0x01 解除：速度降到这里以下
static const float BERTH_SPEED_ALARM_OFF = 0.27f;   // 0x02 解除
static const float BERTH_NEAR_M_OFF      = 0.55f;   // 0x03 解除：距离回升到这里以上
static const float BERTH_NEAR_SPEED_OFF  = 0.09f;   // 0x03 解除：速度降到这里以下

static const float BERTH_DONE_M        = 1.00f; // 靠妥判定：距离要小于这个
static const float BERTH_DONE_SPEED    = 0.03f; // 靠妥判定：速度要小于这个
/* 靠妥判定的辅助条件：观察窗内的极差要小于这个值。
   主判据是上面的速度，极差只用来挡住“一个大浪把船推了一截”。
   原来取 0.03 太严：3.2 秒内变化 3 厘米相当于 0.01 m/s，比速度阈值还严三倍，
   水面上的船一直在晃，永远达不到；放宽到 0.08 才能容忍正常的波浪起伏。 */
static const float BERTH_DONE_STEADY_M = 0.08f;
static const uint32_t BERTH_STEADY_MS  = 3000;  // 靠妥判定与静默共用的观察窗

/* 靠妥判定的三道防护：防止"一开机就在近处"被误判成刚刚靠好 */
static const uint32_t BERTH_MIN_WATCH_MS   = 2000;   // 进入监测后至少观察这么久
static const float    BERTH_APPROACH_MIN_M = 0.30f;  // 必须实际接近过这么多米

/* 靠妥后又离开岸壁：距离超过这个值并持续一段时间，就解除"已靠妥" */
static const float    BERTH_UNDOCK_M       = 1.50f;
static const uint32_t BERTH_UNDOCK_MS      = 1000;
static const float    BERTH_SCREEN_SPEED   = 0.01f;  // 移动超过这个速度才占用屏幕

/* 静默判据：拟合速度小于它就暂停念距离。
   用速度而不是极差——船靠在码头边随浪起伏时距离一直在变，极差永远超标，
   于是每 2 秒念一次没完没了；而浪造成的往复平均速度接近零，用速度就能安静下来。 */
static const float BERTH_QUIET_SPEED   = 0.02f;
static const uint32_t BERTH_ALARM_GAP_MS = 3000; // 告警播报最短间隔

static const uint8_t BERTH_DEBOUNCE_N  = 3;     // 告警去抖：连续多少帧满足才置位

/* ---------------- LoRa SX1278（433MHz，SPI） ----------------
   SCK->GPIO14  MISO->GPIO19  MOSI->GPIO23  NSS->GPIO13  RST->GPIO27
   DIO0 不接：程序用轮询，不需要中断脚。
   两块板子的射频参数必须完全一致，否则收不到。
   注意：这组引脚和接收端工程（WROOM32D_Coord_Receiver）一致，是按现有接线来的。
   如果以后换成 ESP32-S3，GPIO19 是 USB 脚、GPIO23 不存在，那时要改到
   12/13/27/32 一组上。                                              */
static const int  LORA_SCK_PIN  = 14;
static const int  LORA_MISO_PIN = 19;
static const int  LORA_MOSI_PIN = 23;
static const int  LORA_NSS_PIN  = 13;
static const int  LORA_RST_PIN  = 27;
static const int  LORA_DIO0_PIN = -1;
static const long RF_FREQ = 433E6;
static const int  RF_SF   = 10;
static const long RF_BW   = 125E3;
static const int  RF_SYNC = 0x12;

/* ---------------- 本机标识与定期广播 ----------------
   本机标识告诉信标和岸基「这一帧是谁发的」。分配约定：
     本船 = 1，信标 = 11/12/13……，岸基 = 21/22……
   每台设备必须不一样，多只信标、多条船同时工作时才分得清。

   本船每 BOAT_BCAST_MS 广播一次自己的位置（S 帧），岸基靠它知道船在哪；
   这是单向广播，不等应答，所以间隔不能太短，否则会挤掉收信标的时间。 */
static const uint8_t  DEV_ID        = 1;
static const uint32_t BOAT_BCAST_MS = 2000;

/* ---------------- SYN6288 语音模块（UART2） ----------------
   模块 RXD <- GPIO16（必接）  TXD -> GPIO4（可选）  VCC -> 5V      */
static const int      SYN_RX_PIN = 4;
static const int      SYN_TX_PIN = 16;
static const uint32_t SYN_BAUD   = 9600;
static const uint8_t  SYN_VOLUME = 16;

/* ---------------- 播报节奏 ---------------- */
static const uint32_t LINK_LOST_MS        = 15000;  // 超时未收信标就播报失去联系
static const uint32_t ANNOUNCE_MIN_GAP_MS = 8000;   // 两次播报最短间隔，避免语音排队
static const uint32_t ANNOUNCE_MAX_MS     = 20000;  // 坐标没变也最多 20 秒重播一次
static const float    ANNOUNCE_MIN_MOVE_M = 3.0f;   // 目标移动超过 3 米就重播

/* ---------------- WiFi ----------------
   小米路由器有两个 WiFi 名字：不带 _5G 的是 2.4GHz（ESP32 必需），
   带 _5G 的是 5GHz，连不上。                                        */
#define WIFI_SSID   "Xiaomi_AE4D"
#define WIFI_PASS   "123456780"
#define WEB_PORT    80

/* 固定 IP：1 = 用固定地址（网页地址不会变），0 = 由路由器自动分配。
   前三段要和路由器同网段，小米路由器一般是 192.168.31.x，网关 .1。 */
#define USE_STATIC_IP 1
#define STATIC_IP_A   192
#define STATIC_IP_B   168
#define STATIC_IP_C   31
#define STATIC_IP_D   200
#define STATIC_GW_A   192
#define STATIC_GW_B   168
#define STATIC_GW_C   31
#define STATIC_GW_D   1

/* 路由器连不上时自己开热点兜底，保证网页一定能打开 */
#define AP_FALLBACK 1
#define AP_SSID     "Beidou-GPS"
#define AP_PASS     "12345678"

/* ---------------- 调试串口（USB） ---------------- */
static const uint32_t DBG_BAUD = 115200;

/* ---------------- 蜂鸣器（有源，3.3V 供电） ----------------
   VCC -> 3.3V    GND -> GND    I/O -> GPIO33

   触发极性看手上的模块：
     MH-FMG：高电平触发（高响、低停）  → BUZZER_ACTIVE_LOW 设 0
     MH-FMD：低电平触发（低响、高停）  → BUZZER_ACTIVE_LOW 设 1

   注意：模块说明书里的示例写的是 GPIO27，但 27 已经被 LoRa 的 RST 占用了；
   GPIO33 完全空闲，没有任何特殊功能，用它最省心。                     */
#define BUZZER_ENABLE 1
#define BUZZER_PIN    33
#define BUZZER_ACTIVE_LOW 0    // 1 = MH-FMD（低电平触发）  0 = MH-FMG（高电平触发）

/* ---------------- 走锚监测（锚泊位移监测） ----------------
   靠好或抛锚稳定后，用网页上的“设基准”按钮把当前位置记为原点，之后持续监测位移。
   判定按文档附录 B：
     0x21 疑似走锚：位移超限并持续一段时间，提醒级
     0x22 走锚    ：位移持续增大且速率没有放缓，严重级

   ⚠ 北斗单点定位误差约 2.5 米，而阈值默认才 2 米，所以必须对**位置**做长时间
     平滑（不是对位移平滑——位移恒为正，平均会带正偏置），否则船没动噪声就把告警刷满。 */
#define ANCHOR_ENABLE 1

static const float    ANCHOR_DRIFT_M     = 2.00f;   // 位移阈值（文档默认 2 米）
static const float    ANCHOR_DRIFT_OFF_M = 1.80f;   // 解除阈值（滞回）
static const uint32_t ANCHOR_HOLD_MS     = 60000;   // 疑似走锚要持续这么久
static const float    ANCHOR_SLOPE_DRAG  = 0.020f;  // 位移增长率门限（米每秒），超过算“持续增大”
#define ANCHOR_SAMPLE_MS 2000UL                     // 采样间隔

#define ANCHOR_POS_N   15       // 位置平滑窗（15 × 2 秒 = 30 秒）
#define ANCHOR_HIST_N  30       // 位移历史（30 × 2 秒 = 60 秒）

/* 激光辅助：停靠在码头时，与岸壁距离的变化比 GNSS 灵敏得多（毫米级）。
   横向位移超过这个值并持续一段时间，也判疑似走锚。 */
static const float    ANCHOR_LASER_D_M   = 0.50f;
static const uint32_t ANCHOR_LASER_MS    = 10000;

static const uint32_t ANCHOR_REPEAT_MS   = 20000;   // 告警持续时的重播间隔

#endif
