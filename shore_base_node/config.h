/* =====================================================================
   config.h   岸基节点 · 全部可调参数
   ---------------------------------------------------------------------
   接线与射频参数刻意和船端（beidou_gps_test/config.h）保持一致，
   两块板子可以直接对测、也可以直接对接信标。

   本节点用五个模块：
     北斗定位 ATGM336H-5N（UART1） + LoRa SX1278（SPI，433MHz）
     + SYN6288 语音播报（UART2） + SSD1306 OLED 显示屏（I2C）
     + VL53L1X 激光测距（I2C，和 OLED 共用一条总线）
   没有姿态、磁力计、蜂鸣器。
   ===================================================================== */

#ifndef SB_CONFIG_H
#define SB_CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <VL53L1X.h>
#include <LoRa.h>

/* ---------------- 北斗定位模块 ATGM336H-5N（UART1） ----------------
   和船端相同：模块 TX -> GPIO17   模块 RX -> GPIO18   5V/GND
   接 ESP32 时模块上的 USB 口不要同时插电脑，那是同一路串口。        */
static const int      GPS_RX_PIN = 17;
static const int      GPS_TX_PIN = 18;
static const uint32_t GPS_BAUD   = 9600;

/* ---------------- LoRa SX1278（433MHz，SPI） ----------------
   和船端完全相同：SCK->GPIO14  MISO->GPIO19  MOSI->GPIO23
                    NSS->GPIO13  RST->GPIO27   DIO0 不接（轮询）
   射频参数两端必须一模一样，否则收不到：
     433MHz / SF10 / BW125kHz / 同步字 0x12 / 开启 CRC          */
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

/* ---------------- SYN6288 语音模块（UART2） ----------------
   接线和船端完全一致：模块 RXD <- GPIO16（必接）
                       模块 TXD -> GPIO4 （可选，接上能读到应答 0x41）
                       模块 VCC -> 5V，GND 共地，喇叭接 SPK+/SPK-
   GPIO16 / GPIO4 在本节点是空的（北斗 17/18，LoRa 13/14/19/23/27）。 */
static const int      SYN_RX_PIN = 4;
static const int      SYN_TX_PIN = 16;
static const uint32_t SYN_BAUD   = 9600;
static const uint8_t  SYN_VOLUME = 16;

/* ---------------- 激光测距模块 VL53L1X（I2C） ----------------
   和船端接法一致：VIN -> 3.3V（切勿接 5V）  GND -> GND
                   SDA -> GPIO21   SCL -> GPIO22
   和 OLED 共用同一条 I2C 总线：VL53L1X = 0x29，SSD1306 = 0x3C，
   地址不同，挂在一起互不冲突（船端那条总线上还多一个 MPU6050）。
   量程 0.04~4 米，用在岸侧做独立测距。                             */
static const int TOF_SDA_PIN = 21;
static const int TOF_SCL_PIN = 22;
#define TOF_LONG_RANGE 1        // 1 = 远距模式（最远约 4 米），0 = 短距
#define TOF_READ_MS    50       // 读取间隔（毫秒）

/* ---------------- OLED 显示屏（SSD1306 128x64，I2C） ----------------
   4 脚 I2C 版，和船端接法完全一致：
     GND -> GND         VCC -> 3.3V（切勿接 5V）
     SDA -> GPIO21      SCL -> GPIO22
   地址 0x3C（少数模块是 0x3D，程序开机自动扫描并兼容这两个地址）。
   本节点这条 I2C 上还挂着激光 VL53L1X(0x29)，地址不同不冲突
   （船端那条总线上则多挂 MPU6050(0x68) 和 VL53L1X(0x29)）。
   GPIO21/22 没被北斗(17/18)、LoRa(13/14/19/23/27)、语音(16/4)占用。 */
static const int      OLED_SDA_PIN    = 21;
static const int      OLED_SCL_PIN    = 22;
static const uint8_t  OLED_ADDR       = 0x3C;   // 扫描不到就试试 0x3D
static const uint32_t OLED_REFRESH_MS = 250;    // 两次刷屏的最短间隔

/* ---------------- 播报节奏（和船端取值一致） ---------------- */
static const uint32_t ANNOUNCE_MIN_GAP_MS = 8000;   // 两次播报最短间隔，避免语音排队
static const uint32_t ANNOUNCE_MAX_MS     = 20000;  // 坐标没变也最多 20 秒重播一次
static const float    ANNOUNCE_MIN_MOVE_M = 3.0f;   // 目标移动超过 3 米就重播

/* ---------------- 设备编号 ----------------
   告诉对方"这一帧是谁发的"，三端必须各不相同：
     船端 = 1      信标 = 11 / 12 / 13……      岸基 = 21 / 22……
   （岸基节点这台用 21；如果场地上有多台岸基，各给一个不同的号。） */
#ifndef DEV_ID
#define DEV_ID   21
#endif

/* ---------------- 船岸链路协议（文本，三端一致） ----------------
   前缀区分角色，第 2 个字段统一是发送者编号：

     信标 -> 所有： M,<信标ID>,<序号>,P,<定位有效 0/1>,<纬度>,<经度>
     船端 -> 所有： S,<船ID>,<序号>,P,<定位有效 0/1>,<纬度>,<经度>,
                      <对地速度节>,<航向度>,<卫星数>
     本节点 -> 所有：R,<岸基ID>,<序号>,P,<本节点定位有效 0/1>,<纬度>,<经度>
     应答 -> 对方： A,<自己ID>,<序号>,C,<自己定位有效 0/1>,<纬度>,<经度>

   为什么本节点用 R 不用 S：船端已经占了 S（本船广播），
   两个都用 S 的话船端分不清收到的到底是船还是岸基。
   R = Reference（参考站），语义也清楚。

   老格式（没有发送者编号）也认：
     M,<序号>,P,…        S,<序号>,P,…        A,<序号>,C,…
   判断办法：第 3 个字段直接以 P 或 C 开头就是老格式。这样旧固件的
   船端/信标也能接进来，不用三端同时换固件。                        */
static const uint32_t SHORE_SEND_MS  = 2000;    // 本节点广播 R 参考帧的间隔
static const uint32_t VESSEL_LOST_MS = 15000;   // 超过这么久没收到船端 S 帧就算离线
static const uint32_t BEACON_LOST_MS = 15000;   // 超过这么久没收到信标 M 帧就算离线

/* 同时跟踪几只信标。每只有自己的槽位，按编号区分，不会互相覆盖。
   文档按三只配置，这里留 4 个位置。                               */
#define MAX_BEACONS 4

/* ---------------- WiFi ----------------
   小米路由器有两个 WiFi 名字：不带 _5G 的是 2.4GHz（ESP32 必需）。 */
#define WIFI_SSID   "Xiaomi_AE4D"
#define WIFI_PASS   "123456780"
#define WEB_PORT    80

/* 固定 IP：岸基节点用 .201，和船端（.200）区分开，互不冲突。
   前三段要和路由器同网段，小米路由器一般是 192.168.31.x，网关 .1。 */
#define USE_STATIC_IP 1
#define STATIC_IP_A   192
#define STATIC_IP_B   168
#define STATIC_IP_C   31
#define STATIC_IP_D   201
#define STATIC_GW_A   192
#define STATIC_GW_B   168
#define STATIC_GW_C   31
#define STATIC_GW_D   1

/* 路由器连不上时自己开热点兜底，保证网页一定能打开 */
#define AP_FALLBACK 1
#define AP_SSID     "Shore-Base"
#define AP_PASS     "12345678"

/* ---------------- 调试串口（USB） ---------------- */
static const uint32_t DBG_BAUD = 115200;

#endif
