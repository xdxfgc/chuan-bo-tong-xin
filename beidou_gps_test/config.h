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

/* ---------------- LoRa SX1278（433MHz，SPI） ----------------
   SCK->GPIO14  MISO->GPIO12  MOSI->GPIO13  NSS->GPIO27  RST->GPIO32
   DIO0 不接：程序用轮询，不需要中断脚。
   两块板子的射频参数必须完全一致，否则收不到。                     */
static const int  LORA_SCK_PIN  = 14;
static const int  LORA_MISO_PIN = 12;
static const int  LORA_MOSI_PIN = 13;
static const int  LORA_NSS_PIN  = 27;
static const int  LORA_RST_PIN  = 32;
static const int  LORA_DIO0_PIN = -1;
static const long RF_FREQ = 433E6;
static const int  RF_SF   = 10;
static const long RF_BW   = 125E3;
static const int  RF_SYNC = 0x12;

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

#endif
