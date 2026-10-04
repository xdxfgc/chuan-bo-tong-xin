# chuan-bo-tong-xin
通导一体化水上安全终端 —— 船端 + 信标 两个工程

## 目录结构

```
chuan-bo-tong-xin/
├─ beidou_gps_test/     船端（移动终端）
└─ C3_Coord_Sender/     信标（坐标发送端）
```

### 一、`beidou_gps_test/` —— 船端（移动终端）

硬件：ESP32（ESP32 Dev Module）+ 北斗 ATGM336H-5N + VL53L1X 激光测距
+ MPU6050 + 磁力计 + OLED + LoRa SX1278 + SYN6288 语音 + 蜂鸣器

- `beidou_gps_test.ino` 主程序：只做初始化和调度
- `README.md` 全部引脚接线、功能说明、烧录与 WiFi 配置

已实现：落水搜救（收信标、算方位距离、语音播报、OLED/网页显示）、
靠泊辅助（分级告警 0x01/0x02/0x03、靠妥判定）、走锚监测（0x21/0x22）、
LoRa 广播与 ACK、网页七个标签页、蜂鸣器告警。

### 二、`C3_Coord_Sender/` —— 信标（坐标发送端）

硬件：ESP32-C3 SuperMini + SX1278 433MHz。每 2 秒广播一次自己的坐标，
收到船端应答才算这一轮成功，收不到自动重传（最多 3 次）。

- `C3_Coord_Sender.ino` 主程序：只做初始化和调度
- `config.h` 引脚、射频、节奏、`DEV_ID`、坐标来源 `SRC_MODE`
- `README.md` 接线、`SRC_MODE` 说明、编译验证结果

## 两个工程怎么配合

| 方向 | 帧格式 |
| --- | --- |
| 信标 → 船端 / 岸基 | `M,<信标ID>,<序号>,P,<定位有效>,<纬度>,<经度>` |
| 船端 → 信标 | `A,<船ID>,<序号>,C,<本船定位有效>,<纬度>,<经度>` |

船端收到信标帧后算出方位和距离，语音播报、OLED 和网页显示，并回一帧应答。
信标 `DEV_ID` 11/12/13……，船端是 1。

**射频参数两边必须完全一致**：433 MHz / SF10 / BW125k / SYNC 0x12 / CRC 开。
只要有一项不同，两边就互相收不到。

## 开发板与烧录

两个工程都是 Arduino IDE 里编译上传。开发板分别选 **ESP32 Dev Module**（船端）
和 **ESP32C3 Dev Module**（信标），串口监视器都是 **115200**。

船端程序较大，分区方案要选 **Huge APP (3MB No OTA/1MB SPIFFS)**。
往工程里加过新文件后，要把 IDE 关掉重新打开一次，让它重新扫描文件列表。

两个工程的详细接线和参数都在各自的 `README.md` 里。
