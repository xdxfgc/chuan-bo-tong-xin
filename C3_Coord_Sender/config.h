/* =====================================================================
   config.h   信标（坐标发送端）的全部可调参数
   ---------------------------------------------------------------------
   换接线、改参数只动这一个文件。
   ===================================================================== */

#ifndef CB_CONFIG_H
#define CB_CONFIG_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>      // 库集中在这里包含：Arduino 的库发现机制有时扫不到
                       // 只被额外 .cpp 包含的库，写在 config.h 里最稳

/* ---------------- 本机标识 ----------------
   每只信标一个不同的编号，船端和岸基靠它区分“这一帧是谁发的”。
   第一只用 11，第二只 12，第三只 13……（船端是 1）。
   三只信标烧同一个程序时，改这一行后再烧下一只。                       */
/* 这里用 #ifndef 包一层：既能在本文件里改，也能在编译时用
   -DDEV_ID=12 覆盖（一次编译出多只信标时方便）。                */
#ifndef DEV_ID
#define DEV_ID    11
#endif

/* ---------------- 引脚（ESP32-C3 SuperMini + SX1278） ----------------
   SCK -> GPIO6     MISO -> GPIO2     MOSI -> GPIO7
   NSS -> GPIO10    RST  -> GPIO8     DIO0 -> 不要接！
   3V3 -> 3V3       GND  -> GND

   ⚠ DIO0 千万不能接：那个位置是 GPIO9，是启动模式脚，
     接上之后芯片复位就进下载模式，程序根本不跑。
     本程序用轮询读中断标志，不需要 DIO0。                            */
#define SCK_PIN   6
#define MISO_PIN  2
#define MOSI_PIN  7
#define NSS_PIN   10
#define RST_PIN   8
#define DIO0_PIN  -1

/* ---------------- 射频参数（必须与船端完全一致） ---------------- */
#define RF_FREQ   433E6
#define RF_SF     10
#define RF_BW     125E3
#define SYNC_WORD 0x12

/* ---------------- 通信节奏 ---------------- */
#define ROUND_PERIOD_MS 2000      // 每轮间隔
#define ACK_TIMEOUT_MS  1200      // 等应答最长时间
#define MAX_RETRY       3         // 等不到应答最多重发几次
#define TX_TIMEOUT_MS   800       // 单次发送等 TxDone 的上限

/* ---------------- 水感检测（落水触发） ----------------
   硬件：两片不锈钢电极，间距 3~5 毫米，分别接下面两个脚。
         自制电路，不需要任何电阻电容。平时两个脚是高阻，零功耗。

   临时测试不用做电极：两根杜邦线插这两个脚，线头碰一起就是"入水"。
   以后换上真电极，代码一行都不用改。

   脚位只能从空闲脚里挑：GPIO0 / 1 / 3 / 4 / 5
   （2、6、7、8、10 被 LoRa 占了，9 是启动脚，20/21 是北斗）        */
/* 下面几个也包了 #ifndef：既能在本文件里改，也能编译时用 -D 覆盖
   （一次编译出两种版本时方便）。                                   */
#ifndef WATER_PIN_A
#define WATER_PIN_A      3
#endif
#ifndef WATER_PIN_B
#define WATER_PIN_B      4
#endif
#define WATER_SAMPLE_MS  500      // 采样间隔（越小越灵敏，越大越省电）
#define WATER_CONFIRM_MS 2000     // 连续湿多久才算真落水（三重确认的"持续时间"）

/* 定期状态行：每 WATER_STATUS_MS 毫秒打一行，不管有没有水都打。
   串口监视器上一直能看到水感的当前状态，不用碰线头、不用敲命令。
   不想看就设成 0（只在状态变化时打印）。                             */
#ifndef WATER_STATUS_MS
#define WATER_STATUS_MS  5000
#endif

/* 串口模拟命令：1 = 开放 wet on / wet off / wet（演示用，不用碰硬件）
   注意：SRC_MODE = 3 也要读串口，两个不能同时开。                    */
#ifndef WATER_SIM_CMD
#define WATER_SIM_CMD    1
#endif

/* 只在判定入水后才上报：
     0 = 不管有没有入水都照常上报（现在这个，联调方便）
     1 = 平时静默，入水才开始上报（真正信标该有的行为，演示用）        */
#ifndef SEND_ONLY_WET
#define SEND_ONLY_WET    0
#endif

/* ---------------- 坐标来源（改这里切换） ----------------
   0 = 固定坐标（联调最省事，但发的是假坐标）
   1 = 绕固定点缓慢转圈（演示用，能看方向和距离一直在变）
   2 = 本端接北斗模块，用真实定位   ← 现在用这个
   3 = 串口手动输入目标坐标（不用重烧，串口里敲 “纬度,经度” 回车即可）

   改回 0 也很简单：把下面那个 2 改回 0，重新烧录即可。
   ===================================================================== */
#ifndef SRC_MODE
#define SRC_MODE 2
#endif

/* 模式 0/1 用的基坐标（十进制度，正数 = 北纬 / 东经） */
static const double BASE_LAT = 26.208676;
static const double BASE_LON = 111.599388;

/* 模式 1：以 BASE 为圆心、半径多少米、多久转一圈 */
static const double DRIFT_RADIUS_M = 30.0;
static const double DRIFT_PERIOD_S = 60.0;

/* 模式 2：本端北斗模块接在 C3 的哪两个脚（模块 TX->RX 脚，模块 RX->TX 脚） */
#define GNSS_RX_PIN 20
#define GNSS_TX_PIN 21
#define GNSS_BAUD   9600

#endif
