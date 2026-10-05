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
     0 = 不管有没有入水都照常上报（联调时用，方便先确认链路通不通）
     1 = 平时静默，入水才开始上报（现在这个，真正信标该有的行为）      */
#ifndef SEND_ONLY_WET
#define SEND_ONLY_WET    1
#endif

/* ---------------- 姿态传感器 MPU6050（I2C） ----------------
   接线（野火 MPU6050 模块）：
     3V3 -> 3V3（⚠ 只能 3.3V，接 5V 直接烧）
     GND -> GND
     SDA -> GPIO0
     SCL -> GPIO1
     A0  -> GND（地址固定 0x68）    INT -> 不接

   为什么是 GPIO0/1：其它脚都有主了——
     LoRa 用 2/6/7/8/10，水感用 3/4，北斗用 20/21，9 是启动脚。
     GPIO22 在 ESP32-C3 上根本不存在，别照抄普通 ESP32 的接法。

   三重确认里它管"姿态判定"：分辨"在船上被雨淋/上浪"和"真的漂在水里"。
   注意：下面那几个阈值是经验值，必须用实测数据标定过再用（见 README）。      */
#ifndef POSTURE_SDA_PIN
#define POSTURE_SDA_PIN   0
#endif
#ifndef POSTURE_SCL_PIN
#define POSTURE_SCL_PIN   1
#endif
#define POSTURE_ADDR      0x68     // A0 接 GND 就是 0x68；悬空则是 0x69
#define POSTURE_READ_MS   20       // 读取间隔（20ms = 50Hz，对应文档采集频率）
#define POSTURE_PRINT_MS  1000     // 标定用的数据行打印间隔

/* 判据①：入水冲击的波形特征 —— 先失重（合加速度掉下去），
   紧接着一个冲击尖峰（砸进水里）。被浪打到身上几乎不会出现这个波形。 */
#define POSTURE_FREEFALL_G    0.6f   // 低于这个值算"失重"
#define POSTURE_IMPACT_G      1.8f   // 失重之后又高于这个值算"入水冲击"
#define POSTURE_IMPACT_WIN_MS 800    // 失重后多久内出现冲击，才算同一次落水

/* ---------------- 姿态判定的三条子判据 ----------------
   ⚠⚠ 下面这几个阈值是**推算出来的首版值，没有经过完整实测标定** ⚠⚠

   数据来源：
     · 静止 / 走动 / 抖动  三组是实测的（2026-10-05）
     · 漂浮那一组是用波浪加速度 A·ω² 推算的，没实测
   所以现阶段只保证"能用"，**报告里写"实测标定"之前必须补测漂浮数据**。

   实测依据：
     · 失重门槛 0.6g —— 实测走路最低 0.983g、抖动最低 0.921g，都离得很远，安全
     · 冲击门槛 2.0g —— 实测抖动能冲到 3.69g，所以**单看冲击不行**，
       必须靠"先失重"当前提才能把上浪排除掉
     · 摇摆频率改用频率而不是幅度 —— 实测手持晃动 0.115~0.182g 和走路
       0.059~0.215g 完全重叠，幅度分不开；但走路是 1~2Hz 步态，
       波浪只有 0.1~0.5Hz，频率分得开
   ================================================================ */
#define POSTURE_TILT_MIN_DEG     55.0f   // 判据②：相对基准倾角超过这个算"翻转过"
#define POSTURE_CALM_MAX_G       0.30f   // 判据③：运动（半秒峰峰值）小于这个算"平静"
#define POSTURE_IMPACT_KEEP_MS   30000   // 判据①：冲击事件在这么久内算有效

/* 摇摆频率：**只用来打印展示，不参与激活判定**。
   原因是它需要 8 秒窗口才能算准，而文档要求 3 秒内激活，两者冲突。
   激活判定用"平静度"（判据③）就够 —— 实测抖动时运动 1.3~4.2g，
   而落水后应该很小，差两个数量级。
   频率本身对报告有用（能佐证"确实是波浪的节奏"），所以还是算出来打印。 */
#define POSTURE_SWAY_WIN_MS      8000    // 频率统计窗口长度（8 秒）

/* 姿态判定是否接进"三重确认"。
   1 = 接入（现在这个）。想退回"只采集不判定"方便调传感器，就改成 0。 */
#ifndef POSTURE_JUDGE
#define POSTURE_JUDGE     1
#endif

/* ---------------- 落水激活状态机 ----------------
   三重确认 = 水感导通 + 姿态符合漂浮 + 持续够久。
   一旦确认激活就**一直保持**，直到人工复位
   （文档 6.3：打捞后擦干，长按复位键五秒退出激活状态）。            */
#define TRIGGER_WAIT_MS   15000    // 水感确认后最多等这么久让姿态条件成立
#define TRIGGER_SIM_ENABLE 1       // 是否开放 sim 命令（演示时强制条件通过）

/* ---------------- 复位按键（板载 BOOT 键） ----------------
   BOOT 键在 GPIO9 上，长按 5 秒退出激活状态，不用额外加硬件。
   ⚠ 上电时别按着它 —— 按住 BOOT 上电，芯片会进下载模式，程序不跑。 */
#ifndef BTN_PIN
#define BTN_PIN           9
#endif
#define BTN_ACTIVE_LOW    1        // 按下时引脚是低电平
#define BTN_RESET_MS      5000     // 长按多久算复位

/* ---------------- 坐标来源（改这里切换） ----------------
   0 = 固定坐标（联调最省事，但发的是假坐标）
   1 = 绕固定点缓慢转圈（演示用，能看方向和距离一直在变）
   2 = 本端接北斗模块，用真实定位
   3 = 串口手动输入目标坐标（不用重烧，串口里敲 “纬度,经度” 回车即可）

   现在用 0：室内演示时先用固定坐标。北斗模块可以一直接着不动，
   程序只是不去读它；等要在室外测真实定位了，再把下面改回 2 重新烧录。
   ===================================================================== */
#ifndef SRC_MODE
#define SRC_MODE 0
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
