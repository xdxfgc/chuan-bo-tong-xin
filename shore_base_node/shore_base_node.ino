/* =====================================================================
   岸基节点（主程序）
   通导一体化水上安全终端 · 岸基节点
   ---------------------------------------------------------------------
   硬件：ESP32 + 北斗 ATGM336H-5N + LoRa SX1278（433MHz）+ SYN6288 语音
          + SSD1306 OLED + VL53L1X 激光测距（岸侧独立测距，挂 I2C 总线）
   接线与射频参数和船端 beidou_gps_test 完全一致，两块板子可直接对测。

   代码结构（和船端一致的模块化写法）：
     config.h               全部引脚与参数，换接线只改这一个文件
     gnss.h / gnss.cpp      北斗定位：NMEA 解析、位姿帧、定位失锁告警
     lora_link.h / .cpp     LoRa 收发：轮询收包 / 发 ACK 与 R 参考帧
     track.h / track.cpp    信标（可多只）与船端跟踪：距离、方位、链路超时
     gb2312_voice.h         SYN6288 播报用的 GB2312 词表（与船端逐字节一致）
     voice.h / voice.cpp    SYN6288 语音播报：落水告警、失联与恢复
     tof.h / tof.cpp        VL53L1X 激光测距：岸侧本地测距
     net.h / net.cpp        WiFi 与网页服务（页面 + JSON 接口）
     oled.h / oled.cpp      OLED 显示：信标界面与岸基状态

   三端协议（详见 config.h）：前缀区分角色，第 2 个字段是发送者编号
     M,<信标ID>,<序号>,P,…      信标上报落水位置
     S,<船ID>,<序号>,P,…,…      船端广播本船位置
     R,<岸基ID>,<序号>,P,…      本节点广播参考站位置
     A,<自己ID>,<序号>,C,…      应答（里面带着应答者的位置）
   编号：船端 1，信标 11/12/13…，岸基 21/22…
   ===================================================================== */

#include "config.h"
#include "gnss.h"
#include "lora_link.h"
#include "track.h"
#include "voice.h"
#include "tof.h"
#include "net.h"
#include "oled.h"


/* LoRa 统一轮询：收一包并按类型分发
     M = 信标 → 跟踪 + 回 ACK（带上本节点定位）
     S = 船端 → 跟踪
     A = 应答 → 里面带着船端位置，也当船端跟踪（兼容"船端只回 ACK"的情况）
   射频没起来时每 2 秒重试初始化。 */
static void loraPollAll() {
  if (!loraIsReady()) {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry >= 2000) {
      lastRetry = millis();
      loraBegin();
    }
    return;
  }

  LoraPacket pkt;
  if (!loraPoll(&pkt)) return;

  trackOnPacket(pkt);

  if (pkt.kind == LK_BEACON) {                // 只有信标要 ACK，船端不用
    const GpsStatus& g = gpsGet();
    loraSendAck((int)pkt.seq, g.valid, g.lat, g.lon);
  }
}

/* 定时广播本节点参考站位置（R 帧），船端收到后显示“距岸基 / 岸基方位”。
   加一点随机抖动：船端也在按 2 秒节奏发 S 帧，同频同周期长期撞包会互相收不到。

   这里用 R 不用 S —— 船端已经占了 S（本船广播），两个都用 S 的话
   船端分不清收到的是船还是岸基。                                     */
static void shoreBroadcast() {
  static unsigned long lastSend = 0;
  static unsigned short txSeq = 0;
  if (millis() - lastSend < SHORE_SEND_MS + (unsigned long)random(0, 400)) return;
  lastSend = millis();

  const GpsStatus& g = gpsGet();
  char tx[64];
  if (g.valid)
    snprintf(tx, sizeof(tx), "R,%d,%u,P,1,%.6f,%.6f", DEV_ID, (unsigned)txSeq++, g.lat, g.lon);
  else
    snprintf(tx, sizeof(tx), "R,%d,%u,P,0,0,0", DEV_ID, (unsigned)txSeq++);
  loraSendText(tx);
}

void setup() {
  Serial.begin(DBG_BAUD);
  delay(300);

  Serial.println();
  Serial.println("==================================================");
  Serial.println(" 岸基节点：北斗定位 + LoRa 信标接收 + 船岸链路");
  Serial.println(" 通导一体化水上安全终端 · 岸基节点");
  Serial.println("==================================================");
  Serial.printf("本机编号 DEV_ID = %d（船端是 1，信标是 11/12/13…）\n", DEV_ID);

  gpsBegin();     // 北斗定位（UART1，GPIO17/18）
  loraBegin();    // LoRa（失败会在 loraPollAll 里每 2 秒重试）
  voiceBegin();   // 语音串口（UART2，GPIO16/4）
  trackBegin();   // 信标与船端跟踪状态
  tofBegin();     // 激光测距（I2C 与 OLED 共用 GPIO21/22）
  oledBegin();    // OLED 屏幕（放在 netBegin 前面，别让屏陪着一起等 WiFi）
  netBegin();     // WiFi + 网页服务

  voiceSpeakStartup();
  Serial.println("初始化完成，开始接收信标与船端数据。");
}

void loop() {
  gpsUpdate();      // 读定位并解析
  loraPollAll();    // 统一收 LoRa 一包并分发（信标 / 船端）
  shoreBroadcast(); // 每 2 秒广播本节点参考站位置（R 帧）
  trackUpdate();    // 刷新距离方位、判断链路超时
  tofUpdate();      // 读一次激光测距
  netLoop();        // 处理网页请求与 WiFi 重连
  oledUpdate();     // 刷新屏幕

  // 每秒打印一次状态到串口监视器
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    gpsPrintReport();
    trackPrintReport();
    tofPrintReport();
  }
}
