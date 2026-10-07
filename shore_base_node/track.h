/* =====================================================================
   track.h   信标与船端跟踪
   ---------------------------------------------------------------------
   岸基节点把收到的目标统一管理：
     信标（M 帧）-> 每只一个槽位，按编号区分，最多 MAX_BEACONS 只
     船端（S 帧）-> 一个槽位
   各自记录链路在线状态、坐标、信号，并用本节点北斗位置算出距离与方位。

   落水播报与失联播报在 track.cpp 里触发；船端只跟踪、不播报。
   ===================================================================== */

#ifndef SB_TRACK_H
#define SB_TRACK_H

#include "config.h"
#include "lora_link.h"

struct TrackTarget {
  int           id        = 0;       // 发送者编号；0 = 老格式帧没带编号
  bool          linkUp    = false;   // 链路是否在线
  unsigned long lastMs    = 0;       // 最近一次收包时刻
  bool          has       = false;   // 是否收到过
  bool          valid     = false;   // 对方定位是否有效
  uint32_t      seq       = 0;
  double        lat       = 0.0;
  double        lon       = 0.0;
  int           rssi      = 0;
  float         snr       = 0.0f;

  /* 附加字段：只有船端的 S 帧带（对地速度、航向、卫星数），信标没有 */
  bool          hasExtra  = false;
  float         sogKnots  = 0.0f;   // 对地速度（节）
  float         cogDeg    = 0.0f;   // 对地航向（度）
  int           sats      = 0;      // 参与定位的卫星数

  /* 信标的水感电极是否确认导通。
     信标现在一直发（链路随时在线），靠这个位区分"真入水"和"正常值守"，
     不然岸基平时也会一直报落水。老格式帧没这个字段，按"有水"处理。    */
  bool          hasWater  = false;
  bool          waterOn   = false;

  bool          haveDir   = false;   // 本节点已定位且对方坐标有效
  float         distM     = 0.0f;
  float         bearing   = 0.0f;
  int           sector    = 0;

  /* 漂移速度（米/秒）：用相邻两帧的位移除以时间，再做平滑。
     多目标优先级排序要用它 —— 漂得快的先救。                      */
  float         driftMps  = 0.0f;
  double        prevLat   = 0.0;
  double        prevLon   = 0.0;
  unsigned long prevMs    = 0;
};

void trackBegin();
void trackOnPacket(const LoraPacket& pkt);   // 收到一包后调用
void trackUpdate();                          // 周期调用：刷新方位、判断链路超时
void trackPrintReport();

const TrackTarget& trackBeacon();            // 最近活跃的那只信标（兼容旧调用）
const TrackTarget& trackVessel();            // 船端

int                trackBeaconCount();       // 已经收到过数据的信标只数
const TrackTarget& trackBeaconAt(int i);     // 第 i 只（0 ≤ i < count）

const char*        trackDirText(const TrackTarget& t);   // 方位文字，如“东北”

/* 多目标救援优先级（文档 8.4）：四项加权，返回 0~100，越高越紧急。
   权重和满分量在 config.h 里配（PRIO_W_* / PRIO_*_FULL_*）。 */
float              trackPriority(const TrackTarget& t);

/* 这只信标离船端有多远（米）；坐标不齐时返回 -1。
   优先级里的“离最近船舶的距离”就是它。                        */
float              trackDistToVessel(const TrackTarget& t);

/* 距最近一次收到这个目标的数据过了多久（毫秒）。
   从来没收到过返回 0xFFFFFFFF —— 网页上"最近更新"靠它显示"3 秒前"这种。 */
uint32_t           trackAgeMs(const TrackTarget& t);

/* 人工确认告警：确认之后停止重复播报（落水、失联、恢复都不再念），
   但网页和屏上的显示照旧。等有新信标上线（从离线变在线）会自动重新允许播报，
   免得值班员确认过一次之后，后面真的又出事就不响了。                 */
void trackAcknowledge();
bool trackAcked();

#endif
