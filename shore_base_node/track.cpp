/* =====================================================================
   track.cpp   信标与船端跟踪实现
   ---------------------------------------------------------------------
   信标按编号分槽位（最多 MAX_BEACONS 只），不会互相覆盖 —— 文档按三只
   信标配置，同时落水时岸基要能分清是哪一只。
   信标的落水播报与失联播报在这里触发；船端只跟踪、不播报，
   免得码头一直响。
   ===================================================================== */

#include "track.h"
#include "gnss.h"
#include "voice.h"

static const char* const GEO_DIR_UTF8[8] = { "正北", "东北", "正东", "东南",
                                             "正南", "西南", "正西", "西北" };

static double toRad(double d) { return d * M_PI / 180.0; }
static double toDeg(double r) { return r * 180.0 / M_PI; }

// 两点间大圆距离（米）
static double geoDistanceM(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double p1 = toRad(lat1), p2 = toRad(lat2);
  double dp = toRad(lat2 - lat1);
  double dl = toRad(lon2 - lon1);
  double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  if (a > 1.0) a = 1.0;
  return 2.0 * R * asin(sqrt(a));
}

// 从点 1 指向点 2 的方位角（度，0 = 正北，顺时针）
static double geoBearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double p1 = toRad(lat1), p2 = toRad(lat2);
  double dl = toRad(lon2 - lon1);
  double y = sin(dl) * cos(p2);
  double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  double b = toDeg(atan2(y, x));
  if (b < 0) b += 360.0;
  return b;
}

static int geoDirSector(double bearingDeg) {
  int s = (int)((bearingDeg + 22.5) / 45.0) % 8;
  if (s < 0) s += 8;
  return s;
}

/* ---------------- 状态 ---------------- */

static TrackTarget s_beacons[MAX_BEACONS];
static TrackTarget s_vessel;

/* 每只信标各自的播报去重状态：同一条坐标不反复念，否则语音会排队堆积 */
struct AnnState {
  bool          announced     = false;
  bool          lastHaveDir   = false;
  double        annLat = 0.0, annLon = 0.0;
  unsigned long lastAnnMs     = 0;
  bool          reportedNoFix = false;
};
static AnnState s_ann[MAX_BEACONS];

// 找到（或腾出）这只信标的槽位
static int slotFor(int id) {
  for (int i = 0; i < MAX_BEACONS; i++)
    if (s_beacons[i].has && s_beacons[i].id == id) return i;

  for (int i = 0; i < MAX_BEACONS; i++)
    if (!s_beacons[i].has) return i;

  // 槽位满了：覆盖最久没收到包的那一格
  int oldest = 0;
  for (int i = 1; i < MAX_BEACONS; i++)
    if (s_beacons[i].lastMs < s_beacons[oldest].lastMs) oldest = i;
  return oldest;
}

// 用本节点定位算到目标的距离与方位
static void refreshGeo(TrackTarget& t) {
  t.haveDir = false;
  t.distM   = 0.0f;
  t.bearing = 0.0f;
  t.sector  = 0;

  const GpsStatus& g = gpsGet();
  if (t.has && t.valid && g.valid) {
    t.distM   = (float)geoDistanceM(g.lat, g.lon, t.lat, t.lon);
    t.bearing = (float)geoBearingDeg(g.lat, g.lon, t.lat, t.lon);
    t.sector  = geoDirSector(t.bearing);
    t.haveDir = true;
  }
}

/* 漂移速度估算：拿上一帧的位置和这一帧比，除以时间，再做一次平滑。
   几个"不更新"的护栏：
     · 两帧都要有有效坐标（刚接上定位的那一帧不算）
     · 时间间隔 0.5~60 秒（太短测不准，太长中间漏包太多）
     · 位移超过 200 米当作跳变（信标重启、坐标从无到有），不算漂移
   这个速度只服务"多目标优先级排序"，不影响告警。               */
static void updateDrift(TrackTarget& t, bool valid, double lat, double lon) {
  unsigned long now = millis();

  bool usable = t.valid && valid && (t.prevMs != 0);
  uint32_t dt = usable ? (now - t.prevMs) : 0;

  if (usable && dt >= 500 && dt <= 60000) {
    float d = (float)geoDistanceM(t.prevLat, t.prevLon, lat, lon);
    if (d <= 200.0f) {
      float v = d / ((float)dt / 1000.0f);
      /* 首次直接取实测值，之后按 0.7 旧 + 0.3 新 平滑，免得数字乱跳 */
      t.driftMps = (t.driftMps > 0.0f) ? (0.7f * t.driftMps + 0.3f * v) : v;
    }
  }

  t.prevLat = lat;
  t.prevLon = lon;
  t.prevMs  = now;
}

/* ---------------- 信标落水播报 ----------------
   去重条件（和船端一致）：首次、方位有无变化、目标移动过、或超时才念一遍。
   再加一个全局节流：任意两次播报至少隔 ANNOUNCE_MIN_GAP_MS，
   多只信标同时落水时不会把语音队列堆爆。                           */
static unsigned long s_lastAnyAnnMs = 0;

/* 人工确认标志：确认之后不再重复播报，直到有新信标上线（新事件）才复位 */
static bool s_acked = false;

static void maybeAnnounceBeacon(int slot) {
  if (s_acked) return;                        // 已经人工确认过，别再吵

  /* 上一句还没念完就先别说 —— 正在念同为 3 级（落水/失联）的话才等，
     别的低级别播报可以顶掉。信标每 2 秒还会再发一帧，下个包自然会重试。 */
  if (voiceBusyPrio() >= VOICE_PRIO_SOS) return;

  TrackTarget& t = s_beacons[slot];
  AnnState&    a = s_ann[slot];

  if (a.announced && (millis() - a.lastAnnMs < ANNOUNCE_MIN_GAP_MS)) return;
  if (millis() - s_lastAnyAnnMs < ANNOUNCE_MIN_GAP_MS) return;

  bool first     = !a.announced;
  bool dirChange = (t.haveDir != a.lastHaveDir);
  bool moved     = a.announced && t.haveDir &&
                   geoDistanceM(a.annLat, a.annLon, t.lat, t.lon) >= ANNOUNCE_MIN_MOVE_M;
  bool timeout   = a.announced && (millis() - a.lastAnnMs >= ANNOUNCE_MAX_MS);

  if (!(first || dirChange || moved || timeout)) return;

  voiceAnnounce(t.haveDir, t.lat, t.lon, t.distM, t.sector);

  a.announced   = true;
  a.lastHaveDir = t.haveDir;
  a.annLat      = t.lat;
  a.annLon      = t.lon;
  a.lastAnnMs   = millis();
  s_lastAnyAnnMs = millis();
}

/* ---------------- 对外接口 ---------------- */

void trackBegin() {
  for (int i = 0; i < MAX_BEACONS; i++) {
    s_beacons[i] = TrackTarget();
    s_ann[i]     = AnnState();
  }
  s_vessel = TrackTarget();
  s_acked  = false;
}

void trackOnPacket(const LoraPacket& pkt) {
  /* 船端的来源有两种：
       'S' —— 船端主动广播的本船位置（新协议，首选）
       'A' —— 船端收到信标后回的应答，里面也带着船的位置
     两种都当"船端位置"处理；本节点自己发的 A 帧（srcId == DEV_ID）忽略。 */
  if (pkt.kind == LK_ACK && pkt.srcId == DEV_ID) return;

  bool isVessel = (pkt.kind == LK_VESSEL) || (pkt.kind == LK_ACK);
  if (!isVessel && pkt.kind != LK_BEACON) return;    // R 帧不处理

  const char* name = isVessel ? "船端" : "信标";

  if (isVessel) {
    TrackTarget& t = s_vessel;
    bool wasDown = !t.linkUp;
    t.lastMs = millis();
    t.linkUp = true;

    if (pkt.duplicate) {
      Serial.printf("[%s] #%lu 是重传包，已忽略\n", name, (unsigned long)pkt.seq);
      return;
    }

    t.id    = pkt.srcId;
    t.has   = true;
    t.valid = pkt.valid;
    t.seq   = pkt.seq;
    t.lat   = pkt.lat;
    t.lon   = pkt.lon;
    t.rssi  = pkt.rssi;
    t.snr   = pkt.snr;

    /* 船端的 S 帧带对地速度/航向/卫星数；如果这包没带（比如收到的是
       船端回的 A 帧），就保留上一次的值，别清零。 */
    if (pkt.hasExtra) {
      t.hasExtra = true;
      t.sogKnots = pkt.sogKnots;
      t.cogDeg   = pkt.cogDeg;
      t.sats     = pkt.sats;
    }

    Serial.printf("[%s] 收到 %d #%lu  定位=%s  %.6f, %.6f  RSSI=%d dBm  SNR=%.1f dB\n",
                  name, pkt.srcId, (unsigned long)pkt.seq,
                  pkt.valid ? "有效" : "无效", pkt.lat, pkt.lon, pkt.rssi, pkt.snr);

    refreshGeo(t);
    (void)wasDown;                      // 船端不播报，避免码头一直响
    return;
  }

  /* ---------------- 信标 ---------------- */
  int slot = slotFor(pkt.srcId);
  TrackTarget& t = s_beacons[slot];
  AnnState&    a = s_ann[slot];

  /* 槽位被回收给另一只信标了（原主人太久没发，被覆盖）：旧状态要清掉，
     否则新信标会继承老信标的"已播报过""链路在线"等状态 */
  if (t.has && t.id != pkt.srcId) {
    t = TrackTarget();
    a = AnnState();
  }

  bool wasDown = !t.linkUp;
  t.lastMs = millis();
  t.linkUp = true;

  /* 有信标从离线变在线 —— 这是新事件，把"已确认"清掉，让播报重新生效。
     不然值班员确认过一次之后，后面真的又出事就不响了。 */
  if (wasDown) s_acked = false;

  if (pkt.duplicate) {
    Serial.printf("[信标 %d] #%lu 是重传包，已忽略\n",
                  pkt.srcId, (unsigned long)pkt.seq);
    return;
  }

  /* 先算漂移速度：此时 t 里还是上一帧的位置和定位有效性 */
  updateDrift(t, pkt.valid, pkt.lat, pkt.lon);

  t.id    = pkt.srcId;
  t.has   = true;
  t.valid = pkt.valid;
  t.seq   = pkt.seq;
  t.lat   = pkt.lat;
  t.lon   = pkt.lon;
  t.rssi  = pkt.rssi;
  t.snr   = pkt.snr;

  // 水感：老格式帧没这个字段，当作"有水"，退回原来的报警行为
  t.hasWater = pkt.hasWater;
  t.waterOn  = pkt.hasWater ? pkt.waterOn : true;

  Serial.printf("[信标 %d] 收到 #%lu  定位=%s  %.6f, %.6f  RSSI=%d dBm  SNR=%.1f dB\n",
                pkt.srcId, (unsigned long)pkt.seq,
                pkt.valid ? "有效" : "无效", pkt.lat, pkt.lon, pkt.rssi, pkt.snr);

  refreshGeo(t);

  if (wasDown && a.announced && !s_acked) voiceSpeakLinkBack();
  /* 链路恢复 = 上次事件已经过去，播报状态清零：
     万一马上又落水，按新事件重新播报。 */
  if (wasDown) a.announced = false;

  /* 报警条件 = 坐标有效 + 水感确认导通。
     信标现在一直发（链路随时在线），光看"收到坐标"会一直响；
     真正的触发条件是帧里那个水感位。老格式帧没这个字段，退回原行为。 */
  bool waterAlarm = t.valid && t.waterOn;

  /* 本次落水事件结束：已确认过 + 这一帧报"没水"（人离开水面了）。
     清掉"已确认"和"已播报过"，下次再落水当成新事件重新报警 ——
     信标平时一直发心跳、链路不断，只在失联时复位的话整场只会响一次。 */
  if (s_acked && t.hasWater && !t.waterOn) {
    s_acked         = false;
    a.announced     = false;      // 下次落水按首次处理
    a.reportedNoFix = false;
    Serial.printf("[信标 %d] 已离水，本次落水事件结束（再落水会重新报警）\n", t.id);
  }

  if (waterAlarm) {
    a.reportedNoFix = false;
    maybeAnnounceBeacon(slot);
  } else if (!t.valid && !a.reportedNoFix) {
    /* 上一句没念完就再等一轮，下个包再报（这句是一次性的，不能丢） */
    if (voiceBusy()) { voiceNoteSkip(); return; }
    a.reportedNoFix = true;
    if (!s_acked) voiceSpeakTargetNoPos();
  }
}

void trackUpdate() {
  for (int i = 0; i < MAX_BEACONS; i++) {
    if (s_beacons[i].has && s_beacons[i].linkUp &&
        (millis() - s_beacons[i].lastMs > BEACON_LOST_MS)) {
      s_beacons[i].linkUp = false;
      Serial.printf("[信标 %d] 超过 15 秒没收到信标帧\n", s_beacons[i].id);
      if (!s_acked) voiceSpeakLinkLost();     // 已确认过就不再念
    }
    if (s_beacons[i].has) refreshGeo(s_beacons[i]);
  }

  if (s_vessel.linkUp && (millis() - s_vessel.lastMs > VESSEL_LOST_MS)) {
    s_vessel.linkUp = false;
    Serial.println("[船端] 超过 15 秒没收到船端帧");
  }
  refreshGeo(s_vessel);
}

void trackPrintReport() {
  const GpsStatus& g = gpsGet();
  Serial.println("============== 目标跟踪 ==============");
  Serial.printf("本节点定位: %s\n", g.valid ? "有效" : "未定位");

  int n = trackBeaconCount();
  if (n == 0) {
    Serial.println("落水信标  : 还没收到数据");
  } else {
    for (int i = 0; i < n; i++) {
      const TrackTarget& t = trackBeaconAt(i);
      Serial.printf("落水信标%d : %s  #%lu  %s  %.6f, %.6f  RSSI %d\n",
                    t.id, t.linkUp ? "在线" : "离线",
                    (unsigned long)t.seq, t.valid ? "有效" : "未定位",
                    t.lat, t.lon, t.rssi);
      if (t.haveDir)
        Serial.printf("            方位 %s  距离 %.0f 米\n",
                      GEO_DIR_UTF8[t.sector], t.distM);
    }
  }

  Serial.printf("船端      : %s  ", s_vessel.linkUp ? "在线" : "离线");
  if (s_vessel.has) {
    Serial.printf("%d #%lu  %s  %.6f, %.6f  RSSI %d\n",
                  s_vessel.id, (unsigned long)s_vessel.seq,
                  s_vessel.valid ? "有效" : "未定位",
                  s_vessel.lat, s_vessel.lon, s_vessel.rssi);
    if (s_vessel.haveDir)
      Serial.printf("            方位 %s  距离 %.0f 米\n",
                    GEO_DIR_UTF8[s_vessel.sector], s_vessel.distM);
  } else {
    Serial.println("还没收到数据");
  }
  Serial.println("======================================");
}

int trackBeaconCount() {
  int n = 0;
  for (int i = 0; i < MAX_BEACONS; i++) if (s_beacons[i].has) n++;
  return n;
}

const TrackTarget& trackBeaconAt(int i) {
  /* 只数"收到过数据"的槽位，所以先把有效槽位按顺序挑出来 */
  int n = 0;
  for (int k = 0; k < MAX_BEACONS; k++) {
    if (!s_beacons[k].has) continue;
    if (n == i) return s_beacons[k];
    n++;
  }
  return s_beacons[0];                       // 越界就给第一个，调用方用 count 兜着
}

const TrackTarget& trackBeacon() {
  /* 兼容旧调用：返回最近活跃的那只信标 */
  int best = -1;
  for (int i = 0; i < MAX_BEACONS; i++) {
    if (!s_beacons[i].has) continue;
    if (best < 0 || s_beacons[i].lastMs > s_beacons[best].lastMs) best = i;
  }
  if (best < 0) return s_beacons[0];
  return s_beacons[best];
}

const TrackTarget& trackVessel() { return s_vessel; }

const char* trackDirText(const TrackTarget& t) {
  return t.haveDir ? GEO_DIR_UTF8[t.sector] : "--";
}

void trackAcknowledge() {
  s_acked = true;
  /* 顺便把每只信标的播报状态也清掉，这样解除确认之后
     （比如又有新信标上线）能从头重新播报一次。 */
  for (int i = 0; i < MAX_BEACONS; i++) {
    s_ann[i].announced     = false;
    s_ann[i].lastHaveDir   = false;
    s_ann[i].reportedNoFix = false;
  }
  Serial.println("[告警] 已人工确认，停止重复语音播报（有新信标上线会重新报警）");
}

bool trackAcked() { return s_acked; }

uint32_t trackAgeMs(const TrackTarget& t) {
  if (!t.has) return 0xFFFFFFFFUL;           // 从来没收到过
  return millis() - t.lastMs;
}

/* ---------------- 多目标救援优先级（文档 8.4） ---------------- */

/* 这只信标离船端有多远（米）。船端不在线或坐标不齐时返回 -1。 */
float trackDistToVessel(const TrackTarget& t) {
  if (!(t.has && t.valid && s_vessel.linkUp && s_vessel.valid)) return -1.0f;
  return (float)geoDistanceM(s_vessel.lat, s_vessel.lon, t.lat, t.lon);
}

/* 优先级评分 0~100，越高越紧急。
   四项（文档 8.4）：漂移速度、离岸距离、剩余电量、离最近船舶的距离。
   每项先归一化到 0~1（除以"满分量"），再按权重做加权平均，最后 ×100。

   为什么有的项会"不参与"：
     · 离岸距离：算不出来（本节点没定位、或信标没定位）时不参与
     · 离最近船舶的距离：船端不在线时不参与 —— 没有船，这个量没意义
     · 剩余电量：信标目前还没有上报电量，这一项永远不参与；
       等以后信标把电量发过来，在这里把归一化分数补上就能自动生效。
   不参与的项目权重会自动从分母里扣掉，所以分数不会因此变低。          */
float trackPriority(const TrackTarget& t) {
  if (!t.has) return 0.0f;

  float wSum = 0.0f, sSum = 0.0f;

  /* 1) 漂移速度 */
  if (PRIO_W_DRIFT > 0.0f) {
    float r = t.driftMps / PRIO_DRIFT_FULL_MPS;
    if (r < 0.0f) r = 0.0f;
    if (r > 1.0f) r = 1.0f;
    sSum += r * PRIO_W_DRIFT;
    wSum += PRIO_W_DRIFT;
  }

  /* 2) 离岸距离（= 岸基节点到信标的距离，岸基就钉在码头岸壁上） */
  if (PRIO_W_OFFSHORE > 0.0f && t.haveDir) {
    float r = t.distM / PRIO_OFFSHORE_FULL_M;
    if (r < 0.0f) r = 0.0f;
    if (r > 1.0f) r = 1.0f;
    sSum += r * PRIO_W_OFFSHORE;
    wSum += PRIO_W_OFFSHORE;
  }

  /* 3) 剩余电量：信标还没上报，跳过（预留位置） */

  /* 4) 离最近船舶（船端）的距离 */
  if (PRIO_W_VESSEL_DIST > 0.0f) {
    float dv = trackDistToVessel(t);
    if (dv >= 0.0f) {
      float r = dv / PRIO_VESSEL_FULL_M;
      if (r < 0.0f) r = 0.0f;
      if (r > 1.0f) r = 1.0f;
      sSum += r * PRIO_W_VESSEL_DIST;
      wSum += PRIO_W_VESSEL_DIST;
    }
  }

  if (wSum <= 0.0f) return 0.0f;
  return 100.0f * sSum / wSum;
}
