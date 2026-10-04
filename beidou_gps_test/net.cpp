/* =====================================================================
   net.cpp   网络模块实现
   ===================================================================== */

#include "net.h"
#include "gnss.h"
#include "tof.h"
#include "imu.h"
#include "mag.h"
#include "beacon.h"
#include "berth.h"
#include "buzzer.h"
#include "anchor.h"

static WebServer server(WEB_PORT);
static unsigned long lastWifiTry = 0;
static bool apMode = false;            // 是否运行在热点兜底模式

/* 固定 IP（网段在 config.h 里改） */
static IPAddress STATIC_IP     (STATIC_IP_A, STATIC_IP_B, STATIC_IP_C, STATIC_IP_D);
static IPAddress STATIC_GATEWAY(STATIC_GW_A, STATIC_GW_B, STATIC_GW_C, STATIC_GW_D);
static IPAddress STATIC_SUBNET (255, 255, 255, 0);
static IPAddress STATIC_DNS    (STATIC_GW_A, STATIC_GW_B, STATIC_GW_C, STATIC_GW_D);

/* ---------------- 网页 ---------------- */

static const char INDEX_HTML[] = R"HTML(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>船舶终端</title>
<style>
  :root{--bg:#0f1720;--card:#18222e;--line:#26333f;--txt:#e6edf3;--dim:#8b98a5;--accent:#38bdf8}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--txt);
       font-family:-apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
  .wrap{max-width:860px;margin:0 auto;padding:20px}
  h1{font-size:22px;margin:4px 0 2px}
  .sub{color:var(--dim);font-size:13px;margin:0 0 16px}
  .banner{padding:14px 16px;border-radius:12px;font-size:17px;font-weight:600;
          margin-bottom:16px;background:#22303c;border:1px solid var(--line)}
  .banner.ok{background:#0f2e1c;border-color:#1f6f3f;color:#7ee2a8}
  .banner.bad{background:#331717;border-color:#7f2d2d;color:#ff9b9b}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
  .card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px}
  .k{color:var(--dim);font-size:12px;margin-bottom:6px}
  .v{font-size:19px;font-weight:600;font-variant-numeric:tabular-nums}
  .v.small{font-size:15px}
  h2{font-size:16px;margin:26px 0 10px;color:var(--txt);font-weight:700;
     padding-bottom:6px;border-bottom:1px solid var(--line)}
  .hex,.raw{background:#0b1219;border:1px solid var(--line);border-radius:10px;
            padding:12px;font-family:Consolas,Menlo,monospace;font-size:12.5px;
            word-break:break-all;color:#9fe0b0}
  .raw{color:#9ec7e8}
  a{color:var(--accent)}
  .bar{margin:-6px 0 16px}
  .btn{padding:10px 16px;border-radius:10px;border:1px solid var(--line);
       background:#22303c;color:var(--txt);font-size:15px;cursor:pointer}
  .btn.off{background:#3a2020;border-color:#7f2d2d;color:#ff9b9b}
  canvas{width:100%;height:120px;background:#0b1219;border:1px solid var(--line);border-radius:10px}
  .tabs{display:flex;flex-wrap:wrap;gap:8px;margin:0 0 14px}
  .tab{padding:8px 16px;border-radius:999px;border:1px solid var(--line);
       background:#1a232e;color:var(--dim);font-size:14px;cursor:pointer}
  .tab.on{background:#173148;border-color:var(--accent);color:var(--txt);font-weight:600}
  .pane{display:none}
</style>
</head>
<body>
<div class="wrap">
  <h1>船舶终端</h1>
  <p class="sub">北斗定位 · 激光测距 · LoRa 信标接收 · 语音播报</p>
  <div class="banner" id="banner">正在等待数据…</div>
  <div class="bar">
    <button id="btnBuzz" class="btn" onclick="toggleBuzz()">蜂鸣器：--</button>
    <button id="btnMoor" class="btn" onclick="toggleMoor()">设锚泊基准</button>
  </div>
  <div class="tabs">
    <button class="tab" data-t="all" onclick="showTab('all')">全部</button>
    <button class="tab on" data-t="1" onclick="showTab('1')">① 定位</button>
    <button class="tab" data-t="2" onclick="showTab('2')">② 测距</button>
    <button class="tab" data-t="3" onclick="showTab('3')">③ 姿态</button>
    <button class="tab" data-t="4" onclick="showTab('4')">④ 靠泊</button>
    <button class="tab" data-t="5" onclick="showTab('5')">⑤ 锚泊</button>
    <button class="tab" data-t="6" onclick="showTab('6')">⑥ 信标</button>
    <button class="tab" data-t="7" onclick="showTab('7')">⑦ 原始数据</button>
  </div>

  <section class="pane" data-p="1" style="display:block">
  <h2>① 北斗定位</h2>
  <div class="grid">
    <div class="card"><div class="k">定位状态</div><div class="v small" id="ftype">--</div></div>
    <div class="card"><div class="k">纬度</div><div class="v" id="lat">--</div></div>
    <div class="card"><div class="k">经度</div><div class="v" id="lon">--</div></div>
    <div class="card"><div class="k">海拔</div><div class="v" id="alt">--</div></div>
    <div class="card"><div class="k">卫星（参与 / 可见）</div><div class="v" id="sat">--</div></div>
    <div class="card"><div class="k">HDOP（越小越好）</div><div class="v" id="hdop">--</div></div>
    <div class="card"><div class="k">对地速度</div><div class="v" id="spd">--</div></div>
    <div class="card"><div class="k">航向</div><div class="v" id="crs">--</div></div>
    <div class="card"><div class="k">北京时间</div><div class="v small" id="bj">--</div></div>
    <div class="card"><div class="k">日期</div><div class="v small" id="date">--</div></div>
    <div class="card"><div class="k">天线状态</div><div class="v small" id="ant">--</div></div>
  </div>

  </section>

  <section class="pane" data-p="2">
  <h2>② 激光测距</h2>
  <div class="grid">
    <div class="card"><div class="k">距岸距离</div><div class="v" id="tof">--</div></div>
  </div>

  </section>

  <section class="pane" data-p="3">
  <h2>③ 姿态与航向</h2>
  <div class="grid">
    <div class="card"><div class="k">俯仰角</div><div class="v" id="pitch">--</div></div>
    <div class="card"><div class="k">横滚角</div><div class="v" id="roll">--</div></div>
    <div class="card"><div class="k">航向角（磁北）</div><div class="v" id="hdg">--</div></div>
    <div class="card"><div class="k">磁力计状态</div><div class="v small" id="mag">--</div></div>
  </div>

  </section>

  <section class="pane" data-p="4">
  <h2>④ 靠泊辅助</h2>
  <div class="grid">
    <div class="card"><div class="k">距岸距离</div><div class="v" id="bdist">--</div></div>
    <div class="card"><div class="k">接近速度</div><div class="v" id="bspd">--</div></div>
    <div class="card"><div class="k">靠泊状态</div><div class="v small" id="bstate">--</div></div>
  </div>

  </section>

  <section class="pane" data-p="5">
  <h2>⑤ 锚泊监测（走锚）</h2>
  <div class="grid">
    <div class="card"><div class="k">离基准位移</div><div class="v" id="adrift">--</div></div>
    <div class="card"><div class="k">漂移方向</div><div class="v" id="adir">--</div></div>
    <div class="card"><div class="k">漂移速率</div><div class="v" id="aspeed">--</div></div>
    <div class="card"><div class="k">锚泊状态</div><div class="v small" id="astate">--</div></div>
  </div>
  <p class="sub">位移趋势（红线为 2 米告警阈值）</p>
  <canvas id="curve" width="800" height="120"></canvas>

  </section>

  <section class="pane" data-p="6">
  <h2>⑥ 信标搜救</h2>
  <div class="grid">
    <div class="card"><div class="k">信标链路</div><div class="v small" id="link">--</div></div>
    <div class="card"><div class="k">信标编号</div><div class="v small" id="bid">--</div></div>
    <div class="card"><div class="k">信标距离</div><div class="v" id="dist">--</div></div>
    <div class="card"><div class="k">信标方位</div><div class="v" id="dir">--</div></div>
  </div>

  </section>

  <section class="pane" data-p="7">
  <h2>⑦ 原始数据</h2>
  <p class="sub">位姿帧（文档表27 · 0x01）</p>
  <div class="hex" id="frame">--</div>
  <p class="sub">原始语句（GGA）</p>
  <div class="raw" id="raw">--</div>
  <p class="sub">地图</p>
  <div class="raw" id="maplink">待定位</div>
  <p class="sub" id="foot">运行时间 -- 秒</p>
  </section>
</div>
<script>
async function tick(){
  try{
    const d = await (await fetch('/data',{cache:'no-store'})).json();
    const b = document.getElementById('banner');
    if(d.linkUp && d.targetValid && d.haveDir){
      b.className='banner bad';
      b.textContent='收到信标 · ' + d.dirText + '方向 约 ' + d.distM.toFixed(0) + ' 米';
    } else if(d.berthAlarm){
      b.className='banner bad';
      b.textContent='靠泊告警 · ' + d.berthAlarmText;
    } else if(d.anchorAlarm){
      b.className='banner bad';
      b.textContent='锚泊告警 · ' + d.anchorAlarmText;
    } else if(d.valid){
      b.className='banner ok';
      b.textContent='定位成功 · ' + (d.fixType===3?'三维定位':'二维定位');
    } else {
      b.className='banner bad';
      b.textContent = d.alarm;
    }
    document.getElementById('ftype').textContent = d.fixType===3?'三维定位':(d.fixType===2?'二维定位':'未定位');
    document.getElementById('lat').textContent   = d.valid ? d.lat.toFixed(6)+'° N' : '--';
    document.getElementById('lon').textContent   = d.valid ? d.lon.toFixed(6)+'° E' : '--';
    document.getElementById('alt').textContent   = d.valid ? d.alt.toFixed(1)+' m' : '--';
    document.getElementById('sat').textContent   = d.satsUsed + ' / ' + d.satsView;
    document.getElementById('hdop').textContent  = d.hdop.toFixed(1);
    document.getElementById('spd').textContent   = d.speedKmh.toFixed(2)+' km/h';
    document.getElementById('crs').textContent   = d.course.toFixed(1)+'°';
    document.getElementById('bj').textContent    = d.bj;
    document.getElementById('date').textContent  = d.date;
    document.getElementById('ant').textContent   = d.antenna;
    document.getElementById('tof').textContent   = d.tofText;
    document.getElementById('pitch').textContent = d.imuReady ? (d.pitch.toFixed(1) + '°') : '--';
    document.getElementById('roll').textContent  = d.imuReady ? (d.roll.toFixed(1) + '°') : '--';
    document.getElementById('link').textContent  = d.linkUp ? ('在线 ' + d.rssi + ' dBm') : '离线';
    document.getElementById('bid').textContent   = d.hasTarget ? (d.beaconId > 0 ? ('信标 ' + d.beaconId) : '老格式') : '--';
    document.getElementById('dist').textContent  = d.haveDir ? (d.distM.toFixed(0) + ' m') : '--';
    document.getElementById('dir').textContent   = d.haveDir ? (d.useRel ? d.relDirText : d.dirText) : '--';
    document.getElementById('hdg').textContent   = (d.magOk && d.magCal) ? (d.heading.toFixed(0) + '°') : '--';
    document.getElementById('mag').textContent   = d.magOk ? (d.magCal ? '已标定' : '未标定') : '未连接';
    document.getElementById('bdist').textContent = d.berthValid ? d.berthDistText : '--';
    document.getElementById('bspd').textContent  = d.berthActive ? (d.berthSpeed.toFixed(2) + ' m/s') : '--';
    document.getElementById('bstate').textContent = d.berthDocked ? '已靠妥'
                                                  : (d.berthAlarm ? d.berthAlarmText
                                                  : (d.berthActive ? '监测中' : '待机'));
    const bb = document.getElementById('btnBuzz');
    bb.dataset.on = d.buzzOn ? '1' : '0';
    bb.textContent = '蜂鸣器：' + d.buzzState + '（点击切换）';
    bb.className = 'btn' + (d.buzzOn ? '' : ' off');
    const bm = document.getElementById('btnMoor');
    bm.dataset.on = d.anchorOn ? '1' : '0';
    bm.textContent = d.anchorOn ? '清除锚泊基准' : '设锚泊基准';
    bm.className = 'btn' + (d.anchorOn ? ' off' : '');
    document.getElementById('adrift').textContent = d.anchorOn ? (d.anchorDrift.toFixed(2) + ' m') : '--';
    document.getElementById('adir').textContent   = d.anchorOn ? (d.anchorDir.toFixed(0) + '°') : '--';
    document.getElementById('aspeed').textContent = d.anchorOn ? (d.anchorSpeed.toFixed(3) + ' m/s') : '--';
    document.getElementById('astate').textContent = d.anchorState;
    drawCurve(d.anchorHist);
    document.getElementById('frame').textContent = d.frame;
    document.getElementById('raw').textContent   = d.raw;
    const ml = document.getElementById('maplink');
    if(d.valid){
      ml.innerHTML = '<a target="_blank" href="https://www.openstreetmap.org/?mlat='+d.lat+
                     '&mlon='+d.lon+'#map=17/'+d.lat+'/'+d.lon+'">在 OpenStreetMap 上查看当前位置</a>';
    } else { ml.textContent = '待定位'; }
    document.getElementById('foot').textContent = '运行时间 ' + d.runSec + ' 秒';
  }catch(e){
    const b = document.getElementById('banner');
    b.className='banner bad';
    b.textContent='与 ESP32 的连接中断，请确认手机或电脑连的是同一个 WiFi';
  }
}
function drawCurve(hist){
  const c = document.getElementById('curve');
  if(!c) return;
  const g = c.getContext('2d');
  const W = c.width, H = c.height;
  g.clearRect(0, 0, W, H);
  if(!hist) return;
  const v = hist.split(',').map(Number).filter(function(x){ return !isNaN(x); });
  if(v.length < 2) return;
  let maxV = 2.5;
  for(let i = 0; i < v.length; i++) if(v[i] > maxV) maxV = v[i];
  maxV *= 1.15;
  g.strokeStyle = '#38bdf8'; g.lineWidth = 2; g.beginPath();
  for(let i = 0; i < v.length; i++){
    const x = i * (W - 1) / (v.length - 1);
    const y = H - (v[i] / maxV) * (H - 8) - 4;
    if(i) g.lineTo(x, y); else g.moveTo(x, y);
  }
  g.stroke();
  const yt = H - (2.0 / maxV) * (H - 8) - 4;
  g.strokeStyle = '#ef4444'; g.setLineDash([6, 6]); g.beginPath();
  g.moveTo(0, yt); g.lineTo(W, yt); g.stroke(); g.setLineDash([]);
}

async function toggleMoor(){
  const on = document.getElementById('btnMoor').dataset.on === '1';
  try { await fetch('/moor?set=' + (on ? '0' : '1'), {cache:'no-store'}); } catch(e) {}
  tick();
}

async function toggleBuzz(){
  const cur = document.getElementById('btnBuzz').dataset.on === '1';
  try { await fetch('/buzz?on=' + (cur ? '0' : '1'), {cache:'no-store'}); } catch(e) {}
  tick();
}
function showTab(t){
  document.querySelectorAll('.pane').forEach(function(p){
    p.style.display = (t === 'all' || p.dataset.p === t) ? 'block' : 'none';
  });
  document.querySelectorAll('.tab').forEach(function(b){
    b.classList.toggle('on', b.dataset.t === t);
  });
}

tick(); setInterval(tick, 1000);
</script>
</body>
</html>
)HTML";

/* ---------------- JSON 接口 ---------------- */

static String escapeJson(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if (c == '\r') o += "\\r";
    else o += c;
  }
  return o;
}

static String buildJson() {
  const GpsStatus& d = gpsGet();
  char num[32];

  String j = "{";
  j += "\"valid\":";       j += (d.valid ? "true" : "false");
  j += ",\"fixType\":";    j += d.fixType;
  j += ",\"fixQuality\":"; j += d.fixQuality;
  snprintf(num, sizeof(num), "%.6f", d.lat); j += ",\"lat\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", d.lon); j += ",\"lon\":"; j += num;
  snprintf(num, sizeof(num), "%.1f", d.altitude); j += ",\"alt\":"; j += num;
  j += ",\"satsUsed\":";   j += d.satsUsed;
  j += ",\"satsView\":";   j += d.satsView;
  snprintf(num, sizeof(num), "%.1f", d.hdop); j += ",\"hdop\":"; j += num;
  snprintf(num, sizeof(num), "%.2f", d.speedKmh); j += ",\"speedKmh\":"; j += num;
  snprintf(num, sizeof(num), "%.1f", d.course); j += ",\"course\":"; j += num;
  j += ",\"utc\":\"";      j += gpsUtcTime();      j += "\"";
  j += ",\"bj\":\"";       j += gpsBeijingTime();  j += "\"";
  j += ",\"date\":\"";     j += gpsDateText();     j += "\"";
  j += ",\"antenna\":\"";  j += escapeJson(gpsAntenna()); j += "\"";
  j += ",\"alarmCode\":";  j += gpsAlarmCode();
  j += ",\"alarm\":\"";    j += escapeJson(gpsAlarmText()); j += "\"";
  j += ",\"frame\":\"";    j += gpsPoseFrameHex(); j += "\"";
  j += ",\"raw\":\"";      j += escapeJson(gpsLastGga()); j += "\"";
  j += ",\"tofReady\":";   j += (tofIsReady() ? "true" : "false");
  j += ",\"tofValid\":";   j += (tofIsValid() ? "true" : "false");
  j += ",\"tofMm\":";      j += tofDistanceMm();
  j += ",\"tofText\":\"";  j += escapeJson(tofText()); j += "\"";
  j += ",\"imuReady\":";   j += (imuGet().ready ? "true" : "false");
  snprintf(num, sizeof(num), "%.1f", imuGet().pitch); j += ",\"pitch\":"; j += num;
  snprintf(num, sizeof(num), "%.1f", imuGet().roll);  j += ",\"roll\":";  j += num;
  snprintf(num, sizeof(num), "%.1f", imuGet().temperature); j += ",\"imuTemp\":"; j += num;
  j += ",\"linkUp\":";      j += (beaconLinkUp() ? "true" : "false");
  j += ",\"hasTarget\":";   j += (beaconHasTarget() ? "true" : "false");
  j += ",\"targetValid\":"; j += (beaconTargetValid() ? "true" : "false");
  j += ",\"beaconId\":";    j += beaconId();
  j += ",\"rssi\":";        j += beaconRssi();
  j += ",\"haveDir\":";     j += (beaconHaveDir() ? "true" : "false");
  snprintf(num, sizeof(num), "%.6f", beaconLat()); j += ",\"tLat\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", beaconLon()); j += ",\"tLon\":"; j += num;
  snprintf(num, sizeof(num), "%.0f", beaconDistM()); j += ",\"distM\":"; j += num;
  j += ",\"dirText\":\"";   j += escapeJson(String(beaconDirText())); j += "\"";
  j += ",\"useRel\":";      j += (beaconUseRel() ? "true" : "false");
  j += ",\"relDirText\":\""; j += escapeJson(String(beaconRelDirText())); j += "\"";
  snprintf(num, sizeof(num), "%.0f", beaconRelBearing()); j += ",\"relBearing\":"; j += num;
  j += ",\"magOk\":";       j += (magPresent() ? "true" : "false");
  j += ",\"magCal\":";      j += (magCalibrated() ? "true" : "false");
  snprintf(num, sizeof(num), "%.1f", magHeadingDeg()); j += ",\"heading\":"; j += num;
  snprintf(num, sizeof(num), "%.1f", magDeviation());  j += ",\"headingDev\":"; j += num;
  j += ",\"berthActive\":"; j += (berthActive() ? "true" : "false");
  j += ",\"berthDocked\":"; j += (berthDocked() ? "true" : "false");
  j += ",\"berthValid\":";  j += (berthValid() ? "true" : "false");
  snprintf(num, sizeof(num), "%.2f", berthDistanceM()); j += ",\"berthDist\":"; j += num;
  snprintf(num, sizeof(num), "%.3f", berthSpeedMps());  j += ",\"berthSpeed\":"; j += num;
  j += ",\"berthAlarm\":";  j += berthAlarmCode();
  j += ",\"berthAlarmText\":\""; j += escapeJson(berthAlarmText()); j += "\"";
  j += ",\"berthDistText\":\"";  j += escapeJson(berthDistanceText()); j += "\"";
  j += ",\"buzzOn\":";      j += (buzzerUserOn() ? "true" : "false");
  j += ",\"buzzState\":\""; j += escapeJson(buzzerStateText()); j += "\"";
  j += ",\"buzzLevel\":";   j += buzzerLevel();
  j += ",\"beaconAlarm\":"; j += (beaconAlarmActive() ? "true" : "false");
  j += ",\"anchorOn\":";     j += (anchorActive() ? "true" : "false");
  j += ",\"anchorState\":\""; j += escapeJson(anchorStateText()); j += "\"";
  snprintf(num, sizeof(num), "%.2f", anchorDriftM());   j += ",\"anchorDrift\":"; j += num;
  snprintf(num, sizeof(num), "%.0f", anchorDriftDir()); j += ",\"anchorDir\":";   j += num;
  snprintf(num, sizeof(num), "%.3f", anchorDriftSpeed()); j += ",\"anchorSpeed\":"; j += num;
  j += ",\"anchorAlarm\":";  j += anchorAlarmCode();
  j += ",\"anchorAlarmText\":\""; j += escapeJson(anchorAlarmText()); j += "\"";
  j += ",\"anchorHist\":\""; j += anchorDriftHistory(); j += "\"";
  j += ",\"runSec\":";     j += (millis() / 1000);
  j += "}";
  return j;
}

static void handleRoot() {
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleData() {
  server.send(200, "application/json; charset=utf-8", buildJson());
}

static void handleNotFound() {
  server.send(404, "text/plain; charset=utf-8", "404 Not Found");
}

/* ---------------- WiFi ---------------- */

// 连不上路由器时的兜底：ESP32 自己开热点，网页照常可用
static void startApMode() {
  WiFi.mode(WIFI_AP);
  delay(200);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println();
  Serial.println("========== 已切换到热点模式 ==========");
  if (!ok) {
    Serial.println("热点启动失败。");
    return;
  }
  apMode = true;
  Serial.print("热点名称: "); Serial.println(AP_SSID);
  Serial.print("热点密码: "); Serial.println(AP_PASS);
  Serial.print("网页地址: http://"); Serial.println(WiFi.softAPIP());
  Serial.println("用手机连上这个热点，再打开上面的地址即可查看页面。");
  Serial.println("（修好路由器密码后重新上电，会自动改回连路由器。）");
}

static void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // 关闭省电模式，避免周期性掉线
  delay(100);

  // 开机先扫描一遍，把 ESP32 能看到的 WiFi 打印出来
  // （ESP32 只支持 2.4GHz，看不到 5GHz 的网络，用这个就能确认 SSID 该填哪个）
  Serial.println("正在扫描附近的 WiFi…");
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("没有扫描到任何网络，请确认路由器已开启 2.4GHz 频段。");
  } else {
    Serial.printf("扫描到 %d 个网络（ESP32 只能看到 2.4GHz 的）：\n", n);
    for (int i = 0; i < n; i++) {
      Serial.printf("  %2d) %s   信号 %d dBm   信道 %d\n",
                    i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
    }
    Serial.print("代码里 WIFI_SSID 必须是上面其中之一，当前填的是：");
    Serial.println(WIFI_SSID);
  }
  WiFi.scanDelete();
  WiFi.mode(WIFI_STA);
  delay(100);

#if USE_STATIC_IP
  if (WiFi.config(STATIC_IP, STATIC_GATEWAY, STATIC_SUBNET, STATIC_DNS)) {
    Serial.print("使用固定 IP: ");
    Serial.println(STATIC_IP);
  } else {
    Serial.println("固定 IP 配置失败，改用路由器自动分配。");
  }
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("正在连接 WiFi: %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

#if USE_STATIC_IP
  // 固定 IP 没连上（多半是网段不对或地址冲突），自动回退成自动获取
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("固定 IP 没有连上，改用路由器自动分配重试…");
    WiFi.disconnect(true, true);
    delay(200);
    WiFi.mode(WIFI_STA);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);   // 取消固定 IP
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("正在连接 WiFi: %s", WIFI_SSID);
    t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
  }
#endif

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi 连接成功，IP 地址: ");
    Serial.println(WiFi.localIP());
    Serial.print("请在浏览器打开测试页面: http://");
    Serial.println(WiFi.localIP());
    return;
  }

  // 连不上路由器：先说清原因，再开热点兜底
  int s = (int)WiFi.status();
  Serial.printf("WiFi 连接失败，状态码 %d。", s);
  if (s == WL_NO_SSID_AVAIL) {
    Serial.println("找不到这个网络名称，请对照上面的扫描结果核对 SSID。");
  } else if (s == WL_CONNECT_FAILED) {
    Serial.println("密码不对，请核对 WIFI_PASS。");
  } else {
    Serial.println("请确认 SSID 是 2.4GHz 频段且密码正确。");
  }

#if AP_FALLBACK
  startApMode();
#else
  Serial.println("后台会持续重试连接路由器。");
#endif
}

/* ---------------- 对外接口 ---------------- */

void netBegin() {
  connectWifi();
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/buzz", []() {
    bool on = (server.arg("on") == "1");
    buzzerSetUserOn(on);
    server.send(200, "application/json; charset=utf-8",
                on ? "{\"buzzOn\":true}" : "{\"buzzOn\":false}");
  });
  server.on("/moor", []() {
    if (server.arg("set") == "1") anchorSetReference();
    else                          anchorClearReference();
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
  });
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("网页服务已启动。");
}

void netLoop() {
  server.handleClient();

  // 热点兜底模式下不再去重连路由器，避免把热点断掉
  if (apMode) return;

  // WiFi 掉线后自动重连：确认已经断开时才重试，
  // 避免在「正在连接」的状态下反复触发 wifi:sta is connecting 报错
  if (WiFi.status() == WL_DISCONNECTED && millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();
    Serial.println("WiFi 已断开，正在重连…");
    WiFi.reconnect();
  }
}

bool netConnected() { return WiFi.status() == WL_CONNECTED; }
String netIP()      { return WiFi.localIP().toString(); }
