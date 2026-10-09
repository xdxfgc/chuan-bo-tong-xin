/* =====================================================================
   net.cpp   网络模块实现
   ===================================================================== */

#include "net.h"
#include "gnss.h"
#include "ownpos.h"     // 本船位置来源（北斗 / 手动坐标）
#include "tof.h"
#include "imu.h"
#include "mag.h"
#include "beacon.h"
#include "shore.h"
#include "berth.h"
#include "buzzer.h"
#include "anchor.h"
#include "logbook.h"
#include "voice.h"      // 网页上的语音信息与音量控制
#include "motor.h"      // 网页上的电机控制

static WebServer server(WEB_PORT);
static unsigned long lastWifiTry = 0;
static bool apMode = false;            // 是否运行在热点兜底模式

/* 固定 IP（网段在 config.h 里改） */
static IPAddress STATIC_IP     (STATIC_IP_A, STATIC_IP_B, STATIC_IP_C, STATIC_IP_D);
static IPAddress STATIC_GATEWAY(STATIC_GW_A, STATIC_GW_B, STATIC_GW_C, STATIC_GW_D);
static IPAddress STATIC_SUBNET (255, 255, 255, 0);
static IPAddress STATIC_DNS    (STATIC_GW_A, STATIC_GW_B, STATIC_GW_C, STATIC_GW_D);

/* ---------------- 网页 ---------------- */

/* 下面这整段 HTML/JS 是随程序一起烧进 flash 的，页面越大越占程序空间。
   所以里面没有缩进也没有注释 —— 不是写坏了，是故意压掉省 flash：
   默认分区只给程序留 1.2MB，页面每多 1KB，程序空间就少 1KB。
   改页面时保持这个紧凑风格；页面各功能的说明都写在 README.md 里。 */
static const char INDEX_HTML[] = R"HTML(<!DOCTYPE html>
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
word-break:break-all;color:#9fe0b0;white-space:pre-wrap}
.raw{color:#9ec7e8}
a{color:var(--accent)}
.bar{margin:-6px 0 16px}
.inp{margin:0 8px 8px 0;padding:9px 12px;width:150px;border-radius:10px;
border:1px solid var(--line);background:#0b1219;color:var(--txt);font-size:15px}
.vinfo{display:inline-block;margin-left:10px;color:var(--dim);font-size:13px;line-height:38px}
.btn{padding:10px 16px;border-radius:10px;border:1px solid var(--line);
background:#22303c;color:var(--txt);font-size:15px;cursor:pointer}
.btn.off{background:#3a2020;border-color:#7f2d2d;color:#ff9b9b}
canvas{width:100%;height:120px;background:#0b1219;border:1px solid var(--line);border-radius:10px}
canvas.map{width:100%;height:auto;max-width:540px;margin:0 auto;display:block}
.mapwrap{background:#0b1219;border:1px solid var(--line);border-radius:12px;padding:10px}
.mapbar{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;margin:0 0 10px}
.legend{display:flex;flex-wrap:wrap;gap:16px;justify-content:center;
margin-top:10px;color:var(--dim);font-size:12.5px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:50%;
margin-right:5px;vertical-align:middle}
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
<button id="btnAck" class="btn" onclick="ackAlarm()">无落水告警</button>
<button id="btnMoor" class="btn" onclick="toggleMoor()">设锚泊基准</button>
</div>
<div class="bar">
<button class="btn" onclick="volStep(-2)">音量 −</button>
<button class="btn" onclick="volStep(2)">音量 +</button>
<button class="btn" onclick="voiceTest()">试听</button>
<span class="vinfo" id="vinfo">语音：--</span>
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
<button class="tab" data-t="8" onclick="showTab('8')">⑧ 数据记录</button>
<button class="tab" data-t="9" onclick="showTab('9')">⑨ 岸基</button>
<button class="tab" data-t="10" onclick="showTab('10')">⑩ 电机</button>
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
<div class="card"><div class="k">位置来源</div><div class="v small" id="psrc">--</div></div>
</div>
<p class="sub">室内演示用：北斗定不上位时，在这里手动填一个本船坐标，信标方位/距离、态势图、岸基距离马上就能算出来。
坐标存在 flash 里，重启不丢；手动坐标期间走锚监测不可用，要测走锚请先点“切回北斗”。</p>
<div class="bar">
<input class="inp" id="plat" placeholder="纬度 26.212676">
<input class="inp" id="plon" placeholder="经度 111.599388">
<button class="btn" onclick="setPos()">设为手动坐标</button>
<button class="btn" onclick="clearPos()">切回北斗</button>
</div>
</section>
<section class="pane" data-p="2">
<h2>② 激光测距</h2>
<div class="grid">
<div class="card"><div class="k">前方距离（船头）</div><div class="v" id="tof">--</div></div>
<div class="card"><div class="k">右舷距离（到码头）</div><div class="v" id="tof2">--</div></div>
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
<div class="card"><div class="k">数据来源</div><div class="v small" id="bsrc">--</div></div>
</div>
<p class="sub">靠泊距离曲线（最近 1.5 分钟。蓝线 3.5 米=进入监测，黄线 1 米=靠妥判定，红线 0.5 米=距岸过近）</p>
<canvas id="bcurve" width="800" height="120"></canvas>
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
<h2>信标—本船相对态势</h2>
<div class="mapwrap">
<div class="mapbar">
<button class="btn" id="btnMapMode" onclick="toggleMapMode()">视图：船头朝上</button>
<button class="btn" onclick="clearMapTrail()">清除航迹</button>
</div>
<canvas id="map" class="map" width="720" height="720"></canvas>
<div class="legend">
<span><i style="background:#38bdf8"></i>本船</span>
<span><i style="background:#ef4444"></i>信标（落水点）</span>
<span><i style="background:#f59e0b"></i>本船航迹</span>
<span id="mapScale">量程 --</span>
</div>
<p class="sub" id="mapNote">等待数据…</p>
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
<section class="pane" data-p="8">
<h2>⑧ 数据记录</h2>
<div class="grid">
<div class="card"><div class="k">记录状态</div><div class="v small" id="logstate">--</div></div>
<div class="card"><div class="k">已记录 / 容量</div><div class="v" id="logcount">--</div></div>
<div class="card"><div class="k">覆盖时长</div><div class="v" id="logspan">--</div></div>
</div>
<div class="bar">
<button id="btnLog" class="btn" onclick="toggleLog()">暂停记录</button>
<a id="btnCsv" class="btn" href="/log" download="ship_log.csv">下载 CSV</a>
<button class="btn" onclick="clearLog()">清空</button>
</div>
<p class="sub">回放：先点「加载回放数据」，然后拖滑块或用播放按钮逐条浏览</p>
<div class="bar">
<button class="btn" onclick="loadPlayback()">加载回放数据</button>
<button class="btn" onclick="togglePlay()">播放 / 暂停</button>
</div>
<input type="range" id="pbRange" min="0" max="0" value="0"
style="width:100%;margin:6px 0 12px" oninput="showRec(this.value)">
<div class="raw" id="pbInfo">未加载</div>
</section>

<section class="pane" data-p="9">
<h2>⑨ 岸基节点</h2>
<div class="grid">
<div class="card"><div class="k">岸基链路</div><div class="v small" id="shlink">--</div></div>
<div class="card"><div class="k">岸基编号</div><div class="v small" id="shid">--</div></div>
<div class="card"><div class="k">距岸基</div><div class="v" id="shdist">--</div></div>
<div class="card"><div class="k">岸基方位</div><div class="v" id="shdir">--</div></div>
</div>
<p class="sub">岸基节点固定在码头上，每 2 秒广播一条 R 参考帧。
船在远处时靠它就知道离码头还有多远（激光只能看 4 米）。岸基只做参考点，不告警不播报。</p>
</section>

<section class="pane" data-p="10">
<h2>⑩ 电机（TB6612）</h2>
<div class="grid">
<div class="card"><div class="k">驱动状态</div><div class="v small" id="mstate">--</div></div>
<div class="card"><div class="k">目标油门</div><div class="v" id="mtgt">--</div></div>
<div class="card"><div class="k">实际输出</div><div class="v" id="mout">--</div></div>
</div>
<p class="sub">油门 -100% ~ +100%，负数是倒车。松手才生效；软启动约 1.3 秒到全速，
换向会先停稳 0.3 秒保护 H 桥。上电默认停止。</p>
<p class="sub">油门上限：正常 30%；靠泊告警 0x01（速度偏大）时降到 10%；
0x02（速度过大）或 0x03（距岸过近）时自动刹车并联锁，需重新给油门才恢复。</p>
<div class="bar">
<input type="range" id="mthr" min="-100" max="100" value="0" step="5"
 style="width:100%" onchange="setMotor(this.value)">
</div>
<div class="bar">
<button class="btn" onclick="motorGo(30)">前进 30%</button>
<button class="btn" onclick="motorGo(-20)">倒车 20%</button>
<button class="btn" onclick="motorAct('stop')">滑行停</button>
<button class="btn" onclick="motorAct('estop')">急停</button>
</div>
<p class="sub">⚠ 螺旋桨转起来以后别用手碰；第一次测试先把船架起来或者按住船体。
急停 = 油门清零并刹车，比"滑行停"停得快。</p>
</section>
</div>
<script>
async function tick(){
try{
const d = await (await fetch('/data',{cache:'no-store'})).json();
lastData = d;
drawMap(d);
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
b.textContent = d.posManual ? '位置：手动坐标（模拟）· 其余功能真实运行'
                            : ('定位成功 · ' + (d.fixType===3?'三维定位':'二维定位'));
} else {
b.className='banner bad';
b.textContent = d.alarm;
}
document.getElementById('ftype').textContent = d.posManual ? '手动坐标（模拟）'
                                             : (d.fixType===3?'三维定位':(d.fixType===2?'二维定位':'未定位'));
document.getElementById('psrc').textContent  = d.posManual ? '手动坐标（模拟）' : d.posSrc;
document.getElementById('vinfo').textContent = '语音：音量 ' + d.vol + '/16 · '
+ (d.voiceBusy ? ('正在念（还剩 ' + (d.voiceLeftMs / 1000).toFixed(1) + ' 秒）') : '空闲')
+ (d.voiceCount ? (' · 最近：' + d.voiceLast + '（' + d.voiceAgo + ' 秒前）') : ' · 还没播过')
+ (d.voiceSkip ? (' · 等不上的跳过 ' + d.voiceSkip + ' 次') : '');
document.getElementById('mstate').textContent = d.motorReady ? d.motorState : '未启用';
document.getElementById('mtgt').textContent   = d.motorReady ? ((d.motorTarget * 100).toFixed(0) + '%') : '--';
document.getElementById('mout').textContent   = d.motorReady ? ((d.motorOut * 100).toFixed(0) + '%') : '--';
{
const ms = document.getElementById('mthr');
if(ms && document.activeElement !== ms) ms.value = Math.round(d.motorTarget * 100);
}
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
document.getElementById('tof2').textContent  = d.tof2Ready ? (d.tof2Valid ? d.tof2Text : '无效（超出量程或信号弱）') : '未安装';
document.getElementById('pitch').textContent = d.imuReady ? (d.pitch.toFixed(1) + '°') : '--';
document.getElementById('roll').textContent  = d.imuReady ? (d.roll.toFixed(1) + '°') : '--';
document.getElementById('link').textContent  = d.linkUp ? ('在线 ' + d.rssi + ' dBm') : '离线';
document.getElementById('bid').textContent   = d.hasTarget ? (d.beaconId > 0 ? ('信标 ' + d.beaconId) : '老格式') : '--';
document.getElementById('dist').textContent  = d.haveDir ? (d.distM.toFixed(0) + ' m') : '--';
    document.getElementById('dir').textContent   = d.haveDir ? (d.useRel ? d.relDirText : d.dirText) : '--';
    document.getElementById('shlink').textContent = d.shLink ? ('在线 ' + d.shRssi + ' dBm') : '离线';
    document.getElementById('shid').textContent   = d.shHas ? (d.shId > 0 ? ('岸基 ' + d.shId) : '老格式') : '--';
    document.getElementById('shdist').textContent = d.shHaveDir ? (d.shDist.toFixed(0) + ' m') : '--';
    document.getElementById('shdir').textContent  = d.shHaveDir ? (d.shDirText + '方向') : '--';
document.getElementById('hdg').textContent   = (d.magOk && d.magCal) ? (d.heading.toFixed(0) + '°') : '--';
document.getElementById('mag').textContent   = d.magOk ? (d.magCal ? '已标定' : '未标定') : '未连接';
document.getElementById('bdist').textContent = d.berthValid ? d.berthDistText : '--';
document.getElementById('bspd').textContent  = d.berthActive ? (d.berthSpeed.toFixed(2) + ' m/s') : '--';
document.getElementById('bstate').textContent = d.berthDocked ? '已靠妥'
                                              : (d.berthAlarm ? d.berthAlarmText
                                              : (d.berthActive ? '监测中' : '待机'));
document.getElementById('bsrc').textContent = d.berthSide ? '右舷（侧靠）' : '船头（顶着靠）';
const bb = document.getElementById('btnBuzz');
bb.dataset.on = d.buzzOn ? '1' : '0';
bb.textContent = '蜂鸣器：' + d.buzzState + '（点击切换）';
bb.className = 'btn' + (d.buzzOn ? '' : ' off');
const ba = document.getElementById('btnAck');
ba.dataset.on = d.beaconAlarm ? '1' : '0';
ba.textContent = d.beaconAlarm ? '确认落水告警' : '无落水告警';
ba.className = 'btn' + (d.beaconAlarm ? ' off' : '');
const bm = document.getElementById('btnMoor');
bm.dataset.on = d.anchorOn ? '1' : '0';
bm.textContent = d.anchorOn ? '清除锚泊基准' : '设锚泊基准';
bm.className = 'btn' + (d.anchorOn ? ' off' : '');
document.getElementById('adrift').textContent = d.anchorOn ? (d.anchorDrift.toFixed(2) + ' m') : '--';
document.getElementById('adir').textContent   = d.anchorOn ? (d.anchorDir.toFixed(0) + '°') : '--';
document.getElementById('aspeed').textContent = d.anchorOn ? (d.anchorSpeed.toFixed(3) + ' m/s') : '--';
document.getElementById('astate').textContent = d.anchorState;
drawCurve(d.anchorHist);
drawBerthCurve(d.berthHist);
document.getElementById('logstate').textContent = d.logOn ? '记录中' : '已暂停';
document.getElementById('logcount').textContent = d.logCount + ' / ' + d.logCap;
document.getElementById('logspan').textContent  = d.logSpan + ' 秒';
const bl = document.getElementById('btnLog');
bl.dataset.on = d.logOn ? '1' : '0';
bl.textContent = d.logOn ? '暂停记录' : '开始记录';
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
/* 靠泊距离曲线：横轴是最近 1.5 分钟，纵轴是距岸距离。
   三条水平线是判断门槛，一眼能看出"离告警还有多远"。
   无效读数（-1）处断开，不连线。 */
function drawBerthCurve(hist){
const c = document.getElementById('bcurve');
if(!c) return;
const g = c.getContext('2d');
const W = c.width, H = c.height;
g.clearRect(0, 0, W, H);
if(!hist) return;
const v = hist.split(',').map(Number);
if(v.length < 2) return;
let maxV = 4.0;
for(let i = 0; i < v.length; i++) if(v[i] > maxV) maxV = v[i];
maxV *= 1.15;
const yOf = function(d){ return H - (d / maxV) * (H - 10) - 5; };
g.font = '10px Consolas,Menlo,monospace';
g.textBaseline = 'bottom';
const th = [[3.5, '#38bdf8', '3.5 进入'], [1.0, '#f59e0b', '1.0 靠妥'], [0.5, '#ef4444', '0.5 过近']];
g.setLineDash([5, 5]);
for(let k = 0; k < th.length; k++){
const y = yOf(th[k][0]);
g.strokeStyle = th[k][1];
g.beginPath(); g.moveTo(0, y); g.lineTo(W, y); g.stroke();
g.fillStyle = th[k][1];
g.fillText(th[k][2], 4, y - 1);
}
g.setLineDash([]);
g.strokeStyle = '#7ee2a8'; g.lineWidth = 2; g.beginPath();
let pen = false;
for(let i = 0; i < v.length; i++){
if(v[i] < 0){ pen = false; continue; }
const x = i * (W - 1) / (v.length - 1);
const y = yOf(v[i]);
if(pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; }
}
g.stroke();
}
async function toggleBuzz(){
const cur = document.getElementById('btnBuzz').dataset.on === '1';
try { await fetch('/buzz?on=' + (cur ? '0' : '1'), {cache:'no-store'}); } catch(e) {}
tick();
}
async function ackAlarm(){
if(document.getElementById('btnAck').dataset.on !== '1') return;
try { await fetch('/ack', {cache:'no-store'}); } catch(e) {}
tick();
}
let pbData = null, pbTimer = null, pbIdx = 0;
let pollTimer = null;      // 每秒拉 /data 的定时器（切到后台要停掉，见页面末尾）
async function loadPlayback(){
try{
const t = await (await fetch('/log',{cache:'no-store'})).text();
const all = t.split('\n');
pbData = all.slice(1).filter(function(l){ return l.length > 3; });
if(!pbData.length){
document.getElementById('pbInfo').textContent = '还没有记录。';
document.getElementById('pbRange').max = 0;
return;
}
const r = document.getElementById('pbRange');
r.max = pbData.length - 1;
r.value = pbData.length - 1;
showRec(pbData.length - 1);
}catch(e){
document.getElementById('pbInfo').textContent = '加载失败。';
}
}
function showRec(i){
if(!pbData || !pbData.length) return;
pbIdx = Math.min(Math.max(0, parseInt(i, 10) || 0), pbData.length - 1);
const p = pbData[pbIdx].split(',');
document.getElementById('pbInfo').textContent =
'第 ' + (pbIdx + 1) + ' / ' + pbData.length + ' 条\n' +
'时间     ' + (p[1] || '--') + '\n' +
'位置     ' + p[2] + ', ' + p[3] + '     卫星 ' + p[4] + '     HDOP ' + p[5] + '\n' +
'激光     ' + (p[7] || '--') + ' mm     俯仰 ' + p[8] + '     横滚 ' + p[9] + '\n' +
'航向     ' + (p[10] || '--') + '     锚泊位移 ' + p[11] + ' m\n' +
'靠泊告警 ' + p[12] + '     锚泊告警 ' + p[13] + '     信标 ' + p[15] + ' m';
document.getElementById('pbRange').value = pbIdx;
}
function togglePlay(){
if(pbTimer){ clearInterval(pbTimer); pbTimer = null; return; }
if(!pbData){ loadPlayback(); return; }
pbTimer = setInterval(function(){
if(pbIdx >= pbData.length - 1){ clearInterval(pbTimer); pbTimer = null; return; }
showRec(pbIdx + 1);
}, 300);
}
async function toggleLog(){
const on = document.getElementById('btnLog').dataset.on === '1';
try{ await fetch('/logctl?op=' + (on ? 'off' : 'on'), {cache:'no-store'}); }catch(e){}
tick();
}
async function clearLog(){
try{ await fetch('/logctl?op=clear', {cache:'no-store'}); }catch(e){}
pbData = null; pbIdx = 0;
document.getElementById('pbRange').max = 0;
document.getElementById('pbInfo').textContent = '已清空。';
tick();
}
async function setPos(){
const a = parseFloat(document.getElementById('plat').value);
const b = parseFloat(document.getElementById('plon').value);
if(!isFinite(a) || !isFinite(b) || a < -90 || a > 90 || b < -180 || b > 180 ||
(Math.abs(a) < 1e-6 && Math.abs(b) < 1e-6)){
alert('坐标不合法：纬度要在 -90~90，经度要在 -180~180，且不能填 0,0。');
return;
}
try{ await fetch('/pos?lat=' + a + '&lon=' + b, {cache:'no-store'}); }catch(e){}
tick();
}
async function clearPos(){
try{ await fetch('/pos?mode=auto', {cache:'no-store'}); }catch(e){}
tick();
}
async function volStep(d){
const cur = (lastData && typeof lastData.vol === 'number') ? lastData.vol : 16;
const v = Math.max(0, Math.min(16, cur + d));
try{ await fetch('/vol?v=' + v, {cache:'no-store'}); }catch(e){}
tick();
}
async function voiceTest(){
try{ await fetch('/vol?test=1', {cache:'no-store'}); }catch(e){}
}
async function setMotor(v){
try{ await fetch('/motor?t=' + (v / 100), {cache:'no-store'}); }catch(e){}
tick();
}
async function motorGo(pct){
const ms = document.getElementById('mthr');
if(ms) ms.value = pct;
setMotor(pct);
}
async function motorAct(op){
try{ await fetch('/motor?op=' + op, {cache:'no-store'}); }catch(e){}
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
/* ================= 信标—本船 相对态势图 =================
画法：本船固定在圆心，信标按「东/北」相对位移落点。
船头朝上：整个世界按本船航向反向旋转，画面正上方 = 船首方向。
正北朝上：画面正上方 = 正北，和纸质海图一致。          */
let lastData = null;
let mapMode  = 'head';
let mapTrail = [];
const MAP_RANGE_STEPS = [10, 20, 30, 50, 75, 100, 150, 200, 300, 500, 750, 1000,
1500, 2000, 3000, 5000, 10000];
function toggleMapMode(){
mapMode = (mapMode === 'head') ? 'north' : 'head';
updateMapButton();
if(lastData) drawMap(lastData);
}
function clearMapTrail(){
mapTrail = [];
if(lastData) drawMap(lastData);
}
function updateMapButton(){
const b = document.getElementById('btnMapMode');
if(!b) return;
b.textContent = '视图：' + (mapMode === 'head' ? '船头朝上' : '正北朝上');
}
function enuMeters(lat0, lon0, lat, lon){
const R = 6378137.0;
const rad = Math.PI / 180;
return {
e: (lon - lon0) * rad * R * Math.cos(lat0 * rad),
n: (lat - lat0) * rad * R
};
}
function fmtRange(m){
if(m >= 1000) return (m / 1000).toFixed(m % 1000 === 0 ? 0 : 1) + ' km';
return Math.round(m) + ' m';
}
function drawMap(d){
const c = document.getElementById('map');
if(!c) return;
const g = c.getContext('2d');
const W = c.width, H = c.height, cx = W / 2, cy = H / 2;
const Rpx = Math.min(W, H) / 2 - 46;
d = d || {};
const ownOk  = !!d.valid;
const hasTgt = !!d.haveDir;
const online = !!d.linkUp;
let hdg = null, hdgSrc = '';
if(d.magOk && d.magCal){ hdg = d.heading; hdgSrc = '磁力计航向'; }
else if(ownOk && d.speedKmh > 0.5){ hdg = d.course; hdgSrc = 'GPS 航迹向'; }
const wantingHead = (mapMode === 'head');
const headUp = wantingHead && (hdg !== null);
const rot = headUp ? (hdg * Math.PI / 180) : 0;
if(ownOk){
const t = mapTrail.length ? mapTrail[mapTrail.length - 1] : null;
if(!t || Math.abs(t.lat - d.lat) > 1e-5 || Math.abs(t.lon - d.lon) > 1e-5){
mapTrail.push({lat: d.lat, lon: d.lon});
if(mapTrail.length > 300) mapTrail.shift();
}
}
let maxR = 100;
if(hasTgt){
maxR = MAP_RANGE_STEPS[MAP_RANGE_STEPS.length - 1];
for(let i = 0; i < MAP_RANGE_STEPS.length; i++){
if(d.distM <= MAP_RANGE_STEPS[i] * 0.85){ maxR = MAP_RANGE_STEPS[i]; break; }
}
}
function toScreen(e, n){
const e2 = e * Math.cos(rot) - n * Math.sin(rot);
const n2 = e * Math.sin(rot) + n * Math.cos(rot);
return {x: cx + (e2 / maxR) * Rpx, y: cy - (n2 / maxR) * Rpx};
}
g.fillStyle = '#0b1219';
g.fillRect(0, 0, W, H);
g.lineWidth = 1;
for(let k = 1; k <= 4; k++){
const r = Rpx * k / 4;
g.strokeStyle = (k === 4) ? '#31414f' : '#1e2a35';
g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.stroke();
}
g.strokeStyle = '#1e2a35';
g.beginPath(); g.moveTo(cx - Rpx, cy); g.lineTo(cx + Rpx, cy); g.stroke();
g.beginPath(); g.moveTo(cx, cy - Rpx); g.lineTo(cx, cy + Rpx); g.stroke();
g.fillStyle = '#5d6b78';
g.font = '13px Consolas,Menlo,monospace';
g.textAlign = 'left'; g.textBaseline = 'middle';
for(let k = 1; k <= 4; k++){
g.fillText(fmtRange(maxR * k / 4), cx + 6, cy - Rpx * k / 4);
}
const marks = [[0,'N'],[90,'E'],[180,'S'],[270,'W']];
g.textAlign = 'center'; g.textBaseline = 'middle';
for(let i = 0; i < marks.length; i++){
const a = marks[i][0] * Math.PI / 180 - rot;
const px = cx + Math.sin(a) * (Rpx + 22);
const py = cy - Math.cos(a) * (Rpx + 22);
g.fillStyle = (marks[i][1] === 'N') ? '#ff6b6b' : '#5d6b78';
g.font = (marks[i][1] === 'N' ? 'bold ' : '') + '15px Consolas,Menlo,monospace';
g.fillText(marks[i][1], px, py);
}
if(ownOk && mapTrail.length > 1){
g.strokeStyle = 'rgba(245,158,11,0.55)';
g.lineWidth = 2;
g.beginPath();
for(let i = 0; i < mapTrail.length; i++){
const p = enuMeters(d.lat, d.lon, mapTrail[i].lat, mapTrail[i].lon);
const s = toScreen(p.e, p.n);
if(i) g.lineTo(s.x, s.y); else g.moveTo(s.x, s.y);
}
g.stroke();
}
let tgt = null;
if(ownOk && hasTgt){
const p = enuMeters(d.lat, d.lon, d.tLat, d.tLon);
tgt = toScreen(p.e, p.n);
g.save();
g.setLineDash([8, 6]);
g.strokeStyle = online ? 'rgba(239,68,68,0.75)' : 'rgba(120,130,140,0.5)';
g.lineWidth = 2;
g.beginPath(); g.moveTo(cx, cy); g.lineTo(tgt.x, tgt.y); g.stroke();
g.restore();
const mx = (cx + tgt.x) / 2, my = (cy + tgt.y) / 2;
const txt = fmtRange(d.distM) + ' · ' + (d.dirText || '');
g.font = 'bold 14px Consolas,Menlo,monospace';
g.textAlign = 'center'; g.textBaseline = 'middle';
const tw = g.measureText(txt).width + 14;
g.fillStyle = 'rgba(11,18,25,0.88)';
g.fillRect(mx - tw / 2, my - 12, tw, 24);
g.fillStyle = online ? '#ffb1b1' : '#98a4b0';
g.fillText(txt, mx, my);
}
if(tgt){
if(online){
const pulse = 11 + 6 * (1 + Math.sin(Date.now() / 380)) / 2;
g.strokeStyle = 'rgba(239,68,68,0.35)';
g.lineWidth = 2;
g.beginPath(); g.arc(tgt.x, tgt.y, pulse, 0, Math.PI * 2); g.stroke();
}
g.fillStyle = online ? '#ef4444' : 'rgba(160,170,180,0.7)';
g.beginPath(); g.arc(tgt.x, tgt.y, 9, 0, Math.PI * 2); g.fill();
g.strokeStyle = '#0b1219'; g.lineWidth = 2;
g.beginPath(); g.arc(tgt.x, tgt.y, 9, 0, Math.PI * 2); g.stroke();
const lbl = (d.beaconId > 0 ? ('信标 ' + d.beaconId) : '信标') +
(online ? '' : '（离线·最后位置）');
g.font = 'bold 14px Consolas,Menlo,monospace';
g.textAlign = 'left'; g.textBaseline = 'middle';
g.fillStyle = online ? '#ff9b9b' : '#98a4b0';
let lx = tgt.x + 16;
const ly = tgt.y - 14;
if(lx + g.measureText(lbl).width > W - 6){ g.textAlign = 'right'; lx = tgt.x - 16; }
g.fillText(lbl, lx, ly);
}
g.save();
g.translate(cx, cy);
if(!headUp && hdg !== null) g.rotate(hdg * Math.PI / 180);
g.beginPath();
g.moveTo(0, -18); g.lineTo(13, 15); g.lineTo(0, 8); g.lineTo(-13, 15);
g.closePath();
g.fillStyle = ownOk ? '#38bdf8' : 'rgba(56,189,248,0.35)'; g.fill();
g.strokeStyle = '#0b1219'; g.lineWidth = 2; g.stroke();
g.restore();
let note;
if(!ownOk){
note = '本船北斗未定位，无法算出相对位置。';
}else if(!hasTgt){
note = '还没收到信标坐标，正在等待信标回传。';
}else if(!online){
note = '信标链路已离线，图上显示的是最后一次收到的位置。';
}else{
note = '本船相对信标：' + fmtRange(d.distM) + '，方位 ' +
(d.useRel ? ('相对船头 ' + d.relDirText) : d.dirText);
}
note += '　视图：' + (headUp
? ('船头朝上（用' + hdgSrc + '）')
: (wantingHead ? '正北朝上（没有可用航向，已自动切换）' : '正北朝上'));
const ne = document.getElementById('mapNote');
if(ne) ne.textContent = note;
const sc = document.getElementById('mapScale');
if(sc) sc.textContent = '量程 ' + fmtRange(maxR);
updateMapButton();
}
tick(); pollTimer = setInterval(tick, 1000);
/* 页面被切到后台（换标签页、手机锁屏）就把轮询停掉，回到前台再恢复。
   为什么必须这么做：ESP32 的网页服务同一时刻只能伺候一个客户端，
   开着好几个标签页、每个都每秒拉一次，服务端就排队，表现出来就是"打不开"。 */
document.addEventListener('visibilitychange', function(){
  if(document.hidden){
    if(pollTimer){ clearInterval(pollTimer); pollTimer = null; }
    if(pbTimer){ clearInterval(pbTimer); pbTimer = null; }
  }else{
    tick();
    if(!pollTimer) pollTimer = setInterval(tick, 1000);
  }
});
drawMap(null);
updateMapButton();
setInterval(function(){
if(!lastData || !lastData.linkUp) return;
const c = document.getElementById('map');
if(!c || c.offsetParent === null) return;
drawMap(lastData);
}, 120);
</script>
</body>
</html>)HTML";

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

  /* valid / lat / lon 给的是"系统认为的本船位置"：
     默认是北斗，室内演示切了手动坐标之后就是手动值。
     北斗自己的真实状态另放在 gpsValid / gpsLat / gpsLon，互相不混淆。 */
  bool   mPos = ownPosManual();
  double mLat = ownPosLat();
  double mLon = ownPosLon();

  String j = "{";
  j += "\"valid\":";       j += (ownPosValid() ? "true" : "false");
  j += ",\"posManual\":";  j += (mPos ? "true" : "false");
  j += ",\"posSrc\":\"";   j += escapeJson(ownPosSrcText()); j += "\"";
  j += ",\"gpsValid\":";   j += (d.valid ? "true" : "false");
  j += ",\"fixType\":";    j += d.fixType;
  j += ",\"fixQuality\":"; j += d.fixQuality;
  snprintf(num, sizeof(num), "%.6f", mLat);  j += ",\"lat\":";    j += num;
  snprintf(num, sizeof(num), "%.6f", mLon);  j += ",\"lon\":";    j += num;
  snprintf(num, sizeof(num), "%.6f", d.lat); j += ",\"gpsLat\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", d.lon); j += ",\"gpsLon\":"; j += num;
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
  j += ",\"tof2Ready\":";  j += (tofSideIsReady() ? "true" : "false");
  j += ",\"tof2Valid\":";  j += (tofSideIsValid() ? "true" : "false");
  j += ",\"tof2Mm\":";     j += tofSideDistanceMm();
  j += ",\"tof2Text\":\""; j += escapeJson(tofSideText()); j += "\"";
  j += ",\"berthSide\":";  j += (berthUsingSide() ? "true" : "false");
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
  snprintf(num, sizeof(num), "%.0f", beaconBearing()); j += ",\"bearing\":"; j += num;
  j += ",\"dirText\":\"";   j += escapeJson(String(beaconDirText())); j += "\"";
  j += ",\"useRel\":";      j += (beaconUseRel() ? "true" : "false");
  j += ",\"relDirText\":\""; j += escapeJson(String(beaconRelDirText())); j += "\"";
  snprintf(num, sizeof(num), "%.0f", beaconRelBearing()); j += ",\"relBearing\":"; j += num;

  /* 岸基节点（R/A 帧）：链路、编号、位置、距本船的距离与方位 */
  j += ",\"shLink\":";      j += (shoreLinkUp() ? "true" : "false");
  j += ",\"shHas\":";       j += (shoreHas() ? "true" : "false");
  j += ",\"shValid\":";     j += (shoreValid() ? "true" : "false");
  j += ",\"shId\":";        j += shoreId();
  j += ",\"shSeq\":";       j += (unsigned long)shoreSeq();
  j += ",\"shRssi\":";      j += shoreRssi();
  snprintf(num, sizeof(num), "%.1f", shoreSnr()); j += ",\"shSnr\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", shoreLat()); j += ",\"shLat\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", shoreLon()); j += ",\"shLon\":"; j += num;
  j += ",\"shHaveDir\":";   j += (shoreHaveDir() ? "true" : "false");
  snprintf(num, sizeof(num), "%.0f", shoreDistM()); j += ",\"shDist\":"; j += num;
  j += ",\"shDirText\":\""; j += escapeJson(String(shoreDirText())); j += "\"";

  /* 语音播报信息：音量、在不在念、刚才念的是什么、跳过了几次 */
  j += ",\"vol\":";         j += voiceVolume();
  j += ",\"voiceBusy\":";   j += (voiceBusy() ? "true" : "false");
  j += ",\"voiceLeftMs\":"; j += voiceBusyLeftMs();
  j += ",\"voiceLast\":\""; j += escapeJson(String(voiceLastLabel())); j += "\"";
  j += ",\"voiceAgo\":";    j += (voiceLastMs() ? (millis() - voiceLastMs()) / 1000UL : 0UL);
  j += ",\"voiceCount\":";  j += voiceCount();
  j += ",\"voiceSkip\":";   j += voiceSkipCount();
  j += ",\"voicePrio\":";   j += voiceBusyPrio();

  /* 电机（TB6612）：状态、目标油门、实际输出 */
  j += ",\"motorReady\":";   j += (motorReady() ? "true" : "false");
  j += ",\"motorState\":\""; j += escapeJson(String(motorStateText())); j += "\"";
  snprintf(num, sizeof(num), "%.2f", motorTarget()); j += ",\"motorTarget\":"; j += num;
  snprintf(num, sizeof(num), "%.2f", motorOutput()); j += ",\"motorOut\":"; j += num;

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
  j += ",\"berthHist\":\"";  j += berthDistanceHistory(); j += "\"";
  j += ",\"logOn\":";     j += (logbookOn() ? "true" : "false");
  j += ",\"logCount\":";  j += logbookCount();
  j += ",\"logCap\":";    j += logbookCapacity();
  j += ",\"logSpan\":";   j += logbookSpanSec();
  j += ",\"runSec\":";     j += (millis() / 1000);
  j += "}";
  return j;
}

static void handleRoot() {
  /* 页面本体是固定不变的（实时数据全走 /data 接口），所以让浏览器把它缓存住：
     第一次打开才传这 30 多 KB，之后再打开是"秒开"，不会每次都重新传一遍。
     ⚠ 改过页面内容之后，浏览器可能还拿着旧的缓存 —— 按一次 Ctrl+F5 强制刷新即可。
     send_P 直接从 flash 往外发，不会先拼一个 30KB 的 String 出来占内存。 */
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleData() {
  /* 实时数据绝不能缓存：网页每秒拉一次，拿到旧数据就成假的了 */
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", buildJson());
}

/* 浏览器每次打开页面都会顺手要一次 favicon.ico。
   不专门处理它，就会落到 404 分支上，白白多占一次连接 ——
   而 ESP32 的网页服务同一时刻只能伺候一个客户端，能省一次是一次。 */
static void handleFavicon() {
  server.send(204, "image/x-icon", "");
}

static void handleNotFound() {
  server.send(404, "text/plain; charset=utf-8", "404 Not Found");
}

// 导出记录：分块发送，避免一次性拼出几十 KB 的字符串把堆撑爆
static void handleLogCSV() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv; charset=utf-8", "");
  server.sendContent(logbookHeader());
  server.sendContent("\n");
  for (uint16_t k = 0; k < logbookCount(); k++) server.sendContent(logbookLine(k));
  server.sendContent("");          // 结束分块
}

/* ---------------- WiFi ---------------- */

// 连不上路由器时的兜底：ESP32 自己开热点，网页照常可用
static void startApMode() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);          // 热点模式下也关掉省电，响应更快
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
#if FORCE_AP
  Serial.println("（当前是演示模式 FORCE_AP = 1：想改回连路由器，"
                 "把它改成 0 重新烧录。）");
#else
  Serial.println("（修好路由器密码后重新上电，会自动改回连路由器。）");
#endif
}

static void connectWifi() {
#if FORCE_AP
  /* 演示模式：压根不去碰路由器，开机直接开热点。
     为什么这么做：只要经过路由器，就可能碰上无线隔离、双频混用、
     固定 IP 被占这些说不清的问题；热点模式下中间没有任何设备，
     地址永远是 192.168.4.1，谁都拦不住。
     代价是手机连上热点后没有外网 —— 演示不需要外网。 */
  Serial.println("【演示模式】FORCE_AP = 1，跳过路由器，直接开热点。");
  startApMode();
  return;
#endif

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
  server.on("/favicon.ico", handleFavicon);
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
  server.on("/ack", []() {          // 网页上的“确认落水告警”，和串口敲 ack 等效
    buzzerAcknowledge();
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
  });
  server.on("/log", handleLogCSV);
  /* 音量与试听：/vol?v=8 设音量；/vol?test=1 念一句“你好”试听 */
  server.on("/vol", []() {
    if (server.hasArg("v")) {
      int v = server.arg("v").toInt();
      if (v < 0)  v = 0;
      if (v > 16) v = 16;
      voiceSetVolume((uint8_t)v);
    }
    if (server.hasArg("test")) voiceSpeakTest();
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
  });
  /* 电机：/motor?t=0.6 设油门（-1~1）；/motor?op=stop|brake|estop */
  server.on("/motor", []() {
    if (server.hasArg("t")) motorSetThrottle(server.arg("t").toFloat());
    if (server.hasArg("op")) {
      String op = server.arg("op");
      if      (op == "stop")  motorStop();
      else if (op == "brake") motorBrake();
      else if (op == "estop") motorEmergencyStop();
    }
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
  });
  /* 本船位置来源：/pos?lat=..&lon=.. 设手动坐标；/pos?mode=auto 切回北斗 */
  server.on("/pos", []() {
    if (server.hasArg("mode") && server.arg("mode") == "auto") {
      ownPosClear();
      server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
      return;
    }
    if (server.hasArg("lat") && server.hasArg("lon")) {
      double la = server.arg("lat").toDouble();
      double lo = server.arg("lon").toDouble();
      if (!ownPosSet(la, lo)) {
        server.send(400, "application/json; charset=utf-8",
                    "{\"ok\":false,\"msg\":\"坐标不合法\"}");
        return;
      }
      server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
      return;
    }
    server.send(200, "application/json; charset=utf-8",
                String("{\"ok\":true,\"manual\":") + (ownPosManual() ? "true" : "false") + "}");
  });
  server.on("/logctl", []() {
    String op = server.arg("op");
    if      (op == "on")    logbookSetOn(true);
    else if (op == "off")   logbookSetOn(false);
    else if (op == "clear") logbookClear();
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
  });
  server.onNotFound(handleNotFound);
  server.begin();
  /* 没有客户端连进来时不要每次空转都 delay(1)，
     让 loop 跑得快一点，网页请求能被更及时地接住。 */
  server.enableDelay(false);
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
