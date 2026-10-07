/* =====================================================================
   net.cpp   岸基节点网络模块实现
   ===================================================================== */

#include "net.h"
#include "gnss.h"
#include "lora_link.h"
#include "track.h"
#include "tof.h"
#include "oled.h"
#include "logbook.h"

static WebServer server(WEB_PORT);
static unsigned long lastWifiTry = 0;
static bool apMode = false;            // 是否运行在热点兜底模式

/* 固定 IP（网段在 config.h 里改，本节点默认 .201） */
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
<title>岸基节点</title>
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
  canvas{width:100%;height:130px;background:#0b1219;border:1px solid var(--line);border-radius:10px}
  /* 态势图是正方形，别被上面那条 canvas 规则压扁 */
  #smap{width:100%;height:auto;aspect-ratio:1/1;display:block;
        background:#0b1219;border:1px solid var(--line);border-radius:10px}
  .tabs{display:flex;flex-wrap:wrap;gap:8px;margin:0 0 14px}
  .tab{padding:8px 16px;border-radius:999px;border:1px solid var(--line);
       background:#1a232e;color:var(--dim);font-size:14px;cursor:pointer}
  .tab.on{background:#173148;border-color:var(--accent);color:var(--txt);font-weight:600}
  .pane{display:none}
  .btn{padding:9px 16px;border-radius:10px;border:1px solid var(--line);
       background:#1a232e;color:var(--txt);font-size:14px;cursor:pointer;
       font-family:inherit;margin:0 8px 12px 0;display:inline-block;
       text-decoration:none;line-height:1.2}
  .btn.off{background:#22303c;color:var(--dim)}
  table.tbl{width:100%;border-collapse:collapse;font-size:13.5px;
            font-variant-numeric:tabular-nums}
  table.tbl th{text-align:left;color:var(--dim);font-weight:600;font-size:12px;
               padding:6px 8px;border-bottom:1px solid var(--line)}
  table.tbl td{padding:7px 8px;border-bottom:1px solid #1e2833}
  table.tbl td.ok{color:#7ee2a8}
  table.tbl td.bad{color:#ff9b9b}
  table.tbl td.empty{color:var(--dim);text-align:center}
</style>
</head>
<body>
<div class="wrap">
  <h1>岸基节点（参考站）</h1>
  <p class="sub">北斗定位 · LoRa 信标接收 · 船岸链路 · 岸端参考基准</p>
  <div class="banner" id="banner">正在等待数据…</div>
  <div class="tabs">
    <button class="tab" data-t="all" onclick="showTab('all')">全部</button>
    <button class="tab on" data-t="1" onclick="showTab('1')">① 本节点定位</button>
    <button class="tab" data-t="2" onclick="showTab('2')">② 落水信标</button>
    <button class="tab" data-t="3" onclick="showTab('3')">③ 船端</button>
    <button class="tab" data-t="4" onclick="showTab('4')">④ 原始数据</button>
    <button class="tab" data-t="5" onclick="showTab('5')">⑤ 数据记录</button>
  </div>

  <section class="pane" data-p="1" style="display:block">
  <h2>① 本节点定位（参考站）</h2>
  <div class="grid">
    <div class="card"><div class="k">定位状态</div><div class="v small" id="ftype">--</div></div>
    <div class="card"><div class="k">纬度</div><div class="v" id="lat">--</div></div>
    <div class="card"><div class="k">经度</div><div class="v" id="lon">--</div></div>
    <div class="card"><div class="k">海拔</div><div class="v" id="alt">--</div></div>
    <div class="card"><div class="k">卫星（参与 / 可见）</div><div class="v" id="sat">--</div></div>
    <div class="card"><div class="k">HDOP（越小越好）</div><div class="v" id="hdop">--</div></div>
    <div class="card"><div class="k">北京时间</div><div class="v small" id="bj">--</div></div>
    <div class="card"><div class="k">日期</div><div class="v small" id="date">--</div></div>
    <div class="card"><div class="k">天线状态</div><div class="v small" id="ant">--</div></div>
    <div class="card"><div class="k">LoRa 射频</div><div class="v small" id="lora">--</div></div>
    <div class="card"><div class="k">信道底噪</div><div class="v small" id="chrssi">--</div></div>
    <div class="card"><div class="k">OLED 屏</div><div class="v small" id="oled">--</div></div>
    <div class="card"><div class="k">岸侧测距</div><div class="v small" id="tof">--</div></div>
  </div>
  <p class="sub">本节点每 2 秒发一条 R 帧（参考站定位），船端收到后显示“距岸基”和“岸基方位”。</p>
  </section>

  <section class="pane" data-p="2">
  <h2>② 落水信标</h2>
  <div class="grid">
    <div class="card"><div class="k">信标链路</div><div class="v small" id="blink">--</div></div>
    <div class="card"><div class="k">信标定位</div><div class="v small" id="bstate">--</div></div>
    <div class="card"><div class="k">信标编号</div><div class="v small" id="bid">--</div></div>
    <div class="card"><div class="k">在线信标数</div><div class="v small" id="bcount">--</div></div>
    <div class="card"><div class="k">信标序号</div><div class="v" id="bseq">--</div></div>
    <div class="card"><div class="k">信标坐标</div><div class="v small" id="bpos">--</div></div>
    <div class="card"><div class="k">距信标</div><div class="v" id="bdist">--</div></div>
    <div class="card"><div class="k">信标方位</div><div class="v" id="bdir">--</div></div>
    <div class="card"><div class="k">信号（RSSI / SNR）</div><div class="v small" id="bsig">--</div></div>
    <div class="card"><div class="k">最近更新</div><div class="v small" id="bage">--</div></div>
  </div>
  <div>
    <button class="btn" id="btnAck" onclick="doAck()">确认告警（静音语音播报）</button>
  </div>
  <h2>信标列表（按编号）</h2>
  <table class="tbl">
    <thead><tr>
      <th>编号</th><th>链路</th><th>定位</th><th>坐标</th>
      <th>距岸基</th><th>方位</th><th>RSSI</th><th>更新</th>
    </tr></thead>
    <tbody id="btable"><tr><td class="empty" colspan="8">还没收到信标数据</td></tr></tbody>
  </table>
  <p class="sub">多只信标同时落水时这里会各占一行；顶部横幅按最紧急的那只提示。</p>
  <p class="sub">收到信标 M 帧时本节点会回 ACK，岸端同样具备落水接收能力。</p>

  <h2>救援优先级（文档 8.4）</h2>
  <table class="tbl">
    <thead><tr>
      <th>顺序</th><th>编号</th><th>评分</th><th>漂移速度</th>
      <th>离岸基</th><th>离船端</th><th>方位</th>
    </tr></thead>
    <tbody id="ptable"><tr><td class="empty" colspan="7">还没有信标数据</td></tr></tbody>
  </table>
  <p class="sub">按四项加权打分，分数越高越紧急，值班室按这个顺序施救：
  漂移速度、离岸距离、剩余电量、离最近船舶的距离。
  信标目前还没上报电量，那一项自动不参与；权重在 config.h 里可调（PRIO_W_*）。</p>

  <h2>态势图（以岸基为中心，正北朝上）</h2>
  <canvas id="smap" width="720" height="720"></canvas>
  <p class="sub">中心是岸基节点。蓝点=岸基，圆点=信标（按编号配色），绿点=船端。
  距离圈会自动缩放；远处看全局、近处看细节。</p>
  </section>

  <section class="pane" data-p="3">
  <h2>③ 船端</h2>
  <div class="grid">
    <div class="card"><div class="k">船端链路</div><div class="v small" id="vlink">--</div></div>
    <div class="card"><div class="k">船端定位</div><div class="v small" id="vstate">--</div></div>
    <div class="card"><div class="k">最近更新</div><div class="v small" id="vage">--</div></div>
    <div class="card"><div class="k">船端序号</div><div class="v" id="vseq">--</div></div>
    <div class="card"><div class="k">船端坐标</div><div class="v small" id="vpos">--</div></div>
    <div class="card"><div class="k">距船端</div><div class="v" id="vdist">--</div></div>
    <div class="card"><div class="k">船端方位</div><div class="v" id="vdir">--</div></div>
    <div class="card"><div class="k">对地速度</div><div class="v" id="vsog">--</div></div>
    <div class="card"><div class="k">航向</div><div class="v" id="vcog">--</div></div>
    <div class="card"><div class="k">卫星数</div><div class="v" id="vsats">--</div></div>
    <div class="card"><div class="k">信号（RSSI / SNR）</div><div class="v small" id="vsig">--</div></div>
  </div>
  <p class="sub">船端每 2 秒发一条 S 帧（本船位置，带对地速度/航向/卫星数），本节点收到后显示在这一屏。</p>
  </section>

  <section class="pane" data-p="4">
  <h2>④ 原始数据</h2>
  <p class="sub">本节点位姿帧（文档表27 · 0x01）</p>
  <div class="hex" id="frame">--</div>
  <p class="sub">原始语句（GGA）</p>
  <div class="raw" id="raw">--</div>
  <p class="sub">地图</p>
  <div class="raw" id="maplink">待定位</div>
  <p class="sub" id="foot">运行时间 -- 秒</p>
  </section>

  <section class="pane" data-p="5">
  <h2>⑤ 数据记录</h2>
  <div class="grid">
    <div class="card"><div class="k">记录状态</div><div class="v small" id="logstate">--</div></div>
    <div class="card"><div class="k">已记录 / 容量</div><div class="v" id="logcount">--</div></div>
    <div class="card"><div class="k">覆盖时长</div><div class="v" id="logspan">--</div></div>
  </div>
  <div>
    <button class="btn" id="btnLog" data-on="1" onclick="toggleLog()">暂停记录</button>
    <a class="btn" id="btnCsv" href="/log" download="shore_log.csv">下载 CSV</a>
    <button class="btn" onclick="clearLog()">清空</button>
  </div>
  <p class="sub">回放：先点「加载回放数据」，然后拖滑块逐条看，或者点「播放 / 暂停」自动过。</p>
  <div>
    <button class="btn" onclick="loadPlayback()">加载回放数据</button>
    <button class="btn" onclick="togglePlay()">播放 / 暂停</button>
  </div>
  <input type="range" id="pbRange" min="0" max="0" value="0"
         style="width:100%;margin:6px 0 12px" oninput="showRec(this.value)">
  <div class="raw" id="pbInfo">未加载</div>
  <p class="sub">每秒存一帧：本节点定位、主信标（编号/链路/坐标/距离/方位）、
  船端（链路/坐标/距离/方位/速度/航向/卫星）、岸侧测距。
  CSV 带表头，Excel 直接能开 —— 文档要求的“输入输出全部记录以便复核”就是它。</p>
  <p class="sub">记录只占内存、不写 flash；断电就没了。容量 900 条 ≈ 15 分钟。</p>
  </section>
</div>
<script>
async function tick(){
  try{
    const d = await (await fetch('/data',{cache:'no-store'})).json();
    const b = document.getElementById('banner');
    /* 只要信标在线就报警 —— 不能等"定位有效"才报。
       信标刚落水时北斗还没定上位（冷启动要 30~60 秒），
       那半分钟恰恰是最该让人知道的时刻。 */
    if(d.bLink){
      b.className='banner bad';
      var t = '检测到落水信标';
      if(d.bValid && d.bHaveDir)  t += ' · ' + d.bDirText + '方向 约 ' + d.bDist.toFixed(0) + ' 米';
      else if(d.bValid)           t += ' · 坐标有效（本节点还没定位，算不出方位）';
      else                        t += ' · 定位尚未获取，正在搜星';
      if(d.bCount > 1)            t += '（共 ' + d.bCount + ' 只在线）';
      b.textContent = t;
    } else if(d.vLink){
      b.className='banner ok';
      b.textContent='与船端链路在线' + (d.vHaveDir ? (' · 船端在' + d.vDirText + '方向 约 ' + d.vDist.toFixed(0) + ' 米') : '');
    } else if(d.valid){
      b.className='banner ok';
      b.textContent='本节点定位成功 · ' + (d.fixType===3?'三维定位':'二维定位');
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
    document.getElementById('bj').textContent    = d.bj;
    document.getElementById('date').textContent  = d.date;
    document.getElementById('ant').textContent   = d.antenna;
    document.getElementById('lora').textContent  = d.loraReady ? '就绪' : '初始化失败';
    document.getElementById('chrssi').textContent= d.loraReady ? (d.chRssi + ' dBm') : '--';
    document.getElementById('oled').textContent   = d.oled;
    document.getElementById('tof').textContent    = d.tof;

    document.getElementById('blink').textContent  = d.bLink ? ('在线 ' + d.bRssi + ' dBm') : '离线';
    document.getElementById('bstate').textContent = d.bHas ? (d.bValid ? '定位有效' : '未定位') : '未收到';
    document.getElementById('bid').textContent    = d.bHas ? (d.bId > 0 ? ('信标 ' + d.bId) : '老格式') : '--';
    document.getElementById('bcount').textContent = d.bCount + ' 只';
    document.getElementById('bseq').textContent   = d.bHas ? ('#' + d.bSeq) : '--';
    document.getElementById('bpos').textContent   = (d.bHas && d.bValid) ? (d.bLat.toFixed(5)+', '+d.bLon.toFixed(5)) : '--';
    document.getElementById('bdist').textContent  = d.bHaveDir ? (d.bDist.toFixed(0)+' m') : '--';
    document.getElementById('bdir').textContent   = d.bHaveDir ? (d.bDirText+'方向') : '--';
    document.getElementById('bsig').textContent   = d.bHas ? (d.bRssi + ' dBm / ' + d.bSnr.toFixed(1) + ' dB') : '--';
    document.getElementById('bage').textContent   = fmtAge(d.bAge);

    /* 信标列表：后端按编号分槽位，多只信标同时落水也能各占一行 */
    const tb = document.getElementById('btable');
    if(d.bs && d.bs.length){
      tb.innerHTML = d.bs.map(function(x){
        const pos  = (x.has && x.valid) ? (x.lat.toFixed(5) + ', ' + x.lon.toFixed(5)) : '--';
        const dist = x.haveDir ? (x.dist.toFixed(0) + ' m') : '--';
        const dir  = x.haveDir ? x.dirText : '--';
        const st   = x.has ? (x.valid ? '有效' : '未定位') : '未收到';
        return '<tr>' +
          '<td>' + (x.id > 0 ? ('信标 ' + x.id) : '老格式') + '</td>' +
          '<td class="' + (x.link ? 'ok' : 'bad') + '">' + (x.link ? '在线' : '离线') + '</td>' +
          '<td>' + st + '</td><td>' + pos + '</td>' +
          '<td>' + dist + '</td><td>' + dir + '</td>' +
          '<td>' + x.rssi + ' dBm</td>' +
          '<td>' + fmtAge(x.age) + '</td></tr>';
      }).join('');
    } else {
      tb.innerHTML = '<tr><td class="empty" colspan="8">还没收到信标数据</td></tr>';
    }

    /* 救援优先级：后端算好评分，这里只按分数从高到低排个序。
       分数越高越紧急，排最前面的那只先救。 */
    const pt = document.getElementById('ptable');
    if(d.bs && d.bs.length){
      const arr = d.bs.slice().sort(function(x, y){ return y.score - x.score; });
      pt.innerHTML = arr.map(function(x, i){
        const off = x.haveDir ? (x.dist.toFixed(0) + ' m') : '--';
        const vd  = (x.vdist >= 0) ? (x.vdist.toFixed(0) + ' m') : '--';
        return '<tr>' +
          '<td class="' + (i === 0 ? 'ok' : '') + '">' + (i + 1) + '</td>' +
          '<td>' + (x.id > 0 ? ('信标 ' + x.id) : '老格式') + '</td>' +
          '<td>' + x.score.toFixed(1) + '</td>' +
          '<td>' + x.drift.toFixed(2) + ' m/s</td>' +
          '<td>' + off + '</td><td>' + vd + '</td>' +
          '<td>' + (x.haveDir ? x.dirText : '--') + '</td></tr>';
      }).join('');
    } else {
      pt.innerHTML = '<tr><td class="empty" colspan="7">还没有信标数据</td></tr>';
    }

    /* 确认按钮：确认之后停止重复播报；有新信标上线会自动恢复 */
    const ab = document.getElementById('btnAck');
    ab.textContent = d.acked ? '告警已确认（有新信标上线会重新报警）' : '确认告警（静音语音播报）';
    ab.className   = 'btn' + (d.acked ? ' off' : '');

    document.getElementById('vlink').textContent  = d.vLink ? ('在线 ' + d.vRssi + ' dBm') : '离线';
    document.getElementById('vstate').textContent = d.vHas ? (d.vValid ? '定位有效' : '未定位') : '未收到';
    document.getElementById('vseq').textContent   = d.vHas ? ('#' + d.vSeq) : '--';
    document.getElementById('vpos').textContent   = (d.vHas && d.vValid) ? (d.vLat.toFixed(5)+', '+d.vLon.toFixed(5)) : '--';
    document.getElementById('vdist').textContent  = d.vHaveDir ? (d.vDist.toFixed(0)+' m') : '--';
    document.getElementById('vdir').textContent   = d.vHaveDir ? (d.vDirText+'方向') : '--';
    document.getElementById('vsig').textContent   = d.vHas ? (d.vRssi + ' dBm / ' + d.vSnr.toFixed(1) + ' dB') : '--';
    document.getElementById('vage').textContent   = fmtAge(d.vAge);
    document.getElementById('vsog').textContent   = d.vHasExtra ? (d.vSog.toFixed(1) + ' 节') : '--';
    document.getElementById('vcog').textContent   = d.vHasExtra ? (d.vCog.toFixed(0) + '°') : '--';
    document.getElementById('vsats').textContent  = d.vHasExtra ? (d.vSats + ' 颗') : '--';

    document.getElementById('frame').textContent = d.frame;
    document.getElementById('raw').textContent   = d.raw;
    const ml = document.getElementById('maplink');
    if(d.valid){
      ml.innerHTML = '<a target="_blank" href="https://www.openstreetmap.org/?mlat='+d.lat+
                     '&mlon='+d.lon+'#map=17/'+d.lat+'/'+d.lon+'">在 OpenStreetMap 上查看本节点位置</a>';
    } else { ml.textContent = '待定位'; }
    document.getElementById('foot').textContent = '运行时间 ' + d.runSec + ' 秒';

    /* 数据记录状态 */
    document.getElementById('logstate').textContent = d.logOn ? '记录中' : '已暂停';
    document.getElementById('logcount').textContent = d.logCount + ' / ' + d.logCap;
    document.getElementById('logspan').textContent  = d.logSpan + ' 秒';
    const lb = document.getElementById('btnLog');
    lb.dataset.on = d.logOn ? '1' : '0';
    lb.textContent = d.logOn ? '暂停记录' : '继续记录';
    lb.className = 'btn' + (d.logOn ? '' : ' off');

    /* 态势图 */
    drawMap(d);
  }catch(e){
    const b = document.getElementById('banner');
    b.className='banner bad';
    b.textContent='与 ESP32 的连接中断，请确认手机或电脑连的是同一个 WiFi';
  }
}
function showTab(t){
  document.querySelectorAll('.pane').forEach(function(p){
    p.style.display = (t === 'all' || p.dataset.p === t) ? 'block' : 'none';
  });
  document.querySelectorAll('.tab').forEach(function(b){
    b.classList.toggle('on', b.dataset.t === t);
  });
}
async function doAck(){
  try { await fetch('/ack', {cache:'no-store'}); } catch(e){}
}
/* 把"距上次收到多少秒"变成"3 秒前"这种，-1 表示从来没收到过 */
function fmtAge(s){
  if(s === null || s === undefined || s < 0) return '未收到';
  if(s < 60)   return s + ' 秒前';
  if(s < 3600) return Math.floor(s / 60) + ' 分前';
  return Math.floor(s / 3600) + ' 时前';
}

/* ---------------- 态势图 ----------------
   以岸基为中心、正北朝上：蓝点=岸基，圆点=信标（按编号配色），绿点=船端。
   距离圈自动套到整齐的档位（50/100/200/500/1000/2000/5000 米）。
   ------------------------------------------------------------------ */
const MAP_STEPS  = [50, 100, 200, 500, 1000, 2000, 5000];
const TGT_COLORS = ['#ff6b6b', '#ffb454', '#7ee2a8', '#c792ea'];

function fmtRange(m){
  if(m >= 1000) return (m / 1000).toFixed(m % 1000 ? 1 : 0) + ' km';
  return m.toFixed(0) + ' m';
}

function drawMap(d){
  const c = document.getElementById('smap');
  if(!c) return;
  const g = c.getContext('2d');
  const W = c.width, H = c.height, cx = W / 2, cy = H / 2;
  const Rpx = Math.min(W, H) / 2 - 34;

  /* 量程：取最远的目标，再套到整齐的档位上 */
  let far = 0;
  if(d.bHaveDir) far = Math.max(far, d.bDist);
  if(d.vHaveDir) far = Math.max(far, d.vDist);
  if(d.bs) d.bs.forEach(function(x){ if(x.haveDir) far = Math.max(far, x.dist); });
  let maxR = MAP_STEPS[0];
  for(let i = 0; i < MAP_STEPS.length; i++){
    maxR = MAP_STEPS[i];
    if(far <= MAP_STEPS[i] * 0.8) break;
  }

  function toScreen(brg, dist){
    const a = brg * Math.PI / 180;
    return { x: cx + Math.sin(a) * (dist / maxR) * Rpx,
             y: cy - Math.cos(a) * (dist / maxR) * Rpx };
  }

  g.fillStyle = '#0b1219';
  g.fillRect(0, 0, W, H);

  /* 距离圈 */
  g.lineWidth = 1;
  for(let k = 1; k <= 4; k++){
    g.strokeStyle = (k === 4) ? '#31414f' : '#1e2a35';
    g.beginPath(); g.arc(cx, cy, Rpx * k / 4, 0, Math.PI * 2); g.stroke();
  }
  /* 十字线 */
  g.strokeStyle = '#1e2a35';
  g.beginPath(); g.moveTo(cx - Rpx, cy); g.lineTo(cx + Rpx, cy); g.stroke();
  g.beginPath(); g.moveTo(cx, cy - Rpx); g.lineTo(cx, cy + Rpx); g.stroke();

  /* 距离刻度 */
  g.fillStyle = '#5d6b78';
  g.font = '12px Consolas,Menlo,monospace';
  g.textAlign = 'left'; g.textBaseline = 'middle';
  for(let k = 1; k <= 4; k++) g.fillText(fmtRange(maxR * k / 4), cx + 5, cy - Rpx * k / 4);

  /* 方位标记 */
  const marks = [[0,'N'],[90,'E'],[180,'S'],[270,'W']];
  g.textAlign = 'center';
  for(let i = 0; i < marks.length; i++){
    const a = marks[i][0] * Math.PI / 180;
    const px = cx + Math.sin(a) * (Rpx + 16);
    const py = cy - Math.cos(a) * (Rpx + 16);
    g.fillStyle = (marks[i][1] === 'N') ? '#ff6b6b' : '#5d6b78';
    g.font = (marks[i][1] === 'N' ? 'bold ' : '') + '13px Consolas,Menlo,monospace';
    g.fillText(marks[i][1], px, py);
  }

  function dot(brg, dist, color, label){
    const p = toScreen(brg, dist);
    g.fillStyle = color;
    g.beginPath(); g.arc(p.x, p.y, 6, 0, Math.PI * 2); g.fill();
    g.fillStyle = '#e6edf3';
    g.font = '11px Consolas,Menlo,monospace';
    g.textAlign = 'left'; g.textBaseline = 'middle';
    g.fillText(label, p.x + 9, p.y + 1);
  }

  /* 信标（多只，按编号配色） */
  if(d.bs){
    for(let i = 0; i < d.bs.length; i++){
      const x = d.bs[i];
      if(!x.haveDir) continue;
      dot(x.brg, x.dist, TGT_COLORS[i % TGT_COLORS.length],
          x.id > 0 ? ('信标' + x.id) : '信标');
    }
  }

  /* 船端 */
  if(d.vHaveDir) dot(d.vBrg, d.vDist, '#38bdf8', '船端');

  /* 中心：岸基自己 */
  g.fillStyle = '#e6edf3';
  g.beginPath(); g.arc(cx, cy, 5, 0, Math.PI * 2); g.fill();
  g.font = '11px Consolas,Menlo,monospace';
  g.textAlign = 'left'; g.textBaseline = 'middle';
  g.fillText('岸基', cx + 8, cy + 1);

  /* 右上角标量程 */
  g.fillStyle = '#5d6b78';
  g.textAlign = 'right';
  g.fillText('量程 ' + fmtRange(maxR), cx + Rpx, cy - Rpx - 8);
}

/* ---------------- 数据记录控制 ---------------- */
async function logCtl(op){
  try { await fetch('/logctl?op=' + op, {cache:'no-store'}); } catch(e){}
}
function toggleLog(){
  const on = document.getElementById('btnLog').dataset.on === '1';
  logCtl(on ? 'off' : 'on');
}
function clearLog(){
  logCtl('clear');
  pbData = null; pbIdx = 0;
  document.getElementById('pbRange').max = 0;
  document.getElementById('pbInfo').textContent = '已清空。';
}

/* ---------------- 数据回放 ----------------
   CSV 列顺序（和 logbook.cpp 的表头一致）：
   0 t_ms 1 time 2 node_lat 3 node_lon 4 node_sats 5 node_hdop 6 node_fix 7 tof_mm
   8 b_count 9 b_id 10 b_link 11 b_valid 12 b_lat 13 b_lon 14 b_dist_m 15 b_brg
   16 v_link 17 v_valid 18 v_lat 19 v_lon 20 v_dist_m 21 v_brg 22 v_sog_kn 23 v_cog 24 v_sats */
let pbData = null, pbIdx = 0, pbTimer = null;
async function loadPlayback(){
  try{
    const t = await (await fetch('/log',{cache:'no-store'})).text();
    const all = t.split('\n');
    pbData = all.slice(1).filter(function(l){ return l.length > 3; });
    const r = document.getElementById('pbRange');
    if(!pbData.length){
      document.getElementById('pbInfo').textContent = '还没有记录。';
      r.max = 0;
      return;
    }
    r.max = pbData.length - 1;
    r.value = pbData.length - 1;
    showRec(pbData.length - 1);
  }catch(e){
    document.getElementById('pbInfo').textContent = '加载失败。';
  }
}
function showRec(i){
  if(!pbData || !pbData.length) return;
  pbIdx = Math.min(Math.max(0, parseInt(i,10) || 0), pbData.length - 1);
  const p = pbData[pbIdx].split(',');
  const onoff = function(v){ return v === '1' ? '在线' : '离线'; };
  document.getElementById('pbInfo').textContent =
    '第 ' + (pbIdx + 1) + ' / ' + pbData.length + ' 条\n' +
    '时间     ' + (p[1] || '--') + '     岸基卫星 ' + p[4] + ' 颗   HDOP ' + p[5] + '\n' +
    '岸基位置 ' + p[2] + ', ' + p[3] + '\n' +
    '信标     ' + (p[9] || '--') + ' 号   ' + onoff(p[10]) + '   ' + (p[12] || '--') + ', ' + (p[13] || '--') + '\n' +
    '         距岸基 ' + (p[14] || '--') + ' m   方位 ' + (p[15] || '--') + '°\n' +
    '船端     ' + onoff(p[16]) + '   ' + (p[18] || '--') + ', ' + (p[19] || '--') + '\n' +
    '         距岸基 ' + (p[20] || '--') + ' m   方位 ' + (p[21] || '--') + '°   速度 ' + (p[22] || '--') + ' 节\n' +
    '岸侧测距 ' + (p[7] || '--') + ' mm';
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
  const TrackTarget& b = trackBeacon();
  const TrackTarget& v = trackVessel();
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
  j += ",\"bj\":\"";       j += gpsBeijingTime();  j += "\"";
  j += ",\"date\":\"";     j += gpsDateText();     j += "\"";
  j += ",\"antenna\":\"";  j += escapeJson(gpsAntenna()); j += "\"";
  j += ",\"alarm\":\"";    j += escapeJson(gpsAlarmText()); j += "\"";
  j += ",\"frame\":\"";    j += gpsPoseFrameHex(); j += "\"";
  j += ",\"raw\":\"";      j += escapeJson(gpsLastGga()); j += "\"";
  j += ",\"loraReady\":";  j += (loraIsReady() ? "true" : "false");
  j += ",\"chRssi\":";     j += loraChannelRssi();
  j += ",\"oled\":\"";     j += escapeJson(String(oledStatusText())); j += "\"";
  j += ",\"tof\":\"";      j += escapeJson(tofText()); j += "\"";

  j += ",\"bLink\":";   j += (b.linkUp ? "true" : "false");
  j += ",\"bHas\":";    j += (b.has ? "true" : "false");
  j += ",\"bId\":";     j += b.id;
  j += ",\"bCount\":";  j += trackBeaconCount();
  /* 最近更新：距上次收到过了多少秒；-1 表示从来没收到过 */
  {
    uint32_t ab = trackAgeMs(b);
    snprintf(num, sizeof(num), "%ld", (ab == 0xFFFFFFFFUL) ? -1L : (long)(ab / 1000UL));
    j += ",\"bAge\":"; j += num;
  }
  j += ",\"bValid\":";  j += (b.valid ? "true" : "false");
  j += ",\"bSeq\":";    j += (unsigned long)b.seq;
  j += ",\"bRssi\":";   j += b.rssi;
  snprintf(num, sizeof(num), "%.1f", b.snr); j += ",\"bSnr\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", b.lat); j += ",\"bLat\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", b.lon); j += ",\"bLon\":"; j += num;
  j += ",\"bHaveDir\":"; j += (b.haveDir ? "true" : "false");
  snprintf(num, sizeof(num), "%.0f", b.distM);  j += ",\"bDist\":"; j += num;
  j += ",\"bDirText\":\""; j += escapeJson(String(trackDirText(b))); j += "\"";

  j += ",\"vLink\":";   j += (v.linkUp ? "true" : "false");
  j += ",\"vHas\":";    j += (v.has ? "true" : "false");
  j += ",\"vValid\":";  j += (v.valid ? "true" : "false");
  j += ",\"vSeq\":";    j += (unsigned long)v.seq;
  j += ",\"vRssi\":";   j += v.rssi;
  snprintf(num, sizeof(num), "%.1f", v.snr); j += ",\"vSnr\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", v.lat); j += ",\"vLat\":"; j += num;
  snprintf(num, sizeof(num), "%.6f", v.lon); j += ",\"vLon\":"; j += num;
  j += ",\"vHaveDir\":"; j += (v.haveDir ? "true" : "false");
  snprintf(num, sizeof(num), "%.0f", v.distM);  j += ",\"vDist\":"; j += num;
  snprintf(num, sizeof(num), "%.0f", v.bearing); j += ",\"vBrg\":";  j += num;
  j += ",\"vDirText\":\""; j += escapeJson(String(trackDirText(v))); j += "\"";

  /* 船端的附加数据（只有 S 帧带；收到的是 A 帧时保留上一次的值） */
  {
    uint32_t av = trackAgeMs(v);
    snprintf(num, sizeof(num), "%ld", (av == 0xFFFFFFFFUL) ? -1L : (long)(av / 1000UL));
    j += ",\"vAge\":"; j += num;
  }
  j += ",\"vHasExtra\":"; j += (v.hasExtra ? "true" : "false");
  snprintf(num, sizeof(num), "%.1f", v.sogKnots); j += ",\"vSog\":";  j += num;
  snprintf(num, sizeof(num), "%.0f", v.cogDeg);   j += ",\"vCog\":";  j += num;
  j += ",\"vSats\":";     j += v.sats;

  /* 全部信标（多只时网页要列出每一只，不只是最近活跃那只） */
  j += ",\"acked\":";  j += (trackAcked() ? "true" : "false");
  j += ",\"bs\":[";
  int bn = trackBeaconCount();
  for (int i = 0; i < bn; i++) {
    const TrackTarget& t = trackBeaconAt(i);
    if (i) j += ",";
    j += "{";
    j += "\"id\":";     j += t.id;
    j += ",\"link\":";  j += (t.linkUp ? "true" : "false");
    j += ",\"has\":";   j += (t.has ? "true" : "false");
    j += ",\"valid\":"; j += (t.valid ? "true" : "false");
    j += ",\"seq\":";   j += (unsigned long)t.seq;
    j += ",\"rssi\":";  j += t.rssi;
    snprintf(num, sizeof(num), "%.1f", t.snr);    j += ",\"snr\":";  j += num;
    snprintf(num, sizeof(num), "%.6f", t.lat);    j += ",\"lat\":";  j += num;
    snprintf(num, sizeof(num), "%.6f", t.lon);    j += ",\"lon\":";  j += num;
    j += ",\"haveDir\":"; j += (t.haveDir ? "true" : "false");
    snprintf(num, sizeof(num), "%.0f", t.distM);  j += ",\"dist\":"; j += num;
    snprintf(num, sizeof(num), "%.0f", t.bearing); j += ",\"brg\":"; j += num;
    j += ",\"dirText\":\""; j += escapeJson(String(trackDirText(t))); j += "\"";
    /* 多目标救援优先级：漂移速度 + 综合评分 + 离船端多远 */
    snprintf(num, sizeof(num), "%.2f", t.driftMps); j += ",\"drift\":"; j += num;
    snprintf(num, sizeof(num), "%.1f", trackPriority(t)); j += ",\"score\":"; j += num;
    snprintf(num, sizeof(num), "%.0f", trackDistToVessel(t)); j += ",\"vdist\":"; j += num;
    {
      uint32_t at = trackAgeMs(t);
      snprintf(num, sizeof(num), "%ld", (at == 0xFFFFFFFFUL) ? -1L : (long)(at / 1000UL));
      j += ",\"age\":"; j += num;
    }
    j += "}";
  }
  j += "]";

  j += ",\"runSec\":";     j += (millis() / 1000);
  /* 数据记录状态 */
  j += ",\"logOn\":";      j += (logbookOn() ? "true" : "false");
  j += ",\"logCount\":";   j += logbookCount();
  j += ",\"logCap\":";     j += logbookCapacity();
  j += ",\"logSpan\":";    j += logbookSpanSec();
  j += "}";
  return j;
}

static void handleRoot() {
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleData() {
  server.send(200, "application/json; charset=utf-8", buildJson());
}

/* 下载 CSV：带表头，Excel 直接能开 */
static void handleLogCSV() {
  String csv = logbookCSV();
  server.sendHeader("Content-Disposition", "attachment; filename=shore_log.csv");
  server.send(200, "text/csv; charset=utf-8", csv);
  Serial.printf("[记录] 已导出 CSV：%u 条，约 %u 字节\n",
                (unsigned)logbookCount(), (unsigned)csv.length());
}

static void handleNotFound() {
  server.send(404, "text/plain; charset=utf-8", "404 Not Found");
}

/* ---------------- WiFi ---------------- */

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
}

static void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  delay(100);

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
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("固定 IP 没有连上，改用路由器自动分配重试…");
    WiFi.disconnect(true, true);
    delay(200);
    WiFi.mode(WIFI_STA);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
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
  server.on("/ack", []() {
    trackAcknowledge();
    server.send(200, "text/plain; charset=utf-8", "ok");
  });
  server.on("/log", handleLogCSV);
  server.on("/logctl", []() {
    String op = server.arg("op");
    if      (op == "on")    logbookSetOn(true);
    else if (op == "off")   logbookSetOn(false);
    else if (op == "clear") logbookClear();
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
  });
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("网页服务已启动。");
}

void netLoop() {
  server.handleClient();

  if (apMode) return;

  if (WiFi.status() == WL_DISCONNECTED && millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();
    Serial.println("WiFi 已断开，正在重连…");
    WiFi.reconnect();
  }
}

bool netConnected() { return WiFi.status() == WL_CONNECTED; }
String netIP()      { return WiFi.localIP().toString(); }
