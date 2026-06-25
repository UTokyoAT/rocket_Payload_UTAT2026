#pragma once

static const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CanSat Monitor</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #111; color: #eee; font-family: monospace; padding: 16px; }
    h2 { margin-bottom: 16px; font-size: 1.2em; }
    #status { font-size: 0.9em; }
    #status.connected { color: #4fc; }
    #status.disconnected { color: #f44; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 12px; }
    .card { background: #1e1e1e; border-radius: 8px; padding: 14px; }
    .label { font-size: 0.75em; color: #888; margin-bottom: 6px; letter-spacing: 0.08em; }
    .value { font-size: 1.8em; font-weight: bold; color: #4fc; }
    .value.small { font-size: 1.1em; }
    .state-badge {
      display: inline-block;
      padding: 4px 12px;
      border-radius: 4px;
      font-size: 1.2em;
      font-weight: bold;
      background: #333;
      color: #ff0;
    }
    canvas { display: block; width: 100%; height: 180px; margin-top: 8px; }
  </style>
</head>
<body>
  <h2>CanSat Monitor &nbsp;<span id="status" class="disconnected">● disconnected</span></h2>

  <div class="grid">
    <div class="card">
      <div class="label">STATE</div>
      <div class="state-badge" id="state">--</div>
    </div>
    <div class="card">
      <div class="label">ALTITUDE</div>
      <div class="value"><span id="alt">--</span> <span style="font-size:0.5em">m</span></div>
    </div>
    <div class="card">
      <div class="label">ROLL / PITCH / YAW</div>
      <div class="value small">
        R: <span id="roll">--</span>°&nbsp;
        P: <span id="pitch">--</span>°&nbsp;
        Y: <span id="yaw">--</span>°
      </div>
    </div>
    <div class="card">
      <div class="label">GPS</div>
      <div class="value small">
        <span id="lat">--</span><br>
        <span id="lon">--</span>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="label">ALTITUDE HISTORY</div>
    <canvas id="chart"></canvas>
  </div>

  <script>
    const STATE_NAMES = ["STANDBY","ASCENDING","FALLING","SEPARATING","RUNNING","GOAL"];
    const MAX_PTS = 300;
    const altBuf = [];
    const canvas = document.getElementById('chart');
    const ctx = canvas.getContext('2d');

    function drawChart() {
      canvas.width  = canvas.offsetWidth;
      canvas.height = 180;
      const w = canvas.width, h = canvas.height, pad = 10;
      ctx.fillStyle = '#111'; ctx.fillRect(0, 0, w, h);
      if (altBuf.length < 2) return;
      const max = Math.max(...altBuf, 10);
      const min = Math.min(...altBuf, 0);
      const rng = max - min || 1;
      ctx.strokeStyle = '#4fc'; ctx.lineWidth = 2;
      ctx.beginPath();
      altBuf.forEach((v, i) => {
        const x = (i / (MAX_PTS - 1)) * w;
        const y = h - pad - ((v - min) / rng) * (h - pad * 2);
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.fillStyle = '#4fc';
      ctx.font = '10px monospace';
      ctx.fillText(max.toFixed(1) + 'm', 4, pad + 4);
      ctx.fillText(min.toFixed(1) + 'm', 4, h - 4);
    }

    function connect() {
      const ws = new WebSocket('ws://' + location.host + '/ws');
      const el = document.getElementById('status');

      ws.onopen = () => {
        el.textContent = '● connected';
        el.className = 'connected';
      };
      ws.onclose = () => {
        el.textContent = '● disconnected';
        el.className = 'disconnected';
        setTimeout(connect, 2000);
      };
      ws.onmessage = ({ data }) => {
        const d = JSON.parse(data);
        document.getElementById('state').textContent = STATE_NAMES[d.state] ?? d.state;
        document.getElementById('alt').textContent   = d.alt.toFixed(1);
        document.getElementById('roll').textContent  = d.roll.toFixed(1);
        document.getElementById('pitch').textContent = d.pitch.toFixed(1);
        document.getElementById('yaw').textContent   = d.yaw.toFixed(1);
        document.getElementById('lat').textContent   = d.lat.toFixed(6);
        document.getElementById('lon').textContent   = d.lon.toFixed(6);
        altBuf.push(d.alt);
        if (altBuf.length > MAX_PTS) altBuf.shift();
        drawChart();
      };
    }

    connect();
    window.addEventListener('resize', drawChart);
  </script>
</body>
</html>
)rawhtml";
