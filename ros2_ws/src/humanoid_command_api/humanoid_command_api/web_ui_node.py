"""
web_ui_node.py
==============

AI Step 7 — Web UI (Piece 1: backend + minimal chat page).

The browser-facing front door to the robot. Instead of the terminal REPL in
`llm_dispatcher_node.main()`, you open a web page, type a command, and the
SAME `dispatch()` pipeline (Ollama tool-use + SafetyValidator + ROS2 actions,
AI Steps 4-6) runs it on the robot and shows the reply. Nothing about the
brain changes — this file only adds a new way to *reach* it.

ARCHITECTURE
------------
    Browser (chat page, plain HTML + a little JS)
        │  POST /api/command   {"text": "wave your right hand"}
        ▼
    web_ui_node.py  (this file)
        - holds ONE LLMDispatcherNode  (reuses all of Steps 4-6)
        - a tiny stdlib http.server     (NO Flask/FastAPI -> no pip install;
          the root filesystem is tight, see CLAUDE.md §7)
        - routes:  GET  /              -> the chat page (HTML)
                   GET  /health        -> {"ok": true}   (liveness check)
                   POST /api/command   -> run dispatch(), return {"reply": ...}
        ▼
    LLMDispatcherNode.dispatch(text)  (existing) -> Ollama -> ROS2 robot

WHY A LOCK (the one subtlety)
-----------------------------
`dispatch()` BLOCKS and spins rclpy (`spin_until_future_complete`).
`http.server` (ThreadingHTTPServer) handles each request in its own thread,
so two browser requests could try to spin rclpy at the SAME time -> unsafe.
A single `threading.Lock` serializes commands: exactly one runs at a time.
That isn't a restriction we're inventing — the robot can only perform one
motion at a time, and the dispatcher was designed "one command at a time"
(Step 4). The lock just makes the web layer honour that.

PIECE 1 SCOPE
-------------
Backend + a deliberately minimal chat page, plus an `--echo` debug mode that
SKIPS dispatch and just echoes the text, so the browser<->backend HTTP/JSON
loop can be verified WITHOUT Ollama or the command server running (mirrors
the dispatcher's `--test`). Later pieces: add a Stop button + live status to
the page, and a launch file that brings up command_server + web_ui together.
"""

import sys
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from humanoid_msgs.srv import Stop, GetStatus

from .llm_dispatcher_node import LLMDispatcherNode


# Bind to localhost only: this is a personal control panel, not a public
# service. Change HOST to "0.0.0.0" if you ever want to reach it from another
# device on the LAN (understand the safety implications first).
HOST = "127.0.0.1"
PORT = 8080

# Short timeout for the Stop button / status poll: these must feel snappy and
# fail fast if the command server isn't up (unlike a long-running motion).
EMERGENCY_TIMEOUT_SEC = 2.0


# ── Shared state, set once in main() and read by the request handler ──
# http.server creates a NEW handler instance per request, so the node, the
# serialization lock, and the echo flag live at module scope where every
# handler can see them.
_NODE: "LLMDispatcherNode | None" = None
_DISPATCH_LOCK = threading.Lock()
_ECHO_MODE = False

# A SECOND, independent node + its own lock, dedicated to the Stop button and
# status polling (see _EmergencyNode for why it must be separate).
_EMERGENCY: "Node | None" = None
_EMERGENCY_CTX = None
_EMERGENCY_LOCK = threading.Lock()


# ── The chat page ─────────────────────────────────────────────────────
# Embedded as a string (not a separate file) so there are no ament install
# path / resource-lookup headaches: one self-contained page, served from
# memory. It's intentionally tiny — a message list, an input box, and a
# fetch() POST to /api/command.
INDEX_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Abbes · Humanoid Control</title>
<style>
  :root{
    --bg:#0b0e14; --panel:#11151f; --panel2:#161b27; --line:#222a39;
    --text:#e6edf3; --muted:#8b97a8; --accent:#21d4c8; --accent-d:#0f8e86;
    --you:#1f6feb; --ok:#2ea043; --warn:#d29922; --danger:#f85149;
    --r:14px;
  }
  *{box-sizing:border-box}
  html,body{margin:0;min-height:100%}
  body{
    font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
    color:var(--text); min-height:100vh; line-height:1.45;
    background:
      radial-gradient(1100px 520px at 82% -12%, #15314a 0%, transparent 58%),
      radial-gradient(900px 500px at -10% 110%, #1a2335 0%, transparent 55%),
      var(--bg);
  }
  h1,h2{margin:0; font-weight:650}

  /* ── top bar ─────────────────────────────────────────────── */
  .topbar{
    display:flex; align-items:center; justify-content:space-between; gap:1rem;
    padding:.85rem 1.25rem; border-bottom:1px solid var(--line);
    background:rgba(12,16,24,.7); backdrop-filter:blur(8px);
    position:sticky; top:0; z-index:10; flex-wrap:wrap;
  }
  .brand{display:flex; align-items:center; gap:.8rem}
  .avatar{
    width:46px; height:46px; border-radius:13px; display:grid; place-items:center;
    background:linear-gradient(145deg,var(--accent),var(--accent-d));
    box-shadow:0 0 0 1px #ffffff18, 0 6px 18px #0008; flex:0 0 auto;
  }
  .avatar svg{width:30px; height:30px; display:block}
  .eye{fill:#06202a; animation:blink 4.5s infinite}
  @keyframes blink{0%,92%,100%{transform:scaleY(1)}96%{transform:scaleY(.1)}}
  .brand h1{font-size:1.25rem; letter-spacing:.3px}
  .brand p{margin:.1rem 0 0; color:var(--muted); font-size:.78rem}
  .topright{display:flex; align-items:center; gap:.7rem; flex-wrap:wrap}
  .pill{
    display:inline-flex; align-items:center; gap:.45rem; font-size:.78rem;
    padding:.4rem .7rem; border-radius:999px; border:1px solid var(--line);
    background:var(--panel2); color:var(--muted);
  }
  .pill .dot{width:9px; height:9px; border-radius:50%; background:var(--muted)}
  .pill.online .dot{background:var(--ok); box-shadow:0 0 8px var(--ok)}
  .pill.online{color:#bdebc6}
  .pill.offline .dot{background:var(--danger); box-shadow:0 0 8px var(--danger)}
  .pill.offline{color:#f7b8b4}
  .badge-echo{
    display:none; font-size:.7rem; font-weight:700; letter-spacing:.5px;
    padding:.3rem .55rem; border-radius:999px; color:#1b1300;
    background:var(--warn);
  }
  .stopbtn{
    border:0; cursor:pointer; font-weight:800; letter-spacing:.4px;
    color:#fff; padding:.6rem 1rem; border-radius:11px; font-size:.85rem;
    background:linear-gradient(145deg,#ff5247,#c92e25);
    box-shadow:0 4px 14px #f8514955; transition:transform .06s, filter .15s;
  }
  .stopbtn:hover{filter:brightness(1.08)}
  .stopbtn:active{transform:translateY(1px)}

  /* ── layout ──────────────────────────────────────────────── */
  .grid{
    display:grid; grid-template-columns:1fr 340px; gap:1rem;
    max-width:1180px; margin:1rem auto; padding:0 1rem;
  }
  @media(max-width:880px){ .grid{grid-template-columns:1fr} }
  .panel{
    background:var(--panel); border:1px solid var(--line); border-radius:var(--r);
    box-shadow:0 8px 30px #0004;
  }

  /* ── chat column ─────────────────────────────────────────── */
  .chatcol{display:flex; flex-direction:column; height:calc(100vh - 130px); min-height:440px}
  .chathead{
    display:flex; align-items:center; justify-content:space-between;
    padding:.8rem 1rem; border-bottom:1px solid var(--line); font-weight:600;
  }
  .chathead small{color:var(--muted); font-weight:400}
  .ghost{
    background:transparent; border:1px solid var(--line); color:var(--muted);
    border-radius:8px; padding:.35rem .65rem; cursor:pointer; font-size:.78rem;
  }
  .ghost:hover{color:var(--text); border-color:#33405a}
  .log{flex:1; overflow-y:auto; padding:1rem; display:flex; flex-direction:column; gap:.7rem}
  .row{display:flex; gap:.6rem; max-width:88%}
  .row.you{align-self:flex-end; flex-direction:row-reverse}
  .av{
    width:30px; height:30px; border-radius:9px; flex:0 0 auto; display:grid;
    place-items:center; font-size:.95rem;
  }
  .row.robot .av{background:linear-gradient(145deg,var(--accent),var(--accent-d)); color:#06202a}
  .row.you .av{background:var(--you); color:#fff}
  .bubble{
    padding:.6rem .8rem; border-radius:13px; white-space:pre-wrap;
    word-break:break-word; font-size:.93rem;
  }
  .row.robot .bubble{background:var(--panel2); border:1px solid var(--line); border-top-left-radius:4px}
  .row.you .bubble{background:#1f6feb22; border:1px solid #1f6feb55; border-top-right-radius:4px}
  .row.err .bubble{background:#f8514915; border:1px solid #f8514955; color:#f7b8b4}
  .stamp{display:block; margin-top:.3rem; font-size:.68rem; color:var(--muted)}
  .sys{align-self:center; color:var(--muted); font-size:.8rem; font-style:italic}
  .dots span{
    display:inline-block; width:6px; height:6px; margin:0 1px; border-radius:50%;
    background:var(--accent); animation:bb 1.2s infinite ease-in-out;
  }
  .dots span:nth-child(2){animation-delay:.2s}
  .dots span:nth-child(3){animation-delay:.4s}
  @keyframes bb{0%,80%,100%{opacity:.25; transform:translateY(0)}40%{opacity:1; transform:translateY(-4px)}}
  .composer{display:flex; gap:.5rem; padding:.8rem; border-top:1px solid var(--line)}
  .composer input{
    flex:1; padding:.7rem .85rem; border-radius:11px; border:1px solid var(--line);
    background:var(--bg); color:var(--text); font-size:.95rem; outline:none;
  }
  .composer input:focus{border-color:var(--accent)}
  .composer button{
    border:0; border-radius:11px; padding:.7rem 1.1rem; cursor:pointer; font-weight:700;
    color:#06202a; background:linear-gradient(145deg,var(--accent),var(--accent-d));
  }
  .composer button:disabled{opacity:.45; cursor:default}

  /* ── side column ─────────────────────────────────────────── */
  .sidecol{display:flex; flex-direction:column; gap:1rem}
  .sidecol .panel{padding:1rem}
  .sidecol h2{font-size:.95rem; margin-bottom:.2rem}
  .sub{color:var(--muted); font-size:.78rem; margin:.15rem 0 .9rem}
  .statusgrid{display:flex; gap:1rem; align-items:center}
  .level{
    width:96px; height:96px; border-radius:50%; flex:0 0 auto; position:relative;
    background:radial-gradient(circle at 50% 50%, #0c1422, #060a12);
    border:1px solid var(--line); overflow:hidden;
  }
  .level .ring{position:absolute; inset:0; border-radius:50%;
    background:
      repeating-radial-gradient(circle at 50% 50%, transparent 0 22px, #1c2536 22px 23px);}
  .level .cross::before,.level .cross::after{
    content:""; position:absolute; background:#23304a;
  }
  .level .cross::before{left:50%; top:8%; bottom:8%; width:1px; transform:translateX(-.5px)}
  .level .cross::after{top:50%; left:8%; right:8%; height:1px; transform:translateY(-.5px)}
  .bubblelvl{
    position:absolute; left:50%; top:50%; width:20px; height:20px; border-radius:50%;
    margin:-10px 0 0 -10px; transition:transform .35s ease;
    background:radial-gradient(circle at 35% 30%, #6ff7ec, var(--accent));
    box-shadow:0 0 12px var(--accent), 0 0 0 1px #ffffff22;
  }
  .level.tilted .bubblelvl{background:radial-gradient(circle at 35% 30%,#ffd27a,var(--warn));
    box-shadow:0 0 12px var(--warn)}
  .statusmeta{flex:1; display:flex; flex-direction:column; gap:.35rem}
  .kv{display:flex; justify-content:space-between; font-size:.82rem}
  .kv span{color:var(--muted)}
  .kv b{font-weight:600}
  .act-chip{
    display:inline-block; padding:.12rem .5rem; border-radius:999px; font-size:.78rem;
    background:var(--panel2); border:1px solid var(--line);
  }
  .act-chip.busy{background:#21d4c81f; border-color:#21d4c855; color:var(--accent)}

  .capgrid{display:grid; grid-template-columns:1fr 1fr; gap:.55rem}
  .cap{
    text-align:left; cursor:pointer; color:var(--text);
    background:var(--panel2); border:1px solid var(--line); border-radius:11px;
    padding:.6rem .65rem; display:flex; flex-direction:column; gap:.15rem;
    transition:transform .08s, border-color .15s, background .15s;
  }
  .cap:hover{border-color:var(--accent); background:#21d4c812; transform:translateY(-1px)}
  .cap:active{transform:translateY(0)}
  .cap .ic{font-size:1.15rem}
  .cap .t{font-weight:650; font-size:.86rem}
  .cap .d{color:var(--muted); font-size:.72rem; line-height:1.25}
  .about p{color:var(--muted); font-size:.8rem; margin:.3rem 0 0}
  .about .tip{margin-top:.6rem; color:#cdd6e0}
  kbd{background:#0c1320; border:1px solid var(--line); border-radius:5px;
    padding:.05rem .35rem; font-size:.72rem}
</style>
</head>
<body>
  <header class="topbar">
    <div class="brand">
      <div class="avatar" title="Abbes">
        <svg viewBox="0 0 32 32" aria-hidden="true">
          <rect x="6" y="9" width="20" height="15" rx="4" fill="#06202a"/>
          <rect x="9" y="12" width="14" height="9" rx="2.5" fill="#0c3a44"/>
          <circle class="eye" cx="13" cy="16.5" r="2.1"/>
          <circle class="eye" cx="19" cy="16.5" r="2.1"/>
          <rect x="14.2" y="4.5" width="3.6" height="4" rx="1.4" fill="#06202a"/>
          <circle cx="16" cy="4" r="1.7" fill="#06202a"/>
        </svg>
      </div>
      <div class="brandtext">
        <h1>Abbes</h1>
        <p>INSAT humanoid · 18 DOF · local-AI control</p>
      </div>
    </div>
    <div class="topright">
      <span class="badge-echo" id="echo">ECHO MODE</span>
      <span class="pill" id="conn"><span class="dot"></span> connecting…</span>
      <button id="stop" class="stopbtn" type="button">■ EMERGENCY STOP</button>
    </div>
  </header>

  <main class="grid">
    <!-- chat -->
    <section class="panel chatcol">
      <div class="chathead">
        <span>Talk to Abbes <small>— plain language</small></span>
        <button id="clear" class="ghost" type="button">Clear</button>
      </div>
      <div id="log" class="log"></div>
      <form id="form" class="composer">
        <input id="text" autocomplete="off" placeholder="e.g. “wave your left hand” or “walk forward 1 meter”" autofocus>
        <button id="send" type="submit">Send ➤</button>
      </form>
    </section>

    <!-- sidebar -->
    <aside class="sidecol">
      <div class="panel">
        <h2>Live status</h2>
        <p class="sub">Polled from the robot every few seconds.</p>
        <div class="statusgrid">
          <div class="level" id="level">
            <div class="ring"></div><div class="cross"></div>
            <div class="bubblelvl" id="lvlbubble"></div>
          </div>
          <div class="statusmeta">
            <div class="kv"><span>Action</span><b><span class="act-chip" id="act">…</span></b></div>
            <div class="kv"><span>Pitch</span><b id="pitch">—</b></div>
            <div class="kv"><span>Roll</span><b id="roll">—</b></div>
            <div class="kv"><span>Updated</span><b id="upd">—</b></div>
          </div>
        </div>
      </div>

      <div class="panel">
        <h2>What Abbes can do</h2>
        <p class="sub">Click a card, or just type — the AI picks the motion.</p>
        <div class="capgrid" id="caps">
          <button class="cap" data-cmd="Wave your right hand hello">
            <span class="ic">👋</span><span class="t">Wave</span>
            <span class="d">Greet with a wave of one hand</span>
          </button>
          <button class="cap" data-cmd="Take a bow">
            <span class="ic">🙇</span><span class="t">Bow</span>
            <span class="d">Lean the torso forward at the hips</span>
          </button>
          <button class="cap" data-cmd="Walk forward half a meter">
            <span class="ic">🚶</span><span class="t">Walk</span>
            <span class="d">Step forward/back 0.1–3.0 m</span>
          </button>
          <button class="cap" data-cmd="Sit down">
            <span class="ic">🪑</span><span class="t">Sit</span>
            <span class="d">Lower into a resting sit pose</span>
          </button>
          <button class="cap" data-cmd="Stand up straight">
            <span class="ic">🧍</span><span class="t">Stand</span>
            <span class="d">Return to the neutral home pose</span>
          </button>
          <button class="cap" data-cmd="What is your status?">
            <span class="ic">📊</span><span class="t">Status</span>
            <span class="d">Report current action and tilt</span>
          </button>
        </div>
      </div>

      <div class="panel about">
        <h2>About Abbes</h2>
        <p>A child-sized humanoid built from scratch at INSAT — CAD, 3D-printed
           frame, ESP32 control, 18 servos. Your words are grounded by a local
           <b>llama3.2</b> model into a fixed, safety-checked command vocabulary;
           the AI never drives motors directly.</p>
        <p class="tip">Tip: press <kbd>Enter</kbd> to send. The red button is a
           true emergency stop — it interrupts a motion already in progress.</p>
      </div>
    </aside>
  </main>

<script>
const log   = document.getElementById('log');
const form  = document.getElementById('form');
const text  = document.getElementById('text');
const send  = document.getElementById('send');

function now(){ return new Date().toLocaleTimeString([], {hour:'2-digit', minute:'2-digit'}); }

function bubble(kind, icon, body){
  const row = document.createElement('div');
  row.className = 'row ' + kind;
  const av = document.createElement('div'); av.className = 'av'; av.textContent = icon;
  const b  = document.createElement('div'); b.className = 'bubble';
  b.textContent = body;
  const s  = document.createElement('span'); s.className = 'stamp'; s.textContent = now();
  b.appendChild(s);
  row.appendChild(av); row.appendChild(b);
  log.appendChild(row); log.scrollTop = log.scrollHeight;
  return row;
}
function sysline(html){
  const el = document.createElement('div'); el.className = 'sys'; el.innerHTML = html;
  log.appendChild(el); log.scrollTop = log.scrollHeight; return el;
}

function welcome(){
  bubble('robot', '🤖',
    "Hi, I'm Abbes. Tell me what to do in plain English — try “wave hello”, "
    + "“take a bow”, or “walk forward 1 meter”. Tap a card on the right for ideas.");
}
welcome();

// ── send a command ───────────────────────────────────────────────
let busy = false;
async function sendCommand(msg){
  msg = (msg || '').trim();
  if (!msg || busy) return;
  busy = true; send.disabled = true;
  bubble('you', '🧑', msg);
  text.value = '';

  // animated "thinking…" with an elapsed-seconds counter (cold model loads
  // can take ~45s on this GPU, so show progress instead of a frozen UI).
  const t0 = Date.now();
  const think = sysline('Abbes is thinking <span class="dots"><span></span><span></span><span></span></span> <b id="el">0s</b>');
  const el = think.querySelector('#el');
  const tick = setInterval(()=>{ el.textContent = Math.round((Date.now()-t0)/1000)+'s'; }, 500);

  try{
    const res = await fetch('/api/command', {
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({text: msg}),
    });
    const data = await res.json();
    clearInterval(tick); think.remove();
    if (data.reply) bubble('robot', '🤖', data.reply);
    else bubble('err', '⚠️', 'Error: ' + (data.error || 'no reply'));
  } catch(err){
    clearInterval(tick); think.remove();
    bubble('err', '⚠️', 'Request failed: ' + err);
  } finally {
    busy = false; send.disabled = false; text.focus(); poll();
  }
}
form.addEventListener('submit', e => { e.preventDefault(); sendCommand(text.value); });

// quick-command cards
document.getElementById('caps').addEventListener('click', e => {
  const card = e.target.closest('.cap'); if (!card) return;
  sendCommand(card.dataset.cmd);
});

// clear conversation
document.getElementById('clear').addEventListener('click', () => {
  log.innerHTML = ''; welcome();
});

// ── live status poll ─────────────────────────────────────────────
const conn  = document.getElementById('conn');
const actEl = document.getElementById('act');
const pEl   = document.getElementById('pitch');
const rEl   = document.getElementById('roll');
const updEl = document.getElementById('upd');
const level = document.getElementById('level');
const lvlb  = document.getElementById('lvlbubble');

function setConn(on){
  conn.className = 'pill ' + (on ? 'online' : 'offline');
  conn.lastChild.textContent = on ? ' online' : ' offline';
}

async function poll(){
  try{
    const r = await fetch('/api/status');
    const d = await r.json();
    setConn(true);
    const act = (d.current_action || 'idle');
    actEl.textContent = act;
    actEl.className = 'act-chip' + (act && act !== 'idle' ? ' busy' : '');
    const p = +(d.tilt_pitch_deg ?? 0), rr = +(d.tilt_roll_deg ?? 0);
    pEl.textContent = p.toFixed(1) + '°';
    rEl.textContent = rr.toFixed(1) + '°';
    updEl.textContent = now();
    // move the bubble: roll -> x, pitch -> y, clamped to the dial (~38px)
    const cl = v => Math.max(-38, Math.min(38, v * 2.2));
    lvlb.style.transform = `translate(${cl(rr)}px, ${cl(p)}px)`;
    level.classList.toggle('tilted', Math.abs(p) > 15 || Math.abs(rr) > 15);
  } catch(e){
    setConn(false);
    actEl.textContent = 'offline'; actEl.className = 'act-chip';
  }
}
setInterval(poll, 3000); poll();

// echo-mode badge (front door tells us if the backend is in --echo)
fetch('/health').then(r=>r.json()).then(d=>{
  if (d.echo) document.getElementById('echo').style.display = 'inline-block';
}).catch(()=>{});

// ── emergency stop (independent path, interrupts a running motion) ──
document.getElementById('stop').addEventListener('click', async () => {
  sysline('🛑 <b>EMERGENCY STOP</b> requested…');
  try{
    const r = await fetch('/api/stop', {method:'POST'});
    const d = await r.json();
    sysline(d.ok ? ('🛑 ' + (d.message || 'robot stopped'))
                 : ('stop failed: ' + (d.message || '')));
  } catch(e){ sysline('stop request failed: ' + e); }
  poll();
});
</script>
</body>
</html>
"""


class _EmergencyNode(Node):
    """A SECOND, independent node for the Stop button and status polling.

    WHY separate from the dispatcher node:
    `dispatch()` BLOCKS (spins the dispatcher node) for the whole duration of
    a motion — e.g. a multi-second walk. The command SERVER is multi-threaded,
    and its Stop service sets a flag that every action's trajectory loop checks
    each tick, so an out-of-band Stop WILL interrupt a running motion
    *server-side*. But the web layer can only use that if it can call Stop
    WITHOUT waiting on the in-flight dispatch. A distinct node (own clients,
    spun on the request thread, serialized by _EMERGENCY_LOCK) runs CONCURRENTLY
    with a dispatch — so the red button is a real interrupt, not something
    queued behind the walk.

    CRITICAL: this node runs on its OWN rclpy CONTEXT + executor (see below).
    It is NOT enough to merely use a different node. `dispatch()` and a status
    poll / Stop spin rclpy on DIFFERENT threads at the SAME time, and two
    threads each calling spin_until_future_complete on nodes that share ONE
    context race on that context's wait set — corrupting the dispatcher's
    action clients ("wait set index for cancel client is out of bounds",
    rcl_action/action_client.c:650). A separate context gives this node its
    own rcl machinery, so the two spin loops never touch the same wait set.
    (Reproduced + fix-verified: with a shared context the 2nd action after a
    concurrent poll crashes; with separate contexts 8/8 actions pass.)
    """

    def __init__(self, context):
        super().__init__("web_ui_emergency", context=context)
        # Dedicated executor bound to OUR context — used to spin our own
        # service calls, completely independent of the dispatcher's spinning.
        self._exec = SingleThreadedExecutor(context=context)
        self._exec.add_node(self)
        self.stop_client = self.create_client(Stop, "/humanoid/stop")
        self.status_client = self.create_client(GetStatus, "/humanoid/get_status")

    def _call(self, client, request, name: str, fields: list) -> dict:
        if not client.wait_for_service(timeout_sec=EMERGENCY_TIMEOUT_SEC):
            return {"ok": False,
                    "message": f"{name}: service unavailable "
                               f"(is command_server running?)"}
        future = client.call_async(request)
        self._exec.spin_until_future_complete(
            future, timeout_sec=EMERGENCY_TIMEOUT_SEC)
        resp = future.result()
        if resp is None:
            return {"ok": False, "message": f"{name}: no response (timeout)"}
        # Stop uses .success; GetStatus uses .ok. Normalize to "ok".
        ok = getattr(resp, "success", None)
        if ok is None:
            ok = getattr(resp, "ok", False)
        out = {"ok": bool(ok)}
        for f in fields:
            out[f] = getattr(resp, f, None)
        return out

    def stop(self) -> dict:
        return self._call(self.stop_client, Stop.Request(), "stop", ["message"])

    def status(self) -> dict:
        return self._call(
            self.status_client, GetStatus.Request(), "get_status",
            ["current_action", "tilt_pitch_deg", "tilt_roll_deg", "message"])


class RobotWebHandler(BaseHTTPRequestHandler):
    """One instance per request. Routes GET/POST to the robot brain."""

    # Quieten the default per-request stderr logging; route through the node.
    def log_message(self, fmt, *args):
        if _NODE is not None:
            _NODE.get_logger().debug("http: " + (fmt % args))

    def _send(self, code: int, body: bytes, content_type: str):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, code: int, obj: dict):
        self._send(code, json.dumps(obj).encode("utf-8"), "application/json")

    def _emergency(self, which: str) -> dict:
        """Stop / status via the independent emergency node (or fake data in
        echo mode so the page is fully usable without ROS)."""
        if _ECHO_MODE:
            if which == "stop":
                return {"ok": True, "message": "(echo) stop"}
            return {"ok": True, "current_action": "(echo) idle",
                    "tilt_pitch_deg": 0.0, "tilt_roll_deg": 0.0, "message": "(echo)"}
        if _EMERGENCY is None:
            return {"ok": False, "message": "emergency node not ready"}
        with _EMERGENCY_LOCK:
            return _EMERGENCY.stop() if which == "stop" else _EMERGENCY.status()

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path == "/health":
            self._send_json(200, {"ok": True, "echo": _ECHO_MODE})
        elif self.path == "/api/status":
            self._send_json(200, self._emergency("status"))
        else:
            self._send_json(404, {"error": f"no route {self.path!r}"})

    def do_POST(self):
        # Emergency stop: no body needed, runs on the independent node so it
        # can interrupt an in-flight motion instead of queuing behind it.
        if self.path == "/api/stop":
            self._send_json(200, self._emergency("stop"))
            return

        if self.path != "/api/command":
            self._send_json(404, {"error": f"no route {self.path!r}"})
            return

        # ── Parse the request body as JSON {"text": "..."} ──
        try:
            length = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(length) or b"{}")
            user_text = str(payload.get("text", "")).strip()
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json(400, {"error": f"bad request body: {exc}"})
            return

        if not user_text:
            self._send_json(400, {"error": "empty 'text'"})
            return

        # ── Echo debug mode: prove the HTTP/JSON loop without Ollama/ROS ──
        if _ECHO_MODE:
            self._send_json(200, {"reply": f"(echo) {user_text}"})
            return

        # ── Real path: one command at a time (see "WHY A LOCK") ──
        if _NODE is None:
            self._send_json(503, {"error": "dispatcher not ready"})
            return
        with _DISPATCH_LOCK:
            try:
                reply = _NODE.dispatch(user_text)
            except Exception as exc:  # never let one bad command kill the server
                _NODE.get_logger().error(f"dispatch raised: {exc!r}")
                self._send_json(500, {"error": f"dispatch failed: {exc}"})
                return
        self._send_json(200, {"reply": reply})


def main():
    """Serve the web control panel.

    Prerequisites for the REAL path: Ollama running with the model pulled,
    AND the command server up:  ros2 run humanoid_command_api command_server
    Then:                       ros2 run humanoid_command_api web_ui
    Open http://127.0.0.1:8080 in a browser.

    With  --echo  the server skips dispatch and just echoes text, so the
    browser<->backend loop can be checked without Ollama or the command
    server (the ROS node is still created, which also verifies imports).
    """
    global _NODE, _EMERGENCY, _EMERGENCY_CTX, _ECHO_MODE
    _ECHO_MODE = "--echo" in sys.argv

    # Dispatcher on the DEFAULT context; emergency node on its OWN context so
    # its status-poll / Stop spinning can run CONCURRENTLY with a dispatch
    # without corrupting the dispatcher's action-client wait set (see
    # _EmergencyNode for the gory details).
    rclpy.init()
    _NODE = LLMDispatcherNode()
    _EMERGENCY_CTX = rclpy.Context()
    rclpy.init(context=_EMERGENCY_CTX)
    _EMERGENCY = _EmergencyNode(_EMERGENCY_CTX)

    server = ThreadingHTTPServer((HOST, PORT), RobotWebHandler)
    mode = "ECHO (no Ollama/ROS dispatch)" if _ECHO_MODE else "LIVE"
    _NODE.get_logger().info(
        f"web UI [{mode}] serving on http://{HOST}:{PORT}  (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        _EMERGENCY.destroy_node()
        _NODE.destroy_node()
        rclpy.shutdown()
        rclpy.shutdown(context=_EMERGENCY_CTX)


if __name__ == "__main__":
    main()
