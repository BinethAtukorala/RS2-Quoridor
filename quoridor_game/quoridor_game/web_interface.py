"""Web GUI for the Quoridor game, exposed as a ROS2 node.

Mirrors the command set of the terminal ``user_interface`` node but renders a
modern single-page app. It subscribes to ``/quoridor/board_state`` and republishes
state to connected browsers via Server-Sent Events; user input is sent back via
small JSON POST endpoints.

Runs with the Python standard library only — no extra ROS or pip deps.
"""
import json
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from quoridor_game.quoridor_utils import (
    Move,
    MoveType,
    Orientation,
    Pawn,
    Wall,
)


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8088


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Quoridor · UR3e Control</title>
<script src="https://cdn.tailwindcss.com"></script>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;600&display=swap" rel="stylesheet">
<style>
  :root { color-scheme: light; }
  html, body { font-family: 'Inter', system-ui, sans-serif; }
  .mono { font-family: 'JetBrains Mono', monospace; }
  body {
    background:
      radial-gradient(1200px 600px at 10% -10%, rgba(212,175,55,.22), transparent 60%),
      radial-gradient(900px 500px at 110% 10%, rgba(191,149,63,.18), transparent 55%),
      radial-gradient(700px 500px at 50% 120%, rgba(245,222,179,.35), transparent 60%),
      #f7efe0;
    min-height: 100vh;
    color: #3b2f1c;
  }
  .glass {
    background: linear-gradient(180deg, rgba(255,252,244,0.85), rgba(250,240,220,0.70));
    border: 1px solid rgba(191,149,63,0.30);
    backdrop-filter: blur(10px);
    -webkit-backdrop-filter: blur(10px);
  }
  .soft-glow { box-shadow: 0 0 0 1px rgba(191,149,63,0.15), 0 20px 60px -20px rgba(191,149,63,0.35); }
  .btn {
    display: inline-flex; align-items: center; gap: .5rem;
    padding: .6rem 1rem; border-radius: .75rem; font-weight: 600;
    border: 1px solid rgba(191,149,63,0.30);
    transition: transform .15s ease, box-shadow .15s ease, background .15s ease;
  }
  .btn:hover { transform: translateY(-1px); }
  .btn:active { transform: translateY(0); }
  .btn-primary { background: linear-gradient(135deg,#d4af37,#a8812a); color:#fffdf5; }
  .btn-primary:hover { box-shadow: 0 10px 30px -10px rgba(168,129,42,.6); }
  .btn-secondary { background: rgba(255,250,235,0.9); color:#4a3a1f; }
  .btn-secondary:hover { background: rgba(245,230,200,0.95); }
  .btn-danger { background: linear-gradient(135deg,#b23a3a,#7a1f1f); color:#fffdf5; }
  .btn-danger:hover { box-shadow: 0 10px 30px -10px rgba(178,58,58,.6); }
  .chip {
    padding: .25rem .6rem; border-radius: 999px; font-size: .72rem;
    font-weight: 600; letter-spacing: .05em; text-transform: uppercase;
  }
  .pip { width: 10px; height: 22px; border-radius: 3px; }
  canvas { image-rendering: auto; touch-action: manipulation; }
  .log-line { animation: slideIn .25s ease; }
  @keyframes slideIn { from { opacity:0; transform: translateY(4px);} to { opacity:1; transform:none; } }
  .pulse-dot::before {
    content:''; display:inline-block; width:.55rem; height:.55rem; border-radius:999px;
    background: currentColor; margin-right:.4rem; vertical-align: middle;
    box-shadow: 0 0 0 0 currentColor;
    animation: pulse 1.6s infinite;
  }
  @keyframes pulse {
    0% { box-shadow: 0 0 0 0 rgba(139,101,30,.4); }
    70% { box-shadow: 0 0 0 10px rgba(139,101,30,0); }
    100% { box-shadow: 0 0 0 0 rgba(139,101,30,0); }
  }
  .seg {
    display: inline-flex; padding: 3px; background: rgba(255,250,235,0.8);
    border-radius: .7rem; border: 1px solid rgba(191,149,63,0.30);
  }
  .seg button {
    padding: .35rem .8rem; border-radius: .5rem; font-size:.85rem; font-weight:600;
    color:#6b5330;
  }
  .seg button.active { background: linear-gradient(135deg,#d4af37,#a8812a); color:#fffdf5; }
  .toast {
    position: fixed; right: 1.25rem; bottom: 1.25rem; z-index: 50;
    display: flex; flex-direction: column; gap: .5rem;
  }
  .toast > div {
    padding: .75rem 1rem; border-radius: .6rem; color: #fffdf5;
    border: 1px solid rgba(191,149,63,.25); animation: slideIn .25s ease;
  }
  .scroll-thin::-webkit-scrollbar { width: 6px; }
  .scroll-thin::-webkit-scrollbar-thumb { background: rgba(191,149,63,.3); border-radius: 999px; }
</style>
</head>
<body class="antialiased" style="color:#3b2f1c;">

<header class="max-w-7xl mx-auto px-6 pt-8 pb-4 flex items-center justify-between">
  <div class="flex items-center gap-3">
    <div class="w-10 h-10 rounded-xl grid place-items-center soft-glow" style="background:linear-gradient(135deg,#d4af37,#a8812a);">
      <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#fffdf5" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="3" width="7" height="7" rx="1.2"/><rect x="14" y="3" width="7" height="7" rx="1.2"/><rect x="3" y="14" width="7" height="7" rx="1.2"/><rect x="14" y="14" width="7" height="7" rx="1.2"/></svg>
    </div>
    <div>
      <div class="text-xl font-bold tracking-tight" style="color:#3b2f1c;">Quoridor <span style="color:#a8812a;">· UR3e</span></div>
      <div class="text-xs -mt-0.5" style="color:#8a7145;">Robot control console</div>
    </div>
  </div>
  <div id="connBadge" class="chip pulse-dot" style="background:rgba(178,58,58,.12); color:#7a1f1f; border:1px solid rgba(178,58,58,.35);">Disconnected</div>
</header>

<main class="max-w-7xl mx-auto px-6 pb-10 grid grid-cols-1 lg:grid-cols-[1fr_380px] gap-6">

  <!-- Board Panel -->
  <section class="glass rounded-2xl p-5 soft-glow">
    <div class="flex items-start justify-between mb-4">
      <div>
        <div class="text-sm" style="color:#8a7145;">Board</div>
        <div id="turnLabel" class="text-2xl font-bold tracking-tight" style="color:#3b2f1c;">—</div>
      </div>
      <div class="seg" role="tablist" aria-label="Interaction mode">
        <button id="modeMove" class="active" title="Click a cell to move your pawn">Move</button>
        <button id="modeWallH" title="Place horizontal wall">Wall ─</button>
        <button id="modeWallV" title="Place vertical wall">Wall │</button>
      </div>
    </div>

    <div class="relative">
      <canvas id="board" width="720" height="720" class="w-full rounded-xl" style="background:#0a0a0a; border:1px solid rgba(191,149,63,0.35);"></canvas>
      <div id="banner" class="hidden absolute inset-0 rounded-xl grid place-items-center" style="background:rgba(20,15,5,0.75); backdrop-filter:blur(4px);">
        <div class="text-center">
          <div id="bannerTitle" class="text-4xl font-extrabold tracking-tight"></div>
          <div id="bannerSub" class="mt-2" style="color:#f5e6c4;"></div>
        </div>
      </div>
    </div>

    <div class="mt-4 flex flex-wrap items-center gap-2 text-sm" style="color:#8a7145;">
      <span class="chip" style="background:rgba(139,94,60,.12); color:#6b4a2b; border:1px solid rgba(139,94,60,.35);">P · You</span>
      <span class="chip" style="background:rgba(91,108,77,.12); color:#3f5234; border:1px solid rgba(91,108,77,.35);">B · Bot</span>
      <span class="mx-2 opacity-40">|</span>
      <span>Click a cell to move • Switch mode to place walls • Hover for preview</span>
    </div>
  </section>

  <!-- Side Panel -->
  <aside class="flex flex-col gap-4">
    <!-- Status card -->
    <div class="glass rounded-2xl p-5">
      <div class="text-xs uppercase tracking-widest mb-3" style="color:#8a7145;">Match</div>
      <div class="flex items-center justify-between mb-3">
        <div>
          <div class="text-sm" style="color:#8a7145;">Status</div>
          <div id="statusLabel" class="text-lg font-semibold" style="color:#3b2f1c;">—</div>
        </div>
        <div id="statusChip" class="chip" style="background:rgba(139,101,30,.12); color:#6b5330; border:1px solid rgba(139,101,30,.30);">idle</div>
      </div>

      <div class="space-y-3">
        <div>
          <div class="flex justify-between text-xs mb-1" style="color:#8a7145;">
            <span>You · walls</span><span id="playerWallsLabel">0</span>
          </div>
          <div id="playerWalls" class="flex gap-1"></div>
        </div>
        <div>
          <div class="flex justify-between text-xs mb-1" style="color:#8a7145;">
            <span>Bot · walls</span><span id="botWallsLabel">0</span>
          </div>
          <div id="botWalls" class="flex gap-1"></div>
        </div>
      </div>

      <div class="mt-4 grid grid-cols-2 gap-2 text-sm">
        <div class="rounded-lg p-2.5" style="background:rgba(255,250,235,0.7); border:1px solid rgba(191,149,63,0.25);">
          <div class="text-[10px] uppercase tracking-wider" style="color:#8a7145;">Input</div>
          <div id="inputModeLabel" class="mono font-semibold" style="color:#3b2f1c;">—</div>
        </div>
        <div class="rounded-lg p-2.5" style="background:rgba(255,250,235,0.7); border:1px solid rgba(191,149,63,0.25);">
          <div class="text-[10px] uppercase tracking-wider" style="color:#8a7145;">Bot</div>
          <div id="botThinkLabel" class="mono font-semibold" style="color:#3b2f1c;">idle</div>
        </div>
      </div>
    </div>

    <!-- Controls card -->
    <div class="glass rounded-2xl p-5">
      <div class="text-xs uppercase tracking-widest mb-3" style="color:#8a7145;">Controls</div>
      <div class="grid grid-cols-2 gap-2">
        <button id="btnStart" class="btn btn-primary col-span-2 justify-center">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z"/></svg>
          Start · you first
        </button>
        <button id="btnStartBot" class="btn btn-secondary col-span-2 justify-center">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="7" width="18" height="13" rx="2"/><path d="M8 7V4h8v3"/><circle cx="9" cy="13" r="1"/><circle cx="15" cy="13" r="1"/></svg>
          Start · bot first
        </button>
        <button id="btnToggle" class="btn btn-secondary justify-center">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M17 3l4 4-4 4"/><path d="M21 7H9a4 4 0 0 0-4 4v2"/><path d="M7 21l-4-4 4-4"/><path d="M3 17h12a4 4 0 0 0 4-4v-2"/></svg>
          Toggle input
        </button>
        <button id="btnEstop" class="btn btn-danger justify-center">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor"><rect x="5" y="5" width="14" height="14" rx="2"/></svg>
          E-STOP
        </button>
      </div>
    </div>

    <!-- Activity log -->
    <div class="glass rounded-2xl p-5 flex-1 min-h-[220px] flex flex-col">
      <div class="flex items-center justify-between mb-2">
        <div class="text-xs uppercase tracking-widest" style="color:#8a7145;">Activity</div>
        <button id="btnClearLog" class="text-xs" style="color:#8a7145;">clear</button>
      </div>
      <div id="log" class="mono text-xs overflow-y-auto scroll-thin flex-1 pr-1 space-y-1" style="color:#4a3a1f;"></div>
    </div>
  </aside>
</main>

<div id="toast" class="toast"></div>

<script>
// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
const state = {
  board: null,          // latest board state from SSE
  mode: 'move',         // 'move' | 'wallH' | 'wallV'
  hover: null,          // hover target for preview
  connected: false,
};

const boardCanvas = document.getElementById('board');
const ctx = boardCanvas.getContext('2d');

// ---------------------------------------------------------------------------
// Networking
// ---------------------------------------------------------------------------
function connectSSE() {
  const es = new EventSource('/events');
  es.onopen = () => { setConnected(true); };
  es.onerror = () => {
    setConnected(false);
    es.close();
    setTimeout(connectSSE, 1500);
  };
  es.onmessage = (ev) => {
    try {
      const data = JSON.parse(ev.data);
      applyState(data);
    } catch (e) { console.warn('bad SSE', e); }
  };
}
function setConnected(ok) {
  state.connected = ok;
  const el = document.getElementById('connBadge');
  if (ok) {
    el.textContent = 'Live';
    el.className = 'chip pulse-dot';
    el.style.cssText = 'background:rgba(101,120,74,.15); color:#4a5d30; border:1px solid rgba(101,120,74,.40);';
  } else {
    el.textContent = 'Disconnected';
    el.className = 'chip pulse-dot';
    el.style.cssText = 'background:rgba(178,58,58,.12); color:#7a1f1f; border:1px solid rgba(178,58,58,.35);';
  }
}

async function postJSON(url, body) {
  try {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const t = await r.text();
    let j = null; try { j = JSON.parse(t); } catch {}
    if (!r.ok || (j && j.ok === false)) {
      toast(j && j.error ? j.error : 'Request failed', 'error');
      return null;
    }
    return j;
  } catch (e) {
    toast('Network error', 'error');
    return null;
  }
}

function sendCommand(payload) { return postJSON('/command', payload); }
function sendMove(move)      { return postJSON('/move', move); }

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
function applyState(s) {
  const prev = state.board;
  state.board = s;

  document.getElementById('statusLabel').textContent = prettyStatus(s.game_status);
  document.getElementById('turnLabel').textContent =
    s.game_status !== 'in_progress' ? prettyStatus(s.game_status) :
    (s.current_turn === 'player' ? 'Your turn' : 'Bot · thinking');

  const chip = document.getElementById('statusChip');
  chip.textContent = s.current_turn;
  chip.className = 'chip';
  chip.style.cssText = s.current_turn === 'player'
    ? 'background:rgba(139,94,60,.15); color:#6b4a2b; border:1px solid rgba(139,94,60,.40);'
    : 'background:rgba(91,108,77,.15); color:#3f5234; border:1px solid rgba(91,108,77,.40);';

  renderPips('playerWalls', s.player_walls_remaining, 4, 'player');
  renderPips('botWalls',    s.bot_walls_remaining,    4, 'bot');
  document.getElementById('playerWallsLabel').textContent = s.player_walls_remaining;
  document.getElementById('botWallsLabel').textContent    = s.bot_walls_remaining;
  document.getElementById('inputModeLabel').textContent   = s.input_mode || '—';
  document.getElementById('botThinkLabel').textContent    = s.bot_thinking ? 'thinking…' : 'idle';

  // Banner on terminal state
  const banner = document.getElementById('banner');
  if (s.game_status === 'player_wins' || s.game_status === 'bot_wins') {
    banner.classList.remove('hidden');
    document.getElementById('bannerTitle').textContent =
      s.game_status === 'player_wins' ? 'You win' : 'Bot wins';
    const bt = document.getElementById('bannerTitle');
    bt.className = 'text-5xl font-extrabold tracking-tight';
    bt.style.color = s.game_status === 'player_wins' ? '#d4af37' : '#e8c9a0';
    document.getElementById('bannerSub').textContent = 'Press "Start" to play again';
  } else {
    banner.classList.add('hidden');
  }

  // Log deltas
  if (prev) logDiff(prev, s);

  drawBoard();
}

function renderPips(id, n, max, who) {
  const el = document.getElementById(id);
  el.innerHTML = '';
  const on = who === 'player' ? '#8b5e3c' : '#5b6c4d';
  const off = 'rgba(139,101,30,0.18)';
  for (let i = 0; i < max; i++) {
    const d = document.createElement('div');
    d.className = 'pip';
    d.style.background = i < n ? on : off;
    el.appendChild(d);
  }
}

function prettyStatus(s) {
  if (!s) return '—';
  return s.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase());
}

// ---------- Canvas board ----------
const LAYOUT = {
  pad: 36,
  get size() { return boardCanvas.width; },
  get inner() { return this.size - 2 * this.pad; },
  cell() { return this.inner / 5; },        // n=5
  cellXY(x, y) {
    const c = this.cell();
    // y=0 is visually at the BOTTOM to match the engine's convention
    const cx = this.pad + x * c + c / 2;
    const cy = this.pad + (4 - y) * c + c / 2;
    return { cx, cy, c };
  },
};

function drawBoard() {
  const b = state.board;
  const W = boardCanvas.width, H = boardCanvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.save();

  const n = 5;
  const c = LAYOUT.cell();

  // Grid cells (white) — winning rows get a light border instead of fill color
  for (let y = 0; y < n; y++) {
    for (let x = 0; x < n; x++) {
      const { cx, cy } = LAYOUT.cellXY(x, y);
      const isGoal = (y === 0 || y === n - 1);
      roundRect(ctx, cx - c/2 + 4, cy - c/2 + 4, c - 8, c - 8, 10);
      ctx.fillStyle = '#f8f5ed';
      ctx.fill();
      if (isGoal) {
        ctx.strokeStyle = 'rgba(245,235,210,0.95)';
        ctx.lineWidth = 3;
      } else {
        ctx.strokeStyle = 'rgba(212,175,55,0.25)';
        ctx.lineWidth = 1;
      }
      ctx.stroke();
    }
  }

  if (!b) { ctx.restore(); return; }

  // Legal-move hints (only when it's the player's turn and mode is move)
  if (state.mode === 'move' && b.current_turn === 'player' && b.game_status === 'in_progress') {
    const targets = legalPawnTargetsApprox(b);
    for (const t of targets) {
      const { cx, cy } = LAYOUT.cellXY(t.x, t.y);
      ctx.beginPath();
      ctx.arc(cx, cy, c * 0.12, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(168,129,42,0.55)';
      ctx.fill();
    }
  }

  // Walls (red)
  for (const w of b.walls) {
    drawWall(w.pos[0], w.pos[1], w.orientation, '#c0392b');
  }

  // Hover preview (wall or move)
  if (state.hover) {
    if (state.hover.kind === 'wall') {
      drawWall(state.hover.x, state.hover.y, state.hover.orient, 'rgba(192,57,43,0.40)');
    } else if (state.hover.kind === 'move') {
      const { cx, cy } = LAYOUT.cellXY(state.hover.x, state.hover.y);
      ctx.beginPath();
      ctx.arc(cx, cy, c * 0.36, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(168,129,42,0.75)';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }

  // Pawns — flat, no glass/gradient
  drawPawn(b.player_pos.x, b.player_pos.y, '#8b5e3c', 'P');
  drawPawn(b.bot_pos.x,    b.bot_pos.y,    '#5b6c4d', 'B');

  // Coordinate labels
  ctx.fillStyle = 'rgba(245,222,179,0.75)';
  ctx.font = '600 12px JetBrains Mono, monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (let x = 0; x < n; x++) {
    const { cx } = LAYOUT.cellXY(x, 0);
    ctx.fillText(String(x), cx, LAYOUT.pad + LAYOUT.inner + 18);
    ctx.fillText(String(x), cx, LAYOUT.pad - 18);
  }
  for (let y = 0; y < n; y++) {
    const { cy } = LAYOUT.cellXY(0, y);
    ctx.fillText(String(y), LAYOUT.pad - 18, cy);
    ctx.fillText(String(y), LAYOUT.pad + LAYOUT.inner + 18, cy);
  }

  ctx.restore();
}

function drawPawn(x, y, color, letter) {
  const { cx, cy, c } = LAYOUT.cellXY(x, y);
  const r = c * 0.32;
  // flat body
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI*2);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.strokeStyle = 'rgba(245,222,179,0.85)';
  ctx.stroke();
  // letter
  ctx.fillStyle = '#fdf6e3';
  ctx.font = 'bold ' + Math.round(r*0.9) + 'px Inter, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(letter, cx, cy + 1);
}

function drawWall(wx, wy, orient, color) {
  const c = LAYOUT.cell();
  const thick = Math.max(8, c * 0.12);
  ctx.fillStyle = color;
  ctx.shadowColor = 'rgba(192,57,43,0.55)';
  ctx.shadowBlur = 10;
  if (orient === 'HOR') {
    // Blocks y<->y+1 along cols wx and wx+1 -> sits between rows wy and wy+1
    const left   = LAYOUT.cellXY(wx,     wy).cx - c/2 + 4;
    const right  = LAYOUT.cellXY(wx + 1, wy).cx + c/2 - 4;
    // Between wy and wy+1; remember y increases upward on-screen
    const yMid = (LAYOUT.cellXY(wx, wy).cy + LAYOUT.cellXY(wx, wy + 1).cy) / 2;
    roundRect(ctx, left, yMid - thick/2, right - left, thick, thick/2);
    ctx.fill();
  } else {
    const top    = LAYOUT.cellXY(wx, wy + 1).cy - c/2 + 4;
    const bot    = LAYOUT.cellXY(wx, wy    ).cy + c/2 - 4;
    const xMid = (LAYOUT.cellXY(wx, wy).cx + LAYOUT.cellXY(wx + 1, wy).cx) / 2;
    roundRect(ctx, xMid - thick/2, top, thick, bot - top, thick/2);
    ctx.fill();
  }
  ctx.shadowBlur = 0;
}

function roundRect(ctx, x, y, w, h, r) {
  r = Math.min(r, w/2, h/2);
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y,     x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x,     y + h, r);
  ctx.arcTo(x,     y + h, x,     y,     r);
  ctx.arcTo(x,     y,     x + w, y,     r);
  ctx.closePath();
}

function shade(hex, pct) {
  const num = parseInt(hex.replace('#',''), 16);
  let r = (num >> 16) + pct, g = (num >> 8 & 255) + pct, b = (num & 255) + pct;
  r = Math.max(0, Math.min(255, r));
  g = Math.max(0, Math.min(255, g));
  b = Math.max(0, Math.min(255, b));
  return '#' + ((1<<24) + (r<<16) + (g<<8) + b).toString(16).slice(1);
}

// ---------- Client-side legal-move hint (approximate; server validates) ----------
function legalPawnTargetsApprox(b) {
  const n = 5;
  const cur = b.player_pos, opp = b.bot_pos;
  const walls = b.walls;
  const blocked = (fx, fy, tx, ty) => {
    for (const w of walls) {
      const [wx, wy] = w.pos;
      if (w.orientation === 'HOR') {
        if (fx === tx && fy !== ty) {
          const minY = Math.min(fy, ty);
          if (wy === minY && (fx === wx || fx === wx + 1)) return true;
        }
      } else {
        if (fy === ty && fx !== tx) {
          const minX = Math.min(fx, tx);
          if (wx === minX && (fy === wy || fy === wy + 1)) return true;
        }
      }
    }
    return false;
  };
  const inBounds = (x, y) => x >= 0 && x < n && y >= 0 && y < n;
  const out = [];
  const dirs = [[1,0],[-1,0],[0,1],[0,-1]];
  for (const [dx, dy] of dirs) {
    const t = { x: cur.x + dx, y: cur.y + dy };
    if (!inBounds(t.x, t.y)) continue;
    if (blocked(cur.x, cur.y, t.x, t.y)) continue;
    if (t.x === opp.x && t.y === opp.y) {
      // jump straight
      const j = { x: t.x + dx, y: t.y + dy };
      if (inBounds(j.x, j.y) && !blocked(t.x, t.y, j.x, j.y)) out.push(j);
      else {
        // diagonals
        for (const [ex, ey] of dirs) {
          if (ex === -dx && ey === -dy) continue;
          if (ex === dx && ey === dy) continue;
          const d = { x: t.x + ex, y: t.y + ey };
          if (!inBounds(d.x, d.y)) continue;
          if (!blocked(t.x, t.y, d.x, d.y)) out.push(d);
        }
      }
    } else {
      out.push(t);
    }
  }
  return out;
}

// ---------- Interaction ----------
boardCanvas.addEventListener('mousemove', (ev) => {
  const p = eventPos(ev);
  state.hover = pickTarget(p);
  drawBoard();
});
boardCanvas.addEventListener('mouseleave', () => { state.hover = null; drawBoard(); });
boardCanvas.addEventListener('click', (ev) => {
  const p = eventPos(ev);
  const t = pickTarget(p);
  if (!t) return;
  if (t.kind === 'move') {
    sendMove({ move_type: 'PAWN', target: { x: t.x, y: t.y } });
    log(`> move ${t.x} ${t.y}`);
  } else if (t.kind === 'wall') {
    sendMove({ move_type: 'WALL', wall: { pos: [t.x, t.y], orientation: t.orient } });
    log(`> wall ${t.x} ${t.y} ${t.orient === 'HOR' ? 'h' : 'v'}`);
  }
});

function eventPos(ev) {
  const r = boardCanvas.getBoundingClientRect();
  const sx = boardCanvas.width / r.width;
  const sy = boardCanvas.height / r.height;
  return { x: (ev.clientX - r.left) * sx, y: (ev.clientY - r.top) * sy };
}

function pickTarget(p) {
  if (state.mode === 'move') {
    // Snap to nearest cell center within cell bounds
    const { pad } = LAYOUT, c = LAYOUT.cell();
    const gx = Math.floor((p.x - pad) / c);
    const gyVis = Math.floor((p.y - pad) / c);
    if (gx < 0 || gx >= 5 || gyVis < 0 || gyVis >= 5) return null;
    const y = 4 - gyVis;
    return { kind: 'move', x: gx, y };
  }
  // Wall mode — pick nearest wall intersection (wx, wy in 0..3)
  const orient = state.mode === 'wallH' ? 'HOR' : 'VER';
  let best = null, bestD = Infinity;
  for (let wx = 0; wx < 4; wx++) {
    for (let wy = 0; wy < 4; wy++) {
      const a = LAYOUT.cellXY(wx,     wy);
      const b = LAYOUT.cellXY(wx + 1, wy + 1);
      // Wall anchor point = midpoint between (wx,wy)+(wx+1,wy+1) corners
      const cx = (a.cx + b.cx) / 2, cy = (a.cy + b.cy) / 2;
      const d = (p.x - cx) ** 2 + (p.y - cy) ** 2;
      if (d < bestD) { bestD = d; best = { wx, wy }; }
    }
  }
  if (!best) return null;
  return { kind: 'wall', x: best.wx, y: best.wy, orient };
}

// Mode segmented buttons
const modeBtns = {
  move:  document.getElementById('modeMove'),
  wallH: document.getElementById('modeWallH'),
  wallV: document.getElementById('modeWallV'),
};
for (const [k, el] of Object.entries(modeBtns)) {
  el.addEventListener('click', () => setMode(k));
}
function setMode(m) {
  state.mode = m;
  for (const [k, el] of Object.entries(modeBtns)) el.classList.toggle('active', k === m);
  drawBoard();
}

// Keyboard shortcuts
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
  if (e.key === 'm' || e.key === 'M') setMode('move');
  else if (e.key === 'h' || e.key === 'H') setMode('wallH');
  else if (e.key === 'v' || e.key === 'V') setMode('wallV');
  else if (e.key === 'Escape') sendCommand({ command: 'estop' });
});

// Buttons
document.getElementById('btnStart').onclick    = () => { sendCommand({ command: 'start', bot_first: false }); log('> start'); };
document.getElementById('btnStartBot').onclick = () => { sendCommand({ command: 'start', bot_first: true  }); log('> start bot'); };
document.getElementById('btnToggle').onclick   = () => { sendCommand({ command: 'toggle_input' }); log('> toggle input'); };
document.getElementById('btnEstop').onclick    = () => { sendCommand({ command: 'estop' }); toast('Emergency stop sent', 'warn'); log('> estop'); };
document.getElementById('btnClearLog').onclick = () => { document.getElementById('log').innerHTML = ''; };

// ---------- Log + toast ----------
function log(line, cls = '', color = '') {
  const box = document.getElementById('log');
  const d = document.createElement('div');
  d.className = 'log-line ' + cls;
  if (color) d.style.color = color;
  const ts = new Date().toLocaleTimeString([], { hour12: false });
  d.innerHTML = `<span style="color:#a89368;">${ts}</span>  ${escapeHtml(line)}`;
  box.appendChild(d);
  box.scrollTop = box.scrollHeight;
  while (box.children.length > 200) box.removeChild(box.firstChild);
}
function escapeHtml(s) { return s.replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c])); }

function toast(msg, kind = 'info') {
  const box = document.getElementById('toast');
  const d = document.createElement('div');
  const bg = kind === 'error' ? '#b23a3a'
           : kind === 'warn'  ? '#c9a227'
           : '#a8812a';
  d.style.background = bg;
  d.textContent = msg;
  box.appendChild(d);
  setTimeout(() => d.remove(), 3200);
}

function logDiff(prev, cur) {
  if (prev.player_pos.x !== cur.player_pos.x || prev.player_pos.y !== cur.player_pos.y) {
    log(`player → (${cur.player_pos.x}, ${cur.player_pos.y})`, '', '#6b4a2b');
  }
  if (prev.bot_pos.x !== cur.bot_pos.x || prev.bot_pos.y !== cur.bot_pos.y) {
    log(`bot    → (${cur.bot_pos.x}, ${cur.bot_pos.y})`, '', '#3f5234');
  }
  if (prev.walls.length !== cur.walls.length) {
    const added = cur.walls[cur.walls.length - 1];
    if (added) log(`wall placed (${added.pos[0]}, ${added.pos[1]}) ${added.orientation}`, '', '#b23a3a');
  }
  if (prev.game_status !== cur.game_status && cur.game_status !== 'in_progress') {
    log(`*** ${cur.game_status.toUpperCase().replace('_',' ')} ***`, 'font-semibold', '#a8812a');
  }
  if (prev.input_mode !== cur.input_mode) log(`input mode: ${cur.input_mode}`);
}

// Boot
drawBoard();
connectSSE();
</script>
</body>
</html>
"""


class _SSEClient:
    """A single SSE subscriber. Thread-safe queue of JSON payloads."""

    def __init__(self) -> None:
        self.q: "queue.Queue[str]" = queue.Queue(maxsize=64)
        self.alive = True

    def push(self, payload: str) -> None:
        try:
            self.q.put_nowait(payload)
        except queue.Full:
            # Slow client — drop it.
            self.alive = False


class WebInterface(Node):
    """ROS2 node that serves a single-page web GUI for the Quoridor game."""

    def __init__(self) -> None:
        super().__init__('web_interface')

        self.declare_parameter('host', DEFAULT_HOST)
        self.declare_parameter('port', DEFAULT_PORT)
        host = self.get_parameter('host').get_parameter_value().string_value or DEFAULT_HOST
        port = int(self.get_parameter('port').get_parameter_value().integer_value or DEFAULT_PORT)

        self._last_state_json: str | None = None
        self._sse_clients: list[_SSEClient] = []
        self._sse_clients_lock = threading.Lock()

        # --- publishers ---
        self.pub_player_move = self.create_publisher(String, '/quoridor/player_move', 10)
        self.pub_game_command = self.create_publisher(String, '/quoridor/game_command', 10)

        # --- subscribers ---
        self.sub_board_state = self.create_subscription(
            String, '/quoridor/board_state', self._on_board_state, 10)

        # --- HTTP server ---
        handler = _make_handler(self)
        self._http = ThreadingHTTPServer((host, port), handler)
        self._http_thread = threading.Thread(target=self._http.serve_forever, daemon=True)
        self._http_thread.start()

        self.get_logger().info(f'Web UI listening on http://{host}:{port}')

    # ------------------------------------------------------------------ #
    #  Board state fan-out                                                #
    # ------------------------------------------------------------------ #

    def _on_board_state(self, msg: String) -> None:
        self._last_state_json = msg.data
        self._broadcast(msg.data)

    def _broadcast(self, payload: str) -> None:
        with self._sse_clients_lock:
            dead = []
            for c in self._sse_clients:
                if not c.alive:
                    dead.append(c); continue
                try:
                    c.q.put_nowait(payload)
                except queue.Full:
                    c.alive = False
                    dead.append(c)
            for c in dead:
                if c in self._sse_clients:
                    self._sse_clients.remove(c)

    def _register_client(self, c: _SSEClient) -> None:
        with self._sse_clients_lock:
            self._sse_clients.append(c)
        if self._last_state_json is not None:
            try:
                c.q.put_nowait(self._last_state_json)
            except queue.Full:
                pass

    def _unregister_client(self, c: _SSEClient) -> None:
        with self._sse_clients_lock:
            if c in self._sse_clients:
                self._sse_clients.remove(c)

    # ------------------------------------------------------------------ #
    #  Command / move intake                                              #
    # ------------------------------------------------------------------ #

    def handle_command(self, payload: dict) -> tuple[bool, str]:
        cmd = payload.get('command')
        if cmd not in ('start', 'toggle_input', 'estop'):
            return False, f'unknown command: {cmd!r}'
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_game_command.publish(msg)
        return True, 'ok'

    def handle_move(self, payload: dict) -> tuple[bool, str]:
        try:
            move_type = payload['move_type']
            if move_type == 'PAWN':
                target = payload.get('target') or {}
                move = Move(
                    move_type=MoveType.PAWN,
                    target=Pawn(int(target['x']), int(target['y'])),
                )
            elif move_type == 'WALL':
                w = payload.get('wall') or {}
                pos = w.get('pos') or []
                move = Move(
                    move_type=MoveType.WALL,
                    wall=Wall(
                        pos=(int(pos[0]), int(pos[1])),
                        orientation=Orientation[w['orientation']],
                    ),
                )
            else:
                return False, f'unknown move_type: {move_type!r}'
        except (KeyError, ValueError, TypeError) as e:
            return False, f'malformed move: {e}'

        msg = String()
        msg.data = json.dumps(move.to_dict())
        self.pub_player_move.publish(msg)
        return True, 'ok'

    # ------------------------------------------------------------------ #
    #  Shutdown                                                           #
    # ------------------------------------------------------------------ #

    def shutdown(self) -> None:
        try:
            self._http.shutdown()
            self._http.server_close()
        except Exception:
            pass


def _make_handler(node: WebInterface):
    """Build a BaseHTTPRequestHandler subclass bound to *node*."""

    class Handler(BaseHTTPRequestHandler):
        # Silence default stderr access log — route through ROS logger instead.
        def log_message(self, fmt: str, *args) -> None:  # noqa: N802
            node.get_logger().debug(fmt % args)

        # -------------------- GET --------------------
        def do_GET(self):  # noqa: N802
            path = urlparse(self.path).path
            if path == '/' or path == '/index.html':
                body = INDEX_HTML.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
                return

            if path == '/events':
                self._serve_sse()
                return

            if path == '/healthz':
                self._json(200, {'ok': True})
                return

            self.send_error(404, 'not found')

        # -------------------- POST --------------------
        def do_POST(self):  # noqa: N802
            path = urlparse(self.path).path
            length = int(self.headers.get('Content-Length') or 0)
            raw = self.rfile.read(length) if length > 0 else b''
            try:
                payload = json.loads(raw.decode('utf-8') or '{}')
            except json.JSONDecodeError:
                self._json(400, {'ok': False, 'error': 'invalid JSON'})
                return

            if path == '/command':
                ok, msg = node.handle_command(payload)
                self._json(200 if ok else 400, {'ok': ok, 'error': None if ok else msg})
                return

            if path == '/move':
                ok, msg = node.handle_move(payload)
                self._json(200 if ok else 400, {'ok': ok, 'error': None if ok else msg})
                return

            self.send_error(404, 'not found')

        # -------------------- helpers --------------------
        def _json(self, status: int, body: dict) -> None:
            data = json.dumps(body).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _serve_sse(self) -> None:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()

            client = _SSEClient()
            node._register_client(client)
            try:
                # Initial retry hint for the browser's EventSource.
                self.wfile.write(b'retry: 1500\n\n')
                self.wfile.flush()
                while client.alive:
                    try:
                        payload = client.q.get(timeout=15.0)
                        chunk = f'data: {payload}\n\n'.encode('utf-8')
                        self.wfile.write(chunk)
                        self.wfile.flush()
                    except queue.Empty:
                        # Heartbeat to keep proxies from closing the stream.
                        try:
                            self.wfile.write(b': ping\n\n')
                            self.wfile.flush()
                        except (BrokenPipeError, ConnectionResetError):
                            break
                    except (BrokenPipeError, ConnectionResetError):
                        break
            finally:
                client.alive = False
                node._unregister_client(client)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = WebInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
