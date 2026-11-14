// Browser entry. Loads MuJoCo and Pyodide only when this page opens, then runs the sim.
/* eslint-disable @typescript-eslint/no-explicit-any */
import { Session, TICK_US, type Mode, type TickView } from './session';
import { View } from './render';
import { LEADER, World, type Files } from './world';

const PYODIDE_URL = 'https://cdn.jsdelivr.net/pyodide/v0.29.0/full/';
const MIN_FORCE_N = 1;
const MAX_FORCE_N = 15;
const MAX_TICKS_PER_FRAME = 8;
const TICK_MS = TICK_US / 1000;

const $ = <T extends HTMLElement>(sel: string) => document.querySelector<T>(sel)!;
const body = document.body;
const statusBox = $('#status');
const setState = (state: string) => (body.dataset.state = state);
const detail = (text: string) => ($('#status-detail').textContent = text);

async function bytes(url: string): Promise<Uint8Array> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url}: ${res.status}`);
  return new Uint8Array(await res.arrayBuffer());
}
async function text(url: string): Promise<string> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url}: ${res.status}`);
  return res.text();
}

// The host page may send pause or resume before the simulation has loaded. Listen from the
// start and remember the latest request. start() applies it.
let wantPaused = false;
let applyPause: ((paused: boolean) => void) | null = null;
window.addEventListener('message', (e) => {
  const kind = typeof e.data === 'string' ? e.data : e.data?.type;
  if (kind !== 'pause' && kind !== 'resume') return;
  wantPaused = kind === 'pause';
  applyPause?.(wantPaused);
});

async function boot(): Promise<void> {
  detail('Loading the physics engine and Python…');
  const [{ default: loadMujoco }, { loadPyodide }] = await Promise.all([
    import('@mujoco/mujoco'),
    import('pyodide'),
  ]);
  const [mj, pyodide, manifest, wheel, glue, sceneXml, robotXml] = await Promise.all([
    loadMujoco(),
    loadPyodide({ indexURL: PYODIDE_URL }),
    fetch('scene/manifest.json').then((r) => r.json() as Promise<{ meshes: string[] }>),
    bytes('py/mortar_pestle.whl'),
    text('py/browser.py'),
    text('scene/scene.xml'),
    text('scene/so101.xml'),
  ]);

  detail('Loading the robot model…');
  const files: Files = { 'scene.xml': sceneXml, 'so101.xml': robotXml };
  await Promise.all(
    manifest.meshes.map(async (name) => {
      files[`assets/${name}`] = await bytes(`scene/assets/${name}`);
    }),
  );

  detail('Starting the simulation…');
  const world = World.create(mj, files);
  const session = Session.create({ pyodide, world, wheel, glue, mode: 'mirror' });
  const view = new View($<HTMLCanvasElement>('#view'), world);
  start(session, view);
}

function start(session: Session, view: View): void {
  const hand = session.hand;
  let force = 6;
  let keyDown = false;
  let pointerDown = false;
  let running = true;
  let last = view.renderer.domElement ? performance.now() : 0;
  let acc = 0;
  let latest: TickView | null = null;
  let hudAt = 0;
  let speed = 1;
  let simMs = 0;
  let wallMs = 0;
  const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;

  const canvas = view.renderer.domElement;
  const fit = () => view.resize(canvas.clientWidth, canvas.clientHeight);
  new ResizeObserver(fit).observe(canvas);
  fit();

  const setHand = (auto: boolean) => {
    hand.auto = auto;
    document.querySelectorAll<HTMLButtonElement>('[data-hand]').forEach((b) => b.setAttribute('aria-pressed', String((b.dataset.hand === 'auto') === auto)));
  };
  const setMode = (mode: Mode) => {
    session.setMode(mode);
    document.querySelectorAll<HTMLButtonElement>('[data-mode]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.mode === mode)));
  };
  const setForce = (n: number) => {
    force = Math.min(MAX_FORCE_N, Math.max(MIN_FORCE_N, n));
    hand.pointer.force_n = force;
  };
  hand.pointer.force_n = force;

  document.querySelectorAll<HTMLButtonElement>('[data-mode]').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode as Mode)));
  document.querySelectorAll<HTMLButtonElement>('[data-hand]').forEach((b) => b.addEventListener('click', () => setHand(b.dataset.hand === 'auto')));
  $('#force-up').addEventListener('click', () => setForce(force + 1));
  $('#force-down').addEventListener('click', () => setForce(force - 1));

  const aim = (e: PointerEvent) => {
    const r = canvas.getBoundingClientRect();
    const mm = view.pointerToLeaderMm(((e.clientX - r.left) / r.width) * 2 - 1, -(((e.clientY - r.top) / r.height) * 2 - 1));
    if (!mm) return false;
    const near = Math.hypot(mm[0], mm[1]) < 90;
    if (near) {
      hand.pointer.x_mm = mm[0];
      hand.pointer.y_mm = mm[1];
    }
    return near;
  };
  canvas.addEventListener('pointermove', (e) => {
    if (aim(e) && hand.auto) setHand(false);
  });
  canvas.addEventListener('pointerdown', (e) => {
    if (!aim(e)) return;
    canvas.setPointerCapture(e.pointerId);
    setHand(false);
    pointerDown = true;
    hand.pointer.down = true;
  });
  const release = () => {
    pointerDown = false;
    hand.pointer.down = keyDown;
  };
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
  window.addEventListener('keydown', (e) => {
    if (e.key === ' ' && !(e.target instanceof HTMLButtonElement)) {
      e.preventDefault();
      keyDown = !keyDown;
      hand.pointer.down = keyDown || pointerDown;
      if (hand.auto) setHand(false);
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      setForce(force + 1);
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      setForce(force - 1);
    }
  });

  const pause = (p: boolean) => {
    running = !p;
    last = performance.now();
    setState(p ? 'paused' : 'running');
  };
  applyPause = (p) => pause(p || document.hidden);
  document.addEventListener('visibilitychange', () => pause(document.hidden || wantPaused));
  const fullscreenButton = $<HTMLButtonElement>('#fullscreen');
  if (document.fullscreenEnabled) {
    fullscreenButton.hidden = false;
    fullscreenButton.addEventListener('click', () => {
      if (document.fullscreenElement) void document.exitFullscreen();
      else void document.documentElement.requestFullscreen();
    });
  }

  const gauge = (sel: string, n: number) => (($(`${sel} i`) as HTMLElement).style.height = `${Math.min(100, (n / MAX_FORCE_N) * 100)}%`);
  const hud = () => {
    if (!latest) return;
    const i = latest.intent;
    const handName = hand.auto ? 'demo' : 'you';
    $('#readout').innerHTML =
      `hand <b>${handName}</b> · push <b>${force.toFixed(0)} N</b> · speed <b>${speed.toFixed(1)}x</b><br>` +
      `stroke <b>${i.pattern}</b>` +
      (i.pattern === 'circular' ? ` ${i.rate_hz.toFixed(1)} Hz r${i.radius_mm.toFixed(0)} mm ${i.direction}` : '') +
      `<br>Leader <b class="key">${latest.leader_force_n.toFixed(1)} N</b> · Follower <b class="fkey">${latest.state.contact_force_n.toFixed(1)} N</b>`;
    gauge('#g-leader', latest.leader_force_n);
    gauge('#g-follower', latest.state.contact_force_n);
  };

  const frame = (now: number) => {
    requestAnimationFrame(frame);
    if (!running) return;
    const dt = Math.min(now - last, 100);
    last = now;
    acc += dt;
    let n = 0;
    const t0 = performance.now();
    while (acc >= TICK_MS && n < MAX_TICKS_PER_FRAME) {
      latest = session.tick();
      acc -= TICK_MS;
      n++;
    }
    if (n === MAX_TICKS_PER_FRAME) acc = 0; // too slow: drop the backlog instead of spiralling
    if (n > 0) {
      simMs += n * TICK_MS;
      wallMs += dt;
    }
    void t0;
    if (now - hudAt > 120) {
      hudAt = now;
      if (wallMs > 0) speed = simMs / wallMs;
      simMs = 0;
      wallMs = 0;
      hud();
    }
    view.draw();
  };
  requestAnimationFrame(frame);

  statusBox.hidden = true;
  setState('running');
  if (wantPaused) pause(true);
  (window as any).__sim = { session, view, get latest() { return latest; }, LEADER };
  // Reduced motion: start paused with the poster-like first frame; the user can press a key.
  if (reduced) $('#hint').textContent += ' Animation is on because you opened the demo.';
}

boot().catch((err: unknown) => {
  console.error(err);
  statusBox.classList.add('error');
  $('#status-title').textContent = 'The simulation could not start';
  $('#status-detail').textContent = String((err as Error)?.message ?? err);
  setState('error');
});
