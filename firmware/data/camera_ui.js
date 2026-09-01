(function () {
  'use strict';

  let api = null;
  let state = { available: false, streamUrl: '', activeProfile: 'qvga', profilePending: false };
  let active = false;
  let reconnectTimer = 0;
  let reconnectDelayMs = 1000;
  let capsTimer = 0;

  function installStyles() {
    const style = document.createElement('style');
    style.textContent = `
      .camera-entry{background:#111827;color:#fff;border:0;border-radius:8px;padding:8px 15px;font-size:14px;cursor:pointer;box-shadow:0 4px #6b7280}
      .camera-entry[hidden]{display:none!important}
      body.camera-mode{margin:0;overflow:hidden;background:#050505}
      body.camera-mode>:not(#cameraVideoMode){display:none!important}
      #cameraVideoMode{display:none;position:fixed;inset:0;width:100vw;height:100dvh;background:#050505;color:#fff;z-index:10000;overflow:hidden}
      body.camera-mode #cameraVideoMode{display:flex!important;flex-direction:column}
      .camera-view{position:relative;flex:0 0 45%;min-height:0;background:#000;display:flex;align-items:center;justify-content:center}
      .camera-view img{width:100%;height:100%;object-fit:contain;background:#000}
      .camera-status{position:absolute;left:10px;top:10px;max-width:calc(100% - 20px);padding:6px 10px;border-radius:999px;background:rgba(0,0,0,.72);font:13px system-ui,sans-serif}
      .camera-controls{flex:1;min-height:0;display:flex;flex-direction:column;justify-content:space-evenly;align-items:center;padding:8px 14px 12px;box-sizing:border-box;background:#111827}
      .camera-toolbar{width:min(620px,100%);display:flex;gap:8px;justify-content:space-between;align-items:center}
      .camera-toolbar button,.camera-pad button{min-width:56px;min-height:48px;border:0;border-radius:12px;color:#fff;background:#2563eb;font-size:16px;font-weight:700;touch-action:manipulation}
      .camera-toolbar .camera-exit{background:#4b5563;padding:0 14px}
      .camera-toolbar .camera-profile.active{background:#059669}
      .camera-toolbar button:disabled{opacity:.48}
      .camera-pad{display:grid;grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,52px);gap:8px;align-items:center;justify-content:center}
      .camera-pad button{width:64px;height:52px;font-size:21px}
      .camera-pad .forward{grid-column:2;grid-row:1}.camera-pad .left{grid-column:1;grid-row:2}.camera-pad .stop{grid-column:2;grid-row:2;background:#dc2626}.camera-pad .right{grid-column:3;grid-row:2}.camera-pad .back{grid-column:2;grid-row:3}
      .camera-speed{display:flex;gap:8px}.camera-speed button{min-width:64px;min-height:44px;border:0;border-radius:12px;color:#fff;background:#374151;font-size:16px;font-weight:700;touch-action:manipulation}.camera-speed button.active{background:#7c3aed}
      @media (orientation:landscape) and (max-height:500px){body.camera-mode #cameraVideoMode{flex-direction:row}.camera-view{flex:0 0 58%;height:100%}.camera-controls{height:100%;padding:6px}.camera-pad{grid-template-columns:repeat(3,56px);grid-template-rows:repeat(3,46px);gap:5px}.camera-pad button{width:56px;height:46px;min-height:44px}.camera-toolbar button{min-height:44px}}
    `;
    document.head.appendChild(style);
  }

  function installDom() {
    const entry = document.createElement('button');
    entry.id = 'cameraEntry';
    entry.className = 'camera-entry';
    entry.hidden = true;
    entry.textContent = '📷 图传';
    entry.addEventListener('click', enter);
    const header = document.querySelector('.header-actions-right');
    if (header) header.insertBefore(entry, header.firstChild);

    const root = document.createElement('section');
    root.id = 'cameraVideoMode';
    root.setAttribute('aria-label', 'Camera control');
    root.innerHTML = `
      <div class="camera-view"><img id="cameraStream" alt="Camera stream"><div id="cameraStatus" class="camera-status">等待摄像头</div></div>
      <div class="camera-controls">
        <div class="camera-toolbar">
          <button class="camera-exit" id="cameraExit">高级控制</button>
          <div><button class="camera-profile" data-profile="qvga">低 QVGA</button> <button class="camera-profile" data-profile="vga">高 VGA</button></div>
        </div>
        <div class="camera-pad">
          <button class="forward" data-motion="1" aria-label="前进">▲</button>
          <button class="left" data-motion="4" aria-label="左转">◀</button>
          <button class="stop" id="cameraStop" aria-label="停止">■</button>
          <button class="right" data-motion="5" aria-label="右转">▶</button>
          <button class="back" data-motion="3" aria-label="后退">▼</button>
        </div>
        <div class="camera-speed"><button data-speed="0.33">慢</button><button class="active" data-speed="0.5">中</button><button data-speed="1">快</button></div>
      </div>`;
    document.body.appendChild(root);
    document.getElementById('cameraExit').addEventListener('click', exit);
    document.getElementById('cameraStop').addEventListener('click', () => api.stop());
    root.querySelectorAll('[data-motion]').forEach((button) => {
      button.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        if (!api.guardLowBattery()) api.sendMovement(Number(button.dataset.motion));
      });
      ['pointerup', 'pointercancel', 'pointerleave'].forEach((name) =>
        button.addEventListener(name, () => api.stop()));
    });
    root.querySelectorAll('[data-speed]').forEach((button) => button.addEventListener('click', () => {
      if (api.guardLowBattery()) return;
      api.sendSpeed(Number(button.dataset.speed));
      root.querySelectorAll('[data-speed]').forEach((item) => item.classList.toggle('active', item === button));
    }));
    root.querySelectorAll('[data-profile]').forEach((button) =>
      button.addEventListener('click', () => setProfile(button.dataset.profile)));
    const stream = document.getElementById('cameraStream');
    stream.addEventListener('load', () => { reconnectDelayMs = 1000; setStatus('图传已连接'); });
    stream.addEventListener('error', () => { setStatus('图传连接中…'); closeStream(); scheduleReconnect(); });
  }

  function setStatus(text) {
    const element = document.getElementById('cameraStatus');
    if (element) element.textContent = text;
  }

  function openStream() {
    if (!active || document.hidden || !state.available || !state.streamUrl) return;
    const stream = document.getElementById('cameraStream');
    if (stream && !stream.src) stream.src = state.streamUrl + (state.streamUrl.includes('?') ? '&' : '?') + 't=' + Date.now();
  }

  function closeStream() {
    const stream = document.getElementById('cameraStream');
    if (stream) stream.removeAttribute('src');
  }

  function scheduleReconnect() {
    clearTimeout(reconnectTimer);
    if (!active || !state.available || document.hidden) return;
    reconnectTimer = setTimeout(openStream, reconnectDelayMs);
    reconnectDelayMs = Math.min(reconnectDelayMs * 2, 8000);
  }

  function enter() {
    if (!state.available) return;
    active = true;
    reconnectDelayMs = 1000;
    document.body.classList.add('camera-mode');
    setStatus('图传连接中…');
    openStream();
  }

  function exit() {
    if (!active) return;
    api.stop();
    active = false;
    clearTimeout(reconnectTimer);
    closeStream();
    document.body.classList.remove('camera-mode');
  }

  async function setProfile(profile) {
    if (state.profilePending) return;
    state.profilePending = true;
    renderProfiles();
    setStatus('画质切换中…');
    closeStream();
    try {
      const response = await fetch('/api/camera/profile', {
        method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({profile})
      });
      if (!response.ok) throw new Error('profile rejected');
    } catch (_) {
      state.profilePending = false;
      setStatus('画质切换失败');
      scheduleReconnect();
    }
  }

  function renderProfiles() {
    document.querySelectorAll('[data-profile]').forEach((button) => {
      button.classList.toggle('active', button.dataset.profile === state.activeProfile);
      button.disabled = state.profilePending;
    });
  }

  function applyCaps(caps) {
    const camera = caps && caps.peripherals && caps.peripherals.camera;
    state = {
      available: !!(camera && camera.available),
      streamUrl: camera && camera.streamUrl ? camera.streamUrl : '',
      activeProfile: camera && camera.activeProfile ? camera.activeProfile : 'qvga',
      profilePending: !!(camera && camera.profilePending)
    };
    const entry = document.getElementById('cameraEntry');
    if (entry) entry.hidden = !state.available;
    renderProfiles();
    if (active && !state.available) {
      closeStream();
      setStatus('摄像头离线，可返回高级控制');
    } else if (active && !state.profilePending) {
      openStream();
    }
  }

  async function refreshCaps() {
    try {
      const response = await fetch('/api/caps', {cache: 'no-store'});
      if (response.ok) applyCaps(await response.json());
    } catch (_) {}
  }

  function init(options) {
    if (api) return;
    api = options;
    installStyles();
    installDom();
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) closeStream();
      else if (active) openStream();
    });
    window.addEventListener('pagehide', closeStream);
    capsTimer = window.setInterval(refreshCaps, 2000);
    refreshCaps();
  }

  window.CameraUi = {init, applyCaps};
})();
