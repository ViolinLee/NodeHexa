(function () {
  'use strict';

  let api = null;
  let state = { available: false, streamUrl: '', activeProfile: 'qvga', profilePending: false };
  let active = false;
  let reconnectTimer = 0;
  let reconnectDelayMs = 1000;
  let capsTimer = 0;
  let streamSessionId = 0;
  let motionButtonMode = 'continuous';

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
      .camera-view img{width:100%;height:100%;object-fit:contain;background:#000;-webkit-user-drag:none}
      .camera-status{position:absolute;left:10px;top:10px;max-width:calc(100% - 20px);padding:6px 10px;border-radius:999px;background:rgba(0,0,0,.72);font:13px system-ui,sans-serif}
      .camera-controls{flex:1;min-height:0;display:flex;flex-direction:column;justify-content:space-evenly;align-items:center;padding:8px calc(14px + env(safe-area-inset-right)) calc(12px + env(safe-area-inset-bottom)) calc(14px + env(safe-area-inset-left));box-sizing:border-box;background:#111827}
      .camera-toolbar{width:min(620px,100%);display:flex;gap:8px;justify-content:space-between;align-items:center}
      .camera-toolbar button,.camera-pad button{min-width:56px;min-height:48px;border:0;border-radius:12px;color:#fff;background:#2563eb;font-size:16px;font-weight:700;touch-action:manipulation;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
      .camera-toolbar .camera-exit{background:#4b5563;padding:0 14px}
      .camera-toolbar .camera-profile.active{background:#059669}
      .camera-toolbar button:disabled{opacity:.48}
      .camera-motion-mode{padding:5px 9px;border-radius:999px;background:#1f2937;color:#dbeafe;font:600 13px system-ui,sans-serif;white-space:nowrap}
      .camera-profiles{display:flex;gap:6px;white-space:nowrap}
      .camera-pad{display:grid;grid-template-columns:repeat(3,clamp(76px,21vw,92px));grid-template-rows:repeat(3,clamp(58px,8dvh,72px));gap:clamp(10px,2.8vw,14px);align-items:center;justify-content:center}
      .camera-pad button{width:100%;height:100%;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:3px;font-size:clamp(26px,7vw,30px);line-height:1}
      .camera-pad button small{font-size:12px;line-height:1;font-weight:700}
      .camera-pad .forward{grid-column:2;grid-row:1}.camera-pad .turn-left{grid-column:1;grid-row:2}.camera-pad .stop{grid-column:2;grid-row:2;background:#dc2626}.camera-pad .turn-right{grid-column:3;grid-row:2}.camera-pad .shift-left{grid-column:1;grid-row:3}.camera-pad .back{grid-column:2;grid-row:3}.camera-pad .shift-right{grid-column:3;grid-row:3}
      .camera-speed{display:flex;gap:8px}.camera-speed button{min-width:72px;min-height:48px;border:0;border-radius:12px;color:#fff;background:#374151;font-size:16px;font-weight:700;touch-action:manipulation;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}.camera-speed button.active{background:#7c3aed}
      @media (max-width:479px){.camera-toolbar{display:grid;grid-template-columns:1fr auto}.camera-motion-mode{justify-self:end}.camera-profiles{grid-column:1/-1;justify-self:center}}
      @media (max-width:479px) and (max-height:700px){.camera-view{flex-basis:36%}.camera-controls{padding-top:4px;padding-bottom:calc(6px + env(safe-area-inset-bottom))}.camera-toolbar button{min-height:44px}.camera-pad{grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,52px);gap:6px}.camera-pad button{min-height:44px;font-size:22px;gap:1px}.camera-pad button small{font-size:10px}.camera-speed button{min-height:44px}}
      @media (orientation:landscape) and (max-height:500px){body.camera-mode #cameraVideoMode{flex-direction:row}.camera-view{flex:0 0 56%;height:100%}.camera-controls{height:100%;padding:6px calc(8px + env(safe-area-inset-right)) calc(6px + env(safe-area-inset-bottom)) 8px}.camera-pad{grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,48px);gap:5px}.camera-pad button{min-height:44px;font-size:22px;gap:1px}.camera-pad button small{font-size:10px}.camera-toolbar button{min-height:44px}.camera-speed button{min-width:58px;min-height:44px}}
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
      <div class="camera-view"><div id="cameraStatus" class="camera-status">等待摄像头</div></div>
      <div class="camera-controls">
        <div class="camera-toolbar">
          <button class="camera-exit" id="cameraExit">高级控制</button>
          <div id="cameraMotionMode" class="camera-motion-mode">当前：持续</div>
          <div class="camera-profiles"><button class="camera-profile" data-profile="qvga">低 QVGA</button><button class="camera-profile" data-profile="vga">高 VGA</button></div>
        </div>
        <div class="camera-pad">
          <button type="button" class="forward" data-motion="1" aria-label="前进"><span aria-hidden="true">▲</span><small>前进</small></button>
          <button type="button" class="turn-left" data-motion="4" aria-label="左转"><span aria-hidden="true">↶</span><small>左转</small></button>
          <button type="button" class="stop" id="cameraStop" aria-label="停止"><span aria-hidden="true">■</span><small>停止</small></button>
          <button type="button" class="turn-right" data-motion="5" aria-label="右转"><span aria-hidden="true">↷</span><small>右转</small></button>
          <button type="button" class="shift-left" data-motion="6" aria-label="左移"><span aria-hidden="true">⇦</span><small>左移</small></button>
          <button type="button" class="back" data-motion="3" aria-label="后退"><span aria-hidden="true">▼</span><small>后退</small></button>
          <button type="button" class="shift-right" data-motion="7" aria-label="右移"><span aria-hidden="true">⇨</span><small>右移</small></button>
        </div>
        <div class="camera-speed"><button data-speed="0.33">慢</button><button class="active" data-speed="0.5">中</button><button data-speed="1">快</button></div>
      </div>`;
    document.body.appendChild(root);
    document.getElementById('cameraExit').addEventListener('click', exit);
    document.getElementById('cameraStop').addEventListener('click', () => api.stop());
    root.querySelectorAll('[data-motion]').forEach((button) => {
      button.addEventListener('click', (event) => {
        event.preventDefault();
        if (!api.guardLowBattery()) api.sendMovement(Number(button.dataset.motion));
      });
    });
    root.querySelectorAll('[data-speed]').forEach((button) => button.addEventListener('click', () => {
      if (api.guardLowBattery()) return;
      api.sendSpeed(Number(button.dataset.speed));
      root.querySelectorAll('[data-speed]').forEach((item) => item.classList.toggle('active', item === button));
    }));
    root.querySelectorAll('[data-profile]').forEach((button) =>
      button.addEventListener('click', () => setProfile(button.dataset.profile)));
    renderMotionButtonMode();
  }

  function setStatus(text) {
    const element = document.getElementById('cameraStatus');
    if (element) element.textContent = text;
  }

  function renderMotionButtonMode() {
    const element = document.getElementById('cameraMotionMode');
    if (element) element.textContent = motionButtonMode === 'single_cycle' ? '当前：单周期' : '当前：持续';
  }

  function setMotionButtonMode(mode) {
    motionButtonMode = mode === 'single_cycle' ? 'single_cycle' : 'continuous';
    renderMotionButtonMode();
  }

  function clearReconnectTimer() {
    clearTimeout(reconnectTimer);
    reconnectTimer = 0;
  }

  function removeStreamElement() {
    const stream = document.getElementById('cameraStream');
    if (!stream) return;
    stream.removeAttribute('src');
    stream.remove();
  }

  function openStream() {
    if (!active || document.hidden || !state.available || !state.streamUrl) return;
    if (reconnectTimer) return;
    if (document.getElementById('cameraStream')) return;

    const view = document.querySelector('.camera-view');
    const status = document.getElementById('cameraStatus');
    if (!view || !status) return;

    const sessionId = ++streamSessionId;
    const streamUrl = state.streamUrl;
    const stream = document.createElement('img');
    stream.id = 'cameraStream';
    stream.alt = 'Camera stream';
    stream.draggable = false;
    stream.addEventListener('load', () => {
      if (!active || sessionId !== streamSessionId || stream !== document.getElementById('cameraStream')) return;
      reconnectDelayMs = 1000;
      setStatus(state.binding === 'verified' ? '图传已连接 · 设备已绑定' : '图传已连接 · 未验证绑定（旧固件）');
    });
    stream.addEventListener('error', () => {
      if (!active || sessionId !== streamSessionId || stream !== document.getElementById('cameraStream')) return;
      api.stop();
      setStatus('图传连接中…');
      closeStream();
      scheduleReconnect();
    });
    view.insertBefore(stream, status);
    stream.src = streamUrl + (streamUrl.includes('?') ? '&' : '?') + 't=' + Date.now();
  }

  function closeStream() {
    ++streamSessionId;
    removeStreamElement();
  }

  function scheduleReconnect() {
    clearReconnectTimer();
    if (!active || !state.available || document.hidden) return;
    const scheduledSessionId = streamSessionId;
    reconnectTimer = setTimeout(() => {
      reconnectTimer = 0;
      if (scheduledSessionId !== streamSessionId) return;
      openStream();
    }, reconnectDelayMs);
    reconnectDelayMs = Math.min(reconnectDelayMs * 2, 8000);
  }

  function enter() {
    if (!state.available) return;
    active = true;
    reconnectDelayMs = 1000;
    clearReconnectTimer();
    closeStream();
    document.body.classList.add('camera-mode');
    setStatus('图传连接中…');
    renderMotionButtonMode();
    openStream();
    refreshCaps();
  }

  function exit() {
    if (!active) return;
    api.stop();
    active = false;
    clearReconnectTimer();
    closeStream();
    document.body.classList.remove('camera-mode');
  }

  async function setProfile(profile) {
    if (state.profilePending) return;
    state.profilePending = true;
    renderProfiles();
    setStatus('画质切换中…');
    clearReconnectTimer();
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
    if (api && api.acceptRobotIdentity && !api.acceptRobotIdentity(caps)) {
      state.available = false;
      clearReconnectTimer();
      closeStream();
      if (active) {
        active = false;
        document.body.classList.remove('camera-mode');
      }
      return;
    }
    const camera = caps && caps.peripherals && caps.peripherals.camera;
    const previousStreamUrl = state.streamUrl;
    state = {
      available: !!(camera && camera.available),
      streamUrl: camera && camera.streamUrl ? camera.streamUrl : '',
      activeProfile: camera && camera.activeProfile ? camera.activeProfile : 'qvga',
      profilePending: !!(camera && camera.profilePending),
      binding: camera && camera.binding || 'legacy_unverified'
    };
    const entry = document.getElementById('cameraEntry');
    if (entry) entry.hidden = !state.available;
    renderProfiles();
    if (active && !state.available) {
      api.stop();
      clearReconnectTimer();
      closeStream();
      setStatus('摄像头离线，可返回高级控制');
    } else if (active && state.profilePending) {
      clearReconnectTimer();
      closeStream();
      setStatus('画质切换中…');
    } else if (active && !state.profilePending) {
      if (previousStreamUrl && previousStreamUrl !== state.streamUrl) {
        clearReconnectTimer();
        closeStream();
      }
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
    setMotionButtonMode(api.getMotionButtonMode ? api.getMotionButtonMode() : 'continuous');
    installStyles();
    installDom();
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) {
        if (active) api.stop();
        clearReconnectTimer();
        closeStream();
      }
      else if (active) {
        reconnectDelayMs = 1000;
        openStream();
        refreshCaps();
      }
    });
    window.addEventListener('pagehide', () => {
      if (active) api.stop();
      clearReconnectTimer();
      closeStream();
    });
    capsTimer = window.setInterval(refreshCaps, 2000);
    refreshCaps();
  }

  window.CameraUi = {init, applyCaps, setMotionButtonMode};
})();
