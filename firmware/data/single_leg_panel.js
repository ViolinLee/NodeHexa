(function () {
  const SEND_INTERVAL_MS = 80;
  const HOLD_RESEND_INTERVAL_MS = 120;
  const CHANGE_THRESHOLD = 0.04;
  const DEADZONE = 0.10;

  const TEXTS = {
    title: { zh: '单腿控制', en: 'Single Leg Control' },
    hint: {
      zh: '左摇杆控制前后/左右，右摇杆控制抬腿高度。仅六足可用。',
      en: 'Left joystick controls XY, right joystick controls lift height. Hexapod only.'
    },
    leftLabel: { zh: '左摇杆', en: 'Left Stick' },
    leftHint: { zh: '前后 / 左右', en: 'Forward / Lateral' },
    rightLabel: { zh: '右摇杆', en: 'Right Stick' },
    rightHint: { zh: '抬起 / 放回待机高度', en: 'Lift / Return to Standby Height' },
    exit: { zh: '退出单腿控制', en: 'Exit Single Leg' },
    leg: { zh: '腿', en: 'Leg ' }
  };

  const state = {
    mount: null,
    panel: null,
    legButtons: [],
    leftPad: null,
    rightPad: null,
    leftKnob: null,
    rightKnob: null,
    exitButton: null,
    titleEl: null,
    hintEl: null,
    leftLabelEl: null,
    leftHintEl: null,
    rightLabelEl: null,
    rightHintEl: null,
    options: null,
    lang: 'zh',
    active: false,
    selectedLeg: null,
    supported: false,
    axes: { lx: 0, ly: 0, rz: 0 },
    lastSent: { lx: 0, ly: 0, rz: 0 },
    lastSentAt: 0,
    holdResendTimer: null,
    styleInjected: false
  };

  function getText(key) {
    const entry = TEXTS[key];
    return entry ? entry[state.lang] : '';
  }

  function roundAxis(value) {
    return Math.round(value * 1000) / 1000;
  }

  function applyDeadzone(value) {
    const abs = Math.abs(value);
    if (abs <= DEADZONE) {
      return 0;
    }
    const scaled = (abs - DEADZONE) / (1 - DEADZONE);
    return value < 0 ? -scaled : scaled;
  }

  function shouldSendAxes(nextAxes, force) {
    if (force) {
      return true;
    }

    const changed =
      Math.abs(nextAxes.lx - state.lastSent.lx) > CHANGE_THRESHOLD ||
      Math.abs(nextAxes.ly - state.lastSent.ly) > CHANGE_THRESHOLD ||
      Math.abs(nextAxes.rz - state.lastSent.rz) > CHANGE_THRESHOLD;

    if (!changed) {
      return false;
    }

    return Date.now() - state.lastSentAt >= SEND_INTERVAL_MS;
  }

  function hasNonZeroAxes() {
    return Math.abs(state.axes.lx) > 1e-4 ||
      Math.abs(state.axes.ly) > 1e-4 ||
      Math.abs(state.axes.rz) > 1e-4;
  }

  function startHoldResendTimer() {
    if (state.holdResendTimer) {
      return;
    }
    state.holdResendTimer = window.setInterval(() => {
      if (!state.active || !hasNonZeroAxes()) {
        return;
      }
      sendAxes(true);
    }, HOLD_RESEND_INTERVAL_MS);
  }

  function stopHoldResendTimer() {
    if (!state.holdResendTimer) {
      return;
    }
    window.clearInterval(state.holdResendTimer);
    state.holdResendTimer = null;
  }

  function resetKnobs() {
    if (state.leftKnob) {
      state.leftKnob.style.transform = 'translate(-50%, -50%)';
    }
    if (state.rightKnob) {
      state.rightKnob.style.transform = 'translate(-50%, -50%)';
    }
  }

  function sendAxes(force) {
    if (!state.active || !state.options || typeof state.options.sendWsPayload !== 'function') {
      return;
    }

    const nextAxes = {
      lx: roundAxis(state.axes.lx),
      ly: roundAxis(state.axes.ly),
      rz: roundAxis(state.axes.rz)
    };

    if (!shouldSendAxes(nextAxes, force)) {
      return;
    }

    state.options.sendWsPayload({
      singleLeg: {
        op: 'input',
        lx: nextAxes.lx,
        ly: nextAxes.ly,
        rz: nextAxes.rz
      }
    });
    state.lastSent = nextAxes;
    state.lastSentAt = Date.now();
  }

  function clearAxes(sendZero) {
    state.axes = { lx: 0, ly: 0, rz: 0 };
    resetKnobs();
    if (sendZero) {
      sendAxes(true);
    }
  }

  function refreshLegButtons() {
    state.legButtons.forEach((button, index) => {
      const label = state.lang === 'zh'
        ? (TEXTS.leg.zh + (index + 1))
        : (TEXTS.leg.en + (index + 1));
      button.textContent = label;
      button.classList.toggle('single-leg-button-active', state.active && state.selectedLeg === index);
    });
  }

  function updateTexts() {
    if (!state.panel) {
      return;
    }

    state.titleEl.textContent = getText('title');
    state.hintEl.textContent = getText('hint');
    state.leftLabelEl.textContent = getText('leftLabel');
    state.leftHintEl.textContent = getText('leftHint');
    state.rightLabelEl.textContent = getText('rightLabel');
    state.rightHintEl.textContent = getText('rightHint');
    state.exitButton.textContent = getText('exit');
    refreshLegButtons();
  }

  function setActiveLeg(legIndex) {
    state.active = true;
    state.selectedLeg = legIndex;
    state.lastSent = { lx: 0, ly: 0, rz: 0 };
    state.lastSentAt = 0;
    clearAxes(false);
    refreshLegButtons();
  }

  function resetUiState() {
    state.active = false;
    state.selectedLeg = null;
    state.lastSent = { lx: 0, ly: 0, rz: 0 };
    state.lastSentAt = 0;
    stopHoldResendTimer();
    clearAxes(false);
    refreshLegButtons();
  }

  function sendStop() {
    if (state.options && typeof state.options.sendWsPayload === 'function') {
      state.options.sendWsPayload({ singleLeg: { op: 'stop' } });
    }
    resetUiState();
  }

  function activateLeg(legIndex) {
    if (!state.supported || !state.options) {
      return;
    }
    if (typeof state.options.guardLowBattery === 'function' && state.options.guardLowBattery()) {
      return;
    }

    if (!state.active && typeof state.options.requestExclusiveControl === 'function') {
      state.options.requestExclusiveControl();
    }

    if (typeof state.options.sendWsPayload !== 'function') {
      return;
    }

    state.options.sendWsPayload({
      singleLeg: {
        op: 'start',
        legIndex: legIndex
      }
    });
    setActiveLeg(legIndex);
    startHoldResendTimer();
  }

  function injectStyle() {
    if (state.styleInjected) {
      return;
    }

    const style = document.createElement('style');
    style.textContent = `
      .single-leg-panel {
        display: none;
        max-width: 760px;
        margin: 14px auto 0;
        padding: 16px;
        background: #f5f7fb;
        border-radius: 16px;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.08);
      }
      .single-leg-title {
        font-size: 20px;
        font-weight: 700;
        text-align: center;
        color: #333;
      }
      .single-leg-hint {
        margin-top: 8px;
        font-size: 13px;
        line-height: 1.5;
        text-align: center;
        color: #666;
      }
      .single-leg-legs {
        display: grid;
        grid-template-columns: repeat(3, minmax(0, 1fr));
        gap: 10px;
        margin-top: 16px;
      }
      .single-leg-button {
        min-height: 42px;
        border: 2px solid #1E90FF;
        border-radius: 12px;
        background: #fff;
        color: #1E90FF;
        font-size: 15px;
        font-weight: 600;
        cursor: pointer;
        transition: all 0.2s ease;
      }
      .single-leg-button:hover {
        background: #edf5ff;
      }
      .single-leg-button-active {
        background: #1E90FF;
        color: #fff;
        box-shadow: 0 6px 14px rgba(30, 144, 255, 0.25);
      }
      .single-leg-sticks {
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        gap: 16px;
        margin-top: 18px;
      }
      .single-leg-stick-card {
        padding: 14px;
        background: #fff;
        border-radius: 14px;
        box-shadow: inset 0 0 0 1px rgba(30, 144, 255, 0.08);
      }
      .single-leg-stick-label {
        font-size: 16px;
        font-weight: 700;
        text-align: center;
        color: #333;
      }
      .single-leg-stick-hint {
        margin-top: 4px;
        font-size: 12px;
        text-align: center;
        color: #666;
      }
      .single-leg-pad {
        position: relative;
        width: 180px;
        height: 180px;
        margin: 14px auto 0;
        border-radius: 18px;
        background:
          linear-gradient(transparent 49%, rgba(30, 144, 255, 0.2) 49%, rgba(30, 144, 255, 0.2) 51%, transparent 51%),
          linear-gradient(90deg, transparent 49%, rgba(30, 144, 255, 0.2) 49%, rgba(30, 144, 255, 0.2) 51%, transparent 51%),
          radial-gradient(circle at center, rgba(30, 144, 255, 0.12), rgba(30, 144, 255, 0.03));
        touch-action: none;
        user-select: none;
      }
      .single-leg-pad-vertical::before {
        content: '';
        position: absolute;
        top: 10px;
        bottom: 10px;
        left: 50%;
        width: 4px;
        margin-left: -2px;
        background: rgba(30, 144, 255, 0.18);
        border-radius: 999px;
      }
      .single-leg-knob {
        position: absolute;
        left: 50%;
        top: 50%;
        width: 48px;
        height: 48px;
        border-radius: 50%;
        background: linear-gradient(180deg, #4aa3ff, #1E90FF);
        box-shadow: 0 10px 22px rgba(30, 144, 255, 0.28);
        transform: translate(-50%, -50%);
      }
      .single-leg-actions {
        margin-top: 16px;
        text-align: center;
      }
      .single-leg-stop {
        min-width: 180px;
        min-height: 44px;
        border: none;
        border-radius: 12px;
        background: #3A3A3A;
        color: #fff;
        font-size: 15px;
        font-weight: 600;
        cursor: pointer;
      }
      .single-leg-stop:hover {
        background: #4A4A4A;
      }
      @media (max-width: 700px) {
        .single-leg-sticks {
          grid-template-columns: 1fr;
        }
        .single-leg-legs {
          grid-template-columns: repeat(2, minmax(0, 1fr));
        }
      }
    `;
    document.head.appendChild(style);
    state.styleInjected = true;
  }

  function bindStick(pad, knob, options) {
    let tracking = false;
    let pointerId = null;

    function update(clientX, clientY, forceSend) {
      if (!state.active) {
        return;
      }

      const rect = pad.getBoundingClientRect();
      const centerX = rect.left + rect.width / 2;
      const centerY = rect.top + rect.height / 2;
      const maxRadius = rect.width / 2 - 26;

      let dx = clientX - centerX;
      let dy = clientY - centerY;

      if (options.verticalOnly) {
        dx = 0;
      }

      const length = Math.sqrt(dx * dx + dy * dy);
      if (length > maxRadius && length > 0) {
        const ratio = maxRadius / length;
        dx *= ratio;
        dy *= ratio;
      }

      knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

      const normalizedX = applyDeadzone(dx / maxRadius);
      const normalizedY = applyDeadzone(-dy / maxRadius);

      if (options.verticalOnly) {
        state.axes.rz = normalizedY;
      } else {
        state.axes.lx = normalizedX;
        state.axes.ly = normalizedY;
      }
      sendAxes(!!forceSend);
    }

    function reset(forceSend) {
      tracking = false;
      pointerId = null;
      knob.style.transform = 'translate(-50%, -50%)';
      if (options.verticalOnly) {
        state.axes.rz = 0;
      } else {
        state.axes.lx = 0;
        state.axes.ly = 0;
      }
      if (forceSend) {
        sendAxes(true);
      }
    }

    pad.addEventListener('pointerdown', (event) => {
      if (!state.active) {
        return;
      }
      tracking = true;
      pointerId = event.pointerId;
      pad.setPointerCapture(pointerId);
      update(event.clientX, event.clientY, true);
    });

    pad.addEventListener('pointermove', (event) => {
      if (!tracking || event.pointerId !== pointerId) {
        return;
      }
      update(event.clientX, event.clientY, false);
    });

    pad.addEventListener('pointerup', (event) => {
      if (!tracking || event.pointerId !== pointerId) {
        return;
      }
      reset(true);
    });

    pad.addEventListener('pointercancel', (event) => {
      if (!tracking || event.pointerId !== pointerId) {
        return;
      }
      reset(true);
    });
  }

  function buildPanel() {
    injectStyle();

    const panel = document.createElement('section');
    panel.className = 'single-leg-panel';
    panel.innerHTML = `
      <div class="single-leg-title"></div>
      <div class="single-leg-hint"></div>
      <div class="single-leg-legs"></div>
      <div class="single-leg-sticks">
        <div class="single-leg-stick-card">
          <div class="single-leg-stick-label"></div>
          <div class="single-leg-stick-hint"></div>
          <div class="single-leg-pad">
            <div class="single-leg-knob"></div>
          </div>
        </div>
        <div class="single-leg-stick-card">
          <div class="single-leg-stick-label"></div>
          <div class="single-leg-stick-hint"></div>
          <div class="single-leg-pad single-leg-pad-vertical">
            <div class="single-leg-knob"></div>
          </div>
        </div>
      </div>
      <div class="single-leg-actions">
        <button type="button" class="single-leg-stop"></button>
      </div>
    `;

    state.mount.innerHTML = '';
    state.mount.appendChild(panel);
    state.panel = panel;
    state.titleEl = panel.querySelector('.single-leg-title');
    state.hintEl = panel.querySelector('.single-leg-hint');
    state.leftLabelEl = panel.querySelectorAll('.single-leg-stick-label')[0];
    state.rightLabelEl = panel.querySelectorAll('.single-leg-stick-label')[1];
    state.leftHintEl = panel.querySelectorAll('.single-leg-stick-hint')[0];
    state.rightHintEl = panel.querySelectorAll('.single-leg-stick-hint')[1];
    state.leftPad = panel.querySelectorAll('.single-leg-pad')[0];
    state.rightPad = panel.querySelectorAll('.single-leg-pad')[1];
    state.leftKnob = panel.querySelectorAll('.single-leg-knob')[0];
    state.rightKnob = panel.querySelectorAll('.single-leg-knob')[1];
    state.exitButton = panel.querySelector('.single-leg-stop');

    const legsContainer = panel.querySelector('.single-leg-legs');
    state.legButtons = [];
    for (let i = 0; i < 6; ++i) {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'single-leg-button';
      button.addEventListener('click', () => activateLeg(i));
      legsContainer.appendChild(button);
      state.legButtons.push(button);
    }

    state.exitButton.addEventListener('click', () => {
      if (!state.active) {
        resetUiState();
        return;
      }
      clearAxes(true);
      sendStop();
    });

    bindStick(state.leftPad, state.leftKnob, { verticalOnly: false });
    bindStick(state.rightPad, state.rightKnob, { verticalOnly: true });
    updateTexts();
    resetUiState();
  }

  function applyCaps(caps) {
    const robotType = caps && caps.robot && caps.robot.type
      ? caps.robot.type
      : (state.options && typeof state.options.getRobotType === 'function' ? state.options.getRobotType() : 'hexa');
    state.supported = robotType === 'hexa' && !!(caps && caps.manual && caps.manual.singleLeg);
    if (!state.panel) {
      return;
    }
    state.panel.style.display = state.supported ? 'block' : 'none';
    if (!state.supported) {
      resetUiState();
    }
  }

  function init(options) {
    state.options = options || {};
    state.mount = state.options.mount || null;
    state.lang = typeof state.options.getCurrentLang === 'function'
      ? state.options.getCurrentLang()
      : 'zh';
    if (!state.mount) {
      return;
    }
    buildPanel();

    document.addEventListener('visibilitychange', () => {
      if (document.hidden && state.active) {
        clearAxes(true);
        sendStop();
      }
    });

    window.addEventListener('beforeunload', () => {
      if (!state.active || !state.options || typeof state.options.sendWsPayload !== 'function') {
        return;
      }
      try {
        state.options.sendWsPayload({ singleLeg: { op: 'stop' } });
      } catch (_) {
        // ignore teardown failures
      }
      stopHoldResendTimer();
    });
  }

  window.SingleLegPanel = {
    init,
    applyCaps,
    updateLanguage(lang) {
      state.lang = lang === 'en' ? 'en' : 'zh';
      updateTexts();
    },
    handleSocketClosed() {
      resetUiState();
    },
    forceResetUi() {
      resetUiState();
    }
  };
})();
