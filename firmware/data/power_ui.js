(function(global) {
  const STYLE_ID = 'power-ui-style';
  const DEFAULT_STATE = {
    lowBatteryLatched: false,
    lowBatteryProtectionEnabled: true,
    voltageMv: 0,
    percentEstimate: 0,
    lowBatteryThresholdMv: 7200
  };

  function ensureStyle() {
    if (document.getElementById(STYLE_ID)) return;
    const style = document.createElement('style');
    style.id = STYLE_ID;
    style.textContent = [
      '.power-badge-wrap{display:flex;justify-content:center;align-items:center;margin:10px auto 14px;}',
      '.power-badge{display:inline-flex;align-items:center;gap:8px;padding:8px 14px;border-radius:999px;',
      'font-size:14px;font-weight:700;box-shadow:0 4px 12px rgba(15,23,42,0.08);background:#eef6ff;color:#1d4ed8;}',
      '.power-badge.power-good{background:#ecfdf3;color:#0f766e;}',
      '.power-badge.power-warn{background:#fff7ed;color:#c2410c;}',
      '.power-badge.power-low{background:#fef2f2;color:#b91c1c;}',
      '.power-badge-dot{width:10px;height:10px;border-radius:50%;background:currentColor;opacity:0.8;}'
    ].join('');
    document.head.appendChild(style);
  }

  function createPowerUi(options) {
    const opts = options || {};
    const state = Object.assign({}, DEFAULT_STATE);
    let pollTimer = null;

    function getLang() {
      return typeof opts.getLang === 'function' ? opts.getLang() : 'zh';
    }

    function alertMessage(message) {
      if (typeof opts.alertFn === 'function') {
        if (opts.alertFn === window.alert) {
          window.alert(message);
        } else {
          opts.alertFn(message);
        }
      } else {
        window.alert(message);
      }
    }

    function getLowBatteryMessage() {
      return getLang() === 'en'
        ? 'Low battery. Please power off and recharge.'
        : '电量低，请关闭电源后进行充电！';
    }

    function isLowBatteryErrorMessage(message) {
      const text = String(message || '');
      return text.indexOf('电量低') !== -1 || text.toLowerCase().indexOf('low battery') !== -1;
    }

    function getMount() {
      if (!opts.mountId) return null;
      return document.getElementById(opts.mountId);
    }

    function getBadgeLevel() {
      if (isLowBatteryWarning()) {
        return 'power-low';
      }
      if (state.voltageMv < 7800) {
        return 'power-warn';
      }
      return 'power-good';
    }

    function isLowBatteryWarning() {
      return state.voltageMv <= state.lowBatteryThresholdMv;
    }

    function isLowBatteryBlocked() {
      return state.lowBatteryLatched;
    }

    function isLowBatteryActive() {
      return isLowBatteryWarning() || isLowBatteryBlocked();
    }

    function notifyStateChange() {
      if (typeof opts.onStateChange === 'function') {
        opts.onStateChange(Object.assign({}, state));
      }
    }

    function render() {
      const mount = getMount();
      if (!mount) return;
      ensureStyle();

      let badge = mount.querySelector('.power-badge');
      if (!badge) {
        mount.classList.add('power-badge-wrap');
        badge = document.createElement('div');
        badge.className = 'power-badge';
        badge.innerHTML = '<span class="power-badge-dot"></span><span class="power-badge-text"></span>';
        mount.appendChild(badge);
      }

      const voltageText = (state.voltageMv / 1000).toFixed(2) + 'V';
      const percentText = String(state.percentEstimate) + '%';
      const lowText = getLang() === 'en' ? 'Low' : '低电量';
      const label = getLang() === 'en' ? 'Battery' : '电池';
      const text = label + ' ' + voltageText + ' · ' + percentText + (state.lowBatteryLatched ? (' · ' + lowText) : '');

      badge.className = 'power-badge ' + getBadgeLevel();
      const textEl = badge.querySelector('.power-badge-text');
      if (textEl) textEl.textContent = text;
    }

    function applyCaps(data) {
      if (data && data.power) {
        if (typeof data.power.lowBatteryLatched !== 'undefined') {
          state.lowBatteryLatched = !!data.power.lowBatteryLatched;
        }
        if (typeof data.power.lowBatteryProtectionEnabled !== 'undefined') {
          state.lowBatteryProtectionEnabled = !!data.power.lowBatteryProtectionEnabled;
        }
        if (typeof data.power.voltageMv === 'number') {
          state.voltageMv = data.power.voltageMv;
        }
        if (typeof data.power.percentEstimate === 'number') {
          state.percentEstimate = data.power.percentEstimate;
        }
        if (typeof data.power.lowBatteryThresholdMv === 'number') {
          state.lowBatteryThresholdMv = data.power.lowBatteryThresholdMv;
        }
      }
      render();
      notifyStateChange();
      return Object.assign({}, state);
    }

    async function loadCaps() {
      const fetchCaps = opts.fetchCaps || function() {
        return fetch('/api/caps', { cache: 'no-store' }).then(function(resp) { return resp.json(); });
      };
      const data = await fetchCaps();
      applyCaps(data);
      return data;
    }

    function handleLowBatteryFeedback(message) {
      if (!state.lowBatteryProtectionEnabled) return false;
      state.lowBatteryLatched = true;
      render();
      notifyStateChange();
      alertMessage(isLowBatteryErrorMessage(message) ? getLowBatteryMessage() : (message || getLowBatteryMessage()));
      return true;
    }

    function handleWsMessage(msg) {
      if (!msg) return false;
      if (msg.event === 'lowBattery') {
        return handleLowBatteryFeedback(msg.message);
      }
      if (msg.status === 'error' && isLowBatteryErrorMessage(msg.message)) {
        return handleLowBatteryFeedback(msg.message);
      }
      return false;
    }

    function handleHttpErrorMessage(message) {
      if (isLowBatteryErrorMessage(message)) {
        return handleLowBatteryFeedback(message);
      }
      return false;
    }

    function guardLowBattery() {
      if (state.lowBatteryProtectionEnabled && isLowBatteryBlocked()) {
        alertMessage(getLowBatteryMessage());
        return true;
      }
      return false;
    }

    function setProtectionEnabled(enabled) {
      state.lowBatteryProtectionEnabled = !!enabled;
      if (!state.lowBatteryProtectionEnabled) {
        state.lowBatteryLatched = false;
      }
      render();
      notifyStateChange();
    }

    function startPolling(intervalMs) {
      stopPolling();
      pollTimer = setInterval(function() {
        loadCaps().catch(function() {});
      }, intervalMs || 10000);
    }

    function stopPolling() {
      if (pollTimer) {
        clearInterval(pollTimer);
        pollTimer = null;
      }
    }

    return {
      applyCaps: applyCaps,
      loadCaps: loadCaps,
      render: render,
      guardLowBattery: guardLowBattery,
      handleWsMessage: handleWsMessage,
      handleHttpErrorMessage: handleHttpErrorMessage,
      setProtectionEnabled: setProtectionEnabled,
      startPolling: startPolling,
      stopPolling: stopPolling,
      isLowBatteryActive: isLowBatteryActive,
      getState: function() {
        return Object.assign({}, state);
      }
    };
  }

  global.createPowerUi = createPowerUi;
})(window);
