// AP 配置与状态机（NVS/Preferences 持久化）

#include "ap_config.h"

#include <WiFi.h>
#include <Preferences.h>

namespace apconfig {

namespace {

Preferences prefs;

// NVS 命名空间与键名
static constexpr const char* kNs = "apcfg";
static constexpr const char* kKeySsid = "ssid";
static constexpr const char* kKeyPass = "pass";
static constexpr const char* kKeyPending = "pending";
static constexpr const char* kKeyPrevSsid = "prev_ssid";
static constexpr const char* kKeyPrevPass = "prev_pass";

// 默认配置
static constexpr const char* kDefaultSsidBase = "NodeHexa";
static constexpr const char* kDefaultPass = "roboticscv666";

// 回退超时（毫秒）
static constexpr uint32_t kPendingTimeoutMs = 5 * 60 * 1000;

// 任务句柄
TaskHandle_t pendingMonitorTask = nullptr;

String cachedCurrentSsid;

String generateDefaultSSID() {
  uint64_t mac = ESP.getEfuseMac();
  uint16_t suffix = (uint16_t)(mac & 0xFFFFull);
  char buf[32];
  snprintf(buf, sizeof(buf), "%s-%04X", kDefaultSsidBase, suffix);
  return String(buf);
}

APConfig readConfig() {
  APConfig cfg;
  // 只读打开若命名空间尚未创建会返回 NOT_FOUND，首次需回退到读写创建
  if (!prefs.begin(kNs, true)) {
    prefs.begin(kNs, false);
  }
  cfg.ssid = prefs.getString(kKeySsid, generateDefaultSSID());
  cfg.password = prefs.getString(kKeyPass, kDefaultPass);
  cfg.pending = prefs.getBool(kKeyPending, false);
  cfg.prevSsid = prefs.getString(kKeyPrevSsid, "");
  cfg.prevPassword = prefs.getString(kKeyPrevPass, "");
  prefs.end();
  return cfg;
}

void writeConfig(const APConfig& cfg) {
  prefs.begin(kNs, false);
  prefs.putString(kKeySsid, cfg.ssid);
  prefs.putString(kKeyPass, cfg.password);
  prefs.putBool(kKeyPending, cfg.pending);
  prefs.putString(kKeyPrevSsid, cfg.prevSsid);
  prefs.putString(kKeyPrevPass, cfg.prevPassword);
  prefs.end();
}

void startAP(const String& ssid, const String& pass) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid.c_str(), pass.c_str());
  cachedCurrentSsid = ssid;
}

void monitorPendingTask(void* pv) {
  (void)pv;
  // 延迟到系统稳定
  vTaskDelay(pdMS_TO_TICKS(3000));
  while (true) {
    APConfig cfg = readConfig();
    if (cfg.pending) {
      // 等待超时时间
      vTaskDelay(pdMS_TO_TICKS(kPendingTimeoutMs));
      // 再次读取，确认仍未被清除
      APConfig again = readConfig();
      if (again.pending) {
        // 回退
        if (again.prevSsid.length() > 0) {
          APConfig rolled;
          rolled.ssid = again.prevSsid;
          rolled.password = again.prevPassword;
          rolled.pending = false;
          rolled.prevSsid = "";
          rolled.prevPassword = "";
          writeConfig(rolled);
        } else {
          APConfig rolled;
          rolled.ssid = generateDefaultSSID();
          rolled.password = kDefaultPass;
          rolled.pending = false;
          rolled.prevSsid = "";
          rolled.prevPassword = "";
          writeConfig(rolled);
        }
        // 重启使回退生效
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP.restart();
      }
    } else {
      // 无 pending 时降低检查频率
      vTaskDelay(pdMS_TO_TICKS(10000));
    }
  }
}

void rebootTask(void* p) {
  uint32_t ms = (uint32_t)p;
  vTaskDelay(pdMS_TO_TICKS(ms));
  Serial.println("AP Config: Rebooting now...");
  ESP.restart();
}

}  // namespace

void init() {
  // 启动 AP
  APConfig cfg = readConfig();
  startAP(cfg.ssid, cfg.password);

  // 启动 pending 监控任务
  if (pendingMonitorTask == nullptr) {
    xTaskCreate(
      monitorPendingTask,
      "APPendingMonitor",
      4096,
      nullptr,
      1,
      &pendingMonitorTask
    );
  }
}

String getCurrentSSID() { return cachedCurrentSsid.length() ? cachedCurrentSsid : readConfig().ssid; }

bool isPending() { return readConfig().pending; }

void confirm() {
  APConfig cfg = readConfig();
  if (!cfg.pending) return;
  cfg.pending = false;
  cfg.prevSsid = "";
  cfg.prevPassword = "";
  writeConfig(cfg);
}

bool setNewConfig(const String& ssid, const String& password) {
  APConfig cur = readConfig();

  APConfig next;
  next.ssid = ssid;
  next.password = password;
  next.pending = true;
  next.prevSsid = cur.ssid;
  next.prevPassword = cur.password;
  writeConfig(next);
  return true;
}

void resetToDefault() {
  APConfig dft;
  dft.ssid = generateDefaultSSID();
  dft.password = kDefaultPass;
  dft.pending = false;
  dft.prevSsid = "";
  dft.prevPassword = "";
  writeConfig(dft);
}

void requestReboot(uint32_t delayMs) {
  // 创建短任务延迟重启，避免阻断 HTTP 响应
  xTaskCreate(
    rebootTask,
    "APReboot",
    2048,
    (void*)delayMs,
    1,
    nullptr
  );
}

void printCurrentAPInfo(Stream& out) {
  APConfig cfg = readConfig();
  out.printf("AP SSID: %s\n", cfg.ssid.c_str());
  out.printf("AP Pending: %s\n", cfg.pending ? "true" : "false");
  out.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
}

APConfig getConfig() {
  return readConfig();
}

}  // namespace apconfig


