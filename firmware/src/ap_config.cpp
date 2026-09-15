// AP 配置与状态机（NVS/Preferences 持久化）

#include "ap_config.h"

#include <WiFi.h>
#include <Preferences.h>
#include <esp_err.h>
#include <esp_mac.h>
#include <esp_wifi.h>

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
static constexpr const char* kKeyWifiMigrationVersion = "wifi_mig_ver";
static constexpr uint8_t kWifiMigrationVersion = 1;

// 默认配置
static constexpr const char* kDefaultSsidBase = "NodeHexa";
static constexpr const char* kDefaultPass = "roboticscv666";

// 回退超时（毫秒）
static constexpr uint32_t kPendingTimeoutMs = 5 * 60 * 1000;

// 任务句柄
TaskHandle_t pendingMonitorTask = nullptr;

String cachedCurrentSsid;

bool isValidSSID(const String& ssid) {
  return ssid.length() > 0 && ssid.length() <= 31;
}

bool isValidPassword(const String& password) {
  return password.length() == 0 ||
         (password.length() >= 8 && password.length() <= 63);
}

String formatDefaultSSID(const uint8_t mac[6]) {
  const uint16_t suffix = (static_cast<uint16_t>(mac[4]) << 8) | mac[5];
  char buf[32];
  snprintf(buf, sizeof(buf), "%s-%04X", kDefaultSsidBase, suffix);
  return String(buf);
}

String generateDefaultSSID() {
  uint8_t mac[6]{};
  if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) != ESP_OK &&
      esp_efuse_mac_get_default(mac) != ESP_OK) {
    return String(kDefaultSsidBase);
  }
  return formatDefaultSSID(mac);
}

String generateLegacyDefaultSSID() {
  uint8_t mac[6]{};
  if (esp_efuse_mac_get_default(mac) != ESP_OK) {
    return String();
  }

  // The old implementation copied these bytes into a little-endian uint64_t
  // and used its low 16 bits, producing e.g. 8C:94 -> "948C".
  const uint8_t legacyBytes[6] = {0, 0, 0, 0, mac[1], mac[0]};
  return formatDefaultSSID(legacyBytes);
}

APConfig readConfig() {
  APConfig cfg;
  // 只读打开若命名空间尚未创建会返回 NOT_FOUND，首次需回退到读写创建
  if (!prefs.begin(kNs, true)) {
    if (!prefs.begin(kNs, false)) {
      cfg.ssid = generateDefaultSSID();
      cfg.password = kDefaultPass;
      cfg.pending = false;
      return cfg;
    }
  }
  // Preferences::getString logs an error for a missing key even when a
  // default is supplied. Avoid recurring false error logs on factory NVS.
  cfg.ssid = prefs.isKey(kKeySsid) ? prefs.getString(kKeySsid) : generateDefaultSSID();
  cfg.password = prefs.isKey(kKeyPass) ? prefs.getString(kKeyPass) : String(kDefaultPass);
  cfg.pending = prefs.getBool(kKeyPending, false);
  cfg.prevSsid = prefs.isKey(kKeyPrevSsid) ? prefs.getString(kKeyPrevSsid) : String();
  cfg.prevPassword = prefs.isKey(kKeyPrevPass) ? prefs.getString(kKeyPrevPass) : String();
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

uint8_t readWifiMigrationVersion() {
  if (!prefs.begin(kNs, true)) {
    return 0;
  }
  const uint8_t version = prefs.getUChar(kKeyWifiMigrationVersion, 0);
  prefs.end();
  return version;
}

bool writeWifiMigrationVersion(uint8_t version) {
  if (!prefs.begin(kNs, false)) {
    return false;
  }
  const size_t written = prefs.putUChar(kKeyWifiMigrationVersion, version);
  prefs.end();
  return written == sizeof(version);
}

bool restoreWifiDriverSettings() {
  Serial.printf("AP Config: Migrating WiFi driver settings to version %u\n",
                static_cast<unsigned>(kWifiMigrationVersion));

  // esp_wifi_restore() requires an initialized Wi-Fi driver. Initialize it
  // with flash persistence enabled so the restore targets the driver's old
  // persistent settings, then immediately switch all subsequent settings to
  // RAM. NodeHexa persists its own AP credentials in the "apcfg" namespace.
  WiFi.persistent(true);
  if (!WiFi.mode(WIFI_AP)) {
    WiFi.persistent(false);
    Serial.println("AP Config: WiFi migration could not initialize the driver");
    return false;
  }

  const esp_err_t restoreResult = esp_wifi_restore();
  const esp_err_t storageResult = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  WiFi.mode(WIFI_OFF);
  WiFi.persistent(false);

  if (restoreResult != ESP_OK) {
    Serial.printf("AP Config: WiFi migration restore failed: %s (%d)\n",
                  esp_err_to_name(restoreResult),
                  static_cast<int>(restoreResult));
  }
  if (storageResult != ESP_OK) {
    Serial.printf("AP Config: Failed to select RAM-only WiFi storage: %s (%d)\n",
                  esp_err_to_name(storageResult),
                  static_cast<int>(storageResult));
  }
  return restoreResult == ESP_OK && storageResult == ESP_OK;
}

bool startAP(const String& ssid, const String& pass) {
  // AP credentials are persisted by this module in the "apcfg" namespace.
  // Keep the Wi-Fi driver's own configuration in RAM so an SDK upgrade or a
  // stale nvs.net80211 entry cannot become a second source of truth.
  WiFi.persistent(false);
  if (!WiFi.mode(WIFI_AP)) {
    Serial.println("AP Config: Failed to enable WiFi AP mode");
    cachedCurrentSsid = "";
    return false;
  }
  const esp_err_t storageResult = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (storageResult != ESP_OK) {
    Serial.printf("AP Config: Failed to select RAM-only WiFi storage: %s (%d)\n",
                  esp_err_to_name(storageResult),
                  static_cast<int>(storageResult));
    cachedCurrentSsid = "";
    return false;
  }
  if (!WiFi.softAP(ssid.c_str(), pass.c_str())) {
    Serial.printf("AP Config: Failed to start SoftAP (SSID length=%u, password length=%u)\n",
                  static_cast<unsigned>(ssid.length()),
                  static_cast<unsigned>(pass.length()));
    cachedCurrentSsid = "";
    return false;
  }

  const String runtimeSsid = WiFi.softAPSSID();
  const bool runtimeMatches = runtimeSsid == ssid;
  cachedCurrentSsid = runtimeSsid;
  Serial.printf("AP Config: SoftAP started (configMatch=%s, channel=%ld, MAC=%s)\n",
                runtimeMatches ? "true" : "false",
                static_cast<long>(WiFi.channel()),
                WiFi.softAPmacAddress().c_str());
  return runtimeMatches;
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
  const String legacyDefaultSsid = generateLegacyDefaultSSID();
  if (legacyDefaultSsid.length() > 0 && cfg.ssid == legacyDefaultSsid) {
    cfg.ssid = generateDefaultSSID();
    if (cfg.prevSsid == legacyDefaultSsid) {
      cfg.prevSsid = cfg.ssid;
    }
    writeConfig(cfg);
    Serial.printf("AP Config: Migrated legacy generated SSID to %s\n", cfg.ssid.c_str());
  }
  if (!isValidSSID(cfg.ssid) || !isValidPassword(cfg.password)) {
    Serial.printf("AP Config: Invalid stored configuration; using defaults (SSID length=%u, password length=%u)\n",
                  static_cast<unsigned>(cfg.ssid.length()),
                  static_cast<unsigned>(cfg.password.length()));
    cfg.ssid = generateDefaultSSID();
    cfg.password = kDefaultPass;
    cfg.pending = false;
    cfg.prevSsid = "";
    cfg.prevPassword = "";
    writeConfig(cfg);
  }
  const bool migrationNeeded = readWifiMigrationVersion() < kWifiMigrationVersion;
  const bool migrationSucceeded = !migrationNeeded || restoreWifiDriverSettings();
  const bool apStarted = startAP(cfg.ssid, cfg.password);
  if (!apStarted) {
    Serial.println("AP Config: SoftAP is unavailable");
  } else if (migrationNeeded && migrationSucceeded) {
    if (writeWifiMigrationVersion(kWifiMigrationVersion)) {
      Serial.printf("AP Config: WiFi migration version %u completed\n",
                    static_cast<unsigned>(kWifiMigrationVersion));
    } else {
      Serial.println("AP Config: WiFi migration marker write failed; will retry next boot");
    }
  } else if (migrationNeeded) {
    Serial.println("AP Config: WiFi migration incomplete; will retry next boot");
  }

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
  if (!isValidSSID(ssid) || !isValidPassword(password)) {
    return false;
  }
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
