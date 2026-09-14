/* 六足主程序
 *
 * 移植自项目: [https://github.com/SmallpTsai/hexapod-v2-7697, https://github.com/ViolinLee/PiHexa18]
 * 
 * 腿索引:
 *     leg5   leg0
 *     /        \
 *    /          \
 * leg4          leg1
 *    \          /
 *     \        /
 *    leg3    leg2
 */

/* 架构设计（待删）
_mode和mode的处理在异步的Web服务回调函数中处理
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include <Wifi.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <HardwareSerial.h>
#include <atomic>
#include <esp_wifi.h>
#include <esp_system.h>

#include "debug.h"
#include "hexapod.h"
#include "config.h"
#include "calibration.h"
#include "PinDefines.h"
#include "ap_config.h"
#include "device_settings.h"
#include "robot.h"
#include "motion_controller.h"
#include "performance_controller.h"
#include "single_leg_controller.h"
#include "board_config.h"
#include "servo_output.h"
#include "buzzer.h"
#include "uart_protocol.h"
#include "external_device.h"
#include "camera_link.h"
#include "ultrasonic_sensor.h"
#include "power_manager.h"

// 宏定义
#define REACT_DELAY hexapod::config::movementInterval
#define CALIBRATESTART "CALIBRATESTART"
#define CALIBRATESAVE "CALIBRATESAVE"
#define CALIBRATESTART_EXISTING "CALIBRATESTART_EXISTING"
#define NODEHEXA_STRINGIFY_INNER(value) #value
#define NODEHEXA_STRINGIFY(value) NODEHEXA_STRINGIFY_INNER(value)

// 调试模式控制
// #define DEBUG_ADC_MONITOR  // 注释此行可关闭电池电压ADC调试输出
// #define DEBUG_FRAME_RECEIVE  // 启用帧接收调试输出

// 常量定义

// 串口配置
// 静态变量
static std::atomic<int8_t> _mode(0);  // 六足工作模式：0-运动模式 1-校准模式
static int16_t flag = 0;   // 六足运动模式: 见枚举hexapod:MovementMode
static float test_angle = 0.;

// flag访问保护
SemaphoreHandle_t flagMutex;

// 串口通讯相关变量
static uart_protocol::Parser uartParser;
static SemaphoreHandle_t uartTxMutex = nullptr;
static QueueHandle_t uartFrameQueue = nullptr;
static TaskHandle_t uartDispatchTaskHandle = nullptr;
static std::atomic<uint32_t> uartQueueDrops(0);
static bool serialDispatchV2 = false;
static uint16_t serialDispatchSequence = 0;
static portMUX_TYPE uartSequenceMux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t uartNextSequence = 1;
static portMUX_TYPE uartStatsMux = portMUX_INITIALIZER_UNLOCKED;
static uart_protocol::Stats uartStatsSnapshot{};
// Immutable after setup; identifies this AP and this boot's camera session.
static char robotIdentity[24]{};
static char robotApBssid[18]{};
static char cameraSession[33]{};

// 电池监测相关变量
static const unsigned long LOW_BATTERY_SERIAL_REMINDER_INTERVAL_MS = 10000; // 串口提醒重发间隔
static constexpr const char* kLowBatteryUiMessage = "电量低，请关闭电源后进行充电！";
static constexpr const char* kLowBatteryProtectCode = "LOW_BATTERY_PROTECT";
static unsigned long lastLowBatterySerialNotifyMs = 0;
static unsigned long lastServoReleaseAttemptMs = 0;
static constexpr unsigned long kServoReleaseRetryIntervalMs = 1000;
static constexpr size_t kMaxHttpRequestBodyBytes = 2048;

// 实例
AsyncWebServer server(80);
AsyncWebSocket wsRoverCmd("/cmd");

// 函数声明
void handleRoot(AsyncWebServerRequest *request);
void handleCalibrationPage(AsyncWebServerRequest *request);
void handleCalibrationData(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleCalibrationGet(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);
void handleMotionPlanner(AsyncWebServerRequest *request);
void sendHtmlFromSpiffs(AsyncWebServerRequest *request, const char *path);
void sendFileFromSpiffs(AsyncWebServerRequest *request, const char *path, const char *contentType);
void onRobotCmdWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void normal_loop();
void setting_loop();
static void log_output(const char* log);
static bool parseCalibrationData(const String& jsonString,
                                 CalibrationData& data,
                                 String& errorMessage);
void printWelcomeMessage();

// AP 配置接口声明
void handleApConfigGet(AsyncWebServerRequest *request);
void handleApConfigPostBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleApConfigConfirm(AsyncWebServerRequest *request);
void handleApConfigReset(AsyncWebServerRequest *request);

// 最小能力探测接口（仅返回机型与腿数）
void handleCapsGet(AsyncWebServerRequest *request);

// 通用设置接口（用于承载未来更多配置项）
void handleSettingsGet(AsyncWebServerRequest *request);
void handleSettingsPostBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleCameraProfilePostBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleUltrasonicGet(AsyncWebServerRequest *request);
static String* appendRequestBodyChunk(AsyncWebServerRequest *request, const uint8_t *data, size_t len, size_t index, size_t total);
static void clearRequestBodyChunk(AsyncWebServerRequest *request);

void BatteryMonitorTask(void *pvParameters);
void LEDControllerTask(void *pvParameters);

// 串口通讯相关函数声明
void parseSerialMovementCommand(const String& jsonString);
void SerialCommandTask(void *pvParameters);
void UartDispatchTask(void *pvParameters);
void sendSerialResponse(const String& message);
static bool sendV2Message(uart_protocol::MessageType type,
                          uint8_t flags,
                          uint16_t sequence,
                          const String& message);
static uint16_t nextUartSequence();
static bool sendCameraProvision();
static bool sendCameraProfileRequest(const char* profile);
static void handleV2Event(const uart_protocol::Frame& frame);
void clearMovementFlag();
static const char* motionButtonModeToString(devsettings::MotionButtonMode mode);
static bool parseMotionButtonModeField(JsonVariantConst value, devsettings::MotionButtonMode& mode);

// 低电量锁存辅助
static bool isLowBatteryLatched();
static power::LowBatteryPolicy getEffectiveLowBatteryPolicy();
static bool isMotionBlocked();
static void handleLowBatteryLatchedOnce();
static void retryServoReleaseIfNeeded();
static void sendLowBatteryErrorToWebSocket(AsyncWebSocketClient *client);
static void sendLowBatteryErrorToSerial();
static void sendLowBatteryEventToSerial();
static void maybeRepeatLowBatteryEventToSerial();
static uint16_t getLatestBatteryVoltageMv();
static uint8_t estimateBatteryPercent(uint16_t voltageMv);
static bool shouldUseMovingLowBatteryThreshold();
static void publishUartStats();
static uart_protocol::Stats getUartStatsSnapshot();

struct AdvancedCommandResult {
  bool handled = false;
  bool success = false;
  bool suppressAck = false;
  uint32_t sequenceId = 0;
  String message;
};

struct BasicCommandResult {
  bool handled = false;
  bool success = false;
  String message;
};

static void handleSequenceComplete(uint32_t sequenceId);
static AdvancedCommandResult handleAdvancedMotionCommand(JsonVariantConst json);
static BasicCommandResult handleBasicControlCommand(JsonVariantConst json);
static bool parseMovementModeField(JsonVariantConst value, hexapod::MovementMode& mode);
static bool parsePerformanceKindField(JsonVariantConst value, performance::Kind& kind);
static bool buildActionFromJson(JsonVariantConst obj, motion::Action& action, String& error);
static bool hasActionParameters(JsonVariantConst json);

void setup() {
  // 初始化串口
  Serial.begin(115200);
  Serial.println("Starting...");

  // 初始化UART2用于接收运动指令
  Serial2.begin(board::kUartBaudRate, SERIAL_8N1, board::kUartRxPin, board::kUartTxPin);
  Serial2.setTimeout(100);
  Serial2.flush(); // 清空串口缓冲区
  Serial.printf("UART2 initialized: %lu baud (GPIO%u-RX, GPIO%u-TX)\n",
                static_cast<unsigned long>(board::kUartBaudRate),
                board::kUartRxPin,
                board::kUartTxPin);

  // 初始化电池监测相关硬件
  pinMode(BAT_ADC, INPUT);
  pinMode(BAT_LED, OUTPUT);
  flagMutex = xSemaphoreCreateMutex();
  uartTxMutex = xSemaphoreCreateMutex();
  uartFrameQueue = xQueueCreate(8, sizeof(uart_protocol::Frame));
  if (!uartFrameQueue) {
    Serial.println("[UART] Failed to allocate frame queue; UART control disabled.");
  }

  if (!buzzer::begin()) {
    Serial.println("[Buzzer] Initialization failed; continuing without audio.");
  }
  external_device::begin();
  camera_link::begin();
  if (!ultrasonic::begin()) {
    Serial.println("[Ultrasonic] Driver initialization failed.");
  }

  // 初始化I2C
  Wire.setPins(board::kI2cSdaPin, board::kI2cSclPin);

  // 挂载SPIFFS文件系统
  if (!SPIFFS.begin(true)) {
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
  }

  // 初始化WiFi（动态 AP 配置）
  apconfig::init();
  uint8_t apMac[6]{};
  if (esp_wifi_get_mac(WIFI_IF_AP, apMac) == ESP_OK) {
    snprintf(robotIdentity,sizeof(robotIdentity),"%02x%02x%02x%02x%02x%02x",apMac[0],apMac[1],apMac[2],apMac[3],apMac[4],apMac[5]);
    snprintf(robotApBssid,sizeof(robotApBssid),"%02x:%02x:%02x:%02x:%02x:%02x",apMac[0],apMac[1],apMac[2],apMac[3],apMac[4],apMac[5]);
  }
  for (int i=0;i<4;++i)
    snprintf(cameraSession+i*8,9,"%08lx",static_cast<unsigned long>(esp_random()));
  apconfig::printCurrentAPInfo(Serial);

  // 读取设备设置（NVS）
  devsettings::init();
  Serial.printf("Power: lowBatteryPolicy=%s\n",
                power::policyToString(devsettings::getLowBatteryPolicy()));

  // 初始化Web服务
  server.on("/", HTTP_GET, handleRoot);
  server.on("/planner", HTTP_GET, handleMotionPlanner);
  server.on("/planner.html", HTTP_GET, handleMotionPlanner);
  server.on("/power_ui.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    sendFileFromSpiffs(request, "/power_ui.js", "application/javascript; charset=utf-8");
  });
  server.on("/single_leg_panel.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    sendFileFromSpiffs(request, "/single_leg_panel.js", "application/javascript; charset=utf-8");
  });
  server.on("/camera_ui.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    sendFileFromSpiffs(request, "/camera_ui.js", "application/javascript; charset=utf-8");
  });
  server.on("/calibration", HTTP_GET, handleCalibrationPage);
  server.on("/calibration", HTTP_POST, 
    [](AsyncWebServerRequest *request)
    {
      //Serial.println("1");
    },
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final)
    {
      //Serial.println("2");
    },
    handleCalibrationData);
  server.on("/api/calibration", HTTP_GET, handleCalibrationGet);
  server.onNotFound(handleNotFound);

  // AP 配置接口（先注册更具体的路径，再注册通用路径，避免潜在前缀匹配冲突）
  server.on("/api/ap-config/confirm", HTTP_POST, handleApConfigConfirm);
  server.on("/api/ap-config/reset", HTTP_GET, handleApConfigReset);
  server.on("/api/ap-config", HTTP_GET, handleApConfigGet);
  server.on("/api/ap-config", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {},
    handleApConfigPostBody);

  // UI 能力探测（用于校准页适配四足/六足）
  server.on("/api/caps", HTTP_GET, handleCapsGet);

  // 通用设置接口（承载 WiFi 之外的设置项）
  server.on("/api/settings", HTTP_GET, handleSettingsGet);
  server.on("/api/settings", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {},
    handleSettingsPostBody);
  server.on("/api/camera/profile", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {},
    handleCameraProfilePostBody);
  server.on("/api/ultrasonic", HTTP_GET, handleUltrasonicGet);

  wsRoverCmd.onEvent(onRobotCmdWebSocketEvent);
  server.addHandler(&wsRoverCmd);

  server.begin();
  Serial.println("HTTP server started");

  // 初始化日志记录回调函数&机器人工作模式
  hexapod::initLogOutput(log_output, millis);
  if (hexapod::Robot) {
    hexapod::Robot->init(_mode.load() == 1);
  }
  motion::controller().begin();
  performance::controller().begin();
  singleleg::controller().begin();
  motion::controller().setSequenceCallback(handleSequenceComplete);

  // 创建电池监测任务
  xTaskCreate(
    BatteryMonitorTask,
    "BatteryMonitor",
    4096,
    NULL,
    1,
    NULL
  );

  // 创建LED控制任务
  xTaskCreate(
    LEDControllerTask,
    "LEDController",
    4096,
    NULL,
    1,
    NULL
  );

  if (uartFrameQueue) {
    // RX only parses and enqueues. Business handling runs independently so a
    // slow command can never make the hardware UART overflow silently.
    xTaskCreate(SerialCommandTask, "UartRx", 4096, NULL, 3, NULL);
    xTaskCreate(UartDispatchTask, "UartDispatch", 6144, NULL, 2, NULL);
  }

  printWelcomeMessage();

  buzzer::play(buzzer::Cue::Startup);

  Serial.print("Started, mode=");
  Serial.println(_mode.load());
}

void loop() {
  if (isLowBatteryLatched()) {
    handleLowBatteryLatchedOnce();
    retryServoReleaseIfNeeded();
    maybeRepeatLowBatteryEventToSerial();
  }

  external_device::poll(millis());
  const external_device::Snapshot external = external_device::snapshot();
  if (external.type == external_device::Type::Camera && !external.online) {
    camera_link::setOffline();
  }

  // 锁存事实始终保留；仅阻断策略强制回到运动模式。
  if (isMotionBlocked()) {
    if (_mode.load() != 0) {
      _mode.store(0);
    }
    normal_loop();
    return;
  }

  if (_mode.load() == 0) {
    normal_loop();
  }
  else if (_mode.load() == 1) {
    setting_loop();
  }
}

// 函数定义
/* 日志输出
*/
static void log_output(const char* log) {
  Serial.println(log);
}

/* 常规（运动）循环模式
*/
void normal_loop() {
  // if(wsRoverCmd.count() == 0) {
  //   delay(1000 - REACT_DELAY);
  // }

  const bool motionBlocked = isMotionBlocked();

  auto t0 = millis();

  if (motionBlocked) {
    // Both blocking policies freeze the motion pipeline. HoldAndLock must keep
    // the last valid PWM exactly as-is; ReleasePwm has already closed the
    // software output gate, so neither policy should keep writing standby.
  } else if (singleleg::controller().isActive()) {
    singleleg::controller().onLoopTick(REACT_DELAY);
  } else {
    auto mode = hexapod::MOVEMENT_STANDBY;
    if (motion::controller().hasActiveAction()) {
      mode = motion::controller().activeMode();
    } else {
      if (xSemaphoreTake(flagMutex, portMAX_DELAY) == pdTRUE) {
        for (auto m = hexapod::MOVEMENT_STANDBY; m < hexapod::MOVEMENT_TOTAL; m++) {
          if (flag & (1<<m)) {
            mode = m;
            break;
          }
        }
        xSemaphoreGive(flagMutex);
      }
    }

    if (hexapod::Robot) {
      hexapod::Robot->processMovement(mode, REACT_DELAY);
    }
    // 对四足：动作切换存在“等待 entry/对齐”的过渡期，此时实际执行 mode 可能不同。
    // 为保证序列单位(cycles)的计时准确，应以“实际执行的 mode”来累计 completedCycles。
    const auto executedMode = hexapod::Robot ? hexapod::Robot->executedMovementMode(mode) : mode;
    motion::controller().onLoopTick(executedMode, REACT_DELAY);
  }

  auto spent = millis() - t0;

  if(spent < REACT_DELAY) {
    // Serial.println(spent);
    delay(REACT_DELAY-spent);
  }
  else {
    Serial.println(spent);
  }
}

/* 配置循环模式
*/
void setting_loop() {
  // LOG_INFO("Calibration Mode...\n");
}

/* HandleRoot
*/
void sendFileFromSpiffs(AsyncWebServerRequest *request, const char *path, const char *contentType) {
  if (SPIFFS.exists(path)) {
    request->send(SPIFFS, path, contentType);
  } else {
    String message = "File not found: ";
    message += path;
    request->send(404, "text/plain", message);
  }
}

void sendHtmlFromSpiffs(AsyncWebServerRequest *request, const char *path) {
  // 显式声明 UTF-8，避免不同浏览器对中文编码推断不一致导致乱码
  sendFileFromSpiffs(request, path, "text/html; charset=utf-8");
}

void handleRoot(AsyncWebServerRequest *request) {
    apconfig::autoConfirmIfPending();
    sendHtmlFromSpiffs(request, "/web_controller.html");
}

/* HandleCalibrationPage
*/
void handleCalibrationPage(AsyncWebServerRequest *request) {
  apconfig::autoConfirmIfPending();
  sendHtmlFromSpiffs(request, "/calibration.html");
}

/* handleCalibrationData
*/
void handleCalibrationData(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  String *body = appendRequestBodyChunk(request, data, len, index, total);
  if (!body) {
    return;
  }

  // 低电量锁存后：禁止校准动作（避免电压波动导致舵机异常）
  if (isMotionBlocked()) {
    clearRequestBodyChunk(request);
    request->send(200, "application/json", String("{\"status\":\"error\",\"message\":\"") + kLowBatteryUiMessage + "\"}");
    return;
  }

  CalibrationData calibrationData{};
  String parseError;
  const bool valid = parseCalibrationData(*body, calibrationData, parseError);
  clearRequestBodyChunk(request);
  if (!valid) {
    StaticJsonDocument<192> response;
    response["status"] = "error";
    response["message"] = parseError;
    String payload;
    serializeJson(response, payload);
    request->send(400, "application/json", payload);
    return;
  }
  // Parsing and body assembly can overlap the battery task. Re-check the
  // latched fact immediately before issuing any servo movement.
  if (isMotionBlocked()) {
    request->send(200, "application/json", String("{\"status\":\"error\",\"message\":\"") + kLowBatteryUiMessage + "\"}");
    return;
  }

  if (calibrationData.modeChanged) {
    if ((calibrationData.operation == CALIBRATESTART) && (_mode.load() == 0)) {
      _mode.store(1);
      LOG_INFO("Enter Calibration Mode.");
      if (hexapod::Robot) {
        hexapod::Robot->clearOffset();
        hexapod::Robot->calibrationTestAllLeg(test_angle);
      }

      request->send(200, "application/json", "{\"status\":\"success\"}");
    } else if ((calibrationData.operation == CALIBRATESTART_EXISTING) && (_mode.load() == 0)) {
      // 进入校准模式，但不清零现有偏移；舵机转到90°+offset
      _mode.store(1);
      LOG_INFO("Enter Calibration Mode (use existing offsets).");
      if (hexapod::Robot) {
        hexapod::Robot->calibrationTestAllLeg(test_angle);
      }

      request->send(200, "application/json", "{\"status\":\"success\"}");
    } else if ((calibrationData.operation == CALIBRATESAVE) && (_mode.load() == 1)) {
      if (hexapod::Robot) {
        hexapod::Robot->calibrationSave();
        hexapod::Robot->init(_mode.load() == 1, true);
      }
      _mode.store(0);
      LOG_INFO("Leave Calibration Mode.");
      request->send(200, "application/json", "{\"status\":\"success\",\"redirect\":\"/\"}");
    } else {
      request->send(409, "application/json", "{\"status\":\"error\",\"message\":\"Calibration mode transition rejected\"}");
    }
  } else {
    if (_mode.load() == 1) {
      if (hexapod::Robot) {
        hexapod::Robot->calibrationSet(calibrationData);
        hexapod::Robot->calibrationTest(calibrationData.legIndex, calibrationData.partIndex, test_angle);
      }
      request->send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      request->send(409, "application/json", "{\"status\":\"error\",\"message\":\"Calibration mode is not active\"}");
    }
  }
}

/* 查询当前校准状态与偏移 */
void handleCalibrationGet(AsyncWebServerRequest *request) {
  StaticJsonDocument<1024> doc;

  bool exists = SPIFFS.exists(board::kCalibrationFilePath);
  doc["exists"] = exists;

  JsonArray offsets = doc.createNestedArray("offsets");
  // 按机型导出校准数据（四足=4条腿，六足=6条腿）
  for (int i = 0; i < board::kLegCount; i++) {
    JsonArray leg = offsets.createNestedArray();
    for (int j = 0; j < 3; j++) {
      int offset = 0;
      if (hexapod::Robot) {
        hexapod::Robot->calibrationGet(i, j, offset);
      }
      leg.add(offset);
    }
  }

  String responseStr;
  serializeJson(doc, responseStr);
  request->send(200, "application/json", responseStr);
}

/* UI 能力探测：返回机型、电源与表演能力 */
void handleCapsGet(AsyncWebServerRequest *request) {
  DynamicJsonDocument doc(3072);
  JsonObject robot = doc.createNestedObject("robot");
  robot["type"] = board::kRobotType;
  robot["id"] = robotIdentity;
  robot["legCount"] = board::kLegCount;
  robot["hardwareFamily"] = board::kHardwareFamily;
  robot["hardwareRevision"] = board::kHardwareRevision;
  robot["hardwareProfile"] = board::kHardwareProfile;
  JsonObject firmware = doc.createNestedObject("firmware");
  firmware["version"] = NODEHEXA_STRINGIFY(FIRMWARE_VERSION);
  JsonObject protocol = doc.createNestedObject("protocol");
  protocol["uartCurrent"] = 2;
  protocol["uartLegacy"] = true;
  const uart_protocol::Stats uartStats = getUartStatsSnapshot();
  JsonObject uartDiagnostics = protocol.createNestedObject("diagnostics");
  uartDiagnostics["rxFrames"] = uartStats.rxFrames;
  uartDiagnostics["crcErrors"] = uartStats.crcErrors;
  uartDiagnostics["lengthErrors"] = uartStats.lengthErrors;
  uartDiagnostics["timeouts"] = uartStats.timeouts;
  uartDiagnostics["legacyFrames"] = uartStats.legacyFrames;
  uartDiagnostics["queueDrops"] = uartQueueDrops.load();
  JsonObject peripherals = doc.createNestedObject("peripherals");
  JsonObject buzzerCaps = peripherals.createNestedObject("buzzer");
  buzzerCaps["driverAvailable"] = true;
  JsonObject ultrasonicCaps = peripherals.createNestedObject("ultrasonic");
  const ultrasonic::Reading ultrasonicReading = ultrasonic::latest();
  ultrasonicCaps["driverAvailable"] = true;
  ultrasonicCaps["active"] = ultrasonicReading.pending;
  ultrasonicCaps["online"] = ultrasonic::online();
  const external_device::Snapshot external = external_device::snapshot();
  JsonObject externalCaps = peripherals.createNestedObject("externalDevice");
  externalCaps["type"] = external_device::typeToString(external.type);
  externalCaps["online"] = external.online;
  externalCaps["protocolVersion"] = external.protocolVersion;
  if (external.firmwareVersion[0]) externalCaps["firmware"] = external.firmwareVersion;
  const camera_link::Snapshot camera = camera_link::snapshot();
  JsonObject cameraCaps = peripherals.createNestedObject("camera");
  const bool cameraAvailable = camera.available && millis() - camera.lastStatusMs < 6000 && external.online &&
                               external.type == external_device::Type::Camera;
  cameraCaps["available"] = cameraAvailable;
  cameraCaps["binding"] = camera.bindingRequired ? (camera.bindingVerified ? "verified" : "pending") : "legacy_unverified";
  cameraCaps["cameraId"] = camera.cameraId;
  cameraCaps["wifiPhase"] = camera.wifiPhase;
  cameraCaps["wifiError"] = camera.wifiError;
  cameraCaps["provisionResult"] = camera.provisionResult;
  if (cameraAvailable) {
    char streamUrl[160];
    snprintf(streamUrl, sizeof(streamUrl), "http://%s:%u/stream%s%s", camera.ip, camera.streamPort,
             camera.bindingVerified ? "?session=" : "", camera.bindingVerified ? camera.streamToken : "");
    cameraCaps["streamUrl"] = streamUrl;
  }
  JsonArray profiles = cameraCaps.createNestedArray("profiles");
  profiles.add("qvga");
  profiles.add("vga");
  cameraCaps["activeProfile"] = camera.activeProfile;
  cameraCaps["profilePending"] = camera.profilePending;
  cameraCaps["framesOk"] = camera.framesOk;
  cameraCaps["captureFailures"] = camera.captureFailures;
  cameraCaps["encodeFailures"] = camera.encodeFailures;
  JsonObject power = doc.createNestedObject("power");
  power["lowBatteryLatched"] = isLowBatteryLatched();
  power["lowBatteryPolicy"] = power::policyToString(getEffectiveLowBatteryPolicy());
  power["motionBlocked"] = isMotionBlocked();
  const servo_output::OutputState outputState = servo_output::state();
  power["servoPwmReleased"] = outputState.released;
  power["alarmActive"] = buzzer::alarmActive();
  const uint16_t voltageMv = getLatestBatteryVoltageMv();
  power["voltageMv"] = voltageMv;
  power["percentEstimate"] = estimateBatteryPercent(voltageMv);
  power["lowBatteryThresholdMv"] = power_manager::kWarningThresholdMv;

  JsonObject performanceCaps = doc.createNestedObject("performance");
  performanceCaps["freestyle"] = performance::isSupported(performance::Kind::Freestyle);
  performanceCaps["beatsway"] = performance::isSupported(performance::Kind::BeatSway);
  performanceCaps["showtime"] = performance::isSupported(performance::Kind::Showtime);

  JsonObject manualCaps = doc.createNestedObject("manual");
#ifdef ROBOT_MODEL_NODEQUADMINI
  manualCaps["singleLeg"] = false;
#else
  manualCaps["singleLeg"] = true;
#endif

  String responseStr;
  serializeJson(doc, responseStr);
  request->send(200, "application/json", responseStr);
}

static String* appendRequestBodyChunk(AsyncWebServerRequest *request,
                                      const uint8_t *data,
                                      size_t len,
                                      size_t index,
                                      size_t total) {
  if (!request) {
    return nullptr;
  }
  if (total > kMaxHttpRequestBodyBytes) {
    if (index == 0) {
      request->send(413, "application/json",
                    "{\"status\":\"error\",\"message\":\"Request body too large\"}");
    }
    return nullptr;
  }

  if (index == 0 || request->_tempObject == nullptr) {
    clearRequestBodyChunk(request);
    auto *body = new String();
    body->reserve(total);
    request->_tempObject = body;
  }

  auto *body = static_cast<String*>(request->_tempObject);
  if (!body) {
    return nullptr;
  }

  for (size_t i = 0; i < len; ++i) {
    *body += static_cast<char>(data[i]);
  }

  if (index + len < total) {
    return nullptr;
  }

  return body;
}

static void clearRequestBodyChunk(AsyncWebServerRequest *request) {
  if (!request || !request->_tempObject) {
    return;
  }

  auto *body = static_cast<String*>(request->_tempObject);
  delete body;
  request->_tempObject = nullptr;
}

/* HandleNotFound
*/
void handleNotFound(AsyncWebServerRequest *request) {
    request->send_P(404, "text/plain", "File Not Found");
}

void handleMotionPlanner(AsyncWebServerRequest *request) {
  sendHtmlFromSpiffs(request, "/motion_planner.html");
}

/* AP 配置接口处理
*/
void handleApConfigGet(AsyncWebServerRequest *request) {
  apconfig::APConfig cfg = apconfig::getConfig();
  
  // 检查是否包含密码参数（仅用于调试）
  bool includePassword = false;
  if (request->hasParam("includePassword")) {
    includePassword = request->getParam("includePassword")->value() == "true";
  }
  
  StaticJsonDocument<256> json;
  json["status"] = "success";
  json["ssid"] = cfg.ssid;
  json["pending"] = cfg.pending;
  
  if (cfg.pending) {
    // 如果处于 pending 状态，返回待确认的配置（即当前配置）
    json["nextSSID"] = cfg.ssid;
    if (includePassword) {
      json["nextPassword"] = cfg.password;
    }
    json["currentSSID"] = cfg.prevSsid;
  } else {
    // 正常状态，返回当前配置
    if (includePassword) {
      json["password"] = cfg.password;
    }
  }
  
  String response;
  serializeJson(json, response);
  request->send(200, "application/json", response);
}

void handleApConfigPostBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  String *body = appendRequestBodyChunk(request, data, len, index, total);
  if (!body) {
    return;
  }
  
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, *body);
  clearRequestBodyChunk(request);
  
  if (error) {
    request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    Serial.printf("AP Config: JSON parse error: %s\n", error.c_str());
    return;
  }
  
  if (!doc.containsKey("ssid")) {
    request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing ssid field\"}");
    return;
  }
  
  String ssid = doc["ssid"].as<String>();
  String password = doc.containsKey("password") ? doc["password"].as<String>() : "";
  
  // 验证 SSID 长度（ESP32 限制）
  if (ssid.length() == 0 || ssid.length() > 31) {
    request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"SSID length must be 1-31 characters\"}");
    return;
  }
  
  // 验证密码长度（WPA2 要求 8-63 字符，空密码表示开放网络）
  if (password.length() > 0 && (password.length() < 8 || password.length() > 63)) {
    request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Password must be 8-63 characters or empty for open network\"}");
    return;
  }
  
  // 设置新配置
  bool success = apconfig::setNewConfig(ssid, password);
  
  if (success) {
    // Provision only the running AP on camera HELLO. After reboot this becomes
    // the new configuration (or the rolled-back one), never a future network.
    StaticJsonDocument<256> response;
    response["status"] = "success";
    response["message"] = "AP configuration updated, device will reboot in 3 seconds";
    response["pending"] = true;
    response["nextSSID"] = ssid;
    response["nextPassword"] = password;
    
    String responseStr;
    serializeJson(response, responseStr);
    request->send(200, "application/json", responseStr);
    
    Serial.printf("AP Config: New configuration set - SSID: %s, will reboot...\n", ssid.c_str());
    
    // 延迟重启，让 HTTP 响应先发送
    apconfig::requestReboot(3000);
  } else {
    request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to set configuration\"}");
  }
}

void handleCameraProfilePostBody(AsyncWebServerRequest *request,
                                 uint8_t *data,
                                 size_t len,
                                 size_t index,
                                 size_t total) {
  String *body = appendRequestBodyChunk(request, data, len, index, total);
  if (!body) return;
  StaticJsonDocument<128> doc;
  const DeserializationError error = deserializeJson(doc, *body);
  clearRequestBodyChunk(request);
  const char* profile = doc["profile"] | "";
  if (error || !camera_link::isValidProfile(profile)) {
    request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"profile must be qvga or vga\"}");
    return;
  }
  const external_device::Snapshot external = external_device::snapshot();
  if (!external.online || external.type != external_device::Type::Camera) {
    request->send(409, "application/json", "{\"status\":\"error\",\"message\":\"camera offline\"}");
    return;
  }
  if (!sendCameraProfileRequest(profile)) {
    request->send(503, "application/json", "{\"status\":\"error\",\"message\":\"UART unavailable\"}");
    return;
  }
  camera_link::setProfilePending(true);
  request->send(202, "application/json", "{\"status\":\"accepted_pending\"}");
}

void handleUltrasonicGet(AsyncWebServerRequest *request) {
  bool requested = false;
  if (request->hasParam("fresh") && request->getParam("fresh")->value() == "1") {
    requested = ultrasonic::requestMeasurement();
  }
  const ultrasonic::Reading reading = ultrasonic::latest();
  StaticJsonDocument<384> doc;
  doc["status"] = "success";
  doc["valid"] = reading.valid;
  doc["pending"] = reading.pending;
  doc["requestAccepted"] = requested;
  if (reading.valid) {
    doc["distanceMm"] = reading.distanceMm;
    doc["pulseWidthUs"] = reading.pulseWidthUs;
  } else {
    doc["reason"] = reading.measuredAtMs == 0 ? "not_measured" :
                    (reading.pulseWidthUs == 0 ? "timeout" : "pulse_too_short");
  }
  doc["sampleCount"] = reading.sampleCount;
  doc["timeoutCount"] = reading.timeoutCount;
  doc["errorCount"] = reading.errorCount;
  doc["ageMs"] = reading.ageMs;
  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

void handleApConfigConfirm(AsyncWebServerRequest *request) {
  Serial.println("AP Config: Confirm endpoint hit");
  apconfig::confirm();
  
  StaticJsonDocument<128> response;
  response["status"] = "success";
  response["message"] = "AP configuration confirmed";
  
  String responseStr;
  serializeJson(response, responseStr);
  request->send(200, "application/json", responseStr);
  
  Serial.println("AP Config: Configuration confirmed by user");
}

void handleApConfigReset(AsyncWebServerRequest *request) {
  apconfig::resetToDefault();
  
  StaticJsonDocument<128> response;
  response["status"] = "success";
  response["message"] = "AP configuration reset to default, device will reboot";
  
  String responseStr;
  serializeJson(response, responseStr);
  request->send(200, "application/json", responseStr);
  
  Serial.println("AP Config: Reset to default configuration");

  // 延迟重启，确保HTTP响应发出
  apconfig::requestReboot(3000);
  Serial.println("AP Config: Reboot scheduled in 3000 ms");
}

/* 通用设置接口：GET/POST /api/settings
 * - GET: 返回当前设置
 * - POST: 允许更新已支持的 power/motion 设置
 */
void handleSettingsGet(AsyncWebServerRequest *request) {
  StaticJsonDocument<320> json;
  json["status"] = "success";
  JsonObject power = json.createNestedObject("power");
  const power::LowBatteryPolicy configuredPolicy = devsettings::getLowBatteryPolicy();
  const power::LowBatteryPolicy effectivePolicy = getEffectiveLowBatteryPolicy();
  power["lowBatteryPolicy"] = power::policyToString(configuredPolicy);
  power["effectiveLowBatteryPolicy"] = power::policyToString(effectivePolicy);
  power["appliesAfterRestart"] = isLowBatteryLatched() && configuredPolicy != effectivePolicy;
  JsonObject motion = json.createNestedObject("motion");
  motion["buttonMode"] = motionButtonModeToString(devsettings::getMotionButtonMode());
  String response;
  serializeJson(json, response);
  request->send(200, "application/json", response);
}

void handleSettingsPostBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  String *body = appendRequestBodyChunk(request, data, len, index, total);
  if (!body) {
    return;
  }

  StaticJsonDocument<384> doc;
  DeserializationError error = deserializeJson(doc, *body);
  clearRequestBodyChunk(request);
  if (error) {
    request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    return;
  }

  const bool hasPowerObject = doc.containsKey("power");
  const bool hasMotionObject = doc.containsKey("motion");
  if (!hasPowerObject && !hasMotionObject) {
    request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid payload: missing power/motion object\"}");
    return;
  }

  bool hasPowerUpdate = false;
  power::LowBatteryPolicy lowBatteryPolicy = devsettings::getLowBatteryPolicy();
  if (hasPowerObject) {
    if (!doc["power"].is<JsonObjectConst>()) {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid payload: power must be an object\"}");
      return;
    }
    JsonObjectConst p = doc["power"].as<JsonObjectConst>();
    if (p.containsKey("lowBatteryPolicy")) {
      if (!power::policyFromString(p["lowBatteryPolicy"].as<const char*>(), lowBatteryPolicy)) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid power.lowBatteryPolicy\"}");
        return;
      }
    } else if (p.containsKey("lowBatteryProtectionEnabled")) {
      lowBatteryPolicy = power::policyFromLegacyProtection(
        p["lowBatteryProtectionEnabled"].as<bool>()
      );
    } else {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid payload: missing power.lowBatteryPolicy\"}");
      return;
    }
    hasPowerUpdate = true;
  }

  bool hasMotionUpdate = false;
  devsettings::MotionButtonMode buttonMode = devsettings::getMotionButtonMode();
  if (hasMotionObject) {
    if (!doc["motion"].is<JsonObjectConst>()) {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid payload: motion must be an object\"}");
      return;
    }
    JsonObjectConst m = doc["motion"].as<JsonObjectConst>();
    if (!m.containsKey("buttonMode") || !parseMotionButtonModeField(m["buttonMode"], buttonMode)) {
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid payload: motion.buttonMode must be continuous or single_cycle\"}");
      return;
    }
    hasMotionUpdate = true;
  }

  if (hasPowerUpdate) {
    if (!devsettings::setLowBatteryPolicy(lowBatteryPolicy)) {
      request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to persist power settings\"}");
      return;
    }
  }

  if (hasMotionUpdate) {
    if (!devsettings::setMotionButtonMode(buttonMode)) {
      request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to persist motion settings\"}");
      return;
    }
  }

  StaticJsonDocument<320> resp;
  resp["status"] = "success";
  JsonObject power = resp.createNestedObject("power");
  const power::LowBatteryPolicy configuredPolicy = devsettings::getLowBatteryPolicy();
  const power::LowBatteryPolicy effectivePolicy = getEffectiveLowBatteryPolicy();
  power["lowBatteryPolicy"] = power::policyToString(configuredPolicy);
  power["effectiveLowBatteryPolicy"] = power::policyToString(effectivePolicy);
  power["appliesAfterRestart"] = isLowBatteryLatched() && configuredPolicy != effectivePolicy;
  JsonObject motion = resp.createNestedObject("motion");
  motion["buttonMode"] = motionButtonModeToString(devsettings::getMotionButtonMode());
  String response;
  serializeJson(resp, response);
  request->send(200, "application/json", response);
}

static BasicCommandResult handleBasicControlCommand(JsonVariantConst json) {
  BasicCommandResult result;
  const bool hasMovement = json.containsKey("movementMode");
  const bool hasSpeed = json.containsKey("speed");
  const bool hasSpeedLevel = json.containsKey("speedLevel");
  const bool hasGait = json.containsKey("gaitMode");
  result.handled = hasMovement || hasSpeed || hasSpeedLevel || hasGait;
  if (!result.handled) {
    result.message = "No valid command field found";
    return result;
  }

  const int movementMode = hasMovement ? json["movementMode"].as<int>() : 0;
  const int speedLevel = hasSpeedLevel ? json["speedLevel"].as<int>() : 0;
  const int gaitMode = hasGait ? json["gaitMode"].as<int>() : 0;
  const uint16_t supportedMovementMask =
    static_cast<uint16_t>((1U << hexapod::MOVEMENT_TOTAL) - 1U);
  if ((hasMovement && (!json["movementMode"].is<int>() || movementMode < 0 ||
                       movementMode > UINT16_MAX ||
                       (static_cast<uint16_t>(movementMode) & ~supportedMovementMask) != 0)) ||
      (hasSpeed && !json["speed"].is<float>()) ||
      (hasSpeedLevel && (!json["speedLevel"].is<int>() ||
                         speedLevel < hexapod::SPEED_SLOWEST ||
                         speedLevel > hexapod::SPEED_FAST)) ||
      (hasGait && (!json["gaitMode"].is<int>() || gaitMode < 0 || gaitMode >= 4))) {
    result.message = "Invalid basic control value";
    return result;
  }

  if (hasMovement) {
    if (singleleg::controller().isActive()) {
      singleleg::controller().stop("[SingleLeg] overridden by movementMode");
    }
    if (performance::controller().isActive()) {
      motion::controller().clear("[Performance] overridden by movementMode");
      performance::controller().clear("[Performance] overridden by movementMode");
    }
    if (xSemaphoreTake(flagMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      result.message = "System busy, command ignored";
      return result;
    }
    flag = movementMode;
    xSemaphoreGive(flagMutex);
  }
  if (hasSpeed && hexapod::Robot) {
    hexapod::Robot->setMovementSpeed(json["speed"].as<float>());
  }
  if (hasSpeedLevel && hexapod::Robot) {
    hexapod::Robot->setMovementSpeedLevel(static_cast<hexapod::SpeedLevel>(speedLevel));
  }
  if (hasGait && hexapod::Robot) {
    hexapod::Robot->setGaitMode(gaitMode);
  }
  result.success = true;
  result.message = "Command applied";
  return result;
}

/* 机器人指令回调处理
*/
void onRobotCmdWebSocketEvent(AsyncWebSocket *server, 
                              AsyncWebSocketClient *client, 
                              AwsEventType type,
                              void *arg, 
                              uint8_t *data, 
                              size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      clearMovementFlag();
      motion::controller().clear("[Motion] websocket disconnected");
      performance::controller().clear("[Performance] websocket disconnected");
      singleleg::controller().stop("[SingleLeg] websocket disconnected");
      break;
    case WS_EVT_DATA:
      AwsFrameInfo *info;
      info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        // 为了支持包含 sequence 数组的高级运动指令，将文档容量从 128 增大，并显式传入长度
        StaticJsonDocument<1024> json;
        DeserializationError err = deserializeJson(json, data, len);
        if (err) {
          Serial.print(F("deserializeJson() failed with code: "));
          Serial.println(err.c_str());
          if (client) {
            StaticJsonDocument<160> ack;
            ack["status"] = "error";
            ack["message"] = "Invalid JSON format";
            String payload;
            serializeJson(ack, payload);
            client->text(payload);
          }
          return;
        }

        // New pages attach the expected robot identity. Old clients remain compatible.
        if (json.containsKey("robotId") &&
            (!json["robotId"].is<const char*>() || strcmp(json["robotId"].as<const char*>(),robotIdentity))) {
          client->text("{\"status\":\"error\",\"code\":\"ROBOT_ID_MISMATCH\",\"message\":\"Robot changed; reconnect and confirm\"}");
          return;
        }
        // 低电量锁存后：屏蔽所有控制指令（包括运动模式/速度/步态/序列）
        if (isMotionBlocked()) {
          motion::controller().clear("[Power] low battery, command ignored");
          performance::controller().clear("[Power] low battery, command ignored");
          singleleg::controller().stop("[Power] low battery, single leg ignored", false);
          clearMovementFlag();
          sendLowBatteryErrorToWebSocket(client);
          return;
        }

        AdvancedCommandResult adv = handleAdvancedMotionCommand(json.as<JsonVariantConst>());
        if (adv.handled) {
          if (!adv.suppressAck) {
            if (adv.success) {
              Serial.println("[WebSocket] Advanced motion command accepted");
            } else {
              Serial.printf("[WebSocket] Advanced command failed: %s\n", adv.message.c_str());
            }
          }
          if (!adv.suppressAck && client) {
            StaticJsonDocument<160> ack;
            ack["status"] = adv.success ? "success" : "error";
            ack["message"] = adv.message;
            if (adv.sequenceId) {
              ack["sequenceId"] = adv.sequenceId;
            }
            String payload;
            serializeJson(ack, payload);
            client->text(payload);
          }
          return;
        }

        const BasicCommandResult basic =
          handleBasicControlCommand(json.as<JsonVariantConst>());
        if (!basic.success && client) {
          StaticJsonDocument<160> ack;
          ack["status"] = "error";
          ack["message"] = basic.message;
          String payload;
          serializeJson(ack, payload);
          client->text(payload);
        }
      }
      break;
    case WS_EVT_PONG:
      break;
    case WS_EVT_ERROR:
      clearMovementFlag();
      motion::controller().clear("[Motion] websocket error");
      performance::controller().clear("[Performance] websocket error");
      singleleg::controller().stop("[SingleLeg] websocket error");
      break;
    default:
      break;
  }
}

/* 解析舵机校准数据
*/
static bool parseCalibrationData(const String& jsonString,
                                 CalibrationData& data,
                                 String& errorMessage) {
  // {"legIndex": 0, "partIndex": 0, "offset": 0}
  StaticJsonDocument<200> doc;
  const DeserializationError error = deserializeJson(doc, jsonString);

  if (error || !doc.is<JsonObject>()) {
    Serial.print(F("deserializeJson() failed: "));
    if (error) {
      Serial.println(error.f_str());
    } else {
      Serial.println("root is not an object");
    }
    errorMessage = "Invalid calibration JSON";
    return false;
  }

  if (!doc["modeChanged"].is<bool>()) {
    errorMessage = "modeChanged must be a boolean";
    return false;
  }

  data.modeChanged = doc["modeChanged"].as<bool>();
  if (data.modeChanged) {
    if (!doc["operation"].is<const char*>()) {
      errorMessage = "operation must be a string";
      return false;
    }
    data.operation = doc["operation"].as<String>();
    if (data.operation != CALIBRATESTART &&
        data.operation != CALIBRATESTART_EXISTING &&
        data.operation != CALIBRATESAVE) {
      errorMessage = "Unsupported calibration operation";
      return false;
    }
  } else {
    if (!doc["legIndex"].is<int>() || !doc["partIndex"].is<int>() ||
        !doc["offset"].is<int>()) {
      errorMessage = "legIndex, partIndex and offset must be integers";
      return false;
    }
    data.legIndex = doc["legIndex"].as<int>();
    data.partIndex = doc["partIndex"].as<int>();
    data.offset = doc["offset"].as<int>();
    if (data.legIndex < 0 || data.legIndex >= board::kLegCount ||
        data.partIndex < 0 || data.partIndex >= 3) {
      errorMessage = "Calibration joint index out of range";
      return false;
    }
  }

  return true;
}

// 电池监测任务
void BatteryMonitorTask(void *pvParameters) {
  const uint8_t SAMPLE_SIZE = 10;
  uint16_t adcReadings[SAMPLE_SIZE] = {0};
  uint8_t readIndex = 0;
  uint32_t adcSum = 0;
  uint8_t actualSampleCount = 0;  // 实际采集的样本数量

  while(1) {
    // 采集ADC值并做移动平均滤波
    if (actualSampleCount < SAMPLE_SIZE) {
      // 采样初期，直接累加
      adcReadings[actualSampleCount] = analogRead(BAT_ADC);
      adcSum += adcReadings[actualSampleCount];
      actualSampleCount++;
    } else {
      // 采样数量达到设定值后，使用移动平均
      adcSum -= adcReadings[readIndex];
      adcReadings[readIndex] = analogRead(BAT_ADC);
      adcSum += adcReadings[readIndex];
      readIndex = (readIndex + 1) % SAMPLE_SIZE;
    }

    uint16_t adcAverage = adcSum / actualSampleCount;
    const uint16_t rawAdc = adcReadings[actualSampleCount < SAMPLE_SIZE ? actualSampleCount - 1 : readIndex == 0 ? SAMPLE_SIZE - 1 : readIndex - 1];

    const uint16_t voltageMv = power_manager::voltageMvFromAdcAverage(adcAverage);
    const float voltage = voltageMv / 1000.0f;
    const bool useMovingThreshold = shouldUseMovingLowBatteryThreshold();
    const power_manager::SampleResult sample = power_manager::observeVoltage(
      voltageMv, useMovingThreshold, devsettings::getLowBatteryPolicy());

    if (sample.newlyLatched) {
      buzzer::setMandatoryLowBatteryAlarm(true);
#ifdef DEBUG_ADC_MONITOR
      Serial.println("WARNING: Low voltage detected! (latched)");
#endif
    }

    #ifdef DEBUG_ADC_MONITOR
    Serial.printf("ADC Debug - Raw: %u, Avg: %u, Samples: %u, Voltage: %.2fV (%umV), Warning: %.2fV, Latch: %.2fV [%s], LowNow: %s, LowCount: %u/%u, Latched: %s, Handled: %s\n",
                  rawAdc,
                  adcAverage,
                  actualSampleCount,
                  voltage,
                  voltageMv,
                  power_manager::kWarningThresholdMv / 1000.0f,
                  sample.thresholdMv / 1000.0f,
                  useMovingThreshold ? "moving" : "standby",
                  sample.belowThreshold ? "yes" : "no",
                  sample.state.consecutiveLowSamples,
                  power_manager::kConsecutiveSamplesToLatch,
                  sample.state.lowBatteryLatched ? "yes" : "no",
                  sample.state.latchHandled ? "yes" : "no");
    #endif

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// LED控制任务
void LEDControllerTask(void *pvParameters) {
  bool lastState = false;

  while(1) {
    const bool currentState = power_manager::snapshot().lowBatteryLatched;

    if(currentState) {
      // 低电压时闪烁
      digitalWrite(BAT_LED, !digitalRead(BAT_LED));
      vTaskDelay(pdMS_TO_TICKS(300));
    } else {
      // 正常时常灭
      if (lastState) {
        digitalWrite(BAT_LED, LOW);
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    lastState = currentState;
  }
}

// 打印欢迎语（用于确认SPIFFS文件系统正常工作）
void printWelcomeMessage() {
  File file = SPIFFS.open("/text.txt");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  
  while(file.available()){
    Serial.write(file.read());
  }
  Serial.println();
  file.close();
}

void clearMovementFlag() {
  if (xSemaphoreTake(flagMutex, portMAX_DELAY) == pdTRUE) {
    flag = 0;
    xSemaphoreGive(flagMutex);
  }
}

static bool isLowBatteryLatched() {
  return power_manager::snapshot().lowBatteryLatched;
}

static bool isMotionBlocked() {
  return isLowBatteryLatched() &&
         power::policyBlocksMotion(getEffectiveLowBatteryPolicy());
}

static power::LowBatteryPolicy getEffectiveLowBatteryPolicy() {
  const power_manager::Snapshot state = power_manager::snapshot();
  if (!state.lowBatteryLatched) {
    return devsettings::getLowBatteryPolicy();
  }
  return state.latchedPolicy;
}

static uint16_t getLatestBatteryVoltageMv() {
  return power_manager::snapshot().voltageMv;
}

static bool shouldUseMovingLowBatteryThreshold() {
  // 校准模式下机器人不应按“运动中压降”放宽阈值，仍按待机/静止逻辑处理。
  if (_mode.load() == 1) {
    return false;
  }

  if (motion::controller().hasActiveAction()) {
    return motion::controller().activeMode() != hexapod::MOVEMENT_STANDBY;
  }

  bool moving = false;
  if (xSemaphoreTake(flagMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (auto m = hexapod::MOVEMENT_FORWARD; m < hexapod::MOVEMENT_TOTAL; m++) {
      if (flag & (1 << m)) {
        moving = true;
        break;
      }
    }
    xSemaphoreGive(flagMutex);
  }
  return moving;
}

static uint8_t estimateBatteryPercent(uint16_t voltageMv) {
  if (voltageMv >= 8300) return 100;
  if (voltageMv >= 8100) return 85;
  if (voltageMv >= 7900) return 70;
  if (voltageMv >= 7700) return 55;
  if (voltageMv >= 7500) return 40;
  if (voltageMv >= 7300) return 25;
  if (voltageMv >= 7200) return 10;
  return 0;
}

static void sendLowBatteryErrorToWebSocket(AsyncWebSocketClient *client) {
  StaticJsonDocument<160> ack;
  ack["status"] = "error";
  ack["code"] = kLowBatteryProtectCode;
  ack["message"] = kLowBatteryUiMessage;
  String payload;
  serializeJson(ack, payload);
  if (client) {
    client->text(payload);
  }
}

static void sendLowBatteryErrorToSerial() {
  StaticJsonDocument<160> ack;
  ack["status"] = "error";
  ack["code"] = kLowBatteryProtectCode;
  ack["message"] = kLowBatteryUiMessage;
  String payload;
  serializeJson(ack, payload);
  sendSerialResponse(payload);
}

static void sendLowBatteryEventToSerial() {
  StaticJsonDocument<320> doc;
  doc["event"] = "lowBattery";
  doc["code"] = kLowBatteryProtectCode;
  doc["policy"] = power::policyToString(getEffectiveLowBatteryPolicy());
  doc["motionBlocked"] = isMotionBlocked();
  doc["servoPwmReleased"] = servo_output::state().released;
  doc["alarmActive"] = buzzer::alarmActive();
  doc["voltageMv"] = getLatestBatteryVoltageMv();
  doc["message"] = kLowBatteryUiMessage;

  String payload;
  serializeJson(doc, payload);
  sendSerialResponse(payload);
  lastLowBatterySerialNotifyMs = millis();
}

static void maybeRepeatLowBatteryEventToSerial() {
  const unsigned long now = millis();
  if (lastLowBatterySerialNotifyMs != 0 &&
      now - lastLowBatterySerialNotifyMs < LOW_BATTERY_SERIAL_REMINDER_INTERVAL_MS) {
    return;
  }

  sendLowBatteryEventToSerial();
  Serial.println("[Power] Low battery serial reminder sent.");
}

static void handleLowBatteryLatchedOnce() {
  if (!power_manager::claimLatchHandling()) {
    return;
  }

  if (isMotionBlocked()) {
    motion::controller().clear("[Power] low battery, force standby");
    performance::controller().clear("[Power] low battery, clear performance");
    singleleg::controller().stop("[Power] low battery, clear single leg", false);
    clearMovementFlag();
  }
  if (power::policyReleasesPwm(getEffectiveLowBatteryPolicy())) {
    const bool released = servo_output::releaseAll();
    lastServoReleaseAttemptMs = millis();
    Serial.printf("[Power] Servo PWM release requested: %s.\n", released ? "ok" : "failed");
  }

  StaticJsonDocument<320> doc;
  doc["event"] = "lowBattery";
  doc["code"] = kLowBatteryProtectCode;
  doc["policy"] = power::policyToString(getEffectiveLowBatteryPolicy());
  doc["motionBlocked"] = isMotionBlocked();
  doc["servoPwmReleased"] = servo_output::state().released;
  doc["alarmActive"] = buzzer::alarmActive();
  doc["voltageMv"] = getLatestBatteryVoltageMv();
  doc["message"] = kLowBatteryUiMessage;

  String payload;
  serializeJson(doc, payload);

  // Avoid allocating and immediately deleting a broadcast buffer when no
  // WebSocket client exists (the bundled server's old cleanup loop is unsafe).
  if (wsRoverCmd.count() > 0) {
    wsRoverCmd.textAll(payload);
  }
  sendLowBatteryEventToSerial();

  Serial.printf("[Power] Low battery latched: policy=%s, motionBlocked=%s.\n",
                power::policyToString(getEffectiveLowBatteryPolicy()),
                isMotionBlocked() ? "true" : "false");
}

static void retryServoReleaseIfNeeded() {
  if (!power::policyReleasesPwm(getEffectiveLowBatteryPolicy()) ||
      servo_output::state().released) {
    return;
  }
  const unsigned long now = millis();
  if (lastServoReleaseAttemptMs != 0 &&
      now - lastServoReleaseAttemptMs < kServoReleaseRetryIntervalMs) {
    return;
  }
  lastServoReleaseAttemptMs = now;
  const bool released = servo_output::releaseAll();
  Serial.printf("[Power] Servo PWM release retry: %s.\n", released ? "ok" : "failed");
}

static const char* motionButtonModeToString(devsettings::MotionButtonMode mode) {
  return mode == devsettings::MotionButtonMode::SingleCycle ? "single_cycle" : "continuous";
}

static bool parseMotionButtonModeField(JsonVariantConst value, devsettings::MotionButtonMode& mode) {
  if (value.is<const char*>()) {
    String lower = value.as<const char*>();
    lower.toLowerCase();
    if (lower == "continuous") {
      mode = devsettings::MotionButtonMode::Continuous;
      return true;
    }
    if (lower == "single_cycle" || lower == "singlecycle") {
      mode = devsettings::MotionButtonMode::SingleCycle;
      return true;
    }
    return false;
  }

  if (value.is<int>()) {
    int raw = value.as<int>();
    if (raw == static_cast<int>(devsettings::MotionButtonMode::Continuous)) {
      mode = devsettings::MotionButtonMode::Continuous;
      return true;
    }
    if (raw == static_cast<int>(devsettings::MotionButtonMode::SingleCycle)) {
      mode = devsettings::MotionButtonMode::SingleCycle;
      return true;
    }
  }

  return false;
}

static void handleSequenceComplete(uint32_t sequenceId) {
  StaticJsonDocument<128> doc;
  doc["event"] = "sequenceComplete";
  doc["sequenceId"] = sequenceId;

  String payload;
  serializeJson(doc, payload);

  sendSerialResponse(payload);
  if (wsRoverCmd.count() > 0) wsRoverCmd.textAll(payload);
  Serial.printf("[MotionController] Sequence %u completed\n", sequenceId);
  performance::controller().onSequenceComplete(sequenceId);
}

struct ModeNameEntry {
  const char* name;
  hexapod::MovementMode mode;
};

static bool parseMovementModeField(JsonVariantConst value, hexapod::MovementMode& mode) {
  if (value.isNull()) {
    return false;
  }

  if (value.is<int>()) {
    int raw = value.as<int>();
    if (raw >= 0 && raw < hexapod::MOVEMENT_TOTAL) {
      mode = static_cast<hexapod::MovementMode>(raw);
      return true;
    }
    for (int i = 0; i < hexapod::MOVEMENT_TOTAL; ++i) {
      if (raw & (1 << i)) {
        mode = static_cast<hexapod::MovementMode>(i);
        return true;
      }
    }
    return false;
  }

  if (value.is<const char*>()) {
    static constexpr ModeNameEntry kModeNames[] = {
      {"standby", hexapod::MOVEMENT_STANDBY},
      {"forward", hexapod::MOVEMENT_FORWARD},
      {"forwardfast", hexapod::MOVEMENT_FORWARDFAST},
      {"forward_fast", hexapod::MOVEMENT_FORWARDFAST},
      {"backward", hexapod::MOVEMENT_BACKWARD},
      {"turnleft", hexapod::MOVEMENT_TURNLEFT},
      {"turn_left", hexapod::MOVEMENT_TURNLEFT},
      {"turnright", hexapod::MOVEMENT_TURNRIGHT},
      {"turn_right", hexapod::MOVEMENT_TURNRIGHT},
      {"shiftleft", hexapod::MOVEMENT_SHIFTLEFT},
      {"shift_left", hexapod::MOVEMENT_SHIFTLEFT},
      {"shiftright", hexapod::MOVEMENT_SHIFTRIGHT},
      {"shift_right", hexapod::MOVEMENT_SHIFTRIGHT},
      {"climb", hexapod::MOVEMENT_CLIMB},
      {"rotatex", hexapod::MOVEMENT_ROTATEX},
      {"rotate_x", hexapod::MOVEMENT_ROTATEX},
      {"rotatey", hexapod::MOVEMENT_ROTATEY},
      {"rotate_y", hexapod::MOVEMENT_ROTATEY},
      {"rotatez", hexapod::MOVEMENT_ROTATEZ},
      {"rotate_z", hexapod::MOVEMENT_ROTATEZ},
      {"twist", hexapod::MOVEMENT_TWIST},
      {"beatsway", hexapod::MOVEMENT_BEATSWAY},
      {"beat_sway", hexapod::MOVEMENT_BEATSWAY},
    };
    String lower = value.as<const char*>();
    lower.toLowerCase();
    for (auto entry : kModeNames) {
      if (lower == entry.name) {
        mode = entry.mode;
        return true;
      }
    }
  }

  return false;
}

static bool parsePerformanceKindField(JsonVariantConst value, performance::Kind& kind) {
  if (value.isNull() || !value.is<const char*>()) {
    return false;
  }

  String lower = value.as<const char*>();
  lower.toLowerCase();
  if (lower == "freestyle") {
    kind = performance::Kind::Freestyle;
    return true;
  }
  if (lower == "beatsway" || lower == "beat_sway") {
    kind = performance::Kind::BeatSway;
    return true;
  }
  if (lower == "showtime") {
    kind = performance::Kind::Showtime;
    return true;
  }
  return false;
}

static bool buildActionFromJson(JsonVariantConst obj, motion::Action& action, String& error) {
  JsonVariantConst modeField = obj["movementMode"];
  if (modeField.isNull()) {
    modeField = obj["mode"];
  }
  if (!parseMovementModeField(modeField, action.mode)) {
    error = "movementMode missing or invalid";
    return false;
  }

  if (obj.containsKey("speedOverride")) {
    action.speed = obj["speedOverride"].as<float>();
  }

  if (obj.containsKey("durationMs")) {
    error = "durationMs is not supported (use cycles/steps/distance/angle)";
    return false;
  }

  if (obj.containsKey("cycles")) {
    action.unit = motion::Unit::Cycles;
    action.value = obj["cycles"].as<float>();
  } else if (obj.containsKey("steps")) {
    action.unit = motion::Unit::Steps;
    action.value = obj["steps"].as<float>();
  } else if (obj.containsKey("distance")) {
    action.unit = motion::Unit::Distance;
    action.value = obj["distance"].as<float>();
  } else if (obj.containsKey("angle")) {
    action.unit = motion::Unit::Angle;
    action.value = obj["angle"].as<float>();
  } else {
    error = "missing cycles/steps/distance/angle";
    return false;
  }

  if (action.value <= 0.0f) {
    error = "value must be positive";
    return false;
  }
  return true;
}

static bool hasActionParameters(JsonVariantConst json) {
  return json.containsKey("cycles")
      || json.containsKey("steps")
      || json.containsKey("distance")
      || json.containsKey("angle");
}

static AdvancedCommandResult handleAdvancedMotionCommand(JsonVariantConst json) {
  AdvancedCommandResult result;

  if (json.containsKey("stop") && json["stop"].as<bool>()) {
    result.handled = true;
    motion::controller().clear("[Motion] stop command");
    performance::controller().clear("[Motion] stop command");
    singleleg::controller().stop("[SingleLeg] stop command");
    clearMovementFlag();
    result.success = true;
    result.message = "Motion stopped";
    return result;
  }

  if (json.containsKey("clearQueue") && json["clearQueue"].as<bool>()) {
    result.handled = true;
    motion::controller().clear("[Motion] queue cleared");
    performance::controller().clear("[Motion] queue cleared");
    singleleg::controller().stop("[SingleLeg] queue cleared");
    result.success = true;
    result.message = "Queue cleared";
    return result;
  }

  if (json.containsKey("singleLeg")) {
    result.handled = true;

    JsonObjectConst singleLeg = json["singleLeg"].as<JsonObjectConst>();
    if (singleLeg.isNull()) {
      result.success = false;
      result.message = "singleLeg must be an object";
      return result;
    }

    String op = singleLeg["op"] | "";
    op.toLowerCase();

    if (op == "start") {
      const int legIndex = singleLeg["legIndex"] | -1;
      if (legIndex < 0 || legIndex >= 6) {
        result.success = false;
        result.message = "invalid leg index";
        return result;
      }

      motion::controller().clear("[SingleLeg] start override");
      performance::controller().clear("[SingleLeg] start override");
      clearMovementFlag();
      singleleg::controller().stop("[SingleLeg] restart");

      String error;
      if (!singleleg::controller().start(static_cast<uint8_t>(legIndex), error)) {
        result.success = false;
        result.message = error;
        return result;
      }

      result.success = true;
      result.message = "single leg control started";
      return result;
    }

    if (op == "input") {
      if (!singleleg::controller().isActive()) {
        result.success = false;
        result.message = "single leg control not active";
        return result;
      }

      singleleg::InputAxes axes;
      axes.lx = singleLeg["lx"] | 0.0f;
      axes.ly = singleLeg["ly"] | 0.0f;
      axes.rz = singleLeg["rz"] | 0.0f;
      singleleg::controller().updateInput(axes);

      result.success = true;
      result.suppressAck = true;
      result.message = "single leg input accepted";
      return result;
    }

    if (op == "stop") {
      singleleg::controller().stop("[SingleLeg] stop command");
      result.success = true;
      result.message = "single leg control stopped";
      return result;
    }

    result.success = false;
    result.message = "unsupported singleLeg operation";
    return result;
  }

  if (json.containsKey("performance")) {
    result.handled = true;
    if (singleleg::controller().isActive()) {
      singleleg::controller().stop("[SingleLeg] overridden by performance");
    }
    performance::Kind kind;
    if (!parsePerformanceKindField(json["performance"], kind)) {
      result.success = false;
      result.message = "unsupported performance";
      return result;
    }

    const bool repeat = json["repeat"] | false;
    const uint32_t requestedSequenceId = json["sequenceId"] | 0U;
    String error;
    uint32_t acceptedSequenceId = 0;
    if (!performance::controller().start(kind, repeat, requestedSequenceId, error, acceptedSequenceId)) {
      result.success = false;
      result.message = error;
      return result;
    }

    clearMovementFlag();
    result.success = true;
    result.sequenceId = acceptedSequenceId;
    result.message = "performance accepted";
    return result;
  }

  if (json.containsKey("sequence")) {
    result.handled = true;
    if (singleleg::controller().isActive()) {
      singleleg::controller().stop("[SingleLeg] overridden by sequence");
    }
    performance::controller().clear("[Performance] overridden by sequence");
    JsonArrayConst seq = json["sequence"].as<JsonArrayConst>();
    if (seq.isNull() || seq.size() == 0 || seq.size() > 5) {
      result.success = false;
      result.message = "sequence size must be 1-5";
      return result;
    }

    bool append = json["append"] | false;
    uint32_t seqId = json["sequenceId"] | (uint32_t)millis();
    motion::Action actions[5];
    for (size_t i = 0; i < seq.size(); ++i) {
      String err;
      if (!buildActionFromJson(seq[i], actions[i], err)) {
        result.success = false;
        result.message = err;
        return result;
      }
      actions[i].sequenceId = seqId;
      actions[i].sequenceTail = (i == seq.size() - 1);
    }

    if (!append) {
      motion::controller().clear("[Motion] sequence override");
    }

    if (!motion::controller().enqueueSequence(actions, seq.size())) {
      result.success = false;
      result.message = "queue full";
      return result;
    }

    clearMovementFlag();
    result.success = true;
    result.sequenceId = seqId;
    result.message = "sequence accepted";
    return result;
  }

  if (hasActionParameters(json)) {
    result.handled = true;
    if (singleleg::controller().isActive()) {
      singleleg::controller().stop("[SingleLeg] overridden by single action");
    }
    performance::controller().clear("[Performance] overridden by single action");
    motion::Action action;
    String error;
    if (!buildActionFromJson(json, action, error)) {
      result.success = false;
      result.message = error;
      return result;
    }

    if (json.containsKey("sequenceId")) {
      action.sequenceId = json["sequenceId"].as<uint32_t>();
      action.sequenceTail = true;
    }

    bool append = json["append"] | false;
    if (!append) {
      motion::controller().clear("[Motion] single action override");
    }

    if (!motion::controller().enqueue(action)) {
      result.success = false;
      result.message = "queue full";
      return result;
    }

    clearMovementFlag();
    result.success = true;
    result.sequenceId = action.sequenceId;
    result.message = "action accepted";
    return result;
  }

  return result;
}


/* 解析串口运动指令
*/
void parseSerialMovementCommand(const String& jsonString) {
  // The protocol payload limit is 512 bytes; leave room for ArduinoJson's
  // object/array metadata and duplicated strings.
  StaticJsonDocument<1024> json;
  DeserializationError err = deserializeJson(json, jsonString);
  
  if (err) {
    Serial.print(F("Serial deserializeJson() failed with code: "));
    Serial.println(err.c_str());
    // 发送错误响应
    sendSerialResponse("{\"status\":\"error\",\"message\":\"Invalid JSON format\"}");
    return;
  }

  // 低电量锁存后：屏蔽所有控制指令（包括运动模式/速度/步态/序列）
  if (isMotionBlocked()) {
    motion::controller().clear("[Power] low battery, command ignored");
    performance::controller().clear("[Power] low battery, command ignored");
    singleleg::controller().stop("[Power] low battery, single leg ignored", false);
    clearMovementFlag();
    sendLowBatteryErrorToSerial();
    return;
  }

  AdvancedCommandResult adv = handleAdvancedMotionCommand(json.as<JsonVariantConst>());
  if (adv.handled) {
    if (!adv.suppressAck) {
      StaticJsonDocument<192> response;
      response["status"] = adv.success ? "success" : "error";
      response["message"] = adv.message;
      if (adv.sequenceId) {
        response["sequenceId"] = adv.sequenceId;
      }
      String payload;
      serializeJson(response, payload);
      sendSerialResponse(payload);
    }
    return;
  }

  const BasicCommandResult basic = handleBasicControlCommand(json.as<JsonVariantConst>());
  StaticJsonDocument<192> response;
  response["status"] = basic.success ? "success" : "error";
  response["message"] = basic.message;
  if (json.containsKey("movementMode")) response["movementMode"] = json["movementMode"];
  if (json.containsKey("speedLevel")) response["speedLevel"] = json["speedLevel"];
  if (hexapod::Robot && (json.containsKey("speed") || json.containsKey("speedLevel"))) {
    response["speed"] = hexapod::Robot->getMovementSpeed();
  }
  String responseStr;
  serializeJson(response, responseStr);
  sendSerialResponse(responseStr);
}

/* 串口指令处理任务
*/
void SerialCommandTask(void *pvParameters) {
  Serial.println("UartRx task started - waiting for UART2 data...");
  uart_protocol::Frame frame;

  while(1) {
    while (Serial2.available()) {
      const uint8_t byte = static_cast<uint8_t>(Serial2.read());
      if (!uartParser.feed(byte, millis(), frame)) {
        continue;
      }

      if (xQueueSend(uartFrameQueue, &frame, 0) != pdTRUE) {
        uartQueueDrops.fetch_add(1);
      }
    }

    uartParser.pollTimeout(millis());
    publishUartStats();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void UartDispatchTask(void *pvParameters) {
  Serial.println("UartDispatch task started.");
  uartDispatchTaskHandle = xTaskGetCurrentTaskHandle();
  uart_protocol::Frame frame;

  while (1) {
    if (xQueueReceive(uartFrameQueue, &frame, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    serialDispatchV2 = frame.format == uart_protocol::Format::V2;
    serialDispatchSequence = frame.sequence;
    if (serialDispatchV2) {
      external_device::observeV2Frame(millis());
    } else {
      external_device::observeLegacyFrame(millis());
    }

    if (frame.format == uart_protocol::Format::V2 &&
        frame.messageType == uart_protocol::MessageType::Hello) {
      StaticJsonDocument<512> hello;
      StaticJsonDocument<768> peerHello;
      const DeserializationError helloError = deserializeJson(peerHello, frame.payload, frame.payloadLength);
      const char* peerDevice = helloError ? "generic_host" : (peerHello["device"] | "generic_host");
      external_device::observeHello(peerDevice,
                                    helloError ? "" : (peerHello["deviceId"] | ""),
                                    helloError ? "" : (peerHello["firmware"] | ""),
                                    2,
                                    millis());
      hello["device"] = "nodehexa";
      hello["robotType"] = board::kRobotType;
      hello["firmware"] = NODEHEXA_STRINGIFY(FIRMWARE_VERSION);
      hello["protocol"] = 2;
      JsonObject helloPower = hello.createNestedObject("power");
      helloPower["lowBatteryLatched"] = isLowBatteryLatched();
      JsonArray capabilities = hello.createNestedArray("capabilities");
      capabilities.add("control");
      capabilities.add("power_event");
      capabilities.add("camera_provision");
      capabilities.add("camera_binding_v1");
      String response;
      serializeJson(hello, response);
      sendSerialResponse(response);
      if (strcmp(peerDevice, "camera") == 0) {
        bool supportsBinding = false;
        for (JsonVariantConst capability : peerHello["capabilities"].as<JsonArrayConst>())
          if (capability.is<const char*>() && strcmp(capability.as<const char*>(),"camera_binding_v1") == 0)
            supportsBinding = true;
        camera_link::expectBinding(supportsBinding,peerHello["deviceId"] | "",
                                   robotIdentity,robotApBssid,cameraSession);
        sendCameraProvision();
      }
    } else if (frame.format == uart_protocol::Format::Legacy ||
               frame.messageType == uart_protocol::MessageType::Request) {
      parseSerialMovementCommand(reinterpret_cast<const char*>(frame.payload));
    } else if (frame.messageType == uart_protocol::MessageType::Heartbeat) {
      StaticJsonDocument<128> heartbeat;
      heartbeat["status"] = "success";
      JsonObject heartbeatPower = heartbeat.createNestedObject("power");
      heartbeatPower["lowBatteryLatched"] = isLowBatteryLatched();
      String response;
      serializeJson(heartbeat, response);
      sendSerialResponse(response);
    } else if (frame.messageType == uart_protocol::MessageType::Event) {
      handleV2Event(frame);
    } else if (frame.messageType == uart_protocol::MessageType::Response) {
      StaticJsonDocument<256> response;
      if (!deserializeJson(response,frame.payload,frame.payloadLength))
        camera_link::provisionResponse(frame.sequence,response["status"] | "error");
    }
    serialDispatchV2 = false;
    serialDispatchSequence = 0;
  }
}

static void publishUartStats() {
  const uart_protocol::Stats snapshot = uartParser.stats();
  portENTER_CRITICAL(&uartStatsMux);
  uartStatsSnapshot = snapshot;
  portEXIT_CRITICAL(&uartStatsMux);
}

static uart_protocol::Stats getUartStatsSnapshot() {
  uart_protocol::Stats snapshot{};
  portENTER_CRITICAL(&uartStatsMux);
  snapshot = uartStatsSnapshot;
  portEXIT_CRITICAL(&uartStatsMux);
  return snapshot;
}

/* 发送串口响应
*/
void sendSerialResponse(const String& message) {
  const bool inDispatchTask = uartDispatchTaskHandle &&
                              xTaskGetCurrentTaskHandle() == uartDispatchTaskHandle;
  const external_device::Snapshot external = external_device::snapshot();
  const bool useV2 = (inDispatchTask && serialDispatchV2) ||
                     (!inDispatchTask && external.online && external.protocolVersion == 2);

  if (useV2) {
    const uart_protocol::MessageType type = inDispatchTask
      ? uart_protocol::MessageType::Response
      : uart_protocol::MessageType::Event;
    sendV2Message(type, inDispatchTask ? 0x01 : 0x00,
                  inDispatchTask ? serialDispatchSequence : 0, message);
  } else {
    if (uartTxMutex && xSemaphoreTake(uartTxMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
      Serial.println("[UART] TX mutex timeout; response dropped.");
      return;
    }
    Serial2.print('$');
    Serial2.println(message);
    if (uartTxMutex) xSemaphoreGive(uartTxMutex);
  }
}

static bool sendV2Message(uart_protocol::MessageType type,
                          uint8_t flags,
                          uint16_t sequence,
                          const String& message) {
  if (message.length() > uart_protocol::kMaxPayloadLength) return false;
  if (uartTxMutex && xSemaphoreTake(uartTxMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  uint8_t encoded[uart_protocol::kMaxPayloadLength + 11];
  const size_t encodedLength = uart_protocol::encodeV2(
    type, flags, sequence, reinterpret_cast<const uint8_t*>(message.c_str()),
    static_cast<uint16_t>(message.length()), encoded, sizeof(encoded));
  if (encodedLength > 0) Serial2.write(encoded, encodedLength);
  if (uartTxMutex) xSemaphoreGive(uartTxMutex);
  return encodedLength > 0;
}

static uint16_t nextUartSequence() {
  portENTER_CRITICAL(&uartSequenceMux);
  uint16_t sequence = uartNextSequence++;
  if (uartNextSequence == 0) uartNextSequence = 1;
  portEXIT_CRITICAL(&uartSequenceMux);
  return sequence;
}

static bool sendCameraProvision() {
  const external_device::Snapshot external = external_device::snapshot();
  if (!external.online || external.type != external_device::Type::Camera ||
      external.protocolVersion != 2) return false;
  wifi_config_t runtime{};
  if (esp_wifi_get_config(WIFI_IF_AP,&runtime) != ESP_OK) return false;
  char ssid[33]{}, password[65]{};
  memcpy(ssid,runtime.ap.ssid,sizeof(runtime.ap.ssid));
  memcpy(password,runtime.ap.password,sizeof(runtime.ap.password));
  StaticJsonDocument<640> doc;
  JsonObject provision = doc.createNestedObject("cameraProvision");
  provision["ssid"] = ssid;
  provision["password"] = password;
  provision["streamPort"] = 81;
  if (camera_link::snapshot().bindingRequired) {
    if (!robotIdentity[0] || !robotApBssid[0]) return false;
    provision["bssid"] = robotApBssid;
    provision["robotId"] = robotIdentity;
    provision["session"] = cameraSession;
  }
  String payload;
  serializeJson(doc, payload);
  if (doc.overflowed() || payload.length() > uart_protocol::kMaxPayloadLength) return false;
  const uint16_t sequence = nextUartSequence();
  camera_link::provisionSent(sequence);
  return sendV2Message(uart_protocol::MessageType::Request, 0x02,
                       sequence, payload);
}

static bool sendCameraProfileRequest(const char* profile) {
  StaticJsonDocument<128> doc;
  JsonObject camera = doc.createNestedObject("camera");
  camera["op"] = "setProfile";
  camera["profile"] = profile;
  String payload;
  serializeJson(doc, payload);
  return sendV2Message(uart_protocol::MessageType::Request, 0x02,
                       nextUartSequence(), payload);
}

static void handleV2Event(const uart_protocol::Frame& frame) {
  StaticJsonDocument<1280> doc;
  if (deserializeJson(doc, frame.payload, frame.payloadLength)) return;
  const char* event = doc["event"] | "";
  if (strcmp(event, "cameraStatus") != 0) return;
  const bool bindingOk = camera_link::checkBinding(doc["bound"] | false,
                            doc["cameraId"] | "", doc["robotId"] | "",
                            doc["bssid"] | "", doc["session"] | "",
                            doc["streamToken"] | "", doc["wifiPhase"] | "", doc["wifiError"] | 0);
  camera_link::updateStatus(bindingOk && (doc["online"] | false),
                            doc["ip"] | "",
                            doc["streamPort"] | 81,
                            doc["framesize"] | "qvga",
                            doc["sensor"] | "",
                            doc["framesOk"] | 0,
                            doc["captureFailures"] | 0,
                            doc["encodeFailures"] | 0,
                            millis());
}
