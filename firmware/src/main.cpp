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

#include "debug.h"
#include "hexapod.h"
#include "config.h"
#include "calibration.h"
#include "PinDefines.h"
#include "ap_config.h"
#include "motion_controller.h"

// 宏定义
#define REACT_DELAY hexapod::config::movementInterval
#define CALIBRATESTART "CALIBRATESTART"
#define CALIBRATESAVE "CALIBRATESAVE"
#define CALIBRATESTART_EXISTING "CALIBRATESTART_EXISTING"

// 调试模式控制
// #define DEBUG_ADC_MONITOR  // 注释此行可关闭电池电压ADC调试输出
// #define DEBUG_FRAME_RECEIVE  // 启用帧接收调试输出

// 常量定义

// 串口配置
#define UART2_BAUD_RATE 115200

// 静态变量
static int8_t _mode = 0;  // 六足工作模式：0-运动模式 1-校准模式
static int16_t flag = 0;   // 六足运动模式: 见枚举hexapod:MovementMode
static float test_angle = 0.;

// flag访问保护
SemaphoreHandle_t flagMutex;

// 串口通讯相关变量
static String serialBuffer = "";  // 串口数据接收缓冲区
static const unsigned long SERIAL_TIMEOUT = 1000;  // 串口数据超时时间(ms)
static unsigned long lastSerialDataTime = 0;  // 上次接收串口数据的时间
static bool frameStarted = false;  // 帧接收状态：false-等待起始符$，true-正在接收数据

// 电池监测相关变量
static const float LOW_VOLTAGE_THRESHOLD = 6.4f; // 低电压阈值(V)
static const uint16_t ADC_THRESHOLD = (uint16_t)(LOW_VOLTAGE_THRESHOLD * 47.0f / (100.0f + 47.0f) / 3.3f * 4095.0f); // ADC阈值
static bool lowVoltageFlag = false;
SemaphoreHandle_t voltageMutex;

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
void onRobotCmdWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void normal_loop();
void setting_loop();
static void log_output(const char* log);
CalibrationData parseCalibrationData(const String& jsonString);
void printWelcomeMessage();

// AP 配置接口声明
void handleApConfigGet(AsyncWebServerRequest *request);
void handleApConfigPostBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleApConfigConfirm(AsyncWebServerRequest *request);
void handleApConfigReset(AsyncWebServerRequest *request);

void BatteryMonitorTask(void *pvParameters);
void LEDControllerTask(void *pvParameters);

// 串口通讯相关函数声明
void parseSerialMovementCommand(const String& jsonString);
void SerialCommandTask(void *pvParameters);
void sendSerialResponse(const String& message);
void testUART2Connection();
void clearMovementFlag();

struct AdvancedCommandResult {
  bool handled = false;
  bool success = false;
  uint32_t sequenceId = 0;
  String message;
};

static void handleSequenceComplete(uint32_t sequenceId);
static AdvancedCommandResult handleAdvancedMotionCommand(JsonVariantConst json);
static bool parseMovementModeField(JsonVariantConst value, hexapod::MovementMode& mode);
static bool buildActionFromJson(JsonVariantConst obj, motion::Action& action, String& error);

void setup() {
  // 初始化串口
  Serial.begin(115200);
  Serial.println("Starting...");

  // 初始化UART2用于接收运动指令
  Serial2.begin(UART2_BAUD_RATE, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17 (默认引脚)
  Serial2.setTimeout(100);
  Serial2.flush(); // 清空串口缓冲区
  Serial.printf("UART2 initialized: %d baud (GPIO16-RX, GPIO17-TX)\n", UART2_BAUD_RATE);

  // 初始化电池监测相关硬件
  pinMode(BAT_ADC, INPUT);
  pinMode(BAT_LED, OUTPUT);
  voltageMutex = xSemaphoreCreateMutex();
  flagMutex = xSemaphoreCreateMutex();

  // 初始化I2C
  Wire.setPins(21, 22);

  // 挂载SPIFFS文件系统
  if (!SPIFFS.begin(true)) {
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
  }

  // 初始化WiFi（动态 AP 配置）
  apconfig::init();
  apconfig::printCurrentAPInfo(Serial);

  // 初始化Web服务
  server.on("/", HTTP_GET, handleRoot);
  server.on("/planner", HTTP_GET, handleMotionPlanner);
  server.on("/planner.html", HTTP_GET, handleMotionPlanner);
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

  wsRoverCmd.onEvent(onRobotCmdWebSocketEvent);
  server.addHandler(&wsRoverCmd);

  server.begin();
  Serial.println("HTTP server started");

  // 初始化日志记录回调函数&机器人工作模式
  hexapod::initLogOutput(log_output, millis);
  hexapod::Hexapod.init(_mode == 1);
  motion::controller().begin();
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

  // 创建串口指令处理任务
  xTaskCreate(
    SerialCommandTask,
    "SerialCommand",
    4096,
    NULL,
    2,  // 提高优先级到2，确保串口数据及时处理
    NULL
  );

  printWelcomeMessage();

  // 测试UART2连接
  testUART2Connection();

  Serial.print("Started, mode=");
  Serial.println(_mode);
}

void loop() {
  if (_mode == 0) {
    normal_loop();
  }
  else if (_mode == 1) {
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

  auto t0 = millis();

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

  hexapod::Hexapod.processMovement(mode, REACT_DELAY);
  motion::controller().onLoopTick(mode, REACT_DELAY);

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
void sendHtmlFromSpiffs(AsyncWebServerRequest *request, const char *path) {
  if (SPIFFS.exists(path)) {
    request->send(SPIFFS, path, "text/html");
  } else {
    String message = "File not found: ";
    message += path;
    request->send(404, "text/plain", message);
  }
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
  String body = String((char*)data).substring(0, len);

  CalibrationData calibrationData = parseCalibrationData(body);
  if (calibrationData.modeChanged) {
    if ((calibrationData.operation == CALIBRATESTART) && (_mode == 0)) {
      _mode = 1;
      LOG_INFO("Enter Calibration Mode.");
      hexapod::Hexapod.clearOffset();
      hexapod::Hexapod.calibrationTestAllLeg(test_angle);

      request->send(200, "application/json", "{\"status\":\"success\"}");
    } else if ((calibrationData.operation == CALIBRATESTART_EXISTING) && (_mode == 0)) {
      // 进入校准模式，但不清零现有偏移；舵机转到90°+offset
      _mode = 1;
      LOG_INFO("Enter Calibration Mode (use existing offsets).");
      hexapod::Hexapod.calibrationTestAllLeg(test_angle);

      request->send(200, "application/json", "{\"status\":\"success\"}");
    } else if ((calibrationData.operation == CALIBRATESAVE) && (_mode == 1)) {
      hexapod::Hexapod.calibrationSave();
      hexapod::Hexapod.init(_mode == 1, true);
      _mode = 0;
      LOG_INFO("Leave Calibration Mode.");

      sendHtmlFromSpiffs(request, "/web_controller.html");
    }
  } else {
    if (_mode == 1) {
      hexapod::Hexapod.calibrationSet(calibrationData);
      hexapod::Hexapod.calibrationTest(calibrationData.legIndex, calibrationData.partIndex, test_angle);
      request->send(200, "application/json", "{\"status\":\"success\"}");
    }
  }
}

/* 查询当前校准状态与偏移 */
void handleCalibrationGet(AsyncWebServerRequest *request) {
  StaticJsonDocument<1024> doc;

  bool exists = SPIFFS.exists("/calibration.json");
  doc["exists"] = exists;

  JsonArray offsets = doc.createNestedArray("offsets");
  for (int i = 0; i < 6; i++) {
    JsonArray leg = offsets.createNestedArray();
    for (int j = 0; j < 3; j++) {
      int offset = 0;
      hexapod::Hexapod.calibrationGet(i, j, offset);
      leg.add(offset);
    }
  }

  String responseStr;
  serializeJson(doc, responseStr);
  request->send(200, "application/json", responseStr);
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
  String body = String((char*)data).substring(0, len);
  
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, body);
  
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
  if (password.length() > 0 && password.length() < 8) {
    request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Password must be at least 8 characters or empty for open network\"}");
    return;
  }
  
  // 设置新配置
  bool success = apconfig::setNewConfig(ssid, password);
  
  if (success) {
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
      // 使用短超时时间获取锁
      if (xSemaphoreTake(flagMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        flag = 0;  
        xSemaphoreGive(flagMutex);
      }
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
          return;
        }
        
        AdvancedCommandResult adv = handleAdvancedMotionCommand(json.as<JsonVariantConst>());
        if (adv.handled) {
          StaticJsonDocument<160> ack;
          ack["status"] = adv.success ? "success" : "error";
          ack["message"] = adv.message;
          if (adv.sequenceId) {
            ack["sequenceId"] = adv.sequenceId;
          }
          String payload;
          serializeJson(ack, payload);
          if (adv.success) {
            Serial.println("[WebSocket] Advanced motion command accepted");
          } else {
            Serial.printf("[WebSocket] Advanced command failed: %s\n", adv.message.c_str());
          }
          if (client) {
            client->text(payload);
          }
          return;
        }

        if (json.containsKey("movementMode")) {
          int16_t movementMode = json["movementMode"];

          // 使用短超时时间获取锁
          if (xSemaphoreTake(flagMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (flag != movementMode) {
              flag = movementMode; 
              Serial.printf("Receive Movement Command Flag: %d\n", movementMode);
            }
            xSemaphoreGive(flagMutex);
          } else {
            Serial.println("WebSocket: Failed to acquire flag lock, command ignored");
          }
        }
        
        // Handle speed control
        if (json.containsKey("speed")) {
          float speed = json["speed"];
          hexapod::Hexapod.setMovementSpeed(speed);
          Serial.printf("WebSocket: Speed set to %.2f\n", speed);
        }
        
        // Handle speed level control
        if (json.containsKey("speedLevel")) {
          int level = json["speedLevel"];
          if (level >= hexapod::SPEED_SLOWEST && level <= hexapod::SPEED_FAST) {
            hexapod::Hexapod.setMovementSpeedLevel((hexapod::SpeedLevel)level);
            Serial.printf("WebSocket: Speed level set to %d\n", level);
          } else {
            Serial.printf("WebSocket: Invalid speed level %d\n", level);
          }
        }
      }
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
    default:
      break;
  }
}

/* 解析舵机校准数据
*/
CalibrationData parseCalibrationData(const String& jsonString) {
  // {"legIndex": 0, "partIndex": 0, "offset": 0}

  CalibrationData data;

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return data; // 返回默认数据
  }

  // 赋值给结构体
  data.modeChanged = doc["modeChanged"];
  if (!data.modeChanged) {
    data.legIndex = doc["legIndex"];
    data.partIndex = doc["partIndex"];
    data.offset = doc["offset"];
  } else {
    data.operation = doc["operation"].as<String>();
  }

  return data;
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

    // 打印ADC调试信息
    #ifdef DEBUG_ADC_MONITOR
    // 计算实际电压值 (V)
    float voltage = (float)adcAverage * 3.3f / 4095.0f * (100.0f + 47.0f) / 47.0f;
    Serial.printf("ADC Debug - Raw: %d, Average: %d, SampleCount: %d, Voltage: %.2fV, Threshold: %.2fV\n", 
                  adcReadings[actualSampleCount < SAMPLE_SIZE ? actualSampleCount - 1 : readIndex == 0 ? SAMPLE_SIZE - 1 : readIndex - 1], 
                  adcAverage, 
                  actualSampleCount,
                  voltage, 
                  LOW_VOLTAGE_THRESHOLD);
    #endif

    bool newFlag = (adcAverage < ADC_THRESHOLD);
    
    xSemaphoreTake(voltageMutex, portMAX_DELAY);
    if(lowVoltageFlag != newFlag) {
      lowVoltageFlag = newFlag;
      
      // 打印电压状态变化
      #ifdef DEBUG_ADC_MONITOR
      if(newFlag) {
        Serial.println("WARNING: Low voltage detected!");
      } else {
        Serial.println("Voltage returned to normal level.");
      }
      #endif
    }
    xSemaphoreGive(voltageMutex);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// LED控制任务
void LEDControllerTask(void *pvParameters) {
  bool lastState = false;

  while(1) {
    // 读取电压标志（使用"共享变量+互斥锁"这种FreeRTOS中常见的线程安全通信方式）
    xSemaphoreTake(voltageMutex, portMAX_DELAY);
    bool currentState = lowVoltageFlag;
    xSemaphoreGive(voltageMutex);

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

static void handleSequenceComplete(uint32_t sequenceId) {
  StaticJsonDocument<128> doc;
  doc["event"] = "sequenceComplete";
  doc["sequenceId"] = sequenceId;

  String payload;
  serializeJson(doc, payload);

  sendSerialResponse(payload);
  wsRoverCmd.textAll(payload);
  Serial.printf("[MotionController] Sequence %u completed\n", sequenceId);
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
    action.unit = motion::Unit::DurationMs;
    action.durationMs = obj["durationMs"].as<uint32_t>();
    action.value = action.durationMs;
    return action.durationMs > 0;
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
    error = "missing duration/cycles/steps/distance/angle";
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
      || json.containsKey("angle")
      || json.containsKey("durationMs");
}

static AdvancedCommandResult handleAdvancedMotionCommand(JsonVariantConst json) {
  AdvancedCommandResult result;

  if (json.containsKey("stop") && json["stop"].as<bool>()) {
    result.handled = true;
    motion::controller().clear("[Motion] stop command");
    clearMovementFlag();
    result.success = true;
    result.message = "Motion stopped";
    return result;
  }

  if (json.containsKey("clearQueue") && json["clearQueue"].as<bool>()) {
    result.handled = true;
    motion::controller().clear("[Motion] queue cleared");
    result.success = true;
    result.message = "Queue cleared";
    return result;
  }

  if (json.containsKey("sequence")) {
    result.handled = true;
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
  StaticJsonDocument<512> json;
  DeserializationError err = deserializeJson(json, jsonString);
  
  if (err) {
    Serial.print(F("Serial deserializeJson() failed with code: "));
    Serial.println(err.c_str());
    // 发送错误响应
    sendSerialResponse("{\"status\":\"error\",\"message\":\"Invalid JSON format\"}");
    return;
  }
  
  AdvancedCommandResult adv = handleAdvancedMotionCommand(json.as<JsonVariantConst>());
  if (adv.handled) {
    StaticJsonDocument<192> response;
    response["status"] = adv.success ? "success" : "error";
    response["message"] = adv.message;
    if (adv.sequenceId) {
      response["sequenceId"] = adv.sequenceId;
    }
    String payload;
    serializeJson(response, payload);
    sendSerialResponse(payload);
    return;
  }

  bool hasValidCommand = false;

  // 检查是否包含movementMode字段
  if (json.containsKey("movementMode")) {
    hasValidCommand = true;
    int16_t movementMode = json["movementMode"];
    
    // 验证运动模式范围
    // if (movementMode >= 0 && movementMode < hexapod::MOVEMENT_TOTAL) {
    if (movementMode >= 0) {
      // 使用短超时时间获取锁，避免长时间等待
      if (xSemaphoreTake(flagMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (flag != movementMode) {
          flag = movementMode;
          Serial.printf("UART2: Receive Movement Command Flag: %d\n", movementMode);
          
          // 发送成功响应
          StaticJsonDocument<128> response;
          response["status"] = "success";
          response["movementMode"] = movementMode;
          response["message"] = "Movement command executed";
          
          String responseStr;
          serializeJson(response, responseStr);
          sendSerialResponse(responseStr);
        } else {
          // 发送重复指令响应
          StaticJsonDocument<128> response;
          response["status"] = "success";
          response["movementMode"] = movementMode;
          response["message"] = "Movement mode already set";
          
          String responseStr;
          serializeJson(response, responseStr);
          sendSerialResponse(responseStr);
        }
        xSemaphoreGive(flagMutex);
      } else {
        Serial.println("UART2: Failed to acquire flag lock, command ignored");
        sendSerialResponse("{\"status\":\"error\",\"message\":\"System busy, command ignored\"}");
      }
    } else {
      sendSerialResponse("{\"status\":\"error\",\"message\":\"Invalid movement mode\"}");
    }
  }
  
  // Handle speed control
  if (json.containsKey("speed")) {
    hasValidCommand = true;
    float speed = json["speed"];
    hexapod::Hexapod.setMovementSpeed(speed);
    Serial.printf("UART2: Speed set to %.2f\n", speed);
    
    StaticJsonDocument<128> response;
    response["status"] = "success";
    response["speed"] = hexapod::Hexapod.getMovementSpeed();
    response["message"] = "Speed updated";
    
    String responseStr;
    serializeJson(response, responseStr);
    sendSerialResponse(responseStr);
  }
  
  // Handle speed level control
  if (json.containsKey("speedLevel")) {
    hasValidCommand = true;
    int level = json["speedLevel"];
    if (level >= hexapod::SPEED_SLOWEST && level <= hexapod::SPEED_FAST) {
      hexapod::Hexapod.setMovementSpeedLevel((hexapod::SpeedLevel)level);
      Serial.printf("UART2: Speed level set to %d\n", level);
      
      StaticJsonDocument<128> response;
      response["status"] = "success";
      response["speedLevel"] = level;
      response["speed"] = hexapod::Hexapod.getMovementSpeed();
      response["message"] = "Speed level updated";
      
      String responseStr;
      serializeJson(response, responseStr);
      sendSerialResponse(responseStr);
    } else {
      sendSerialResponse("{\"status\":\"error\",\"message\":\"Invalid speed level\"}");
    }
  }
  
  if (!hasValidCommand) {
    // 如果不是有效的指令格式，发送错误响应
    sendSerialResponse("{\"status\":\"error\",\"message\":\"No valid command field found\"}");
  }
}

/* 串口指令处理任务
*/
void SerialCommandTask(void *pvParameters) {
  Serial.println("SerialCommandTask started - waiting for UART2 data...");
  
  // 初始化串口缓冲区状态
  serialBuffer = "";
  frameStarted = false;
  lastSerialDataTime = 0;
  
  while(1) {
    // 检查UART2是否有可用数据
    while (Serial2.available()) {
      char c = Serial2.read();
      lastSerialDataTime = millis();
      
      // 调试输出：显示接收到的字符
      #ifdef DEBUG_FRAME_RECEIVE
      Serial.printf("UART2 received char: '%c' (0x%02X)\n", c, c);
      #endif
      
      // 帧接收状态机
      if (!frameStarted) {
        // 等待起始符$
        if (c == '$') {
          frameStarted = true;
          serialBuffer = "";  // 清空缓冲区，准备接收新帧
          #ifdef DEBUG_FRAME_RECEIVE
          Serial.println("Frame start detected: $");
          #endif
        }
        // 其他字符都丢弃
      } else {
        // 正在接收数据帧
        if (c == '\n' || c == '\r') {
          // 帧结束，处理接收到的数据
          if (serialBuffer.length() > 0) {
            Serial.printf("Serial2 received complete frame: [%s]\n", serialBuffer.c_str());
            
            // 检查是否是简单测试消息
            if (serialBuffer == "Hello from NodeMCU!") {
              Serial.println("Received test message from NodeMCU!");
              Serial2.println("Hello back from Hexapod!");
              Serial.println("Test response sent via UART2");
            } else {
              // 解析串口指令
              parseSerialMovementCommand(serialBuffer);
            }
          }
          
          // 重置帧接收状态
          frameStarted = false;
          serialBuffer = "";
        } else {
          // 将字符添加到缓冲区
          serialBuffer += c;
        }
      }
    }
    
    // 检查串口数据超时，防止缓冲区溢出
    if (serialBuffer.length() > 0 && (millis() - lastSerialDataTime) > SERIAL_TIMEOUT) {
      Serial.printf("Serial2 timeout, received: [%s]\n", serialBuffer.c_str());
      Serial.println("Serial2 command timeout, clearing buffer");
      serialBuffer = "";
      frameStarted = false;  // 超时时也重置帧状态
    }
    
    // 减少任务延时，提高响应速度
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms延时，100Hz处理频率，提高响应速度
  }
}

/* 发送串口响应
*/
void sendSerialResponse(const String& message) {
  // 通过UART2发送响应，保持与接收格式一致
  Serial2.print("$");
  Serial2.println(message);
  
  // 同时在调试串口打印响应信息
  Serial.printf("Response sent via UART2: $%s\n", message.c_str());
}

/* 测试UART2连接
*/
void testUART2Connection() {
  Serial.println("Testing UART2 connection...");
  
  // 发送测试消息
  Serial2.println("UART2 Test Message from Hexapod");
  Serial.println("Test message sent via UART2");
  
  // 等待一小段时间
  delay(100);
  
  // 检查是否有响应
  if (Serial2.available()) {
    Serial.println("UART2 response detected:");
    while (Serial2.available()) {
      char c = Serial2.read();
      Serial.printf("Response char: '%c' (0x%02X)\n", c, c);
    }
  } else {
    Serial.println("No UART2 response detected");
  }
  
  Serial.println("UART2 test completed");
}