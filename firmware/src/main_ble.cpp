/* 六足主程序 - BLE实时步态模式
 *
 * 使用BLE遥控器控制，支持实时步态生成
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

#include "config.h"

#ifdef USE_BLE_CONTROL

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

#include "debug.h"
#include "hexapod.h"
#include "ble_comm.h"
#include "gait_parameters.h"
#include "PinDefines.h"

// 宏定义
#define REACT_DELAY hexapod::config::movementInterval

// 调试模式控制
// #define DEBUG_ADC_MONITOR  // 电池电压ADC调试输出

// 电池监测相关变量
static const float LOW_VOLTAGE_THRESHOLD = 6.4f;
static const uint16_t ADC_THRESHOLD = (uint16_t)(LOW_VOLTAGE_THRESHOLD * 47.0f / (100.0f + 47.0f) / 3.3f * 4095.0f);
static bool lowVoltageFlag = false;
SemaphoreHandle_t voltageMutex;

// 心跳包相关
static unsigned long lastHeartbeatTime = 0;
static const unsigned long HEARTBEAT_INTERVAL = 1000; // 1秒
static unsigned long lastConnectionCheck = 0;
static const unsigned long CONNECTION_TIMEOUT = 3000; // 3秒无心跳则进入待机

// 校准模式相关
static bool calibrationMode = false;  // 校准模式标志
static float test_angle = 0.0;        // 校准测试角度

// 函数声明
static void log_output(const char* log);
void printWelcomeMessage();
void BatteryMonitorTask(void *pvParameters);
void LEDControllerTask(void *pvParameters);
void HeartbeatTask(void *pvParameters);

// BLE回调实现
class NodeHexaBLECallback : public hexapod::BLECommCallback {
public:
    void onModeChanged(hexapod::ControlMode mode) override {
        hexapod::Hexapod.setControlMode(mode);
        
        // 更新心跳时间（收到任何命令）
        lastHeartbeatTime = millis();
        
        const char* modeName = mode == hexapod::MODE_STAND ? "站立" : 
                              mode == hexapod::MODE_WALK ? "行走" : "特技";
        LOG_INFO("模式切换: %s", modeName);
    }
    
    void onStandControl(const hexapod::BodyPose& pose) override {
        hexapod::Hexapod.setBodyPose(pose);
        
        // 更新心跳时间
        lastHeartbeatTime = millis();
        
        LOG_DEBUG("站立控制: roll=%.1f, pitch=%.1f, yaw=%.1f, z=%.1f",
                  pose.roll, pose.pitch, pose.yaw, pose.z);
    }
    
    void onWalkControl(const hexapod::Velocity& vel, float pitch,
                      const hexapod::GaitParameters& params) override {
        hexapod::Hexapod.setVelocity(vel);
        hexapod::Hexapod.setBodyPitch(pitch);
        hexapod::Hexapod.setGaitParameters(params);
        
        // 更新心跳时间
        lastHeartbeatTime = millis();
        
        LOG_DEBUG("行走控制: vx=%.1f, vy=%.1f, vyaw=%.1f, pitch=%.1f",
                  vel.vx, vel.vy, vel.vyaw, pitch);
    }
    
    void onTrickAction(hexapod::TrickAction action) override {
        hexapod::Hexapod.executeTrick(action);
        
        // 更新心跳时间
        lastHeartbeatTime = millis();
    }
    
    void onEmergencyStop() override {
        LOG_INFO("紧急停止");
        hexapod::Velocity vel;
        hexapod::Hexapod.setVelocity(vel);
        hexapod::Hexapod.setControlMode(hexapod::MODE_STAND);
        
        // 更新心跳时间
        lastHeartbeatTime = millis();
    }
    
    // 校准相关回调
    void onCalibrationStart() override {
        LOG_INFO("进入校准模式");
        calibrationMode = true;
        
        // 清除所有偏移
        hexapod::Hexapod.clearOffset();
        
        // 测试所有舵机到测试角度
        hexapod::Hexapod.calibrationTestAllLeg(test_angle);
        
        // 发送校准模式状态
        hexapod::BLE.sendCalibrationStatus("mode_changed", "Entered calibration mode", true);
        
        // 更新心跳时间
        lastHeartbeatTime = millis();
    }
    
    void onCalibrationAdjust(int legIndex, int partIndex, int offset, float testAngle) override {
        if (!calibrationMode) {
            hexapod::BLE.sendError(5001, "Not in calibration mode", "warning");
            return;
        }
        
        LOG_INFO("校准调整: leg=%d, part=%d, offset=%d, angle=%.1f", 
                 legIndex, partIndex, offset, testAngle);
        
        // 设置校准偏移
        hexapod::Hexapod.calibrationSet(legIndex, partIndex, offset);
        
        // 测试舵机
        hexapod::Hexapod.calibrationTest(legIndex, partIndex, testAngle);
        
        // 发送成功反馈
        hexapod::BLE.sendCalibrationStatus("adjust", "Calibration adjusted", false);
        
        // 更新心跳时间
        lastHeartbeatTime = millis();
    }
    
    void onCalibrationGet(int legIndex, int partIndex) override {
        int offset = 0;
        hexapod::Hexapod.calibrationGet(legIndex, partIndex, offset);
        
        // 发送校准值
        hexapod::BLE.sendCalibrationValue(legIndex, partIndex, offset);
        
        LOG_INFO("读取校准值: leg=%d, part=%d, offset=%d", legIndex, partIndex, offset);
        
        // 更新心跳时间
        lastHeartbeatTime = millis();
    }
    
    void onCalibrationSave() override {
        if (!calibrationMode) {
            hexapod::BLE.sendError(5001, "Not in calibration mode", "warning");
            return;
        }
        
        LOG_INFO("保存校准数据");
        
        // 保存到SPIFFS
        hexapod::Hexapod.calibrationSave();
        
        // 重新初始化（加载校准数据）
        hexapod::Hexapod.init(false, true);
        
        // 退出校准模式
        calibrationMode = false;
        
        // 发送保存成功反馈
        hexapod::BLE.sendCalibrationStatus("save", "Calibration data saved", false);
        
        // 更新心跳时间
        lastHeartbeatTime = millis();
    }
    
    void onCalibrationExit() override {
        LOG_INFO("退出校准模式");
        
        // 退出校准模式
        calibrationMode = false;
        
        // 重新初始化
        hexapod::Hexapod.init(false, true);
        
        // 发送退出反馈
        hexapod::BLE.sendCalibrationStatus("exit", "Exited calibration mode", false);
        
        // 更新心跳时间
        lastHeartbeatTime = millis();
    }
};

static NodeHexaBLECallback bleCallback;

void setup() {
    // 初始化串口
    Serial.begin(115200);
    Serial.println("Starting BLE Mode...");
    
    // 初始化电池监测相关硬件
    pinMode(BAT_ADC, INPUT);
    pinMode(BAT_LED, OUTPUT);
    voltageMutex = xSemaphoreCreateMutex();
    
    // 初始化I2C
    Wire.setPins(21, 22);
    
    // 挂载SPIFFS文件系统
    if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    
    // 初始化日志记录回调函数
    hexapod::initLogOutput(log_output, millis);
    
    // 初始化六足机器人
    hexapod::Hexapod.init(false);
    
    // 初始化BLE通讯
    hexapod::BLE.init("NodeHexa");
    hexapod::BLE.setCallback(&bleCallback);
    
    // 初始化心跳时间
    lastHeartbeatTime = millis();
    
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
    
    // 创建心跳包任务
    xTaskCreate(
        HeartbeatTask,
        "Heartbeat",
        4096,
        NULL,
        1,
        NULL
    );
    
    printWelcomeMessage();
    
    Serial.println("BLE Mode Started");
}

void loop() {
    auto t0 = millis();
    
    // 处理BLE通讯
    hexapod::BLE.process();
    
    // 检查心跳超时（3秒无心跳则进入待机）
    if (hexapod::BLE.isConnected() && !calibrationMode) {
        if (millis() - lastHeartbeatTime > CONNECTION_TIMEOUT) {
            LOG_INFO("心跳超时，进入待机模式");
            hexapod::Velocity vel;  // 零速度
            hexapod::Hexapod.setVelocity(vel);
            hexapod::Hexapod.setControlMode(hexapod::MODE_STAND);
            lastHeartbeatTime = millis();  // 重置计时器
        }
    }
    
    // 更新实时步态（校准模式下不更新）
    if (!calibrationMode) {
        hexapod::Hexapod.updateRealtimeGait(REACT_DELAY);
    }
    
    auto spent = millis() - t0;
    
    if (spent < REACT_DELAY) {
        delay(REACT_DELAY - spent);
    } else {
        Serial.println(spent);
    }
}

// 日志输出
static void log_output(const char* log) {
    Serial.println(log);
}

// 电池监测任务
void BatteryMonitorTask(void *pvParameters) {
    const uint8_t SAMPLE_SIZE = 10;
    uint16_t adcReadings[SAMPLE_SIZE] = {0};
    uint8_t readIndex = 0;
    uint32_t adcSum = 0;
    uint8_t actualSampleCount = 0;
    
    while(1) {
        if (actualSampleCount < SAMPLE_SIZE) {
            adcReadings[actualSampleCount] = analogRead(BAT_ADC);
            adcSum += adcReadings[actualSampleCount];
            actualSampleCount++;
        } else {
            adcSum -= adcReadings[readIndex];
            adcReadings[readIndex] = analogRead(BAT_ADC);
            adcSum += adcReadings[readIndex];
            readIndex = (readIndex + 1) % SAMPLE_SIZE;
        }
        
        uint16_t adcAverage = adcSum / actualSampleCount;
        
        #ifdef DEBUG_ADC_MONITOR
        float voltage = (float)adcAverage * 3.3f / 4095.0f * (100.0f + 47.0f) / 47.0f;
        Serial.printf("ADC: %d, Voltage: %.2fV\n", adcAverage, voltage);
        #endif
        
        bool newFlag = (adcAverage < ADC_THRESHOLD);
        
        xSemaphoreTake(voltageMutex, portMAX_DELAY);
        lowVoltageFlag = newFlag;
        xSemaphoreGive(voltageMutex);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// LED控制任务
void LEDControllerTask(void *pvParameters) {
    bool lastState = false;
    
    while(1) {
        xSemaphoreTake(voltageMutex, portMAX_DELAY);
        bool currentState = lowVoltageFlag;
        xSemaphoreGive(voltageMutex);
        
        if (currentState) {
            digitalWrite(BAT_LED, !digitalRead(BAT_LED));
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            if (lastState) {
                digitalWrite(BAT_LED, LOW);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        lastState = currentState;
    }
}

// 心跳包任务
void HeartbeatTask(void *pvParameters) {
    while(1) {
        if (hexapod::BLE.isConnected()) {
            // 读取电池电压
            xSemaphoreTake(voltageMutex, portMAX_DELAY);
            bool lowVoltage = lowVoltageFlag;
            xSemaphoreGive(voltageMutex);
            
            // 计算电池电量百分比
            uint16_t adcValue = analogRead(BAT_ADC);
            float voltage = (float)adcValue * 3.3f / 4095.0f * (100.0f + 47.0f) / 47.0f;
            int battery = map(voltage * 100, 640, 840, 0, 100);
            battery = constrain(battery, 0, 100);
            
            // 获取当前模式
            const char* mode = "stand";
            if (hexapod::Hexapod.getControlMode() == hexapod::MODE_WALK) {
                mode = "walk";
            } else if (hexapod::Hexapod.getControlMode() == hexapod::MODE_TRICK) {
                mode = "trick";
            }
            
            // 发送状态信息
            hexapod::BLE.sendStatus("connected", battery, mode, voltage, 35.0);
            
            // 检查低电压
            if (lowVoltage) {
                hexapod::BLE.sendError(1001, "Battery voltage too low", "warning");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL));
    }
}

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

#endif // USE_BLE_CONTROL

