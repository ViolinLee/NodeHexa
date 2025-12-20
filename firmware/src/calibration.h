#pragma once

// Arduino 环境下使用 String；非 Arduino 环境下（例如 IDE/clang 静态分析）退化为 std::string
#ifdef ARDUINO
  #include <Arduino.h> // for String
#else
  #include <string>
  using String = std::string;
#endif

struct CalibrationData {
    int legIndex;  // 腿的索引
    int partIndex; // 关节的索引
    int offset;    // 偏移量
    
    bool modeChanged;
    String operation;
};
