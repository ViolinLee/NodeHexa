#pragma once

struct CalibrationData {
    int legIndex;  // 腿的索引
    int partIndex; // 关节的索引
    int offset;    // 偏移量
    
    bool modeChanged;
    String operation;
};
