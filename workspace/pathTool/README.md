## pathTool

用于生成固件离线动作表（头文件），输出到：`firmware/src/generated/`

### 使用方式

- **生成六足动作表**

```bash
python3 workspace/pathTool/src/main.py --robot hexapod
```

- **生成四足动作表**

```bash
python3 workspace/pathTool/src/main.py --robot quad
```

### 说明

- **输出文件**：
  - 六足：`firmware/src/generated/movement_table.h`
  - 四足：`firmware/src/generated/movement_table_quad.h`
- **依赖**：六足生成需要 `numpy`（首次缺失可执行：`python3 -m pip install --user -U numpy`）
