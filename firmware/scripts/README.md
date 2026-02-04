# 脚本使用说明

## release.py - 固件发布脚本

自动构建、打包并发布固件到 GitHub Release。

### 环境要求

- Python 3.6+
- PlatformIO
- Git 仓库
- `GITHUB_TOKEN` 环境变量（可通过 `scripts/.env` 文件配置）
- `python-dotenv` 包

### 使用方法

```bash
# 构建所有环境（六足 + 四足）
python scripts/release.py

# 只构建六足机器人固件
python scripts/release.py -e nodemcu-32s

# 只构建四足机器人固件
python scripts/release.py -e nodequadmini

# 构建多个指定环境
python scripts/release.py -e nodemcu-32s -e nodequadmini
```

### 支持的环境

- `nodemcu-32s` - NodeHexa (六足机器人)
- `nodequadmini` - NodeQuadMini (四足机器人)

### 功能

1. 从 `platformio.ini` 读取各环境的版本号
2. 构建指定环境的固件
3. 为每个环境创建独立的发布包（ZIP）
4. 上传到 GitHub Release

### 配置

在 `platformio.ini` 的每个环境块中设置版本号：
```ini
build_flags = -D FIRMWARE_VERSION="1.0.0"
```

在 `scripts/.env` 中添加 GitHub Token：
```
GITHUB_TOKEN=your_github_token_here
```

### 自定义 Release 描述

#### 方法一：通过环境变量设置更新内容（推荐）

在发布前设置 `RELEASE_CHANGELOG` 环境变量，支持多行文本：

**Windows PowerShell:**
```powershell
$env:RELEASE_CHANGELOG = @"
- 新增功能：添加了新的运动模式
- 修复问题：修复了校准数据保存的 bug
- 性能优化：提升了运动控制的响应速度
"@
python scripts/release.py
```

**Linux/macOS:**
```bash
export RELEASE_CHANGELOG="- 新增功能：添加了新的运动模式
- 修复问题：修复了校准数据保存的 bug
- 性能优化：提升了运动控制的响应速度"
python scripts/release.py
```

**或在 `.env` 文件中配置：**
```
RELEASE_CHANGELOG=- 新增功能：添加了新的运动模式
- 修复问题：修复了校准数据保存的 bug
- 性能优化：提升了运动控制的响应速度
```

#### 方法二：在代码中修改默认内容

1. 打开 `scripts/release.py`
2. 找到 `get_changelog` 方法（约第 340 行）
3. 修改 `default_changelog` 变量的内容

#### 自定义完整 Release 描述

如果需要修改 Release 的完整结构，可以编辑 `create_github_release` 方法中的 `release_body` 变量（约第 360-385 行）。

当前 Release 描述包含：
- **更新内容**：从环境变量或代码中读取（可自定义）
- **构建信息**：构建时间和包含的固件数量（自动生成）
- **固件列表**：所有环境的固件包列表（自动生成）
- **下载说明**：使用说明
- **烧录地址**：固件烧录地址
- **技术支持**：联系方式

---

## setup_platformio_path.ps1 - PlatformIO PATH 设置

将 PlatformIO 添加到当前 PowerShell 会话的 PATH 中。

### 使用方法

在 PowerShell 中运行：
```powershell
.\scripts\setup_platformio_path.ps1
```

### 说明

- 只影响当前 PowerShell 会话
- 如需永久生效，请手动添加到系统环境变量
- 或每次使用前运行此脚本
