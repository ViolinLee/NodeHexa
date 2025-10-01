#!/usr/bin/env python3
"""
NodeHexa 固件发布脚本
支持 Windows、Linux、macOS
自动构建、打包并发布到 GitHub Release
"""

import os
import re
import sys
import json
import zipfile
import subprocess
import requests
from pathlib import Path
from datetime import datetime
from dotenv import load_dotenv

# 加载环境变量
load_dotenv()

# 获取脚本所在目录的父目录（项目根目录）
SCRIPT_DIR = Path(__file__).parent
PROJECT_ROOT = SCRIPT_DIR.parent

# 配置
PROJECT_NAME = "NodeHexa"
BUILD_ENV = "nodemcu-32s"
BUILD_DIR = PROJECT_ROOT / f".pio/build/{BUILD_ENV}"
RELEASE_DIR = PROJECT_ROOT / "release"
BIN_FILES = ["bootloader.bin", "firmware.bin", "partitions.bin", "spiffs.bin"]

class ReleaseManager:
    def __init__(self):
        self.version = None
        self.zip_filename = None
        self.platformio_cmd = None
        
    def get_version_from_platformio(self):
        """从 platformio.ini 中读取版本号"""
        try:
            platformio_ini_path = PROJECT_ROOT / 'platformio.ini'
            with open(platformio_ini_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 查找 FIRMWARE_VERSION 定义
            match = re.search(r'FIRMWARE_VERSION="([^"]+)"', content)
            if match:
                self.version = match.group(1)
                print(f"✓ 找到版本号: {self.version}")
                return True
            else:
                print("❌ 未在 platformio.ini 中找到 FIRMWARE_VERSION")
                return False
        except FileNotFoundError:
            print(f"❌ 未找到 platformio.ini 文件: {platformio_ini_path}")
            return False
    
    def check_platformio(self):
        """检查 PlatformIO 是否安装"""
        # 尝试多个可能的 PlatformIO 路径
        possible_paths = [
            'platformio',  # 系统 PATH 中
            'pio',         # 系统 PATH 中
        ]
        
        # Windows 特定路径
        if os.name == 'nt':
            user_profile = os.environ.get('USERPROFILE', '')
            if user_profile:
                possible_paths.extend([
                    os.path.join(user_profile, '.platformio', 'penv', 'Scripts', 'platformio.exe'),
                    os.path.join(user_profile, '.platformio', 'penv', 'Scripts', 'pio.exe'),
                ])
        
        # Linux/macOS 特定路径
        else:
            home = os.environ.get('HOME', '')
            if home:
                possible_paths.extend([
                    os.path.join(home, '.platformio', 'penv', 'bin', 'platformio'),
                    os.path.join(home, '.platformio', 'penv', 'bin', 'pio'),
                ])
        
        for pio_path in possible_paths:
            try:
                result = subprocess.run([pio_path, '--version'], 
                                      capture_output=True, text=True, check=True)
                print(f"✓ PlatformIO 版本: {result.stdout.strip()}")
                self.platformio_cmd = pio_path
                return True
            except (subprocess.CalledProcessError, FileNotFoundError):
                continue
        
        print("❌ PlatformIO 未安装或不在 PATH 中")
        print("请访问 https://platformio.org/install 安装 PlatformIO")
        return False
    
    def build_firmware(self):
        """构建固件"""
        print("🔨 开始构建固件...")
        try:
            # 切换到项目根目录
            original_cwd = os.getcwd()
            os.chdir(PROJECT_ROOT)
            
            # 清理并构建
            subprocess.run([self.platformio_cmd, 'run', '--target', 'clean'], 
                         check=True, capture_output=True)
            
            # 构建固件
            result = subprocess.run([self.platformio_cmd, 'run', '-e', BUILD_ENV], 
                                  check=True, capture_output=True, text=True)
            print("✓ 固件构建成功")
            
            # 构建文件系统镜像
            print("📁 构建 SPIFFS 文件系统镜像...")
            result = subprocess.run([self.platformio_cmd, 'run', '-e', BUILD_ENV, '--target', 'buildfs'], 
                                  check=True, capture_output=True, text=True)
            print("✓ SPIFFS 文件系统镜像构建成功")
            
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ 构建失败: {e}")
            print(f"错误输出: {e.stderr}")
            return False
        finally:
            # 恢复原始工作目录
            os.chdir(original_cwd)
    
    def create_readme(self):
        """创建烧录说明文档"""
        readme_content = f"""# {PROJECT_NAME} 固件烧录说明

## 版本信息
- **固件版本**: {self.version}
- **构建时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
- **目标板**: ESP32 (NodeMCU-32S)

## 文件说明
本固件包包含以下文件：

| 文件名 | 说明 | 烧录地址 |
|--------|------|----------|
| `bootloader.bin` | 引导加载程序 | 0x1000 |
| `partitions.bin` | 分区表 | 0x8000 |
| `firmware.bin` | 主应用程序固件 | 0x10000 (app0分区) |
| `spiffs.bin` | SPIFFS 文件系统映像 | 0x290000 (spiffs分区) |

## 烧录步骤

### 1. 下载烧录工具
下载并安装 [ESP32 Flash Download Tool](https://www.espressif.com/en/support/download/other-tools)

### 2. 连接设备
- 将 ESP32 开发板通过 USB 连接到电脑
- 确保驱动已正确安装

### 3. 配置烧录工具
1. 打开 `flash_download_tool`
2. 选择芯片类型：**ESP32**
3. 选择工作模式：**develop**

### 4. 添加文件
在烧录工具中添加以下文件：

| 文件路径 | 烧录地址 | 文件大小 |
|----------|----------|----------|
| `bootloader.bin` | 0x1000 | 自动检测 |
| `partitions.bin` | 0x8000 | 自动检测 |
| `firmware.bin` | 0x10000 | 自动检测 |
| `spiffs.bin` | 0x290000 | 自动检测 |

### 5. 烧录设置
- **波特率**: 115200 (推荐)
- **Flash 模式**: DIO
- **Flash 大小**: 4MB (32Mbit)
- **Flash 频率**: 40MHz

### 6. 开始烧录
1. 选择正确的串口
2. 点击 **START** 开始烧录
3. 等待烧录完成

## 注意事项
- 烧录前请确保设备已正确连接
- 如果烧录失败，请检查串口权限和驱动
- 建议使用高质量的 USB 数据线
- 烧录过程中请勿断开连接

## 故障排除
- **串口无法识别**: 检查驱动安装和 USB 线质量
- **烧录失败**: 尝试降低波特率或更换 USB 端口
- **设备无响应**: 按住 BOOT 按钮后重新烧录

## 技术支持
如有问题，请访问项目 GitHub 页面提交 Issue。
"""
        
        readme_path = PROJECT_ROOT / 'README.md'
        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write(readme_content)
        print("✓ 创建烧录说明文档")
    
    def create_zip_package(self):
        """创建 ZIP 发布包"""
        if not BUILD_DIR.exists():
            print(f"❌ 构建目录不存在: {BUILD_DIR}")
            return False
        
        # 确保 release 目录存在
        RELEASE_DIR.mkdir(exist_ok=True)
        
        self.zip_filename = f"{PROJECT_NAME}-Firmware-v{self.version}.zip"
        zip_path = RELEASE_DIR / self.zip_filename
        
        print(f"📦 创建发布包: {self.zip_filename}")
        
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            # 添加 bin 文件
            for bin_file in BIN_FILES:
                bin_path = BUILD_DIR / bin_file
                if bin_path.exists():
                    zipf.write(bin_path, bin_file)
                    print(f"  ✓ 添加: {bin_file}")
                else:
                    print(f"  ⚠️  文件不存在: {bin_file}")
            
            # 添加说明文档
            readme_path = PROJECT_ROOT / 'README.md'
            if readme_path.exists():
                zipf.write(readme_path, 'README.md')
                print("  ✓ 添加: README.md")
            
            # 添加版本信息
            version_info = {
                "version": self.version,
                "build_time": datetime.now().isoformat(),
                "build_env": BUILD_ENV,
                "files": BIN_FILES
            }
            zipf.writestr('version.json', json.dumps(version_info, indent=2))
            print("  ✓ 添加: version.json")
        
        print(f"✓ 发布包创建完成: {zip_path}")
        
        # 删除临时 README 文档
        if readme_path.exists():
            readme_path.unlink()
            print("✓ 删除临时 README 文档")
        
        return True
    
    def create_github_release(self):
        """创建 GitHub Release"""
        github_token = os.getenv('GITHUB_TOKEN')
        if not github_token:
            print("❌ 请设置环境变量 GITHUB_TOKEN")
            print("获取 Token: https://github.com/settings/tokens")
            return False
        
        # 获取仓库信息
        try:
            # 切换到项目根目录执行 git 命令
            original_cwd = os.getcwd()
            os.chdir(PROJECT_ROOT)
            
            result = subprocess.run(['git', 'remote', 'get-url', 'origin'], 
                                  capture_output=True, text=True, check=True)
            repo_url = result.stdout.strip()
            # 解析仓库名 (假设格式: https://github.com/owner/repo.git)
            match = re.search(r'github\.com[:/]([^/]+)/([^/]+?)(?:\.git)?$', repo_url)
            if not match:
                print("❌ 无法解析 GitHub 仓库信息")
                return False
            owner, repo = match.groups()
        except subprocess.CalledProcessError:
            print("❌ 无法获取 Git 远程仓库信息")
            return False
        finally:
            # 恢复原始工作目录
            os.chdir(original_cwd)
        
        print(f"📤 准备发布到 GitHub: {owner}/{repo}")
        
        # 创建 Release
        release_data = {
            "tag_name": f"v{self.version}",
            "name": f"{PROJECT_NAME} Firmware v{self.version}",
            "body": f"""## {PROJECT_NAME} 固件 v{self.version}

### 更新内容
- 固件版本: {self.version}
- 构建时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

### 下载说明
请下载 `{self.zip_filename}` 文件，解压后按照 README.md 中的说明进行烧录。

### 烧录地址
- bootloader.bin → 0x1000
- partitions.bin → 0x8000  
- firmware.bin → 0x10000
- spiffs.bin → 0x290000

### 技术支持
如有问题，请提交 Issue 或联系开发团队。
""",
            "draft": False,
            "prerelease": False
        }
        
        headers = {
            'Authorization': f'token {github_token}',
            'Accept': 'application/vnd.github.v3+json'
        }
        
        # 创建 Release
        url = f'https://api.github.com/repos/{owner}/{repo}/releases'
        response = requests.post(url, json=release_data, headers=headers)
        
        if response.status_code != 201:
            print(f"❌ 创建 Release 失败: {response.status_code}")
            print(f"错误信息: {response.text}")
            return False
        
        release_info = response.json()
        release_id = release_info['id']
        print(f"✓ Release 创建成功: {release_info['html_url']}")
        
        # 上传文件
        upload_url = f'https://uploads.github.com/repos/{owner}/{repo}/releases/{release_id}/assets?name={self.zip_filename}'
        
        zip_path = RELEASE_DIR / self.zip_filename
        with open(zip_path, 'rb') as f:
            headers['Content-Type'] = 'application/zip'
            response = requests.post(upload_url, headers=headers, data=f)
            
            if response.status_code != 201:
                print(f"❌ 上传文件失败: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
        
        print(f"✓ 文件上传成功: {self.zip_filename}")
        print(f"🎉 发布完成! 访问: {release_info['html_url']}")
        return True
    
    def run(self):
        """执行完整的发布流程"""
        print(f"🚀 开始 {PROJECT_NAME} 固件发布流程")
        print("=" * 50)
        
        # 1. 检查环境
        if not self.check_platformio():
            return False
        
        # 2. 获取版本号
        if not self.get_version_from_platformio():
            return False
        
        # 3. 构建固件
        if not self.build_firmware():
            return False
        
        # 4. 创建说明文档
        self.create_readme()
        
        # 5. 创建发布包
        if not self.create_zip_package():
            return False
        
        # 6. 发布到 GitHub
        if not self.create_github_release():
            return False
        
        print("=" * 50)
        print("🎉 固件发布流程完成!")
        print(f"📁 发布包位置: {RELEASE_DIR / self.zip_filename}")
        return True

def main():
    """主函数"""
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print(f"""
{PROJECT_NAME} 固件发布脚本

用法:
    python release.py

环境要求:
    - Python 3.6+
    - PlatformIO
    - Git 仓库
    - GITHUB_TOKEN 环境变量 (可通过 .env 文件配置)
    - python-dotenv 包

功能:
    1. 从 platformio.ini 读取版本号
    2. 构建固件
    3. 创建发布包
    4. 发布到 GitHub Release

配置:
    在 platformio.ini 中添加:
    build_flags = -D FIRMWARE_VERSION="1.0.0"
    
    在 scripts/.env 中添加:
    GITHUB_TOKEN=your_github_token_here
""")
        return
    
    manager = ReleaseManager()
    success = manager.run()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
