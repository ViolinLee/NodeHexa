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
# 支持的环境列表
SUPPORTED_ENVS = {
    "nodemcu-32s": {
        "name": "NodeHexa (六足机器人)",
        "description": "ESP32 (NodeMCU-32S) - 六足机器人固件",
        "package_name": "NodeHexa"
    },
    "nodequadmini": {
        "name": "NodeQuadMini (四足机器人)",
        "description": "ESP32 (NodeMCU-32S) - 四足机器人固件",
        "package_name": "NodeQuadMini"
    }
}
RELEASE_DIR = PROJECT_ROOT / "release"
BIN_FILES = ["bootloader.bin", "firmware.bin", "partitions.bin", "spiffs.bin"]

class ReleaseManager:
    def __init__(self, build_envs=None):
        """
        初始化发布管理器
        :param build_envs: 要构建的环境列表，None 表示构建所有环境
        """
        self.build_envs = build_envs or list(SUPPORTED_ENVS.keys())
        self.platformio_cmd = None
        self.release_packages = []  # 存储每个环境的发布包信息
        
    def get_version_from_platformio(self, env_name):
        """从 platformio.ini 中读取指定环境的版本号"""
        try:
            platformio_ini_path = PROJECT_ROOT / 'platformio.ini'
            with open(platformio_ini_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 查找指定环境的 FIRMWARE_VERSION 定义
            # 先找到环境块的开始位置
            env_start_pattern = rf'\[env:{re.escape(env_name)}\]'
            env_start_match = re.search(env_start_pattern, content)
            if not env_start_match:
                print(f"❌ 未在 platformio.ini 中找到环境 {env_name}")
                return None
            
            # 找到下一个环境块的开始位置（或文件结尾）
            env_start_pos = env_start_match.end()
            next_env_match = re.search(r'\n\[env:', content[env_start_pos:])
            if next_env_match:
                env_end_pos = env_start_pos + next_env_match.start()
            else:
                env_end_pos = len(content)
            
            # 在当前环境块内查找 FIRMWARE_VERSION
            env_block = content[env_start_pos:env_end_pos]
            version_pattern = r'FIRMWARE_VERSION="([^"]+)"'
            version_match = re.search(version_pattern, env_block)
            
            if version_match:
                version = version_match.group(1)
                print(f"✓ [{env_name}] 找到版本号: {version}")
                return version
            else:
                print(f"❌ 未在 platformio.ini 中找到环境 {env_name} 的 FIRMWARE_VERSION")
                return None
        except FileNotFoundError:
            print(f"❌ 未找到 platformio.ini 文件: {platformio_ini_path}")
            return None
    
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
    
    def build_firmware(self, env_name):
        """构建指定环境的固件"""
        print(f"🔨 开始构建固件 [{env_name}]...")
        try:
            # 切换到项目根目录
            original_cwd = os.getcwd()
            os.chdir(PROJECT_ROOT)
            
            # 清理并构建
            print(f"  🧹 清理环境 {env_name}...")
            subprocess.run([self.platformio_cmd, 'run', '-e', env_name, '--target', 'clean'], 
                         check=True, capture_output=True)
            
            # 构建固件
            print(f"  ⚙️  构建固件 {env_name}...")
            result = subprocess.run([self.platformio_cmd, 'run', '-e', env_name], 
                                  check=True, capture_output=True, text=True)
            print(f"  ✓ [{env_name}] 固件构建成功")
            
            # 构建文件系统镜像
            print(f"  📁 构建 SPIFFS 文件系统镜像 [{env_name}]...")
            result = subprocess.run([self.platformio_cmd, 'run', '-e', env_name, '--target', 'buildfs'], 
                                  check=True, capture_output=True, text=True)
            print(f"  ✓ [{env_name}] SPIFFS 文件系统镜像构建成功")
            
            return True
        except subprocess.CalledProcessError as e:
            print(f"  ❌ [{env_name}] 构建失败: {e}")
            if hasattr(e, 'stderr') and e.stderr:
                print(f"  错误输出: {e.stderr}")
            return False
        finally:
            # 恢复原始工作目录
            os.chdir(original_cwd)
    
    def create_readme(self, env_name, version, env_info):
        """创建烧录说明文档"""
        readme_content = f"""# {env_info['name']} 固件烧录说明

## 版本信息
- **固件版本**: {version}
- **构建时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
- **目标板**: ESP32 (NodeMCU-32S)
- **机器人类型**: {env_info['description']}

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
- 确保开发板驱动（CP2102）已正确安装

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
- **设备无响应**: 按住 BOOT/IO0 按钮后重新烧录

## 技术支持
如有问题，请访问项目 GitHub 页面提交 Issue。
"""
        
        readme_path = PROJECT_ROOT / f'README_{env_name}.md'
        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write(readme_content)
        print(f"  ✓ 创建烧录说明文档: README_{env_name}.md")
        return readme_path
    
    def create_zip_package(self, env_name, version, env_info):
        """创建 ZIP 发布包"""
        build_dir = PROJECT_ROOT / f".pio/build/{env_name}"
        if not build_dir.exists():
            print(f"  ❌ 构建目录不存在: {build_dir}")
            return None
        
        # 确保 release 目录存在
        RELEASE_DIR.mkdir(exist_ok=True)
        
        # 生成发布包文件名（使用英文名称，不包含中文）
        package_name = env_info.get('package_name', env_name)
        zip_filename = f"{package_name}-Firmware-v{version}.zip"
        zip_path = RELEASE_DIR / zip_filename
        
        print(f"  📦 创建发布包: {zip_filename}")
        
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            # 添加 bin 文件
            for bin_file in BIN_FILES:
                bin_path = build_dir / bin_file
                if bin_path.exists():
                    zipf.write(bin_path, bin_file)
                    print(f"    ✓ 添加: {bin_file}")
                else:
                    print(f"    ⚠️  文件不存在: {bin_file}")
            
            # 添加说明文档
            readme_path = PROJECT_ROOT / f'README_{env_name}.md'
            if readme_path.exists():
                zipf.write(readme_path, 'README.md')
                print(f"    ✓ 添加: README.md")
            
            # 添加版本信息
            version_info = {
                "version": version,
                "build_time": datetime.now().isoformat(),
                "build_env": env_name,
                "robot_type": env_info['description'],
                "files": BIN_FILES
            }
            zipf.writestr('version.json', json.dumps(version_info, indent=2))
            print(f"    ✓ 添加: version.json")
        
        print(f"  ✓ 发布包创建完成: {zip_path}")
        
        # 删除临时 README 文档
        if readme_path.exists():
            readme_path.unlink()
            print(f"  ✓ 删除临时 README 文档")
        
        return {
            "env": env_name,
            "version": version,
            "zip_filename": zip_filename,
            "zip_path": zip_path,
            "env_info": env_info
        }
    
    def get_changelog(self):
        """获取更新内容描述
        
        优先级：
        1. 环境变量 RELEASE_CHANGELOG
        2. 代码中定义的默认内容
        
        返回格式化的 Markdown 文本
        """
        # 从环境变量读取
        changelog = os.getenv('RELEASE_CHANGELOG', '').strip()
        
        if changelog:
            # 如果环境变量中有内容，直接使用
            return changelog
        
        # 默认更新内容（可以在这里修改）
        default_changelog = """- 固件版本更新
- 新增四足机器人固件
- 新增运动规划功能"""
        
        return default_changelog
    
    def create_github_release(self):
        """创建 GitHub Release 并上传所有发布包"""
        github_token = os.getenv('GITHUB_TOKEN')
        if not github_token:
            print("❌ 请设置环境变量 GITHUB_TOKEN")
            print("获取 Token: https://github.com/settings/tokens")
            return False
        
        if not self.release_packages:
            print("❌ 没有可发布的包")
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
        
        # 使用第一个包的版本号作为 Release 版本号
        first_package = self.release_packages[0]
        release_version = first_package['version']
        
        # 获取更新内容描述（可从环境变量或代码中配置）
        changelog = self.get_changelog()
        
        # 生成 Release 说明
        release_body = f"""## {PROJECT_NAME} 固件 v{release_version}

### 更新内容
{changelog}

### 构建信息
- 构建时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
- 包含固件: {len(self.release_packages)} 个环境

### 固件列表
"""
        for pkg in self.release_packages:
            release_body += f"- **{pkg['env_info']['name']}** v{pkg['version']} - `{pkg['zip_filename']}`\n"
        
        release_body += f"""
### 下载说明
请根据您的机器人类型下载对应的固件包，解压后按照 README.md 中的说明进行烧录。

### 烧录地址
- bootloader.bin → 0x1000
- partitions.bin → 0x8000  
- firmware.bin → 0x10000
- spiffs.bin → 0x290000

### 技术支持
如有问题，请提交 Issue 或联系开发团队。
"""
        
        # 创建 Release
        release_data = {
            "tag_name": f"v{release_version}",
            "name": f"{PROJECT_NAME} Firmware v{release_version}",
            "body": release_body,
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
        
        # 上传所有文件
        for pkg in self.release_packages:
            upload_url = f'https://uploads.github.com/repos/{owner}/{repo}/releases/{release_id}/assets?name={pkg["zip_filename"]}'
            
            with open(pkg['zip_path'], 'rb') as f:
                headers['Content-Type'] = 'application/zip'
                response = requests.post(upload_url, headers=headers, data=f)
                
                if response.status_code != 201:
                    print(f"❌ 上传文件失败 {pkg['zip_filename']}: {response.status_code}")
                    print(f"错误信息: {response.text}")
                    return False
                
                print(f"✓ 文件上传成功: {pkg['zip_filename']}")
        
        print(f"🎉 发布完成! 访问: {release_info['html_url']}")
        return True
    
    def run(self):
        """执行完整的发布流程"""
        print(f"🚀 开始 {PROJECT_NAME} 固件发布流程")
        print(f"📋 将构建以下环境: {', '.join(self.build_envs)}")
        print("=" * 50)
        
        # 1. 检查环境
        if not self.check_platformio():
            return False
        
        # 2. 验证环境列表
        invalid_envs = [env for env in self.build_envs if env not in SUPPORTED_ENVS]
        if invalid_envs:
            print(f"❌ 不支持的环境: {', '.join(invalid_envs)}")
            print(f"支持的环境: {', '.join(SUPPORTED_ENVS.keys())}")
            return False
        
        # 3. 为每个环境构建和打包
        for env_name in self.build_envs:
            print(f"\n{'='*50}")
            print(f"📦 处理环境: {env_name} ({SUPPORTED_ENVS[env_name]['name']})")
            print(f"{'='*50}")
            
            # 3.1 获取版本号
            version = self.get_version_from_platformio(env_name)
            if not version:
                print(f"⚠️  跳过环境 {env_name}（无法获取版本号）")
                continue
            
            # 3.2 构建固件
            if not self.build_firmware(env_name):
                print(f"⚠️  跳过环境 {env_name}（构建失败）")
                continue
            
            # 3.3 创建说明文档
            env_info = SUPPORTED_ENVS[env_name]
            readme_path = self.create_readme(env_name, version, env_info)
            
            # 3.4 创建发布包
            package_info = self.create_zip_package(env_name, version, env_info)
            if package_info:
                self.release_packages.append(package_info)
        
        if not self.release_packages:
            print("❌ 没有成功构建的发布包")
            return False
        
        # 4. 发布到 GitHub
        print(f"\n{'='*50}")
        print("📤 准备发布到 GitHub")
        print(f"{'='*50}")
        if not self.create_github_release():
            print("⚠️  GitHub 发布失败，但本地发布包已创建")
        
        print("=" * 50)
        print("🎉 固件发布流程完成!")
        print(f"📁 发布包位置:")
        for pkg in self.release_packages:
            print(f"  - {pkg['zip_filename']}: {pkg['zip_path']}")
        return True

def main():
    """主函数"""
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print(f"""
{PROJECT_NAME} 固件发布脚本

用法:
    python release.py                    # 构建所有环境
    python release.py --all              # 构建所有环境
    python release.py -e nodemcu-32s     # 只构建六足机器人固件
    python release.py -e nodequadmini    # 只构建四足机器人固件
    python release.py -e nodemcu-32s -e nodequadmini  # 构建多个指定环境

环境要求:
    - Python 3.6+
    - PlatformIO
    - Git 仓库
    - GITHUB_TOKEN 环境变量 (可通过 .env 文件配置)
    - python-dotenv 包

支持的环境:
    - nodemcu-32s      : NodeHexa (六足机器人)
    - nodequadmini     : NodeQuadMini (四足机器人)

功能:
    1. 从 platformio.ini 读取各环境的版本号
    2. 构建指定环境的固件
    3. 为每个环境创建独立的发布包
    4. 发布到 GitHub Release

配置:
    在 platformio.ini 的每个环境块中添加:
    build_flags = -D FIRMWARE_VERSION="1.0.0"
    
    在 scripts/.env 中添加:
    GITHUB_TOKEN=your_github_token_here
""")
        return
    
    # 解析命令行参数
    build_envs = None
    i = 1
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg in ['-e', '--env']:
            if i + 1 < len(sys.argv):
                if build_envs is None:
                    build_envs = []
                build_envs.append(sys.argv[i + 1])
                i += 2
            else:
                print("❌ -e/--env 参数需要指定环境名")
                sys.exit(1)
        elif arg in ['--all', '-a']:
            build_envs = None  # None 表示构建所有环境
            i += 1
        else:
            print(f"❌ 未知参数: {arg}")
            print("使用 --help 查看帮助信息")
            sys.exit(1)
    
    manager = ReleaseManager(build_envs=build_envs)
    success = manager.run()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
