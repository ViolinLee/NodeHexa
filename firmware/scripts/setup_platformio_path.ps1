# PlatformIO PATH 设置脚本
# 将 PlatformIO 添加到当前 PowerShell 会话的 PATH 中

Write-Host "🔧 设置 PlatformIO PATH..." -ForegroundColor Green

# 获取用户目录
$userProfile = $env:USERPROFILE
$platformioPath = Join-Path $userProfile ".platformio\penv\Scripts"

# 检查 PlatformIO 是否存在
if (Test-Path $platformioPath) {
    Write-Host "✓ 找到 PlatformIO 安装目录: $platformioPath" -ForegroundColor Green
    
    # 添加到当前会话的 PATH
    $env:PATH = "$platformioPath;$env:PATH"
    
    # 验证 PlatformIO 是否可用
    try {
        $version = & "$platformioPath\platformio.exe" --version
        Write-Host "✓ PlatformIO 版本: $version" -ForegroundColor Green
        Write-Host "✓ PlatformIO 已添加到当前 PowerShell 会话的 PATH" -ForegroundColor Green
        
        # 测试常用命令
        Write-Host "`n🧪 测试 PlatformIO 命令..." -ForegroundColor Yellow
        & "$platformioPath\platformio.exe" --help | Select-Object -First 3
        
    } catch {
        Write-Host "❌ PlatformIO 命令测试失败: $_" -ForegroundColor Red
    }
    
} else {
    Write-Host "❌ 未找到 PlatformIO 安装目录: $platformioPath" -ForegroundColor Red
    Write-Host "请确保已安装 PlatformIO 扩展" -ForegroundColor Yellow
}

Write-Host "`n📝 使用说明:" -ForegroundColor Cyan
Write-Host "1. 此脚本只影响当前 PowerShell 会话" -ForegroundColor White
Write-Host "2. 要永久添加 PATH，请手动添加到系统环境变量" -ForegroundColor White
Write-Host "3. 或者每次使用前运行此脚本" -ForegroundColor White

Write-Host "`n🚀 现在可以使用以下命令:" -ForegroundColor Green
Write-Host "  platformio --version" -ForegroundColor White
Write-Host "  platformio run" -ForegroundColor White
Write-Host "  platformio run -e nodemcu-32s" -ForegroundColor White
