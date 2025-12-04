# Quick MCP Installation Script
# Run this in PowerShell to install MCP servers for Claude Code

Write-Host "🚀 Installing MCP Servers for Claude Code..." -ForegroundColor Cyan
Write-Host ""

# Check if Claude config directory exists
$claudeDir = "$env:APPDATA\Claude"
if (-not (Test-Path $claudeDir)) {
    Write-Host "Creating Claude configuration directory..." -ForegroundColor Yellow
    New-Item -ItemType Directory -Path $claudeDir -Force | Out-Null
}

# Copy MCP configuration
$sourceConfig = "d:\huminiod-robotics\claude_mcp_config.json"
$destConfig = "$claudeDir\claude_desktop_config.json"

if (Test-Path $sourceConfig) {
    Write-Host "✅ Copying MCP configuration..." -ForegroundColor Green
    Copy-Item $sourceConfig $destConfig -Force
    Write-Host "   → Installed to: $destConfig" -ForegroundColor Gray
} else {
    Write-Host "❌ Source config not found: $sourceConfig" -ForegroundColor Red
    exit 1
}

# Create database directory if it doesn't exist
$dbDir = "d:\huminiod-robotics\db"
if (-not (Test-Path $dbDir)) {
    Write-Host "📁 Creating database directory..." -ForegroundColor Yellow
    New-Item -ItemType Directory -Path $dbDir -Force | Out-Null
}

Write-Host ""
Write-Host "✨ MCP Servers installed successfully!" -ForegroundColor Green
Write-Host ""
Write-Host "📝 Next Steps:" -ForegroundColor Cyan
Write-Host "   1. Add your GitHub Personal Access Token to the config"
Write-Host "   2. (Optional) Add Brave Search API key"
Write-Host "   3. Restart Claude Code / Claude Desktop"
Write-Host ""
Write-Host "📚 Documentation: d:\huminiod-robotics\MCP_SETUP.md" -ForegroundColor Gray
Write-Host ""

# Ask if user wants to open config for editing
$response = Read-Host "Would you like to open the config file now to add API keys? (Y/N)"
if ($response -eq "Y" -or $response -eq "y") {
    notepad $destConfig
}

Write-Host ""
Write-Host "🎉 Done! Restart Claude to use MCP servers." -ForegroundColor Green
