# MCP Server Connection Error Fix Script
# Fixes common MCP server connection issues

Write-Host "🔧 MCP Server Connection Error Diagnostic & Fix" -ForegroundColor Cyan
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host ""

# Step 1: Check Node.js and npx
Write-Host "📦 Step 1: Checking Node.js environment..." -ForegroundColor Yellow
try {
    $nodeVersion = node --version
    $npmVersion = npm --version
    Write-Host "  ✅ Node.js: $nodeVersion" -ForegroundColor Green
    Write-Host "  ✅ npm: $npmVersion" -ForegroundColor Green
}
catch {
    Write-Host "  ❌ Node.js not found! Install from https://nodejs.org" -ForegroundColor Red
    exit 1
}

Write-Host ""

# Step 2: Pre-install failing MCP servers
Write-Host "📥 Step 2: Installing MCP server packages..." -ForegroundColor Yellow

$servers = @(
    "@modelcontextprotocol/server-fetch",
    "@modelcontextprotocol/server-sqlite",
    "@modelcontextprotocol/server-memory",
    "@modelcontextprotocol/server-filesystem",
    "@modelcontextprotocol/server-github"
)

foreach ($server in $servers) {
    Write-Host "  Installing $server..." -ForegroundColor Gray
    try {
        npx -y $server --version 2>&1 | Out-Null
        Write-Host "  ✅ $server" -ForegroundColor Green
    }
    catch {
        Write-Host "  ⚠️  $server (may install on first use)" -ForegroundColor Yellow
    }
}

Write-Host ""

# Step 3: Create SQLite database directory
Write-Host "📁 Step 3: Setting up SQLite database..." -ForegroundColor Yellow
$dbDir = "d:\huminiod-robotics\db"
$dbPath = Join-Path $dbDir "robotics.db"

if (-not (Test-Path $dbDir)) {
    New-Item -ItemType Directory -Path $dbDir -Force | Out-Null
    Write-Host "  ✅ Created directory: $dbDir" -ForegroundColor Green
}
else {
    Write-Host "  ✅ Directory exists: $dbDir" -ForegroundColor Green
}

# Create empty database if it doesn't exist
if (-not (Test-Path $dbPath)) {
    # Create empty file
    New-Item -ItemType File -Path $dbPath -Force | Out-Null
    Write-Host "  ✅ Created database: $dbPath" -ForegroundColor Green
}
else {
    Write-Host "  ✅ Database exists: $dbPath" -ForegroundColor Green
}

Write-Host ""

# Step 4: Create simplified MCP config (without problematic servers)
Write-Host "📝 Step 4: Creating clean MCP configuration..." -ForegroundColor Yellow

$cleanConfig = @{
    mcpServers = @{
        filesystem = @{
            command = "npx"
            args    = @(
                "-y",
                "@modelcontextprotocol/server-filesystem",
                "d:\huminiod-robotics\huminiod-robotics",
                "d:\huminiod-robotics",
                "C:\Users\Hp\.gemini"
            )
        }
        github     = @{
            command = "npx"
            args    = @(
                "-y",
                "@modelcontextprotocol/server-github"
            )
            env     = @{
                GITHUB_PERSONAL_ACCESS_TOKEN = "<YOUR_GITHUB_TOKEN_HERE>"
            }
        }
        memory     = @{
            command = "npx"
            args    = @(
                "-y",
                "@modelcontextprotocol/server-memory"
            )
        }
        puppeteer  = @{
            command = "npx"
            args    = @(
                "-y",
                "@modelcontextprotocol/server-puppeteer"
            )
        }
    }
}

$cleanConfigPath = "d:\huminiod-robotics\gemini_mcp_config_clean.json"
$cleanConfig | ConvertTo-Json -Depth 10 | Set-Content $cleanConfigPath

Write-Host "  ✅ Created clean config: gemini_mcp_config_clean.json" -ForegroundColor Green
Write-Host "  ℹ️  This config excludes problematic servers (fetch, sqlite, openai_agent)" -ForegroundColor Cyan

Write-Host ""

# Step 5: Test individual servers
Write-Host "🧪 Step 5: Testing individual servers..." -ForegroundColor Yellow

Write-Host "  Testing filesystem server..." -ForegroundColor Gray
$testFilesystem = Start-Process npx -ArgumentList "-y", "@modelcontextprotocol/server-filesystem", "d:\huminiod-robotics" -NoNewWindow -Wait -PassThru
if ($testFilesystem.ExitCode -eq 0) {
    Write-Host "  ✅ Filesystem server OK" -ForegroundColor Green
}
else {
    Write-Host "  ⚠️  Filesystem server may have issues" -ForegroundColor Yellow
}

Write-Host ""

# Summary
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host "✅ Diagnostic Complete!" -ForegroundColor Green
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host ""

Write-Host "📋 Summary:" -ForegroundColor Cyan
Write-Host "  • MCP server packages installed/cached" -ForegroundColor White
Write-Host "  • SQLite database directory created" -ForegroundColor White
Write-Host "  • Clean MCP config created (minimal, stable servers)" -ForegroundColor White
Write-Host ""

Write-Host "🔧 Recommended Fixes:" -ForegroundColor Cyan
Write-Host ""
Write-Host "Option 1: Use Clean Config (Recommended)" -ForegroundColor Yellow
Write-Host "  Replace your current config with the clean version:" -ForegroundColor White
Write-Host '  Copy-Item "d:\huminiod-robotics\gemini_mcp_config_clean.json" "d:\huminiod-robotics\gemini_mcp_config.json" -Force' -ForegroundColor Gray
Write-Host ""

Write-Host "Option 2: Remove Problematic Servers" -ForegroundColor Yellow
Write-Host "  Edit your Gemini config and remove:" -ForegroundColor White
Write-Host "  • openai_agent (if present)" -ForegroundColor White
Write-Host "  • fetch (causing connection issues)" -ForegroundColor White
Write-Host "  • sqlite (may need additional setup)" -ForegroundColor White
Write-Host ""

Write-Host "Option 3: Update to Latest MCP Packages" -ForegroundColor Yellow
Write-Host "  Clear npm cache and reinstall:" -ForegroundColor White
Write-Host "  npm cache clean --force" -ForegroundColor Gray
Write-Host "  npx -y clear-npx-cache" -ForegroundColor Gray
Write-Host ""

Write-Host "🔄 Next Steps:" -ForegroundColor Cyan
Write-Host "  1. Choose an option above" -ForegroundColor White
Write-Host "  2. Restart Gemini CLI (exit and restart)" -ForegroundColor White
Write-Host "  3. Check for errors - should see fewer or no errors" -ForegroundColor White
Write-Host ""

$applyClean = Read-Host "Apply clean config now? (y/n)"
if ($applyClean -eq "y") {
    Copy-Item $cleanConfigPath "d:\huminiod-robotics\gemini_mcp_config.json" -Force
    Write-Host ""
    Write-Host "✅ Clean config applied!" -ForegroundColor Green
    Write-Host "🔄 Restart Gemini CLI to see the changes." -ForegroundColor Cyan
}

Write-Host ""
Write-Host "✨ Done!" -ForegroundColor Green
