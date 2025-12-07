# 🔧 Quick Fix for MCP Connection Errors

## Problem
You're seeing these errors when starting Gemini CLI:
```
✕ Error during discovery for server 'openai_agent': Connection failed
✕ Error during discovery for server 'fetch': Connection failed
✕ Error during discovery for server 'sqlite': Connection failed
```

## ⚡ Instant Fix (30 seconds)

### Option 1: Use Minimal Stable Config

Run this command:

```powershell
.\fix-mcp-errors.ps1
```

This will:
- Install/cache MCP packages
- Create clean config with only stable servers
- Set up SQLite database directory
- Offer to apply the fix automatically

### Option 2: Manual Quick Fix

Edit your Gemini MCP config:

```powershell
notepad "d:\huminiod-robotics\gemini_mcp_config.json"
```

**Remove or comment out these servers:**
```json
// Remove these sections if they exist:
"openai_agent": { ... },
"fetch": { ... },
"sqlite": { ... }
```

**Keep these working servers:**
- ✅ `filesystem`
- ✅ `github`
- ✅ `memory`
- ✅ `puppeteer`

---

## 🎯 Minimal Working Config

Save this as your `gemini_mcp_config.json`:

```json
{
  "mcpServers": {
    "filesystem": {
      "command": "npx",
      "args": [
        "-y",
        "@modelcontextprotocol/server-filesystem",
        "d:\\huminiod-robotics\\huminiod-robotics",
        "d:\\huminiod-robotics",
        "C:\\Users\\Hp\\.gemini"
      ]
    },
    "github": {
      "command": "npx",
      "args": [
        "-y",
        "@modelcontextprotocol/server-github"
      ],
      "env": {
        "GITHUB_PERSONAL_ACCESS_TOKEN": "<YOUR_TOKEN_HERE>"
      }
    },
    "memory": {
      "command": "npx",
      "args": [
        "-y",
        "@modelcontextprotocol/server-memory"
      ]
    },
    "puppeteer": {
      "command": "npx",
      "args": [
        "-y",
        "@modelcontextprotocol/server-puppeteer"
      ]
    }
  }
}
```

---

## 🔍 Why This Happens

**Common causes:**
1. **First-time package download** - npx hasn't cached the packages yet
2. **Network issues** - Can't download MCP server packages
3. **Missing dependencies** - Some servers need additional setup
4. **Path issues** - SQLite database path doesn't exist

**These specific servers often fail:**
- `openai_agent` - Requires OpenAI API key and special setup
- `fetch` - Can have CORS/network issues
- `sqlite` - Needs database file to exist first

---

## ✅ After Applying Fix

Restart Gemini CLI:
```bash
# Exit current session (Ctrl+C or type 'exit')
# Then restart
gemini
```

You should see:
```
✅ Using: 2 GEMINI.md files | 4-6 MCP servers
```

**No error messages!**

---

## 🧪 Test Your Servers

After restarting, test each server:

**Filesystem:**
```
"List files in the docs directory"
```

**GitHub (need token):**
```
"Search GitHub for ROS2 robotics"
```

**Memory:**
```
"Remember: This is a humanoid robotics book project"
```

**Puppeteer:**
```
"Open localhost:3000 in browser"
```

---

## 📚 Want More Servers Later?

Once the basic setup is working, you can **gradually add** servers:

### Add Fetch Server (HTTP requests)
Uncomment in config after basic servers work

### Add SQLite Server (Database)
1. Create database directory first:
```powershell
New-Item -ItemType Directory -Path "d:\huminiod-robotics\db" -Force
New-Item -ItemType File -Path "d:\huminiod-robotics\db\robotics.db" -Force
```

2. Then add to config:
```json
"sqlite": {
  "command": "npx",
  "args": [
    "-y",
    "@modelcontextprotocol/server-sqlite",
    "--db-path",
    "d:\\huminiod-robotics\\db\\robotics.db"
  ]
}
```

---

## 🆘 Still Having Issues?

Run full diagnostic:
```powershell
.\fix-mcp-errors.ps1
```

Check logs:
- Gemini CLI shows MCP logs in debug mode
- Look for specific error messages
- Check network connectivity

Common fixes:
1. Clear npm cache: `npm cache clean --force`
2. Update npm: `npm install -g npm@latest`
3. Restart terminal/VS Code
4. Check firewall settings

---

**Quick Command:**
```powershell
.\fix-mcp-errors.ps1
```

Then restart Gemini CLI and enjoy error-free MCP servers! ✨
