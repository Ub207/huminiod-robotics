# MCP Servers Configuration for Claude Code

This document explains how to configure Model Context Protocol (MCP) servers with Claude Code to extend its capabilities.

## What are MCP Servers?

MCP (Model Context Protocol) servers allow Claude to interact with external tools and data sources, providing enhanced capabilities like:
- File system access
- GitHub integration
- Database connections
- Web searching
- Memory persistence
- Browser automation

## Configuration File

The MCP configuration is stored in `claude_mcp_config.json` and needs to be copied to Claude's configuration directory.

### Windows Location
Copy the config to: `%APPDATA%\Claude\claude_desktop_config.json`

```powershell
# Copy the configuration
Copy-Item "d:\huminiod-robotics\claude_mcp_config.json" "$env:APPDATA\Claude\claude_desktop_config.json"
```

## Available MCP Servers

### 1. **Filesystem Server** ✅
**What it does**: Access and manage files in specified directories

**Configured paths**:
- `d:\huminiod-robotics` - Your robotics project
- `C:\Users\Hp\.gemini` - Gemini artifacts directory

**No setup required** - Works out of the box!

---

### 2. **GitHub Server** 🔧
**What it does**: Create issues, PRs, search repositories, manage code

**Setup required**:
1. Create GitHub Personal Access Token:
   - Go to https://github.com/settings/tokens
   - Click "Generate new token (classic)"
   - Select scopes: `repo`, `read:org`, `read:user`
   - Copy the token

2. Update config:
```json
"github": {
  "env": {
    "GITHUB_PERSONAL_ACCESS_TOKEN": "ghp_YOUR_TOKEN_HERE"
  }
}
```

---

### 3. **SQLite Server** ✅
**What it does**: Query and manage SQLite databases

**Configured for**: `d:\huminiod-robotics\db\robotics.db`

**No additional setup required** - Will create database if it doesn't exist

---

### 4. **PostgreSQL Server** 🔧
**What it does**: Connect to PostgreSQL databases

**Setup required**:
1. Install PostgreSQL (if not installed)
2. Create database: `robotics_db`
3. Update connection string in config if needed

---

### 5. **Memory Server** ✅
**What it does**: Persistent memory across conversations

**No setup required** - Works automatically!

---

### 6. **Fetch Server** ✅
**What it does**: Make HTTP requests to APIs and websites

**No setup required** - Ready to use!

---

### 7. **Brave Search Server** 🔧
**What it does**: Web search capabilities

**Setup required**:
1. Get Brave Search API key:
   - Go to https://brave.com/search/api/
   - Sign up for API access
   - Copy API key

2. Update config:
```json
"brave-search": {
  "env": {
    "BRAVE_API_KEY": "YOUR_BRAVE_API_KEY_HERE"
  }
}
```

---

### 8. **Puppeteer Server** ✅
**What it does**: Browser automation for testing and scraping

**No setup required** - Will install Chrome automatically on first use

---

### 9. **Sequential Thinking Server** ✅
**What it does**: Enhanced reasoning and problem-solving

**No setup required** - Works out of the box!

---

## Quick Installation

### Step 1: Install the Configuration

```powershell
# Windows PowerShell
Copy-Item "d:\huminiod-robotics\claude_mcp_config.json" "$env:APPDATA\Claude\claude_desktop_config.json"
```

### Step 2: Add Your API Keys (Optional but Recommended)

Edit `$env:APPDATA\Claude\claude_desktop_config.json` and add:

1. **GitHub Token** (line 18): Replace `<YOUR_GITHUB_TOKEN_HERE>`
2. **Brave API Key** (line 32): Replace `<YOUR_BRAVE_API_KEY_HERE>`

### Step 3: Restart Claude Code

Close and reopen Claude Code (VS Code extension) or Claude Desktop App.

### Step 4: Verify Installation

In Claude, you should now be able to:
- Access files in your robotics project
- Query GitHub repositories
- Use web search
- Store persistent memory
- Automate browser tasks

## Usage Examples

### Using Filesystem Server
```
"Can you read the constitution.md file from the robotics project?"
```

### Using GitHub Server
```
"Create a new issue in my repository about adding Chapter 11"
```

### Using Memory Server
```
"Remember that I prefer TypeScript for web projects"
```

### Using Fetch Server
```
"Fetch the latest data from https://api.github.com/repos/facebook/docusaurus"
```

### Using Puppeteer Server
```
"Open the Docusaurus site and take a screenshot of the homepage"
```

## Troubleshooting

### MCP Servers Not Loading

1. **Check file location**:
```powershell
Test-Path "$env:APPDATA\Claude\claude_desktop_config.json"
```

2. **Validate JSON syntax**:
```powershell
Get-Content "$env:APPDATA\Claude\claude_desktop_config.json" | ConvertFrom-Json
```

3. **Restart Claude completely** (not just reload window)

### Individual Server Issues

**GitHub**: Ensure token has correct permissions (repo, read:org, read:user)

**Brave Search**: Verify API key is active and has remaining quota

**PostgreSQL**: Ensure PostgreSQL service is running

**SQLite**: Check that directory `d:\huminiod-robotics\db\` exists

## Advanced Configuration

### Adding More Directories to Filesystem

Edit the filesystem server args:
```json
"filesystem": {
  "args": [
    "-y",
    "@modelcontextprotocol/server-filesystem",
    "d:\\huminiod-robotics",
    "C:\\Users\\Hp\\.gemini",
    "C:\\your\\other\\directory"
  ]
}
```

### Changing Database Paths

Update SQLite path:
```json
"sqlite": {
  "args": [
    "-y",
    "@modelcontextprotocol/server-sqlite",
    "--db-path",
    "path\\to\\your\\database.db"
  ]
}
```

## Security Notes

⚠️ **Important Security Considerations**:

1. **Never commit API keys to Git** - Add config file to `.gitignore`
2. **Use environment variables** for sensitive data in production
3. **Limit filesystem access** to only necessary directories
4. **Review GitHub token permissions** regularly
5. **Rotate tokens** periodically

## Benefits for Your Robotics Project

With these MCP servers, Claude can:

✅ **Read and edit** book chapters directly  
✅ **Search GitHub** for robotics examples  
✅ **Query databases** for lab data  
✅ **Fetch documentation** from external APIs  
✅ **Remember project preferences** across sessions  
✅ **Automate testing** of the Docusaurus site  
✅ **Search web** for latest robotics research  

## Next Steps

1. Install the configuration
2. Add your API keys (especially GitHub)
3. Test each server with simple commands
4. Integrate into your robotics book workflow

## Resources

- [MCP Documentation](https://modelcontextprotocol.io/)
- [Available MCP Servers](https://github.com/modelcontextprotocol/servers)
- [Claude Desktop App](https://claude.ai/download)

---

**Configuration File**: [`claude_mcp_config.json`](file:///d:/huminiod-robotics/claude_mcp_config.json)

**Status**: Ready to install ✅
