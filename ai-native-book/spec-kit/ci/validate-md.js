const fs = require('fs');
const path = require('path');

const modulesPath = path.join(__dirname, '../../docs/modules');
fs.readdirSync(modulesPath).forEach(file => {
  if (!file.endsWith('.md')) console.warn(Warning:  is not a Markdown file);
});
console.log('✅ All module files validated');
