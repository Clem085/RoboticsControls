// Hummingbird KaTeX PDF Export

const vscode = require('vscode');
const fs = require('fs');
const os = require('os');
const path = require('path');
const crypto = require('crypto');
const { pathToFileURL } = require('url');

const MarkdownIt = require('markdown-it');
const markdownItAttrs = require('markdown-it-attrs');
const markdownItKatex = require('markdown-it-katex');
const texmath = require('markdown-it-texmath');
const katex = require('katex');
const puppeteer = require('puppeteer');

function toFileURL(p) {
  return pathToFileURL(p).href; // proper file:/// URL for Windows/macOS/Linux
}

function ensureTrailingSlash(p) {
  return p.endsWith(path.sep) ? p : p + path.sep;
}

function buildMarkdownRenderer() {
  const md = new MarkdownIt({ html: true, linkify: true, typographer: true });
  // Use texmath to robustly handle \[...\], $$...$$, and environments; render with KaTeX
  md.use(texmath, { engine: katex, delimiters: 'dollars', katexOptions: { throwOnError: false } });
  md.use(texmath, { engine: katex, delimiters: 'brackets', katexOptions: { throwOnError: false } });
  md.use(markdownItKatex); // keep for $...$ inline users
  md.use(markdownItAttrs);
  return md;
}
// No preprocessing required with texmath, but keep hook for future tweaks
function preprocessMath(text) { return text; }

function getKatexCssPath() {
  try {
    const katexMain = require.resolve('katex/package.json');
    const katexDir = path.dirname(katexMain);
    const cssPath = path.join(katexDir, 'dist', 'katex.min.css');
    if (fs.existsSync(cssPath)) return cssPath;
  } catch (_) {}
  return '';
}

function coerceImageWidthStyles(html) {
  // Convert width="70%" on <img> into inline style for Chromium print
  return html.replace(/<img([^>]*?)width="(\d+%)"([^>]*?)>/gi, (m, pre, w, post) => {
    const styleMatch = (pre + post).match(/style=\"([^\"]*)\"/i);
    if (styleMatch) {
      const styleFull = styleMatch[0];
      const styleVal = styleMatch[1];
      const newStyleVal = styleVal.includes('width:') ? styleVal : (styleVal.trim().replace(/;?$/, ';') + ` width:${w};`);
      return m.replace(styleFull, `style="${newStyleVal}"`).replace(/\swidth="\d+%"/i, '');
    }
    return m.replace(/\swidth="\d+%"/i, ` style="width:${w};"`);
  });
}

function buildHtml({ title, baseDir, bodyInnerHtml, pageMargins }) {
  bodyInnerHtml = coerceImageWidthStyles(bodyInnerHtml);
  const katexCssPath = getKatexCssPath();
  const katexCssHref = katexCssPath ? toFileURL(katexCssPath) : '';
  const baseHref = toFileURL(ensureTrailingSlash(baseDir));

  const pageMargin = pageMargins || '1in';
  const css = `
    @page { margin: ${pageMargin}; }
    html, body { height: 100%; }
    body { font-family: -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Helvetica, Arial, "Segoe UI Emoji", "Segoe UI Symbol"; font-size: 12pt; line-height: 1.5; color: #111; }
    h1, h2, h3 { break-after: avoid-page; page-break-after: avoid; }
    h1, h2, h3, h4, h5, h6 { break-inside: avoid; page-break-inside: avoid; }
    .katex-display { break-inside: avoid; page-break-inside: avoid; }
    pre, blockquote, table { break-inside: avoid; page-break-inside: avoid; }
    img { max-width: 100%; }
  `;

  return `<!DOCTYPE html>
  <html>
    <head>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1" />
      <title>${escapeHtml(title)}</title>
      <base href="${baseHref}">
      ${katexCssHref ? `<link rel="stylesheet" href="${katexCssHref}">` : ''}
      <style>${css}</style>
    </head>
    <body>
      <div id="content">${bodyInnerHtml}</div>
    </body>
  </html>`;
}

function escapeHtml(str) {
  return String(str).replace(/[&<>\"]+/g, s => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[s]);
}

async function exportActiveMarkdownToPdf() {
  const editor = vscode.window.activeTextEditor;
  if (!editor) { vscode.window.showErrorMessage('No active editor. Open a Markdown file to export.'); return; }

  const doc = editor.document;
  const filePath = doc.fileName;
  const ext = path.extname(filePath).toLowerCase();
  if (ext !== '.md' && doc.languageId !== 'markdown') { vscode.window.showErrorMessage('Active file is not a Markdown (.md) document.'); return; }

  const cfg = vscode.workspace.getConfiguration();
  const pageMargins = cfg.get('hummingbirdKatexPdf.pageMargins', '1in');
  const paperFormat = cfg.get('hummingbirdKatexPdf.paperFormat', 'A4');

  let text = doc.getText();
  try { text = preprocessMath(text); } catch(_) {}

  const md = buildMarkdownRenderer();
  let htmlBody;
  try { htmlBody = md.render(text); } catch (e) { vscode.window.showErrorMessage('Markdown render error: ' + e.message); return; }

  const baseDir = path.dirname(filePath);
  const title = path.basename(filePath);
  const fullHtml = buildHtml({ title, baseDir, bodyInnerHtml: htmlBody, pageMargins });

  const tmpName = 'hb-katex-' + crypto.randomBytes(6).toString('hex') + '.html';
  const tmpPath = path.join(os.tmpdir(), tmpName);
  try { fs.writeFileSync(tmpPath, fullHtml, 'utf8'); } catch (e) { vscode.window.showErrorMessage('Failed to write temporary HTML: ' + e.message); return; }

  const outPdf = path.join(baseDir, path.basename(filePath, ext) + '.katex.pdf');

  let browser;
  try {
    browser = await puppeteer.launch({ headless: 'new', args: ['--allow-file-access-from-files', '--disable-web-security'] });
    const page = await browser.newPage();
    await page.goto(toFileURL(tmpPath), { waitUntil: 'networkidle0' });
    try {
      // Ensure file:// images are fully loaded
      await page.evaluate(() => Promise.all(Array.from(document.images).map(img => {
        if (img.complete && img.naturalWidth > 0) return true;
        return new Promise(resolve => { img.addEventListener('load', resolve, { once: true }); img.addEventListener('error', resolve, { once: true }); });
      })));
    } catch (_) {}
    await page.pdf({ path: outPdf, printBackground: true, format: paperFormat, margin: { top: pageMargins, right: pageMargins, bottom: pageMargins, left: pageMargins } });
  } catch (e) {
    vscode.window.showErrorMessage('PDF export failed: ' + e.message);
    try { if (browser) await browser.close(); } catch (_) {}
    return;
  } finally {
    try { if (browser) await browser.close(); } catch (_) {}
    try { fs.unlinkSync(tmpPath); } catch (_) {}
  }

  vscode.window.showInformationMessage('Exported PDF: ' + outPdf);
}

function activate(context) {
  const disposable = vscode.commands.registerCommand('hummingbird-katex-pdf.exportPdf', exportActiveMarkdownToPdf);
  context.subscriptions.push(disposable);
}

function deactivate() {}

module.exports = { activate, deactivate };
