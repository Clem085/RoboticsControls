const fs = require('fs');
const os = require('os');
const path = require('path');
const { pathToFileURL } = require('url');
const crypto = require('crypto');
const MarkdownIt = require('markdown-it');
const markdownItAttrs = require('markdown-it-attrs');
const markdownItKatex = require('markdown-it-katex');
const puppeteer = require('puppeteer');

function toFileURL(p) { return pathToFileURL(p).href; }
function ensureTrailingSlash(p) { return p.endsWith(path.sep) ? p : p + path.sep; }

function preprocessMath(text) {
  const parts = text.split(/(^```[\s\S]*?^```|^~~~[\s\S]*?^~~~)/m);
  for (let i = 0; i < parts.length; i++) {
    if (/^```|^~~~/m.test(parts[i])) continue;
    let s = parts[i];
    s = s.replace(/\\\((.+?)\\\)/g, (_, inner) => '$' + inner + '$');
    s = s.replace(/\\\[([\s\S]*?)\\\]/g, (_, inner) => '\n$$\n' + inner + '\n$$\n');
    parts[i] = s;
  }
  return parts.join('');
}

function getKatexCssPath() {
  const katexMain = require.resolve('katex/package.json');
  const cssPath = path.join(path.dirname(katexMain), 'dist', 'katex.min.css');
  if (!fs.existsSync(cssPath)) throw new Error('katex.min.css not found');
  return cssPath;
}

function buildHtml(title, baseDir, bodyInnerHtml) {
  const katexCssHref = toFileURL(getKatexCssPath());
  const baseHref = toFileURL(ensureTrailingSlash(baseDir));
  const css = `@page{margin:1in} .katex-display{break-inside:avoid}`;
  return `<!doctype html><html><head><meta charset="utf-8"/><title>${title}</title><base href="${baseHref}"><link rel="stylesheet" href="${katexCssHref}"><style>${css}</style></head><body><div id="content">${bodyInnerHtml}</div></body></html>`;
}

async function main(mdPath) {
  const text = fs.readFileSync(mdPath, 'utf8');
  const md = new MarkdownIt({ html:true, linkify:true, typographer:true });
  md.use(markdownItKatex).use(markdownItAttrs);
  const pre = preprocessMath(text);
  fs.writeFileSync(mdPath + '.pre.tmp.md', pre, 'utf8');
  const html = md.render(pre);
  const htmlAll = buildHtml(path.basename(mdPath), path.dirname(mdPath), html);
  const tmp = path.join(os.tmpdir(), 'hbtest-' + crypto.randomBytes(6).toString('hex') + '.html');
  fs.writeFileSync(tmp, htmlAll, 'utf8');
  const out = mdPath.replace(/\.md$/i, '.katex.test.pdf');
  const browser = await puppeteer.launch({ headless:'new', args:['--allow-file-access-from-files','--disable-web-security'] });
  try {
    const page = await browser.newPage();
    await page.goto(toFileURL(tmp), { waitUntil: 'networkidle0' });
    await page.evaluate(() => Promise.all(Array.from(document.images).map(img => img.complete ? true : new Promise(r=>{img.onload=r;img.onerror=r;}))));
    await page.pdf({ path: out, printBackground: true, format: 'A4', margin: { top: '1in', right: '1in', bottom: '1in', left: '1in' }});
    console.log('Wrote', out);
  } finally {
    await browser.close();
    fs.unlinkSync(tmp);
  }
}

if (require.main === module) {
  const mdPath = process.argv[2];
  if (!mdPath) { console.error('Usage: node tools/test_render.js <path-to-md>'); process.exit(1); }
  main(mdPath).catch(e => { console.error(e); process.exit(1); });
}
