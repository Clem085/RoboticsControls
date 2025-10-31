param(
  [Parameter(Mandatory=$true, Position=0)][string]$InputPath,
  [Parameter(Mandatory=$false, Position=1)][string]$OutputPath
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path $InputPath)) {
  Write-Error "Input Markdown file not found: $InputPath"
}

if (-not $OutputPath -or [IO.Path]::GetExtension($OutputPath).ToLower() -ne '.pdf') {
  $dir = Split-Path -Parent $InputPath
  $name = [IO.Path]::GetFileNameWithoutExtension($InputPath)
  $OutputPath = Join-Path $dir ("$name.pdf")
}

# Normalize to absolute path
$OutputPath = [IO.Path]::GetFullPath($OutputPath)

# Read workspace CSS
$workspaceCssPath = Join-Path (Get-Location) ".vscode/mpe-pdf.css"
if (-not (Test-Path $workspaceCssPath)) {
  Write-Error "Workspace CSS not found at .vscode/mpe-pdf.css"
}
$css = Get-Content -Path $workspaceCssPath -Raw

# Read Markdown
$md = Get-Content -Path $InputPath -Raw

# Base64-encode Markdown to avoid quoting issues
$mdBytes = [System.Text.Encoding]::UTF8.GetBytes($md)
$mdB64   = [Convert]::ToBase64String($mdBytes)

# Build HTML that renders via markdown-it + KaTeX (placeholders replaced later)
$html = @'
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta http-equiv="X-UA-Compatible" content="IE=edge" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Markdown Export</title>
  <style>
%%CSS%%
  </style>
  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css" crossorigin="anonymous">
  <style>
  /* Ensure body background prints */
  html, body { background: #fff; }
  </style>
</head>
<body>
  <div id="app">Rendering…</div>

  <script>
  /* markdown-it */
  %%MDIT%%
  </script>
  <script>
  /* KaTeX */
  %%KATEXJS%%
  </script>
  <script>
  /* markdown-it-katex */
  %%MDKATEX%%
  </script>
  <script>
    const RAW_MD = atob('%%B64%%');
    // Normalize common TeX delimiters so we can pre-render reliably
    const RAW_NORM = RAW_MD
      .replace(/\\\[([\s\S]*?)\\\]/g, (m, g1) => `$$${g1}$$`)
      .replace(/\\\(([^\)]*?)\\\)/g, (m, g1) => `$${g1}$`);
    const md = window.markdownit({ html: true, linkify: true, breaks: false });

    function renderMathHTML(html) {
      // Render display math first
      html = html.replace(/\$\$([\s\S]*?)\$\$/g, function(_m, expr){
        try { return katex.renderToString(expr, {displayMode: true, throwOnError: false}); }
        catch(e) { return _m; }
      });
      // Render inline math (avoid matching $$ and escaped $)
      html = html.replace(/(^|[^\\])\$([^$\n]+?)\$(?!\w)/g, function(_m, p1, expr){
        try { return p1 + katex.renderToString(expr, {displayMode: false, throwOnError: false}); }
        catch(e) { return _m; }
      });
      return html;
    }

    const html0 = md.render(RAW_NORM);
    const html = renderMathHTML(html0);
    document.getElementById('app').innerHTML = html;
  </script>
</body>
</html>
'@

# Inject CSS and data
$html = $html.Replace('%%CSS%%', $css).Replace('%%B64%%', $mdB64)

# Ensure vendor JS is available (download if missing), then inline into HTML
$vendor = Join-Path (Get-Location) "tools/vendor"
New-Item -ItemType Directory -Force -Path $vendor | Out-Null

$mdItPath   = Join-Path $vendor "markdown-it.min.js"
$katexJs    = Join-Path $vendor "katex.min.js"
$autoRenderJs  = Join-Path $vendor "auto-render.min.js"

function Ensure-AnyFile([string[]]$urls, $path) {
  if (Test-Path $path) { return }
  $lastErr = $null
  foreach ($u in $urls) {
    try {
      Invoke-WebRequest -UseBasicParsing -Uri $u -OutFile $path -TimeoutSec 30 | Out-Null
      return
    } catch {
      $lastErr = $_
    }
  }
  throw "Failed to download library to $path from any CDN. Last error: $lastErr"
}

Ensure-AnyFile @(
  "https://cdn.jsdelivr.net/npm/markdown-it@13/dist/markdown-it.min.js",
  "https://cdnjs.cloudflare.com/ajax/libs/markdown-it/13.0.1/markdown-it.min.js",
  "https://unpkg.com/markdown-it@13/dist/markdown-it.min.js"
) $mdItPath

Ensure-AnyFile @(
  "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.js",
  "https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.16.11/katex.min.js",
  "https://unpkg.com/katex@0.16.11/dist/katex.min.js"
) $katexJs

Ensure-AnyFile @(
  "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/contrib/auto-render.min.js",
  "https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.16.11/contrib/auto-render.min.js",
  "https://unpkg.com/katex@0.16.11/dist/contrib/auto-render.min.js"
) $autoRenderJs

$mdItSrc   = Get-Content -Path $mdItPath -Raw
$katexSrc  = Get-Content -Path $katexJs -Raw
$autoSrc    = Get-Content -Path $autoRenderJs -Raw

$html = $html.Replace('%%MDIT%%', $mdItSrc).Replace('%%KATEXJS%%', $katexSrc).Replace('%%MDKATEX%%', $autoSrc)

# Write temp HTML
$tempHtml = Join-Path $env:TEMP ("mpe_export_" + [IO.Path]::GetFileNameWithoutExtension($InputPath) + ".html")
Set-Content -Path $tempHtml -Value $html -Encoding UTF8

# Find Edge or Chrome
$pf86 = ${env:ProgramFiles(x86)}
$pf   = ${env:ProgramFiles}
$candidates = @(
  "$pf86\Microsoft\Edge\Application\msedge.exe",
  "$pf\Microsoft\Edge\Application\msedge.exe",
  "$pf86\Google\Chrome\Application\chrome.exe",
  "$pf\Google\Chrome\Application\chrome.exe"
)

$browser = $candidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
if (-not $browser) {
  Write-Error "No Edge/Chrome executable found. Please install Microsoft Edge or Google Chrome."
}

# Print to PDF via headless Chromium
$tmpUser = Join-Path $env:TEMP ("mpe_ud_" + [guid]::NewGuid().ToString())
New-Item -ItemType Directory -Path $tmpUser -Force | Out-Null

$args = @(
  "--headless=new",
  "--disable-gpu",
  "--print-to-pdf=$OutputPath",
  "--print-to-pdf-no-header",
  "--virtual-time-budget=10000",
  "--no-sandbox",
  "--no-first-run",
  "--no-default-browser-check",
  "--user-data-dir=$tmpUser",
  "--window-size=1280,1696",
  ("file:///" + ($tempHtml -replace "\\","/"))
)

& $browser @args | Out-Null

try { Remove-Item -Recurse -Force -ErrorAction SilentlyContinue $tmpUser } catch {}

Write-Output "PDF written: $OutputPath"
