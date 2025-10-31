param()
$ErrorActionPreference = 'Stop'

$kb = Join-Path $env:APPDATA 'Code\User\keybindings.json'
if (!(Test-Path $kb)) {
  New-Item -ItemType Directory -Force -Path (Split-Path $kb) | Out-Null
  Set-Content -Path $kb -Value '[]' -Encoding UTF8
}

$raw = Get-Content -Path $kb -Raw
if ([string]::IsNullOrWhiteSpace($raw)) { $raw='[]' }

$entry = [pscustomobject]@{
  key = 'ctrl+alt+e'
  command = 'workbench.action.tasks.runTask'
  args = 'Export Markdown to PDF (Puppeteer)'
  when = 'editorTextFocus && resourceExtname == .md'
}

$parsed = $true
try { $obj = $raw | ConvertFrom-Json -ErrorAction Stop } catch { $parsed = $false }

if ($parsed -and ($obj -is [System.Collections.IEnumerable])) {
  $list = [System.Collections.ArrayList]@($obj)
  $existing = $list | Where-Object { $_.key -eq $entry.key -and $_.command -eq $entry.command }
  if ($existing) {
    foreach ($ex in $existing) { $ex.args = $entry.args; $ex.when = $entry.when }
  } else {
    [void]$list.Add($entry)
  }
  $json = $list | ConvertTo-Json -Depth 6
  Set-Content -Path $kb -Value $json -Encoding UTF8
  Write-Output "UPDATED:$kb"
} else {
  # Textual fallback: insert before last closing bracket
  $insert = ($entry | ConvertTo-Json -Compress)
  $trim = $raw.Trim()
  if ($trim -eq '[]') {
    $new = "[\n  $insert\n]"
  } else {
    $idx = $raw.LastIndexOf(']')
    if ($idx -lt 0) { throw 'Invalid keybindings.json content' }
    $prefix = $raw.Substring(0,$idx).TrimEnd()
    if ($prefix.Trim().EndsWith('[')) {
      $new = $prefix + "\n  $insert\n]"
    } else {
      $new = $prefix + ",\n  $insert\n]"
    }
  }
  Set-Content -Path $kb -Value $new -Encoding UTF8
  Write-Output "UPDATED_TEXTUAL:$kb"
}

