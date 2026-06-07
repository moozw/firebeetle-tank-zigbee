<#
.SYNOPSIS
  Build, package, and publish a firmware release for the FireBeetle tank controller.

  Does it all in one shot:
    1. Builds the firmware (idf.py build)
    2. Reads OTA_FW_VERSION from main/app_config.h
    3. Packages the Zigbee .ota (tools/make_ota.py)
    4. Creates a GitHub Release with both assets (.ota for Zigbee, .bin for WiFi)
    5. Rewrites ota/index.json to point at the release .ota
    6. Commits + pushes ota/index.json

  Bump OTA_FW_VERSION in main/app_config.h BEFORE running this.

.EXAMPLE
  pwsh tools/publish_release.ps1
  pwsh tools/publish_release.ps1 -Tag v1.1.0 -Notes "New feature X"
#>
param(
  [string]$Tag,                                   # default: v1.0.<low byte of OTA_FW_VERSION>
  [string]$Repo  = "moozw/firebeetle-tank-zigbee",
  [string]$Notes,
  [string]$IdfPath = "C:\Espressif\frameworks\esp-idf-v5.5"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

# --- activate ESP-IDF + build ---
$env:IDF_PATH = $IdfPath
$env:IDF_TOOLS_PATH = "C:\Espressif"
. "$IdfPath\export.ps1" *> $null
Write-Host "==> Building..." -ForegroundColor Cyan
idf.py build | Out-Null
if ($LASTEXITCODE -ne 0) { throw "idf.py build failed" }

# --- read OTA_FW_VERSION ---
$m = Select-String -Path main\app_config.h -Pattern 'OTA_FW_VERSION\s+(0x[0-9A-Fa-f]+)'
if (-not $m) { throw "OTA_FW_VERSION not found in main/app_config.h" }
$otaHex = $m.Matches[0].Groups[1].Value
$fileVersion = [Convert]::ToInt64($otaHex, 16)
if (-not $Tag) { $Tag = "v1.0.$([int]($fileVersion -band 0xFF))" }
if (-not $Notes) { $Notes = "Firmware $Tag (OTA version $otaHex)." }
Write-Host "==> $Tag  OTA_FW_VERSION=$otaHex ($fileVersion)" -ForegroundColor Cyan

# --- package .ota + copy raw .bin ---
$bin     = "build\firebeetle_tank_zigbee.bin"
$otaFile = "tank2_$Tag.ota"
$binFile = "tank2_$Tag.bin"
python tools\make_ota.py $bin $otaFile --version $otaHex | Out-Null
if ($LASTEXITCODE -ne 0) { throw "make_ota.py failed" }
Copy-Item $bin $binFile -Force

# --- create the GitHub release (both assets) ---
Write-Host "==> Creating release $Tag..." -ForegroundColor Cyan
gh release create $Tag $otaFile $binFile --repo $Repo --title $Tag --notes $Notes
if ($LASTEXITCODE -ne 0) { throw "gh release create failed (tag may already exist)" }

# --- rewrite ota/index.json -> release .ota ---
$otaSize = (Get-Item $otaFile).Length
$sha512  = (Get-FileHash $otaFile -Algorithm SHA512).Hash.ToLower()
$otaUrl  = "https://github.com/$Repo/releases/download/$Tag/$otaFile"
$binUrl  = "https://github.com/$Repo/releases/download/$Tag/$binFile"
$entry = [ordered]@{
  fileName         = $otaFile
  fileVersion      = $fileVersion
  fileSize         = $otaSize
  manufacturerCode = 4644      # 0x1224
  imageType        = 4113      # 0x1011
  sha512           = $sha512
  url              = $otaUrl
  otaHeaderString  = "FB2-C5-TANK"
  releaseNotes     = $Notes
}
($entry | ConvertTo-Json -AsArray -Depth 5) | Set-Content -Encoding UTF8 ota\index.json

# --- commit + push the index ---
git add ota\index.json
git commit -m "Release $Tag" | Out-Null
git push origin main | Out-Null

# --- tidy local (gitignored) artifacts ---
Remove-Item $otaFile, $binFile -ErrorAction SilentlyContinue

Write-Host "`nDone." -ForegroundColor Green
Write-Host "Zigbee OTA: Z2M 'check for updates' (index -> $otaUrl)"
Write-Host "WiFi OTA:   publish to <topic>/ota  {`"url`":`"$binUrl`"}"
