param(
    [string]$BuildDir = "$env:USERPROFILE\git\grblhal-sim-windows-build",
    [string]$Configuration = "Release",
    [string]$ArchiveName = "grblhal-flexihal-sim-windows-x64.zip"
)

$ErrorActionPreference = "Stop"

function Resolve-NativePath {
    param([Parameter(Mandatory = $true)][string]$Path)
    return (Resolve-Path $Path).ProviderPath
}

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$FilePath,
        [string[]]$Arguments = @()
    )

    & $FilePath @Arguments
    $exitCode = if ($null -ne $LASTEXITCODE) { $LASTEXITCODE } elseif ($?) { 0 } else { 1 }

    if ($exitCode -ne 0) {
        throw "$FilePath failed with exit code $exitCode"
    }
}

function Resolve-ToolPath {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [string[]]$Fallbacks = @()
    )

    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) {
        return $cmd.Source
    }

    foreach ($fallback in $Fallbacks) {
        if (Test-Path $fallback) {
            return $fallback
        }
    }

    throw "$Name was not found. Install it or add it to PATH."
}

$Root = Resolve-NativePath (Join-Path $PSScriptRoot "..\..")
$DistDir = Join-Path $Root "dist"
$StageDir = Join-Path $BuildDir "package\grblhal-flexihal-sim"
$ArchivePath = Join-Path $DistDir $ArchiveName
$CMakeExe = Resolve-ToolPath -Name "cmake" -Fallbacks @(
    "C:\Program Files\CMake\bin\cmake.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
    "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
)

Invoke-Checked -FilePath $CMakeExe -Arguments @("-S", $Root, "-B", $BuildDir, "-G", "Visual Studio 17 2022", "-A", "x64", "-DBUILD_WINDOWS_LAUNCHER=ON")
Invoke-Checked -FilePath $CMakeExe -Arguments @("--build", $BuildDir, "--config", $Configuration, "--clean-first")

if (Test-Path $StageDir) {
    Remove-Item $StageDir -Recurse -Force
}

Invoke-Checked -FilePath $CMakeExe -Arguments @("--install", $BuildDir, "--config", $Configuration, "--prefix", $StageDir)
New-Item -ItemType Directory -Force -Path (Join-Path $StageDir "sdcard") | Out-Null
New-Item -ItemType Directory -Force -Path $DistDir | Out-Null

if (Test-Path $ArchivePath) {
    Remove-Item $ArchivePath -Force
}

$PackageParent = Split-Path -Parent $StageDir
$PackageName = Split-Path -Leaf $StageDir
Push-Location $PackageParent
try {
    Invoke-Checked -FilePath $CMakeExe -Arguments @("-E", "tar", "cf", $ArchivePath, "--format=zip", $PackageName)
}
finally {
    Pop-Location
}
Write-Host $ArchivePath
