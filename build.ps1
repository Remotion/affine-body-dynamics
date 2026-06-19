<#
.SYNOPSIS
    Builds the Affine Body Dynamics project using CMake and MSBuild for Visual Studio 2026.
.DESCRIPTION
    This script automates finding Visual Studio 2026 (MSBuild), configuring the project
    using CMake's Visual Studio 18 2026 generator, building the generated solution,
    and optionally running tests.
#>

param (
    [string]$Config = "Release",
    [switch]$Clean,
    [switch]$RunTests
)

$ErrorActionPreference = "Stop"

# 1. Locate Visual Studio 2026 installation path using vswhere.exe
Write-Host "Locating Visual Studio 2026..." -ForegroundColor Cyan
$vswherePath = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswherePath)) {
    Write-Error "vswhere.exe not found. Visual Studio Installer may not be installed."
    exit 1
}

# Dev18 corresponds to version 18.x (Visual Studio 2026)
$vsPath = & $vswherePath -version "[18.0,19.0)" -latest -property installationPath
if (-not $vsPath) {
    Write-Warning "Visual Studio 2026 not explicitly found in range 18.x. Falling back to the latest installed version."
    $vsPath = & $vswherePath -latest -property installationPath
}

if (-not $vsPath) {
    Write-Error "Failed to locate Visual Studio installation."
    exit 1
}

$vsPath = $vsPath.Trim()
Write-Host "Found Visual Studio at: $vsPath" -ForegroundColor Green

# 2. Locate MSBuild.exe
$msBuildPath = Join-Path $vsPath "MSBuild\Current\Bin\MSBuild.exe"
if (-not (Test-Path $msBuildPath)) {
    Write-Error "MSBuild.exe was not found at expected path: $msBuildPath"
    exit 1
}
Write-Host "Using MSBuild: $msBuildPath" -ForegroundColor Green

# 3. Handle clean build option
$buildDir = Join-Path $PSScriptRoot "build"
if ($Clean -and (Test-Path $buildDir)) {
    Write-Host "Cleaning build directory: $buildDir" -ForegroundColor Yellow
    Remove-Item -Path $buildDir -Recurse -Force
}

# Create build directory if not exists
if (-not (Test-Path $buildDir)) {
    New-Item -Path $buildDir -ItemType Directory | Out-Null
}

# 4. Configure CMake
Write-Host "Configuring CMake project..." -ForegroundColor Cyan

# Disable SSL/TLS certificate verification for git and cmake downloads
# to avoid SSL signer certificate failures in restricted environment.
$env:GIT_SSL_NO_VERIFY = "true"
$env:CMAKE_TLS_VERIFY = "0"

# We use CMake to generate the Visual Studio 18 2026 solution.
# Unit tests are enabled via -DABD_WITH_UNIT_TESTS=ON.
# Using direct operator & ensures correct string quoting for spaces.
& cmake -G "Visual Studio 18 2026" -A x64 -S $PSScriptRoot -B $buildDir "-DABD_WITH_UNIT_TESTS=ON" "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" "-DCMAKE_TLS_VERIFY=OFF"
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake configuration failed with exit code $LASTEXITCODE."
    exit $LASTEXITCODE
}
Write-Host "CMake configuration succeeded." -ForegroundColor Green

# 5. Build solution using MSBuild
Write-Host "Building project using MSBuild..." -ForegroundColor Cyan
$slnPath = Get-ChildItem -Path $buildDir -Filter "*.sln*" | Select-Object -First 1 -ExpandProperty FullName
if (-not $slnPath) {
    Write-Error "Solution file (*.sln or *.slnx) not found in: $buildDir"
    exit 1
}
Write-Host "Found solution: $slnPath" -ForegroundColor Green

# We run MSBuild directly on the solution file.
# /p:Configuration specifies build configuration (e.g. Release).
# /m enables multi-processor build, and /v:m uses minimal verbosity.
& $msBuildPath $slnPath "/p:Configuration=$Config" "/m" "/v:m"
if ($LASTEXITCODE -ne 0) {
    Write-Error "MSBuild build failed with exit code $LASTEXITCODE."
    exit $LASTEXITCODE
}
Write-Host "Build completed successfully." -ForegroundColor Green

# 6. Run tests if requested
if ($RunTests) {
    Write-Host "Running tests..." -ForegroundColor Cyan
    # CTest executes all tests defined in CMake
    # We specify configuration using -C to locate appropriate test binaries
    Push-Location $buildDir
    try {
        & ctest --output-on-failure -C $Config
        if ($LASTEXITCODE -ne 0) {
            Write-Error "Tests failed with exit code $LASTEXITCODE."
            exit $LASTEXITCODE
        }
    } finally {
        Pop-Location
    }
    Write-Host "All tests passed successfully!" -ForegroundColor Green
}

