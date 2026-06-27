@echo off
chcp 65001 >nul 2>&1
title Building C++ Modules
cd /d "%~dp0.."

echo ========================================
echo   Building C++ Control & Localize Cores
echo ========================================
echo.

set "CMAKE_GEN=Visual Studio 17 2022"
set "BUILD_DIR=cpp\build"

echo [1/3] Configuring CMake...
cmake -S cpp -B %BUILD_DIR% -G "%CMAKE_GEN%" -A x64
if %errorlevel% neq 0 (
    echo [FAIL] CMake configure failed!
    pause
    exit /b 1
)

echo.
echo [2/3] Building Release...
cmake --build %BUILD_DIR% --config Release
if %errorlevel% neq 0 (
    echo [FAIL] Build failed!
    pause
    exit /b 1
)

echo.
echo [3/3] Copying .pyd files to package...
for /r "%BUILD_DIR%" %%f in (localize_core*.pyd) do copy /y "%%f" "micromouse_sim\" >nul
for /r "%BUILD_DIR%" %%f in (control_core*.pyd) do copy /y "%%f" "micromouse_sim\" >nul

echo.
echo ========================================
echo   Build SUCCESS
echo   Modules: micromouse_sim/localize_core.pyd
echo            micromouse_sim/control_core.pyd
echo ========================================
echo.
echo Test: python -c "import localize_core; import control_core; print('OK')"
pause
