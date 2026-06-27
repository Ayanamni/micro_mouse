@echo off
chcp 65001 >nul 2>&1
title Micromouse Workbench
cd /d "%~dp0"

echo ========================================
echo   Micromouse Simulation Workbench
echo ========================================
echo.
echo   [1] Workbench - Dashboard + tuning (recommended)
echo   [2] Viewer run - 3D auto line following
echo   [3] Interactive - keyboard manual driving
echo.
choice /c 123 /n /m "Select mode [1/2/3]: "
if errorlevel 3 goto interactive
if errorlevel 2 goto batch
if errorlevel 1 goto workbench

:workbench
echo.
echo Select track:
echo   [1] Robotena  (4.80m, tight turns, min radius ~9cm)
echo   [2] 2019Kansai (14.75m, wider layout, min radius ~14cm)
choice /c 12 /n /m "Select track [1/2]: "
if errorlevel 2 goto workbench_kansai
if errorlevel 1 goto workbench_robotena
:workbench_kansai
set TRACK=2019kansai
goto workbench_launch
:workbench_robotena
set TRACK=robotena
:workbench_launch
echo.
echo Starting Workbench: %TRACK% (dashboard only, no 3D viewer)...
echo   Space = Pause   R = Reset   Ctrl+S = Save
echo   Q/Esc = Quit    F12 = Screenshot
echo.
python scripts\workbench.py --track %TRACK%
goto end

:batch
echo.
echo Select track:
echo   [1] Robotena  (4.80m, tight turns, min radius ~9cm)
echo   [2] 2019Kansai (14.75m, wider layout, min radius ~14cm)
choice /c 12 /n /m "Select track [1/2]: "
if errorlevel 2 goto batch_kansai
if errorlevel 1 goto batch_robotena
:batch_kansai
set TRACK=2019kansai
goto batch_launch
:batch_robotena
set TRACK=robotena
:batch_launch
echo.
echo Viewer run: %TRACK%, 15s, 3.5 m/s (auto line following)
echo   Close MuJoCo window or Ctrl+C to stop
python scripts\workbench.py --track %TRACK% --viewer-only --duration 15
goto end

:interactive
echo.
echo Starting Interactive mode (manual keyboard driving)...
echo   Arrows = drive   Q/E = gear   R = reset   Esc = quit
echo.
python scripts\interactive.py
goto end

:end
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Failed to start! Check dependencies:
    echo   pip install mujoco numpy scipy pyyaml h5py matplotlib
    pause
)
