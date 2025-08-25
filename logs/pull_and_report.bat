@echo off
setlocal ENABLEDELAYEDEXPANSION
cd /d "%~dp0"

REM Args: pull_and_report.bat [file] [topN] [midN] [botN]
set "REQFILE=%~1"
set "TOP=%~2"
set "MID=%~3"
set "BOT=%~4"
if not defined TOP set "TOP=10"
if not defined MID set "MID=10"
if not defined BOT set "BOT=20"

REM 1) Pull contents of remote logs into current folder (no nested logs)
adb pull /sdcard/Android/data/com.example.n3appfromscratch/files/logs/. . >nul 2>&1

REM 1a) If a nested .\logs\ appeared anyway, flatten it
if exist ".\logs\run_*.csv" (
  move /Y ".\logs\run_*.csv" "." >nul 2>&1
  rmdir /S /Q ".\logs" >nul 2>&1
)

REM 2) Decide which file to show
if defined REQFILE (
  if exist "%REQFILE%" (
    set "NEWEST=%REQFILE%"
  ) else (
    for /f "usebackq delims=" %%F in (`
      powershell -NoProfile -Command ^
        "(Get-ChildItem -File -Filter '%REQFILE%' | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName"
    `) do set "NEWEST=%%F"
  )
) else (
  for /f "usebackq delims=" %%F in (`
    powershell -NoProfile -Command ^
      "(Get-ChildItem -File -Filter 'run_*.csv' | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName"
  `) do set "NEWEST=%%F"
)

if not defined NEWEST (
  echo No matching CSV files found.
  exit /b 1
)

echo === NEWEST: %NEWEST%
echo.

REM 3) Top N
echo --- TOP %TOP% ---
powershell -NoProfile -Command "Get-Content -Path '%NEWEST%' -Head %TOP%"
echo.

REM 4) Middle N (centered)
echo --- MIDDLE %MID% ---
powershell -NoProfile -Command ^
  "$f='%NEWEST%'; $n=(Get-Content -Path $f).Count; $w=%MID%; $s=[math]::Max([math]::Floor($n/2)-[math]::Floor($w/2),0); Get-Content -Path $f | Select-Object -Skip $s -First $w"
echo.

REM 5) Bottom N
echo --- BOTTOM %BOT% ---
powershell -NoProfile -Command "Get-Content -Path '%NEWEST%' -Tail %BOT%"
echo.

endlocal
