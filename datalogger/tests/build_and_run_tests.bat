@echo off
REM Build and run the RTDE fallback monitor test harness.
REM Requires MinGW-w64 gcc on PATH (x86_64-w64-mingw32). No other dependency.
setlocal
cd /d "%~dp0"

gcc -O2 -static -Wall -Wextra -o test_rtde_fallback_monitor.exe test_rtde_fallback_monitor.c -lws2_32 -lm
if errorlevel 1 (
    echo BUILD FAILED
    exit /b 1
)

.\test_rtde_fallback_monitor.exe
if errorlevel 1 (
    echo TESTS FAILED
    exit /b 1
)

echo TESTS PASSED
exit /b 0
