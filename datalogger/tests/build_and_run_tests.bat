@echo off
REM Build and run the RTDE fallback monitor test harness.
REM Requires MinGW-w64 gcc on PATH (x86_64-w64-mingw32). No other dependency.
REM --repeat N exists because F16 made the fake server's handshake close
REM deterministic: it re-runs the built harness N times and requires the
REM same exit code and the same summary line on every run, which is how
REM that determinism claim is actually checked rather than merely asserted.
REM Control flow below is goto-based, not nested if(...) blocks: cmd.exe
REM silently drops the exit code of an "exit /b" that sits inside a
REM parenthesized if-block which is itself nested inside another block and
REM followed by sibling commands in that same outer block (confirmed by
REM direct testing, 2026-08-20) - it still prints the right message but
REM returns 0. goto out to a top-level label is the reliable escape.
setlocal EnableDelayedExpansion
cd /d "%~dp0"

set "REPEAT=1"
set "ARG1=%~1"

if "%ARG1%"=="" goto :args_done
if /i not "%ARG1%"=="--repeat" goto :usage
if not "%~3"=="" goto :usage
set "REPEATARG=%~2"
if "%REPEATARG%"=="" goto :usage
echo %REPEATARG%| findstr /r "^[1-9][0-9]*$" >nul
if errorlevel 1 goto :usage
set "REPEAT=%REPEATARG%"

:args_done
gcc -O2 -static -Wall -Wextra -o test_rtde_fallback_monitor.exe test_rtde_fallback_monitor.c -lws2_32 -lm
if errorlevel 1 goto :build_failed

if not "%ARG1%"=="" goto :repeat_mode

.\test_rtde_fallback_monitor.exe
if errorlevel 1 goto :single_failed
echo TESTS PASSED
exit /b 0

:single_failed
echo TESTS FAILED
exit /b 1

:build_failed
echo BUILD FAILED
exit /b 1

:repeat_mode
REM --repeat path: every run must reproduce the first run's exit code and
REM its "N checks, N failure(s)" summary line, or the harness is not the
REM deterministic tool F16 claims it to be.
set "FIRST_SUMMARY="
set "ALL_OK=1"
del /q mismatch_run_*.out >nul 2>&1
for /l %%I in (1,1,%REPEAT%) do (
    ".\test_rtde_fallback_monitor.exe" > "run_%%I.out.tmp" 2>&1
    set "RC=!errorlevel!"
    set "SUMMARY="
    set "KEEP="
    for /f "delims=" %%L in ('findstr /c:" checks," "run_%%I.out.tmp"') do set "SUMMARY=%%L"
    if "!FIRST_SUMMARY!"=="" set "FIRST_SUMMARY=!SUMMARY!"
    if not "!RC!"=="0" (
        echo MISMATCH at run %%I: exit code !RC! ^(expected 0^)
        set "ALL_OK=0"
        set "KEEP=1"
    )
    if not "!SUMMARY!"=="!FIRST_SUMMARY!" (
        echo MISMATCH at run %%I: "!SUMMARY!" differs from run 1's "!FIRST_SUMMARY!"
        set "ALL_OK=0"
        set "KEEP=1"
    )
    REM A gate that detects a divergent run and then deletes its output
    REM leaves nothing to diagnose, which is the whole reason that run
    REM mattered. Keep the full stdout of any run that diverged, name it,
    REM and delete the rest.
    if "!KEEP!"=="1" (
        move /y "run_%%I.out.tmp" "mismatch_run_%%I.out" >nul
        echo   full output kept in mismatch_run_%%I.out
    )
)
del /q run_*.out.tmp >nul 2>&1

if "!ALL_OK!"=="0" goto :repeat_failed
echo TESTS PASSED ^(%REPEAT%/%REPEAT% identical: !FIRST_SUMMARY!^)
exit /b 0

:repeat_failed
echo TESTS FAILED - runs are not identical, see MISMATCH line^(s^) above
exit /b 1

:usage
echo Usage: build_and_run_tests.bat [--repeat N]
exit /b 2
