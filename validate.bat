@echo off
setlocal
chcp 65001 >nul
set PYTHONIOENCODING=utf-8
pushd "%~dp0"

REM Interprete l'environnement virtuel local par CHEMIN ABSOLU (Python 3.13,
REM swift-sim + websockets<13), jamais via un `python` nu resolu par PATH.
REM Mesure 2026-09-24 : un `python` nu peut resoudre vers un Python global
REM (y compris celui d'un AUTRE compte Windows, admin, qui a sa propre copie
REM parallele de swift-sim avec un websockets non-epingle) et planter, tres
REM tard, au fond d'un thread Swift plutot qu'au demarrage (CLAUDE.md,
REM "Dependency pinning"). `start` (option 5) herite normalement le PATH
REM active, mais le chemin absolu rend ce fonctionnement independant du
REM contexte d'appel (compte, elevation, fenetre deja ouverte).
set "PY=%~dp0.venv\Scripts\python.exe"
if not exist "%PY%" (
    echo [WARN] .venv absent : utilisation du Python systeme.
    echo        Pour creer le venv : python -m venv .venv ^&^& .venv\Scripts\activate ^&^& pip install -r requirements.txt
    set "PY=python"
) else (
    call ".venv\Scripts\activate.bat"
)

:menu
cls
echo ============================================================
echo  Validation trajectoire UR5 - etalement.script
echo ============================================================
echo.
echo  Cible : P_REF        = p[-0.011, 0.6, -0.3, 0, -3.1416, 0]
echo  Ref   : P_ANCHOR_OLD = p[0.2, -0.335, 0.05, 3.14159, 0, 0]
echo.
echo  ----- Verification cinematique -----
echo   1. Check rapide       (P_REF cible)
echo   2. Check identite     (P_REF = P_ANCHOR_OLD, sanity refactor)
echo.
echo  ----- Visualisation 3 panneaux -----
echo   3. Visualiser         (P_REF cible)
echo   4. Visualiser         (P_REF = P_ANCHOR_OLD)
echo.
echo  ----- Outils combines -----
echo   5. Demarrer les deux  (UI conception + viewer 3D simultanes)
echo   6. UI conception seule (ur5_etalementv6.py)
echo.
echo  ----- Emulateur RTDE (sans robot) -----
echo   7. Emulateur seul     (headless, 2 essais + moniteur)
echo   8. Verifier le CSV    (dernier ACQ_rtde_*.csv)
echo.
echo   0. Quitter
echo.
set /p choice="Choix : "

if "%choice%"=="1" (
    "%PY%" -m ur5_sim --check
    goto pause_back
)
if "%choice%"=="2" (
    "%PY%" -m ur5_sim --check --identity
    goto pause_back
)
if "%choice%"=="3" (
    call :start_monitor
    "%PY%" -m ur5_sim --visualize
    goto pause_back
)
if "%choice%"=="4" (
    call :start_monitor
    "%PY%" -m ur5_sim --visualize --identity
    goto pause_back
)
if "%choice%"=="5" (
    REM Lance l'UI de conception en fenetre detachee. Si etalement.script
    REM existe deja, ouvre aussi le viewer 3D immediatement pour avoir les
    REM deux systemes en parallele (l'UI n'ouvre pas elle-meme le viewer :
    REM son bouton START exporte le script mais ne lance pas ur5_sim).
    start "UR5 - UI conception" "%PY%" ur5_etalementv6.py
    if exist "%~dp0etalement.script" (
        start "UR5 - Viewer 3D" "%PY%" -m ur5_sim --visualize
    ) else (
        echo.
        echo [INFO] etalement.script absent : exportez-le depuis l'UI puis cliquez START.
    )
    goto pause_back
)
if "%choice%"=="6" (
    "%PY%" ur5_etalementv6.py
    goto pause_back
)
if "%choice%"=="7" (
    call :start_monitor
    "%PY%" -m ur5_sim --emulate --runs 2 --pause-at 30
    goto pause_back
)
if "%choice%"=="8" (
    "%PY%" -m ur5_sim --verify-csv auto
    goto pause_back
)
if "%choice%"=="0" goto end
goto menu

:start_monitor
if not exist "%~dp0datalogger\rtde_fallback_monitor.exe" (
    echo [WARN] datalogger\rtde_fallback_monitor.exe absent : lancez datalogger\build.bat
    echo        La visualisation continue sans le moniteur RTDE.
    goto :eof
)
if not exist "%~dp0datalogger\sim_runs" mkdir "%~dp0datalogger\sim_runs"
start "UR5 - Moniteur RTDE" "%~dp0datalogger\rtde_fallback_monitor.exe" 127.0.0.1 30004 "%~dp0datalogger\sim_runs"
goto :eof

:pause_back
echo.
echo ------------------------------------------------------------
pause
goto menu

:end
popd
endlocal
