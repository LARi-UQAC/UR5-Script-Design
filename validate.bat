@echo off
setlocal
chcp 65001 >nul
set PYTHONIOENCODING=utf-8
pushd "%~dp0"

REM Active l'environnement virtuel local (Python 3.13, swift-sim + websockets<13).
REM Le venv isole les conflits avec l'environnement Python global (notamment
REM google-genai qui exige websockets>=13).
if exist ".venv\Scripts\activate.bat" (
    call ".venv\Scripts\activate.bat"
) else (
    echo [WARN] .venv absent : utilisation du Python systeme.
    echo        Pour creer le venv : python -m venv .venv ^&^& .venv\Scripts\activate ^&^& pip install -r requirements.txt
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
echo   0. Quitter
echo.
set /p choice="Choix : "

if "%choice%"=="1" (
    python -m ur5_sim --check
    goto pause_back
)
if "%choice%"=="2" (
    python -m ur5_sim --check --identity
    goto pause_back
)
if "%choice%"=="3" (
    python -m ur5_sim --visualize
    goto pause_back
)
if "%choice%"=="4" (
    python -m ur5_sim --visualize --identity
    goto pause_back
)
if "%choice%"=="5" (
    REM Lance l'UI de conception en fenetre detachee. Le bouton START
    REM interne exporte etalement.script puis ouvre ur5_sim --visualize.
    REM Si etalement.script existe deja, on ouvre aussi le viewer 3D immediatement
    REM pour avoir les deux systemes en parallele.
    start "UR5 - UI conception" python ur5_etalementv6.py
    if exist "%~dp0etalement.script" (
        start "UR5 - Viewer 3D" python -m ur5_sim --visualize
    ) else (
        echo.
        echo [INFO] etalement.script absent : exportez-le depuis l'UI puis cliquez START.
    )
    goto pause_back
)
if "%choice%"=="6" (
    python ur5_etalementv6.py
    goto pause_back
)
if "%choice%"=="0" goto end
goto menu

:pause_back
echo.
echo ------------------------------------------------------------
pause
goto menu

:end
popd
endlocal
