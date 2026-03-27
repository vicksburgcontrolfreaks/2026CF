@echo off
REM Script to sync documentation updates from current branch to gh-pages

echo Syncing documentation to gh-pages...

REM Save current branch
for /f "tokens=*" %%i in ('git branch --show-current') do set CURRENT_BRANCH=%%i
echo Current branch: %CURRENT_BRANCH%

REM Check if there are uncommitted changes
git status -s > nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo Warning: You have uncommitted changes. Consider committing them first.
    set /p CONTINUE=Continue anyway? (y/n):
    if /i not "%CONTINUE%"=="y" (
        echo Aborted.
        exit /b 1
    )
)

REM Switch to gh-pages
echo Switching to gh-pages branch...
git checkout gh-pages

REM Copy documentation files from the original branch
echo Copying documentation files from %CURRENT_BRANCH%...
git checkout %CURRENT_BRANCH% -- SHOOTER_TEST_USAGE.md PID_TUNING_GUIDE.md SHOOTER_SYSTEM_DOCUMENTATION.md SHOOT_WHILE_DRIVING.md

REM Copy and add Jekyll front matter
echo Processing files for GitHub Pages...

REM SHOOTER_TEST_USAGE.md -> shooter-testing.md
(
echo ---
echo layout: default
echo title: Shooter Test System
echo ---
echo.
type SHOOTER_TEST_USAGE.md
) > shooter-testing.md

REM PID_TUNING_GUIDE.md -> pid-tuning.md
(
echo ---
echo layout: default
echo title: PID Tuning Guide
echo ---
echo.
type PID_TUNING_GUIDE.md
) > pid-tuning.md

REM SHOOTER_SYSTEM_DOCUMENTATION.md -> shooter-system.md
(
echo ---
echo layout: default
echo title: Shooter System
echo ---
echo.
type SHOOTER_SYSTEM_DOCUMENTATION.md
) > shooter-system.md

REM SHOOT_WHILE_DRIVING.md -> shoot-while-driving.md
(
echo ---
echo layout: default
echo title: Shoot While Driving
echo ---
echo.
type SHOOT_WHILE_DRIVING.md
) > shoot-while-driving.md

REM Remove the originals
del SHOOTER_TEST_USAGE.md PID_TUNING_GUIDE.md SHOOTER_SYSTEM_DOCUMENTATION.md SHOOT_WHILE_DRIVING.md

REM Stage the changes
echo Staging changes...
git add shooter-testing.md pid-tuning.md shooter-system.md shoot-while-driving.md

REM Check if there are changes to commit
git diff --staged --quiet
if %ERRORLEVEL% EQU 0 (
    echo No documentation changes to sync.
) else (
    REM Commit the changes
    echo Committing changes...
    git commit -m "Sync documentation from %CURRENT_BRANCH%"

    REM Push to origin
    echo Pushing to origin/gh-pages...
    git push origin gh-pages

    echo Documentation synced successfully!
)

REM Switch back to original branch
echo Switching back to %CURRENT_BRANCH%...
git checkout %CURRENT_BRANCH%

echo Done! Your GitHub Pages site will update in a few minutes.
echo View your site at: https://vicksburgcontrolfreaks.github.io/2026CF/
