@echo on
setlocal enabledelayedexpansion
cd ./venv/Scripts
call activate.bat
cd ../..
set /p default="Do you want to use default parameters? [y/n]:":
set /p com="Enter COM port [name]:":
set /p len="Enter length of window [number]:":
set /p log="Enter log parameter [y/n]:":
if %default%==n (
echo Running python script with custom parameters!
echo =============================================
run.py -comm %com% -len %len% -log %log%
pause
) else (
echo Running python script with default parameters
echo =============================================
run.py -comm COM7 -len 1024 -log y
pause
)