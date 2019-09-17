@echo on
setlocal enabledelayedexpansion
cd ./venv/Scripts
call activate.bat
cd ../..
set /p default="Do you want to use default parameters? [y/n]:":
set /p com="Enter communication way:":
set /p len="Enter length of window [number]:":
set /p log="Enter log parameter [y/n]:":
set /p qrs="Enter qrs parameter [y/n]:":
if %default%==n (
echo Running python script with custom parameters!
echo =============================================
run.py -comm %com% -qrs %qrs% -len %len% -log %log%
pause
) else (
echo Running python script with default parameters
echo =============================================
run.py -comm TCP:127.0.0.1:5005 -qrs y -len 4096 -log y
pause
)