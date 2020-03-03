@echo on
setlocal enabledelayedexpansion
cd ./venv/Scripts
call activate.bat
cd ../..
set /p com="Enter communication way:":
echo Running python script with custom parameters!
echo =============================================
python run.py -comm %com% -qrs y -len 4096 -log n
pause