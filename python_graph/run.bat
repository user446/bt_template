cd ./venv/Scripts
call activate.bat
cd ../..
set /p default=Do you want to use default parameters? [y/n]:
if %default%==n (
set /p com=Enter COM port [name]: 
set /p len=Enter length of window [number]: 
set /p log=Enter log parameter [True/False]: 
echo Running python script with custom parameters!
echo =============================================
run.py -com %com% -len %len% -log %log%
) else (
echo Running python script with default parameters
echo =============================================
run.py -com COM7 -len 1024 -log True
)