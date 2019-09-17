@echo on
setlocal enabledelayedexpansion
cd ./venv/Scripts
call activate.bat
cd ../..
run_server.py
pause