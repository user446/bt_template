@echo on
setlocal enabledelayedexpansion
cd ./venv/Scripts
call activate.bat
cd ../..
edf_server.py
pause