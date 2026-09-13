@echo off
setlocal
cd /d "%~dp0"
python "python\simulate_route.py" --source "track_source.jpg" --output "." --frames 546 --fps 20
pause
