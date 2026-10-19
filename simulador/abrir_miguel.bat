@echo off
rem Abre o Miguel 5250 pelo endereco http://localhost:8052 (o Chrome guarda a permissao do microfone).
rem Deixe este arquivo na mesma pasta que miguel-5250.html e de dois cliques nele.
cd /d "%~dp0"
start "Servidor Miguel" /min python -m http.server 8052 --bind 127.0.0.1
timeout /t 2 /nobreak >nul
start chrome "http://localhost:8052/miguel-5250.html" || start msedge "http://localhost:8052/miguel-5250.html"
