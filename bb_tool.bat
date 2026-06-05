@echo off
setlocal
:: Ball Balancer 3RPS - Firmware Tool
:: Carrega o ambiente ESP-IDF (instalacao Espressif/EIM) e abre o menu interativo.
:: idf.py e um alias de PowerShell criado pelo profile -- precisa ser carregado antes.
PowerShell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
  "$prof='C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1'; if(Test-Path $prof){ . $prof }; & '%~dp0bb_tool.ps1'"
endlocal
