@echo off
setlocal
:: Source the ESP-IDF environment (EIM install) then launch the interactive tool.
:: idf.py is a PowerShell alias created by the profile — must be sourced first.
PowerShell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
  "$prof='C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1'; if(Test-Path $prof){ . $prof }; & '%~dp0vk_firmware_tool.ps1'"
endlocal
