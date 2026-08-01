@echo off
REM Prints this machine's local IPv4 address(es), excluding loopback and
REM link-local (169.254.x.x) addresses. Use the Wi-Fi one for CSI_TARGET_IP
REM in test-node/src/credentials.h.

echo Local IPv4 addresses:
echo.
powershell -NoProfile -Command "Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike '169.254.*' -and $_.IPAddress -ne '127.0.0.1' } | Select-Object InterfaceAlias, IPAddress | Format-Table -AutoSize"
