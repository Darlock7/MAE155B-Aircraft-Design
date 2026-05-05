echo off
set LOCALHOST=%COMPUTERNAME%
set KILL_CMD="C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v261\fluent/ntbin/win64/winkill.exe"

start "tell.exe" /B "C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v261\fluent\ntbin\win64\tell.exe" SigaSurfaceMk3 50204 CLEANUP_EXITING
timeout /t 1
"C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v261\fluent\ntbin\win64\kill.exe" tell.exe
if /i "%LOCALHOST%"=="SigaSurfaceMk3" (%KILL_CMD% 70028) 
if /i "%LOCALHOST%"=="SigaSurfaceMk3" (%KILL_CMD% 59996) 
if /i "%LOCALHOST%"=="SigaSurfaceMk3" (%KILL_CMD% 67424)
del "C:\Users\johns\MAE155B-Aircraft-Design\CFD\cleanup-fluent-SigaSurfaceMk3-59996.bat"
