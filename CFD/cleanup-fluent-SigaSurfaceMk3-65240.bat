echo off
set LOCALHOST=%COMPUTERNAME%
set KILL_CMD="C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v261\fluent/ntbin/win64/winkill.exe"

start "tell.exe" /B "C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v261\fluent\ntbin\win64\tell.exe" SigaSurfaceMk3 60639 CLEANUP_EXITING
timeout /t 1
"C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v261\fluent\ntbin\win64\kill.exe" tell.exe
if /i "%LOCALHOST%"=="SigaSurfaceMk3" (%KILL_CMD% 46604) 
if /i "%LOCALHOST%"=="SigaSurfaceMk3" (%KILL_CMD% 65240) 
if /i "%LOCALHOST%"=="SigaSurfaceMk3" (%KILL_CMD% 28524)
del "C:\Users\johns\MAE155B-Aircraft-Design\CFD\cleanup-fluent-SigaSurfaceMk3-65240.bat"
