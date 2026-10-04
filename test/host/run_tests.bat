@echo off
rem Builds and runs the host unit tests for the INS/EKF.
rem ins_ekf.cpp has no Arduino dependencies, so no stubs are needed.
setlocal
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cl /nologo /std:c++17 /EHsc /W3 /Fe:"%~dp0test_ins.exe" /Fo:"%~dp0test_ins.obj" "%~dp0test_ins.cpp"
if errorlevel 1 exit /b 1
"%~dp0test_ins.exe"
