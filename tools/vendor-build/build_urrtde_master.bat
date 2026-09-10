@echo off
set "NoDefaultCurrentDirectoryInExePath="
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
rem force standalone CMake 3.29 (pre-CMP0167) so find_package(Boost) uses the legacy FindBoost module + BOOST_ROOT
set "PATH=C:\Users\ADM-lpachecoalcala\animaquina-build\cmake-3.29.6-windows-x86_64\bin;%PATH%"
rem short temp dir to keep build paths under MAX_PATH (MSBuild FileTracker)
set "TMP=C:\b"
set "TEMP=C:\b"
if not exist C:\b mkdir C:\b
set "BOOST_ROOT=C:\Users\ADM-lpachecoalcala\animaquina-build\boost_1_86_0"
set "BOOST_LIBRARYDIR=C:\Users\ADM-lpachecoalcala\animaquina-build\boost_1_86_0\stage\lib"
set "PYORG=%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
echo === CMAKE IN USE ===
where cmake
cmake --version
echo === PYTHON ===
"%PYORG%" --version
echo === BUILDING ur_rtde WHEEL (cp313, master) ===
"%PYORG%" -m pip wheel "C:\Users\ADM-lpachecoalcala\animaquina-build\ur_rtde_master" --no-deps -w "C:\Users\ADM-lpachecoalcala\animaquina-build\urrtde_wheel_master"
if errorlevel 1 ( echo URRTDE_WHEEL_FAILED & exit /b 1 )
echo === WHEEL OUTPUT ===
dir /b "C:\Users\ADM-lpachecoalcala\animaquina-build\urrtde_wheel_master"
echo URRTDE_WHEEL_DONE
