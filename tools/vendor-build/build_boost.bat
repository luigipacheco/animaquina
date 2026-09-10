@echo off
rem ===========================================================================
rem Build Boost static libs (system, thread, program_options) needed by ur_rtde.
rem One-time per machine / per Boost version. Output: %BOOST_DIR%\stage\lib
rem Prereqs: VS 2022 C++ Build Tools; Boost source extracted at %BOOST_DIR%.
rem ===========================================================================
setlocal
set "BUILD_DIR=%USERPROFILE%\animaquina-build"
set "BOOST_DIR=%BUILD_DIR%\boost_1_86_0"
set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"

rem This env var (set on some corp images) stops cmd finding boost's helper
rem .bat files in the cwd -> bootstrap fails with "guess_toolset.bat not recognized".
set "NoDefaultCurrentDirectoryInExePath="

call "%VCVARS%"
where cl || ( echo NO_MSVC & exit /b 1 )

pushd "%BOOST_DIR%"
if not exist b2.exe call "%BOOST_DIR%\bootstrap.bat"
if not exist b2.exe ( echo BOOTSTRAP_FAILED & popd & exit /b 1 )
b2.exe --with-system --with-thread --with-program_options ^
  link=static runtime-link=shared threading=multi address-model=64 architecture=x86 -j8 stage
if errorlevel 1 ( echo B2_FAILED & popd & exit /b 1 )
echo === stage\lib ===
dir /b stage\lib
popd
echo BOOST_BUILD_DONE
