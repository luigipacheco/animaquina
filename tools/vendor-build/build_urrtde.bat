@echo off
rem ===========================================================================
rem Compile ur_rtde from source for a target Python -> a cpXYZ wheel containing
rem rtde_*.pyd, dashboard_client.pyd, script_client.pyd, rtde.dll, urcl/.
rem ur_rtde has NO prebuilt Windows wheels, so this is the only way to get it
rem for a new Python/Blender version.
rem
rem Prereqs (one-time, see README.md):
rem   - VS 2022 C++ Build Tools (MSVC v143)
rem   - Boost static libs built (run build_boost.bat)
rem   - standalone CMake 3.29.x  (MUST be < 3.30: ur_rtde's CMakeLists needs the
rem     legacy FindBoost module, which CMake >=3.30 drops via policy CMP0167)
rem   - the TARGET python.org interpreter (it has Python.h/.lib; Blender's does not.
rem     cpXYZ ABI is stable across patch releases, so the .pyd loads in Blender.)
rem ===========================================================================
setlocal
set "BUILD_DIR=%USERPROFILE%\animaquina-build"
set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
set "CMAKE_BIN=%BUILD_DIR%\cmake-3.29.6-windows-x86_64\bin"
set "PYTARGET=%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
set "BOOST_ROOT=%BUILD_DIR%\boost_1_86_0"
set "URRTDE_VER=1.6.3"

set "NoDefaultCurrentDirectoryInExePath="
call "%VCVARS%"

rem Put the pre-3.30 CMake first (legacy FindBoost), and ninja off PATH so the
rem default VS generator is used (avoids ninja + '-A x64' conflict).
set "PATH=%CMAKE_BIN%;%PATH%"

rem Short TMP: pip builds in a deep temp dir and MSBuild FileTracker hits MAX_PATH.
set "TMP=C:\b"
set "TEMP=C:\b"
if not exist C:\b mkdir C:\b

set "BOOST_LIBRARYDIR=%BOOST_ROOT%\stage\lib"
where cmake
cmake --version
"%PYTARGET%" --version
"%PYTARGET%" -m pip wheel "ur_rtde==%URRTDE_VER%" --no-deps -w "%BUILD_DIR%\urrtde_wheel"
if errorlevel 1 ( echo URRTDE_WHEEL_FAILED & exit /b 1 )
echo === wheel ===
dir /b "%BUILD_DIR%\urrtde_wheel"
echo URRTDE_WHEEL_DONE
