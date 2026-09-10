@echo off
rem ===========================================================================
rem Compile ur_rtde from a local MASTER checkout (not a PyPI release) -> cp313
rem wheel. Use this rather than build_urrtde.bat when you need PolyScope X
rem support and/or the ANIMAQUINA patch, neither of which is in a release.
rem
rem See README.md for why master is required and what the patch fixes.
rem
rem Prereqs (one-time, same as build_urrtde.bat):
rem   - VS 2022 C++ Build Tools (MSVC v143)
rem   - Boost static libs built (run build_boost.bat)
rem   - standalone CMake 3.29.x  (MUST be < 3.30: ur_rtde's CMakeLists needs the
rem     legacy FindBoost module, which CMake >=3.30 drops via policy CMP0167)
rem   - the TARGET python.org interpreter. Blender's bundled Python has no
rem     Python.h/.lib so it cannot be built against; the cpXYZ ABI is stable
rem     across patch releases, so a python.org-built .pyd loads in Blender.
rem   - ur_rtde master cloned to %URRTDE_SRC%, with the patch in patches/ applied
rem ===========================================================================
setlocal
set "BUILD_DIR=%USERPROFILE%\animaquina-build"
set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
set "CMAKE_BIN=%BUILD_DIR%\cmake-3.29.6-windows-x86_64\bin"
set "PYTARGET=%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
set "BOOST_ROOT=%BUILD_DIR%\boost_1_86_0"
set "URRTDE_SRC=%BUILD_DIR%\ur_rtde_master"
set "WHEEL_OUT=%BUILD_DIR%\urrtde_wheel_master"

set "NoDefaultCurrentDirectoryInExePath="
call "%VCVARS%"

rem Force the pre-3.30 CMake so find_package(Boost) uses the legacy FindBoost
rem module + BOOST_ROOT.
set "PATH=%CMAKE_BIN%;%PATH%"

rem Short TMP: pip builds in a deep temp dir and MSBuild FileTracker hits MAX_PATH.
set "TMP=C:\b"
set "TEMP=C:\b"
if not exist C:\b mkdir C:\b

set "BOOST_LIBRARYDIR=%BOOST_ROOT%\stage\lib"

echo === PREREQ CHECK ===
if not exist "%VCVARS%" ( echo MISSING: VS 2022 C++ Build Tools at "%VCVARS%" & exit /b 1 )
if not exist "%PYTARGET%" ( echo MISSING: python.org Python 3.13 at "%PYTARGET%" & exit /b 1 )
if not exist "%BOOST_ROOT%\stage\lib" ( echo MISSING: Boost static libs - run build_boost.bat first & exit /b 1 )
if not exist "%CMAKE_BIN%\cmake.exe" ( echo MISSING: standalone CMake 3.29.x at "%CMAKE_BIN%" & exit /b 1 )
if not exist "%URRTDE_SRC%" ( echo MISSING: ur_rtde master checkout at "%URRTDE_SRC%" & exit /b 1 )

echo === CMAKE IN USE ===
where cmake
cmake --version
echo === PYTHON ===
"%PYTARGET%" --version
echo === BUILDING ur_rtde WHEEL (cp313, master) ===
"%PYTARGET%" -m pip wheel "%URRTDE_SRC%" --no-deps -w "%WHEEL_OUT%"
if errorlevel 1 ( echo URRTDE_WHEEL_FAILED & exit /b 1 )
echo === WHEEL OUTPUT ===
dir /b "%WHEEL_OUT%"
echo URRTDE_WHEEL_DONE
