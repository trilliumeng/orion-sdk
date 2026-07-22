@echo off
set ROOT=%~dp0

REM Generate C code
"%ROOT%\Protogen\Windows\ProtoGen.exe" "%ROOT%\Communications\OrionPublicProtocol.xml" "%ROOT%\Communications" -no-doxygen

REM Generate Python bindings and compile test harness (if python is available)
where python >nul 2>nul
if %ERRORLEVEL% EQU 0 (
    set PYTHONPATH=%ROOT%\PythonGen
    python -m orion_sdk.cli "%ROOT%\Communications\OrionPublicProtocol.xml" "%ROOT%\Communications\python\orion_sdk" --c-header-path "%ROOT%\Communications\OrionPublicPacket.h" --compile-tests --project-root "%ROOT%" --install-deps
) else (
    echo Warning: python not found, skipping Python bindings generation
)

exit /b 0
