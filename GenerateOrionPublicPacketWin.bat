set ROOT=%~dp0
"%ROOT%\Protogen\Windows\ProtoGen.exe" "%ROOT%\Communications\OrionPublicProtocol.xml" "%ROOT%\Communications" -no-doxygen -no-unrecognized-warnings
exit /b 0
