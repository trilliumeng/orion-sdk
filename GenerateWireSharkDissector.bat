set ROOT=%~dp0
"%ROOT%\Protogen\Windows\3.6k\bin\ProtoGen.exe" "%ROOT%\Communications\OrionPublicProtocol.xml" "%ROOT%\Communications"  -no-doxygen -yes-wireshark -no-code
copy "%ROOT%Communications\OrionPublic.lua" "%ROOT%wireshark\OrionPublic.lua"
exit /b 0
