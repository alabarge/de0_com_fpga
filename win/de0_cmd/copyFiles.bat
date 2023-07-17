ECHO F|xcopy %1%2\%3\%4.exe %1..\..\data\%4.exe /F /Y
ECHO F|xcopy %1driver\wsc64.dll %1..\..\data\wsc64.dll /F /Y
ECHO F|xcopy %1driver\csc64.dll %1..\..\data\csc64.dll /F /Y
ECHO F|xcopy %1driver\wpcap.dll %1..\..\data\wpcap.dll /F /Y
exit 0