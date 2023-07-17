if exist ".\release\" rmdir /s /q .\release
if exist ".\output_files\" rmdir /s /q .\output_files
..\..\utils\fpga_pre_ver.exe fpga_build.inc fpga_build.h fpga_build.vhd ..\..\.git
echo f|xcopy fpga_build.h output_files\fpga_build.h /F /Y /R
echo f|xcopy fpga_build.h ..\..\nios\de0_fw\share\fpga_build.h /F /Y /R
