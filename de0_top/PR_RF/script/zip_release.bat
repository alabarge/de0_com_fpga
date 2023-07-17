rmdir /s /q .\release
echo f|xcopy .\output_files\de0_fpga.sof .\release\DE0_F_FPGA_Project.sof /F /Y /R
echo f|xcopy .\output_files\de0_fpga_v*.sof .\release\de0_fpga_v*.sof /F /Y /R
echo f|xcopy .\output_files\fpga_version.h .\release\fpga_version.h /F /Y /R
echo f|xcopy fpga_build.h .\release\fpga_build.h /F /Y /R
echo f|xcopy fpga_build.vhd .\release\fpga_build.vhd /F /Y /R
echo f|xcopy .\output_files\de0_fpga_crc32.txt .\release\de0_fpga_crc32.txt /F /Y /R
echo f|xcopy .\output_files\de0_fpga_checksum.txt .\release\de0_fpga_checksum.txt /F /Y /R
echo f|xcopy .\output_files\de0_fpga_md5.txt .\release\de0_fpga_md5.txt /F /Y /R
if exist ".\de0_fpga_release_*.zip" del /F de0_fpga_release_*.zip
cd release
..\..\..\utils\fpga_zip.exe
