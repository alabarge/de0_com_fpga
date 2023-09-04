echo \n | ..\..\utils\fpga_checksum.exe  .\output_files\de0_fpga.sof .\output_files\de0_fpga_checksum.txt
echo \n | ..\..\utils\fpga_crc32.exe  .\output_files\de0_fpga.sof .\output_files\de0_fpga_crc32.txt
echo \n | CertUtil -hashfile .\output_files\de0_fpga.sof MD5 > .\output_files\de0_fpga_md5.txt
..\..\utils\fpga_post.exe .\output_files\de0_fpga_checksum.txt .\output_files\de0_fpga_crc32.txt .\output_files\de0_fpga_md5.txt .\output_files\fpga_build.h .\output_files\fpga_version.h
echo f|xcopy .\output_files\fpga_version.h ..\..\nios\de0_fw\share\fpga_version.h /F /Y /R
