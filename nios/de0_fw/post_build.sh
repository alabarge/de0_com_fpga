make mem_init_generate
../../utils/ci_check.exe de0_fw_ci.txt zipfs/de0_fw_ci.txt zipfs/de0_fw_ci.bin
rm -f zipfs.zip
cd zipfs
# only zip info (host = DOS, compression = STORE) works with zipfs.c
"/mnt/c/Program Files/7-Zip/7z.exe" a -tzip -mx0 ../zipfs.zip
# this version from WSL does not work
#7z a -tzip -mx0 ../zipfs.zip *
cd ..
