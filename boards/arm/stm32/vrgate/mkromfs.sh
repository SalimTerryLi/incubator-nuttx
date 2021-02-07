genromfs -f rom.img -d romfs/
genromfs -f romfs.img -d etcfs/
xxd -i romfs.img > include/nsh_romfsimg.h
rm romfs.img
