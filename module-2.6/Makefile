#
# Makefile for the linux ramfs routines.
#

obj-m += mramfs.o

mramfs-objs := compression.o inode.o uncompress.o lookup.o

default:
	CFLAGS="-g" make -C /usr/src/linux-`uname -r`/ SUBDIRS=`pwd` modules

clean:
	rm *.o *.ko 
	rm /lib/modules/`uname -r`/kernel/fs/mramfs.ko
	
install:
	cp mramfs.ko /lib/modules/`uname -r`/kernel/fs/
	depmod
