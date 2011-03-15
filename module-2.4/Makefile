TARGET = mramfs
OBJS = inode.o compression.o lookup.o uncompress.o
MDIR = fs/

#EXTRA_CFLAGS = -DEXPORT_SYMTAB
# EXTRA_CFLAGS = -DDEBUG
CURRENT = $(shell uname -r)
KDIR = /lib/modules/$(CURRENT)/build
PWD = $(shell pwd)
DEST = /lib/modules/$(CURRENT)/kernel/$(MDIR)

obj-m      := $(TARGET).o

default:
	make -C $(KDIR) SUBDIRS=$(PWD) modules

$(TARGET).o: $(OBJS)
	$(LD) $(LD_RFLAG) -r -o $@ $(OBJS)

ifneq (,$(findstring 2.4.,$(CURRENT)))
install:
	su -c "cp -v $(TARGET).o $(DEST) && /sbin/depmod -a"
else
install:
	su -c "cp -v $(TARGET).ko $(DEST) && /sbin/depmod -a"
endif

clean:
	-rm -f *.o *.ko .*.cmd .*.flags *.mod.c

-include $(KDIR)/Rules.make
