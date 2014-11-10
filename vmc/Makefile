obj-m := ibmvmc.o

KDIR  := /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# Temporary until I come up with a better way
install:
	mknod /dev/ibmvmc c 249 0
