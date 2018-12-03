NUX_PATH=/home/horo/arm/prj428/tmp/W55FA92BSP2.6.35_160719/linux-2.6.35.4       
obj-$(CONFIG_CAN0) += can0_drv.o can0_dev.o
all:
	#编译内核模块要用到内核中的Makefile
	    make -C $(LINUX_PATH) M=`pwd` modules
clean:
	    make -C $(LINUX_PATH) M=`pwd` modules clean
