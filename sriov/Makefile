
# 驱动名称，在pag接口中需要修改为zywk
MOD_NAME :=pcie_driver_deamon

obj-m :=$(MOD_NAME).o
KERNELDIR ?=/lib/modules/$(shell uname -r)/build
PWD ?=$(shell pwd)

default:
	$(MAKE) -C ${KERNELDIR} M=${PWD} modules



#清理垃圾的伪代码
clean:
	$(RM) -rf *.o *.ko .tmp_versions *~ Module.symvers .*.cmd *.mod.c modules.*
