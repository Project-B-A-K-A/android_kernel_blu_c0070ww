
ifeq ($(BUILD),debug)
ccflags-y += -DDEBUG
endif
obj-y += chipone-tddi.o
chipone-tddi-objs := cts_i2c_driver.o \
		     cts_core.o  \
		     cts_platform.o \
		     cts_sfctrlv2.o \
		     cts_spi_flash.o \
		     cts_firmware.o \
		     cts_tool.o \
		     cts_sysfs.o \
		     cts_test.o
