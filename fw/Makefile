# put your *.o targets here, make should handle the rest!
CMSIS_PATH 		?= STM32Cube/Drivers/CMSIS
CMSIS_DEV_PATH  ?= $(CMSIS_PATH)/Device/ST/STM32F0xx
HAL_PATH   		?= STM32Cube/Drivers/STM32F0xx_HAL_Driver

CC      := arm-none-eabi-gcc
LD      := arm-none-eabi-ld
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size

CFLAGS  = -g -Wall -std=gnu11 -O2 -fdump-rtl-expand
CFLAGS += -mlittle-endian -mcpu=cortex-m0 -march=armv6-m -mthumb
#CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS = -nostartfiles
#LDFLAGS += -specs=rdimon.specs -DSEMIHOSTING
LDFLAGS += -Wl,-Map=main.map -nostdlib
#LDFLAGS += -Wl,--gc-sections 
LIBS = -lgcc
#LIBS += -lrdimon

# Technically we're using an STM32F030F4, but apart from the TSSOP20 package that one is largely identical to the
# STM32F030*6 and there is no separate device header provided for it, so we're faking a *6 device here. This is
# even documented in stm32f0xx.h. Thanks ST!
CFLAGS += -DSTM32F030x6 -DHSE_VALUE=25000000

LDFLAGS += -Tstm32_flash.ld
CFLAGS += -I$(CMSIS_DEV_PATH)/Include -I$(CMSIS_PATH)/Include -I$(HAL_PATH)/Inc -Iconfig
LDFLAGS += -L$(CMSIS_PATH)/Lib/GCC -larm_cortexM0l_math

###################################################

.PHONY: program clean

all: main.elf

cmsis_exports.c: $(CMSIS_DEV_PATH)/Include/stm32f030x6.h $(CMSIS_PATH)/Include/core_cm0.h
	python3 gen_cmsis_exports.py $^ > $@

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $^
	$(CC) -E $(CFLAGS) -o $(@:.o=.pp) $^

%.o: %.s
	$(CC) -c $(CFLAGS) -o $@ $^
	$(CC) -E $(CFLAGS) -o $(@:.o=.pp) $^

%.dot: %.elf
	r2 -a arm -qc 'aa;agC' $< 2>/dev/null >$@
	
sources.tar.xz: main.c transpose.c transpose.h
	tar -caf $@ $^

sources.c: sources.tar.xz
	xxd -i $< | sed 's/=/__attribute__((section(".text")))=/' > $@

main.elf: main.o startup_stm32f030x6.o system_stm32f0xx.o $(HAL_PATH)/Src/stm32f0xx_ll_utils.o base.o cmsis_exports.o transpose.o bus_addr.o sources.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(OBJCOPY) -O ihex $@ $(@:.elf=.hex)
	$(OBJCOPY) -O binary $@ $(@:.elf=.bin)
	$(OBJDUMP) -St $@ >$(@:.elf=.lst)
	$(SIZE) $@
	
program: main.elf openocd.cfg
	openocd -f openocd.cfg -c "program $< verify reset exit"

transpose.elf: transpose.c transpose_main.c
	gcc -g -Wall -Wpedantic -o $@ $^

.PHONY: transpose_test
transpose_test: transpose.elf
	./transpose.elf

_crc.so: crc.c
	gcc -o $@ -shared -fPIC -g $^

clean:
	rm -f **.o
	rm -f main.elf main.hex main.bin main.map main.lst
	rm -f **.expand
	rm -f transpose.elf
	rm -f crc.so
	rm -f sources.tar.xz
	rm -f sources.c

