CROSS_PFIX := arm-none-eabi-
CC := ${CROSS_PFIX}gcc
GDB := ${CROSS_PFIX}gdb

OELF := out.elf
LDSCR := linker.ld
GDBSCR := debug.gdb

CFLAGS := -Wall -g3 -Os -mcpu=cortex-m3 -mfloat-abi=soft
CFLAGS += -mthumb
CFLAGS += -DSTM32F103xB -DUSE_HAL_DRIVER
CFLAGS += -Isrc
CFLAGS += -I../CMSIS/Include
CFLAGS += -I../STM32F1xx_HAL_Driver/Inc/
CFLAGS += -I../CMSIS/Device/ST/STM32F1xx/Include
LDFLAGS := -Wl,-static -Wl,--gc-sections -Wl,-T${LDSCR}
LDFLAGS += -L../CMSIS/Lib/GCC/

SRCS := ../CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s
SRCS += ../CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c
SRCS += ../STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
SRCS += ../STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
SRCS += ../STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
SRCS += ../STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
SRCS += ../STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
SRCS += src/mgmt_hw.c
SRCS += src/mgmt_pwm.c
SRCS += src/app.c
SRCS += src/random.c
SRCS += src/main.c

OBJS := ${SRCS:%.c=%.o:%.s=%.o}
DEPS := ${OBJS:.o=.d}

.SUFFIXES: .o .c .s

.s.o:
	@echo "  CC  ${.IMPSRC}"
	@${CC} ${CFLAGS} -MD -MF ${.TARGET:R}.d -o ${.TARGET} -c ${.IMPSRC}

.c.o:
	@echo "  CC  ${.IMPSRC}"
	@${CC} ${CFLAGS} -MD -MF ${.TARGET:R}.d -o ${.TARGET} -c ${.IMPSRC}

.for o in ${DEPS}
        sinclude $o
.endfor

.MAIN: all
all: ${OBJS}
	@echo "  LD  ${OELF}"
	@${CC} ${OBJS} ${CFLAGS} ${LDFLAGS} -o ${OELF}

.PHONY: debug
debug: all
	@${GDB} --command=${GDBSCR} ${OELF}

.PHONY: clean
clean:
	@rm -f $(OBJS) $(DEPS) ${OELF}
