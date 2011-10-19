#
# Makefile for the linux kernel.
#

# Common support
obj-y	:= generic.o gic.o timer.o dma.o pcm_irq.o
obj-y	+= gpio.o pmu.o pm.o buslim.o
obj-m	:=
obj-n	:=
obj-	:=

# Specific board support
obj-$(CONFIG_EMEV_BOARD_EVA)	+= emev_board.o
obj-$(CONFIG_EMGR_BOARD_EVA)	+= emev_board.o
obj-$(CONFIG_EMXX_EXTIO)	+= extio.o

obj-$(CONFIG_EMXX_SPI0)		+= spi0.o
obj-$(CONFIG_EMXX_SPI)		+= spi.o
obj-$(CONFIG_EMGR_TI_PMIC)	+= pwc_ext.o
obj-$(CONFIG_EMXX_PWC)		+= pwc.o
obj-$(CONFIG_EMXX_PWM)		+= pwm.o
obj-$(CONFIG_EMXX_DTV)		+= dtv.o
obj-$(CONFIG_EMXX_LED)		+= light.o
obj-$(CONFIG_EMXX_PDMA)		+= emev_pdma.o
obj-$(CONFIG_EMXX_INTERDSP)	+= inter_dsp.o

obj-$(CONFIG_PM)		+= pm_pmu.o

obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o

