#
# Makefile for the EM/EV NTSC device driver
#

emxx_ntsif-y := emxx_nts.o emxx_ntshw.o emxx_nts_image.o
emxx_ntsif-$(CONFIG_EMXX_NTS_ADV7179) += emxx_adv7179.o

obj-$(CONFIG_EMXX_NTS) += emxx_ntsif.o

