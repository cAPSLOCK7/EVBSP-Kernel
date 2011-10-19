
#
# Makefile for AVE driver
#

AVE_SRCS = ave_interface.c ave_debug.c

AVE_OBJS = $(AVE_SRCS:.c=.o)

obj-$(CONFIG_EMXX_AVE) := em_ave.o
em_ave-y := $(AVE_OBJS)

