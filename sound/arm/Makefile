#
# Makefile for ALSA
#

obj-$(CONFIG_SND_EMXX) += snd-emxx.o
snd-emxx-objs := emxx-pcm.o

obj-$(CONFIG_SND_EMXX_MIXER) += snd-emxx-mixer.o
snd-emxx-mixer-objs := emxx-mixer.o

obj-$(CONFIG_SND_EMXX_SPDIF) += snd-emxx-spdif.o
snd-emxx-spdif-objs := emxx-cs8427.o

obj-$(CONFIG_SND_SA11XX_UDA1341) += snd-sa11xx-uda1341.o
snd-sa11xx-uda1341-objs		:= sa11xx-uda1341.o

obj-$(CONFIG_SND_ARMAACI)	+= snd-aaci.o
snd-aaci-objs			:= aaci.o devdma.o

obj-$(CONFIG_SND_PXA2XX_PCM)	+= snd-pxa2xx-pcm.o
snd-pxa2xx-pcm-objs		:= pxa2xx-pcm.o

obj-$(CONFIG_SND_PXA2XX_LIB)	+= snd-pxa2xx-lib.o
snd-pxa2xx-lib-y		:= pxa2xx-pcm-lib.o
snd-pxa2xx-lib-$(CONFIG_SND_PXA2XX_LIB_AC97)	+= pxa2xx-ac97-lib.o

obj-$(CONFIG_SND_PXA2XX_AC97)	+= snd-pxa2xx-ac97.o
snd-pxa2xx-ac97-objs		:= pxa2xx-ac97.o
