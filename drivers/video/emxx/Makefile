#
# Makefile for the EMXX framebuffer device driver
#

obj-$(CONFIG_FB_EMXX)  += emxx_fb.o
obj-$(CONFIG_FB_EMXX)  += emxx_fb_blit.o
obj-$(CONFIG_FB_EMXX)  += emxx_lcd.o
obj-$(CONFIG_FB_EMXX)  += emxx_lcdhw.o
obj-$(CONFIG_FB_EMXX)  += alphacomposite.o
obj-$(CONFIG_FB_EMXX)  += neon_alphablend_type2_0_0_0.o
obj-$(CONFIG_FB_EMXX)  += neon_alphablend_type2_0_1_1.o
obj-$(CONFIG_FB_EMXX)  += neon_alphablend_type2_0_2_2.o

obj-$(CONFIG_EMXX_HDMI_ADV7523) += emxx_adv7523.o

emxxfb-objs := emxx_fblcd.o
emxxhdmi-objs := emxx_adv7523.o
