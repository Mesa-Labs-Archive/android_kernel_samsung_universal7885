#
# Makefile for muic devices
#

ifndef CONFIG_MUIC_S2MU004
obj-y					+= muic-core.o
endif
obj-$(CONFIG_MUIC_NOTIFIER)		+= muic_notifier.o
obj-$(CONFIG_MUIC_MANAGER)		+= muic_manager.o
obj-$(CONFIG_MUIC_S2MU004)		+= muic_core.o s2mu004-muic.o s2mu004-muic-sysfs.o
obj-$(CONFIG_HV_MUIC_S2MU004_AFC)	+= s2mu004-muic-afc.o
obj-$(CONFIG_MUIC_S2MU005)			+= s2mu005-muic.o