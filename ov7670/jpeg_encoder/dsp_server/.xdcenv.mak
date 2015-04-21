#
_XDCBUILDCOUNT = 61
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = 
override XDCROOT = /opt/trik-dsp/xdctools_3_24_07_73
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /opt/trik-dsp/xdctools_3_24_07_73/packages;../../..
HOSTOS = Linux
endif
