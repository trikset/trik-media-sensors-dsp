#
_XDCBUILDCOUNT = 105
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /opt/trik-dsp/codec_engine_3_23_00_07/packages;/opt/trik-dsp/xdais_7_23_00_06/packages;/opt/trik-dsp/bios_6_35_01_29/packages;/opt/trik-dsp/syslink_2_21_01_05/packages;/opt/trik-dsp/ipc_1_25_02_12/packages;/opt/trik-dsp/framework_components_3_23_03_17/packages;/opt/trik-dsp/osal_1_23_00_04/packages;/opt/trik-dsp/linuxutils_3_23_00_01/packages;/opt/trik-dsp/dsplib_3_1_1_1/packages;/opt/trik-dsp/c64plus-imglib_2_02_00_00;/opt/trik-dsp/vlib_c674x_3_2_0_2/packages
override XDCROOT = /opt/trik-dsp/xdctools_3_24_07_73
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = CGTOOLS_C674="/opt/trik-dsp/cgt_c6000_7.4.2"
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /opt/trik-dsp/codec_engine_3_23_00_07/packages;/opt/trik-dsp/xdais_7_23_00_06/packages;/opt/trik-dsp/bios_6_35_01_29/packages;/opt/trik-dsp/syslink_2_21_01_05/packages;/opt/trik-dsp/ipc_1_25_02_12/packages;/opt/trik-dsp/framework_components_3_23_03_17/packages;/opt/trik-dsp/osal_1_23_00_04/packages;/opt/trik-dsp/linuxutils_3_23_00_01/packages;/opt/trik-dsp/dsplib_3_1_1_1/packages;/opt/trik-dsp/c64plus-imglib_2_02_00_00;/opt/trik-dsp/vlib_c674x_3_2_0_2/packages;/opt/trik-dsp/xdctools_3_24_07_73/packages;../../../../..
HOSTOS = Linux
endif
