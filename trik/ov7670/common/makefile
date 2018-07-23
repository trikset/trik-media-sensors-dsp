
# your various installation directories
DEPOT_DSP?=/opt/trik-dsp

XDC_INSTALL_DIR?=$(DEPOT_DSP)/xdctools_3_25_06_96
CODEGEN_INSTALL_DIR?=$(DEPOT_DSP)/ti-cgt-c6000_8.0.1
CE_INSTALL_DIR?=$(DEPOT_DSP)/codec_engine_3_23_00_07
XDAIS_INSTALL_DIR?=$(DEPOT_DSP)/xdais_7_24_00_04
DSPLIB_INSTALL_DIR?=$(DEPOT_DSP)/dsplib_c674x_3_4_0_0
IMGLIB_INSTALL_DIR?=$(DEPOT_DSP)/c64plus-imglib_2_02_00_00
VLIB_INSTALL_DIR?=$(DEPOT_DSP)/vlib_c674x_3_3_0_3

CGTOOLS_C674?=$(CODEGEN_INSTALL_DIR)

#uncomment this for verbose builds
XDCOPTIONS=v

XDCARGS?=CGTOOLS_C674=\"$(CGTOOLS_C674)\"

XDCPATH?=/home/me/trik/trik-media-sensors-dsp/;$(CE_INSTALL_DIR)/packages;$(XDAIS_INSTALL_DIR)/packages;$(DSPLIB_INSTALL_DIR)/packages;$(IMGLIB_INSTALL_DIR);$(VLIB_INSTALL_DIR)/packages

XDC?=$(XDC_INSTALL_DIR)/xdc

all:
	$(XDC) XDCOPTIONS=$(XDCOPTIONS) XDCARGS="$(XDCARGS)" --xdcpath="$(XDCPATH)" release

clean:
	$(XDC) XDCOPTIONS=$(XDCOPTIONS) clean

