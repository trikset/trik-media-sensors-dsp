SUBDIRS = edge_line_sensor line_sensor  mxn_sensor jpeg_encoder motion_sensor object_sensor 

all:
	for d in $(SUBDIRS) ; do $(MAKE) -C $$d && $(MAKE) -C $$d/dsp_server ; done
