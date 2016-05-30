SUBDIRS = #edge_line_sensor line_sensor \
		  mxn_sensor #jpeg_encoder \
		  #motion_sensor object_sensor

SUBDIRSCMD =  $(patsubst %,%-$(CMD),$(SUBDIRS))

.PHONY: subdirs $(SUBDIRSCMD) $(SUBDIRS)

all: subdirs$(CMD)


clean:
	$(MAKE) -C . CMD=clean


subdirs$(CMD): $(SUBDIRS)


$(SUBDIRS):
	$(MAKE) -C $@ $(CMD)
	$(MAKE) -C $@/dsp_server $(CMD)

