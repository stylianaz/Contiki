CONTIKI = ../..

all: MobileNodes Nodes SinkNode SourceNodes StaticNodes \
     distanceExample 

TARGET_LIBFILES = -lm
CONTIKI_WITH_RIME = 1
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
include $(CONTIKI)/Makefile.include
