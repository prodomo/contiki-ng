CONTIKI_PROJECT = udp-client udp-server
all: $(CONTIKI_PROJECT)

CONTIKI=../..

MAKE_MAC = MAKE_MAC_TSCH
MODULES += os/services/shell
PROJECT_SOURCEFILES += command-type.c
# MAKE_ROUTING = MAKE_ROUTING_RPL_CLASSIC



include $(CONTIKI)/Makefile.include
