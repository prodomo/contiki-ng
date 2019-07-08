CONTIKI_PROJECT = udp-client udp-server
all: $(CONTIKI_PROJECT)

CONTIKI=../..

CFLAGS += -DUART1_CONF_BAUD_RATE=9600

MAKE_MAC = MAKE_MAC_TSCH
MODULES += os/services/shell
MODULES += arch/dev/modbus
PROJECT_SOURCEFILES += command-type.c



include $(CONTIKI)/Makefile.include
