CONTIKI_PROJECT=slip-radio
all: $(CONTIKI_PROJECT)

PLATFORMS_EXCLUDE = native nrf52dk

CONTIKI=../../..

 # force Orchestra from command line
MAKE_WITH_ORCHESTRA ?= 0
# force Security from command line
MAKE_WITH_SECURITY ?= 0
 # print #routes periodically, used for regression tests
MAKE_WITH_PERIODIC_ROUTES_PRINT ?= 0

MAKE_MAC = MAKE_MAC_TSCH
MODULES += os/services/shell

ifeq ($(MAKE_WITH_ORCHESTRA),1)
MODULES += os/services/orchestra
endif

ifeq ($(MAKE_WITH_SECURITY),1)
CFLAGS += -DWITH_SECURITY=1
endif


# slip-radio is only intended for platforms with SLIP support
MODULES += os/services/slip-cmd

include $(CONTIKI)/Makefile.identify-target

MODULES_REL += $(TARGET)
PROJECT_SOURCEFILES += slip-net.c

# custom net layer, but with IPv6 enabled
MAKE_NET = MAKE_NET_IPV6
MAKE_ROUTING = MAKE_ROUTING_NULLROUTING
include $(CONTIKI)/Makefile.include
