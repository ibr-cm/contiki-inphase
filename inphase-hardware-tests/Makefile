CONTIKI_PROJECT = inphase-hardware-tests
all: $(CONTIKI_PROJECT)

TARGET = inga
CONTIKI_WITH_RIME = 1
CFLAGS += -DINGA_CONF_RADIO_CHANNEL=26

CONTIKI = ..
include $(CONTIKI)/Makefile.include
