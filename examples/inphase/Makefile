CONTIKI_PROJECT = inphase
all: $(CONTIKI_PROJECT).upload

# Extra defines go here
INGA_CONF_BAUDRATE = 38400
CFLAGS += -DINGA_CONF_RADIO_CHANNEL=25

# Use floating point printf
LDFLAGS += -Wl,-u,vfprintf -lprintf_flt -lm
TARGET_LIBFILES += -lm

TARGET=inga

CONTIKI_WITH_RIME = 1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
