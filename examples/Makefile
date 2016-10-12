UNAME := $(shell uname)
CFLAGS += -I../sml/include/ -g -Wall
OBJS = sml_server.o
OBJSDP = sml_deactivate_pin.o
LIBSML = ../sml/lib/libsml.a

ifeq ($(UNAME), Linux)
LIBS = -luuid
endif

all : sml_server sml_deactivate_pin

sml_server : $(OBJS) $(LIBSML)
	$(CC) $(CFLAGS) $(OBJS) $(LIBS) $(LIBSML) -o sml_server

sml_deactivate_pin : $(OBJSDP) $(LIBSML)
	$(CC) $(CFLAGS) $(OBJSDP) $(LIBS) $(LIBSML) -o sml_deactivate_pin

%.o : %.c
	$(CC) $(CFLAGS) -c $^ -o $@

.PHONY: clean
clean:
	@rm -f *.o
	@rm -f sml_server
	@rm -f sml_deactivate_pin
