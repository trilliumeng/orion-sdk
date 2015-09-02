#
# myapps/hello/Makefile
#

.PHONY: build install clean


BIN			= GeoPoint
SRCS			= $(wildcard *.c)
OBJS			= $(SRCS:.c=.o)

CFLAGS += $(EXTRA_CFLAGS) -I../.. -I../../publiccomms -I../LinuxComm

%.o:%.c
	$(V)$(CC) -c $(CFLAGS) $< -o $@ $(QOUT)

$(BIN): $(OBJS) ../../libOrionComm.a ../LinuxComm/libLinuxComm.a
	$(V)$(CC) $(LDFLAGS) -o $(BIN) $(OBJS) $(QOUT) -L../.. -L../LinuxComm -lOrionComm -lLinuxComm

../../libOrionComm.a:
	make -C ../../

../LinuxComm/libLinuxComm.a:
	make -C ../LinuxComm/

clean: 
	$(V)rm -f $(BIN) *.debug *.o core *~
