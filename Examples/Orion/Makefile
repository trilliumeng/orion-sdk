#
# myapps/hello/Makefile
#

.PHONY: build install clean


BIN			= Orion
SRCS			= $(wildcard *.c)
OBJS			= $(SRCS:.c=.o)

CFLAGS += $(EXTRA_CFLAGS) -I../.. -I../../publiccomms

%.o:%.c
	$(V)$(CC) -c $(CFLAGS) $< -o $@ $(QOUT)

$(BIN): $(OBJS) ../../libOrionComm.a
	$(V)$(CC) $(LDFLAGS) -o $(BIN) $(OBJS) $(QOUT) -L../.. -lOrionComm

../../libOrionComm.a:
	make -C ../../

clean: 
	$(V)rm -f $(BIN) *.debug *.o core *~
