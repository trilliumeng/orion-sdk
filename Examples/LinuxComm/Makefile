#
# myapps/hello/Makefile
#

.PHONY: build install clean


BIN			= libLinuxComm.a
SRCS			= $(wildcard *.c)
OBJS			= $(SRCS:.c=.o)

CFLAGS += $(EXTRA_CFLAGS) -I../.. -I../../publiccomms

%.o:%.c
	$(V)$(CC) -c $(CFLAGS) $< -o $@ $(QOUT)

$(BIN): $(OBJS) ../../libOrionComm.a
	ar rcs $(BIN) $(OBJS)

../../libOrionComm.a:
	make -C ../../

clean: 
	$(V)rm -f $(BIN) *.debug *.o core *~
