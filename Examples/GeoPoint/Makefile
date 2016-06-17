#
# myapps/hello/Makefile
#

.PHONY: build install clean


BIN			= GeoPoint
SRCS			= $(wildcard *.c)
OBJS			= $(SRCS:.c=.o)

CFLAGS += $(EXTRA_CFLAGS) -I../../Communications -I../../Utils -I../LinuxComm

%.o:%.c
	@$(CC) -c $(CFLAGS) $< -o $@ $(QOUT)

$(BIN): ../../Communications/libOrionComm.a ../../Utils/libOrionUtils.a ../LinuxComm/libLinuxComm.a $(OBJS)
	@$(CC) $(LDFLAGS) -o $(BIN) $(OBJS) $(QOUT) -L../../Communications -L../../Utils -L../LinuxComm -lOrionUtils -lOrionComm -lLinuxComm -lm

../../Communications/libOrionComm.a:
	@make -C ../../Communications

../../Utils/libOrionUtils.a:
	@make -C ../../Utils

../LinuxComm/libLinuxComm.a:
	@make -C../LinuxComm

clean: 
	@rm -f $(BIN) *.debug *.o core *~
