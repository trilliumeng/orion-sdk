#
# myapps/hello/Makefile
#

.PHONY: build install clean


BIN				= $(shell basename `pwd`)
SRCS			= $(wildcard *.c)
OBJS			= $(SRCS:.c=.o)

CFLAGS += $(EXTRA_CFLAGS) -O3 -I../../Communications -I../../Utils -I../LinuxComm

%.o:%.c
	@$(CC) -c $(CFLAGS) $< -o $@ $(QOUT)

$(BIN): ../../Communications/libOrionComm.a ../../Utils/libOrionUtils.a ../LinuxComm/libLinuxComm.a $(OBJS)
	@$(CC) $(LDFLAGS) -o $(BIN) $(OBJS) -L../../Communications -L../../Utils -L../LinuxComm -lLinuxComm -lOrionComm -lOrionUtils -lm -lstdc++ $(QOUT)

../../Communications/libOrionComm.a:
	@make -C ../../Communications

../../Utils/libOrionUtils.a:
	@make -C ../../Utils

../LinuxComm/libLinuxComm.a:
	@make -C ../LinuxComm

clean:
	@rm -f $(BIN) *.debug *.o core *~
