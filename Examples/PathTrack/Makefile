#
# myapps/hello/Makefile
#

.PHONY: build install clean


BIN				= $(shell basename `pwd`)
SRCS			= $(wildcard *.c)
OBJS			= $(SRCS:.c=.o)

CFLAGS += $(EXTRA_CFLAGS) -I../../Communications -I../../Utils

%.o:%.c
	@$(CC) -c $(CFLAGS) $< -o $@ $(QOUT)

$(BIN): ../../Communications/libOrionComm.a ../../Utils/libOrionUtils.a $(OBJS)
	@$(CC) $(LDFLAGS) -o $(BIN) $(OBJS) -L../../Communications -L../../Utils -lOrionComm -lOrionUtils -lm $(QOUT)

../../Communications/libOrionComm.a:
	@make -C ../../Communications

../../Utils/libOrionUtils.a:
	@make -C ../../Utils

clean: 
	@rm -f $(BIN) *.debug *.o core *~
