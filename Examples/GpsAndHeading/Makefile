#
# myapps/hello/Makefile
#

ifneq ($(VERBOSE), 1)
    V=@
endif

.PHONY: build install clean


BIN				= $(shell basename `pwd`)
SRCS			= $(wildcard *.c) $(wildcard ../../Utils/*.c)
OBJS			= $(SRCS:.c=.o)

CFLAGS += $(EXTRA_CFLAGS) -I../../Communications -I../../Utils

%.o:%.c
	$(V)$(CC) -c -Wall $(CFLAGS) $< -o $@ $(QOUT)

$(BIN): ../../Communications/libOrionComm.a ../../Utils/libOrionUtils.a $(OBJS)
	$(V)$(CC) $(LDFLAGS) -o $(BIN) $(OBJS) -L../../Communications -L../../Utils -lOrionUtils -lOrionComm -lm $(QOUT)

../../Communications/libOrionComm.a:
	@make -C ../../Communications

../../Utils/libOrionUtils.a:
	@make -C ../../Utils

clean: 
	$(V)rm -f $(BIN) *.debug *.o core *~
