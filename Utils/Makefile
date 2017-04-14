LIB = libOrionUtils.a
SRC = $(wildcard *.c)
OBJ = $(SRC:.c=.o)

ifneq ($(VERBOSE), 1)
    V=@
endif

%.o:%.c
	$(V)$(CC) -c $(CFLAGS) $< -o $@ $(QOUT) -I../Communications

build: $(LIB)
$(LIB): $(OBJ)
	$(V)$(AR) rcs $(LIB) $(OBJ)
	$(V)rm *.zip 2>/dev/null || true

clean:
	$(V)rm -f $(LIB) *.o
