LIB = libOrionUtils.a
SRC = $(wildcard *.c)
OBJ = $(SRC:.c=.o)

%.o:%.c
	@$(CC) -c $(CFLAGS) $< -o $@ $(QOUT) -I../Communications

build: $(LIB)
$(LIB): $(OBJ)
	@$(AR) rcs $(LIB) $(OBJ)
	@rm *.zip 2>/dev/null || true

clean:
	@rm -f $(LIB) *.o
