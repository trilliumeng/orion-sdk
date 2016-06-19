-include autogen.mk

LIB = libOrionComm.a
OBJ = $(SRC:.c=.o)

%.o:%.c
	@$(CC) -c $(CFLAGS) $< -o $@ $(QOUT)

build: $(LIB)
$(LIB): $(OBJ)
	@ar rcs $(LIB) $(OBJ)

# Generate code and populate autogen.mk file with src list
autogen.mk: OrionPublicProtocol.xml 
	@../GenerateOrionPublicPacket.sh || true
	@echo "SRC = `echo *.c`" > autogen.mk

clean:
	@rm -f $(LIB) *.[cho] *.html *.markdown *.css autogen.mk
