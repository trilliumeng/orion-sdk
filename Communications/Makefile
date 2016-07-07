-include autogen.mk

LIB = libOrionComm.a
OBJ = $(SRC:.c=.o)

%.o:%.c
	@$(CC) -c $(CFLAGS) $< -o $@ -I../Utils -I../Communications $(QOUT)

build: $(LIB)
$(LIB): $(OBJ)
	@$(AR) rcs $(LIB) $(OBJ)

# Generate code and populate autogen.mk file with src list
autogen.mk: OrionPublicProtocol.xml 
	@../GenerateOrionPublicPacket.sh || true
	@echo "SRC = `echo *.c`" > autogen.mk

clean:
	@mkdir .save && mv OrionComm*.[ch] .save
	@rm -f $(LIB) *.[cho] *.html *.markdown *.css autogen.mk
	@mv .save/* . && rm -rf .save
