-include autogen.mk

LIB = libOrionComm.a
OBJ = $(SRC:.c=.o)

ifneq ($(VERBOSE), 1)
    V=@
endif

%.o:%.c
	$(V)$(CC) -c $(CFLAGS) $< -o $@ -I../Utils -I../Communications $(QOUT)

build: autogen.mk $(LIB)
$(LIB): $(OBJ)
	$(V)$(AR) rcs $(LIB) $(OBJ)

# Generate code and populate autogen.mk file with src list
autogen.mk: OrionPublicProtocol.xml 
	@../GenerateOrionPublicPacket.sh || true
	@echo "SRC = `echo *.c`" > autogen.mk

clean:
	$(V)mkdir .save && mv OrionComm*.[ch] .save
	$(V)rm -f $(LIB) *.[cho] *.html *.markdown *.css autogen.mk
	$(V)mv .save/* . && rm -rf .save
