-include autogen.mk
include ../common.mk

LIB = $(TARGET)/libOrionComm.a
OBJ = $(SRC:%.c=$(OBJ_DIR)/%.o)

$(OBJ_DIR)/%.o:%.c
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
	$(V)rm -rf $(TARGET) *.[cho] *.html *.markdown *.css autogen.mk
	$(V)mv .save/* . && rm -rf .save
