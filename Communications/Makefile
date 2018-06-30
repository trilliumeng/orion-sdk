-include autogen.mk
include ../common.mk

LIB = libOrionComm.a
OBJ = $(SRC:%.c=$(OBJ_DIR)/%.o)

$(OBJ_DIR)/%.o:%.c
	$(V)mkdir -p $(OBJ_DIR)
	$(V)$(CC) -c $(CFLAGS) $< -o $@ -I../Utils -I../Communications $(QOUT)

build: autogen.mk $(LIB)
$(LIB): $(OBJ)
	$(V)mkdir -p $(OUT_DIR)
	$(V)$(AR) rcs $(OUT_DIR)/$(LIB) $(OBJ)

# Generate code and populate autogen.mk file with src list
autogen.mk: OrionPublicProtocol.xml
	@../GenerateOrionPublicPacket.sh || true
	@echo "SRC = `echo *.c`" > autogen.mk

clean:
	$(V)mkdir .save && mv OrionComm*.[ch] .save
	$(V)rm -f $(LIB) *.[cho] *.html *.markdown *.css autogen.mk
	$(V)rm -rf build/
	$(V)mv .save/* . && rm -rf .save
