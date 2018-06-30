include ../common.mk

LIB = libOrionUtils.a
SRC = $(wildcard *.c)
OBJ = $(SRC:%.c=$(OBJ_DIR)/%.o)

$(OBJ_DIR)/%.o:%.c
	$(V)mkdir -p $(OBJ_DIR)
	$(V)$(CC) -c $(CFLAGS) $< -o $@ $(QOUT) -I../Communications

build: $(LIB)
$(LIB): $(OBJ)
	$(V)mkdir -p $(OUT_DIR)
	$(V)$(AR) rcs $(OUT_DIR)/$(LIB) $(OBJ)
	$(V)rm *.zip 2>/dev/null || true

clean:
	$(V)rm -f $(LIB) *.o
	$(V)rm -rf build/
