include ../common.mk

LIB = $(TARGET)/libOrionUtils.a
SRC = $(wildcard *.c)
OBJ = $(SRC:%.c=$(OBJ_DIR)/%.o)

$(OBJ_DIR)/%.o:%.c
	$(V)$(CC) -c $(CFLAGS) $< -o $@ $(QOUT) -I../Communications -I./

build: $(LIB)
$(LIB): $(OBJ)
	$(V)$(AR) rcs $(LIB) $(OBJ)
	$(V)rm *.zip 2>/dev/null || true

clean:
	$(V)rm -rf $(TARGET)
