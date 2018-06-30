include ../../common.mk

.PHONY: build install clean

BIN				= $(shell basename `pwd`)
SRCS			= $(wildcard *.c)
OBJS			= $(SRCS:%.c=$(OBJ_DIR)/%.o)

CFLAGS += $(EXTRA_CFLAGS) -I../../Communications -I../../Utils

$(OBJ_DIR)/%.o:%.c
	$(V)mkdir -p $(OBJ_DIR)
	$(V)$(CC) -c -Wall $(CFLAGS) $< -o $@ $(QOUT)

$(BIN): ../../Communications/libOrionComm.a ../../Utils/libOrionUtils.a $(OBJS)
	$(V)$(CC) -o $(OUT_DIR)/$(BIN) $(OBJS) -L../../Communications/$(OUT_DIR) -L../../Utils/$(OUT_DIR) -lOrionComm -lOrionUtils -lm $(LDFLAGS) $(QOUT)

../../Communications/libOrionComm.a:
	@make -C ../../Communications

../../Utils/libOrionUtils.a:
	@make -C ../../Utils

clean:
	$(V)rm -f $(BIN) *.debug *.o core *~
	$(V)rm -rf build/
