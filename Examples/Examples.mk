include ../../common.mk

.PHONY: build install clean

BIN				= $(TARGET)/$(shell basename `pwd`)
SRCS			= $(wildcard *.c)
OBJS			= $(SRCS:%.c=$(OBJ_DIR)/%.o)

CFLAGS += $(EXTRA_CFLAGS) -I../../Communications -I../../Utils

$(OBJ_DIR)/%.o:%.c
	$(V)$(CC) -c -Wall $(CFLAGS) $< -o $@ $(QOUT)

$(BIN): ../../Communications/$(TARGET)/libOrionComm.a ../../Utils/$(TARGET)/libOrionUtils.a $(OBJS)
	$(V)$(CC) -o $(BIN) $(OBJS) -L../../Communications/$(TARGET) -L../../Utils/$(TARGET) -lOrionComm -lOrionUtils -lm $(LDFLAGS) $(QOUT)

../../Communications/$(TARGET)/libOrionComm.a:
	@make -C ../../Communications

../../Utils/$(TARGET)/libOrionUtils.a:
	@make -C ../../Utils

clean:
	$(V)rm -rf $(TARGET) *.debug *.o core *~
