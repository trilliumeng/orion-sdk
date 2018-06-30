ifneq ($(VERBOSE), 1)
    V=@
endif

TARGET ?= x86

ifeq ($(TARGET), arm)
	PREFIX=arm-linux-gnueabi-
    CC=$(PREFIX)gcc
    AR=$(PREFIX)ar
else ifeq ($(TARGET), tegra)
	PREFIX=aarch64-unknown-linux-gnu-
    CC=$(PREFIX)gcc
    AR=$(PREFIX)ar
endif

OUT_DIR = build/$(TARGET)
OBJ_DIR = $(OUT_DIR)/obj
