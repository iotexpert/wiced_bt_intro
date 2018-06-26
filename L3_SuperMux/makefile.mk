#
# Lesson 3 - SuperMux
#
APP_SRC +=  L3_SuperMux.c

C_FLAGS += -DWICED_BT_TRACE_ENABLE
C_FLAGS += -DSMUX_CHIP=$(CHIP)
APP_SRC += L3_SuperMux_pin_config.c
