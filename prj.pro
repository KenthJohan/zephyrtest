TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += src/main.c
SOURCES += src/button_svc.c
SOURCES += src/led_svc.c
SOURCES += src/tmc2130.c

INCLUDEPATH += C:/zephyrproject/zephyr/include
INCLUDEPATH += C:/zephyrproject/zephyr/include/drivers
#INCLUDEPATH += C:/zephyrproject/zephyr
INCLUDEPATH += $$PWD/build/zephyr/include/generated
INCLUDEPATH += C:/zephyrproject/zephyr/soc/arm/st_stm32/stm32wb
INCLUDEPATH += C:/zephyrproject/zephyr/drivers
INCLUDEPATH += C:/zephyrproject/zephyr/ext/lib/crypto/tinycrypt/include
INCLUDEPATH += C:/zephyrproject/zephyr/ext/hal/cmsis/Core/Include
INCLUDEPATH += C:/zephyrproject/zephyr/subsys/bluetooth
INCLUDEPATH += C:/zephyrproject/modules/hal/stm32/stm32cube/stm32wbxx/soc
INCLUDEPATH += C:/zephyrproject/modules/hal/stm32/stm32cube/stm32wbxx/drivers/include
INCLUDEPATH += C:/zephyrproject/modules/hal/stm32/stm32cube/stm32wbxx/drivers/include/Legacy
INCLUDEPATH += C:/zephyrproject/modules/hal/stm32/lib/stm32wb/hci


QMAKE_CXXFLAGS_WARN_ON += -Wno-padded
QMAKE_CXXFLAGS_WARN_ON += -Wno-main-return-type
QMAKE_CXXFLAGS_WARN_ON += -Wno-unused-parameter

HEADERS += src/button_svc.h
HEADERS += src/led_svc.h
HEADERS += src/tmc2130.h
