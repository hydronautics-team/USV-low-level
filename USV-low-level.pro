QT += core
QT -= gui
QT += core network
QT += serialport

INCLUDEPATH += /mingw64/include/GeographicLib
LIBS += -L/mingw64/lib -lGeographic

CONFIG += c++17 console
CONFIG -= app_bundle

#INCLUDEPATH += /usr/include/gstreamer-1.0 \
#               /usr/include/glib-2.0 \
#               /usr/lib/arm-linux-gnueabihf/glib-2.0/include

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        VMA_control/vma_control.cpp \
        cs_usv.cpp \
        diagnostic_board/diagnostic_board.cpp \
        logger/logger.cpp \
        protocol_GPS/nmea0183.cpp \
        protocol_echolot/PA500.cpp \
        kx_pult/configdata.cpp \
        kx_pult/kx_protocol.cpp \
        kx_pult/qkx_coeffs.cpp \
        kx_pult/qpiconfig.cpp \
        main.cpp \
        library/pca9685/pca9685.cpp \
        protocol_BW_AH127C/AH127Cprotocol.cpp \
        protocol_gans_ZIMA/protocolzima.cpp \
        coordSSP/coordssp.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += -L"/home/hydronautics/rpi/sysroot/usr/lib"
LIBS += -lwiringPi
LIBS += -L"/home/hydronautics/rpi/sysroot/usr/lib/arm-linux-gnueabihf"
LIBS += -li2c
#LIBS += -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0

HEADERS += \
    VMA_control/vma_control.h \
    cs_usv.h \
    diagnostic_board/diagnostic_board.h \
    logger/logger.h \
    protocol_GPS/nmea0183.h \
    protocol_echolot/PA500.h \
    kx_pult/configdata.h \
    kx_pult/kx_protocol.h \
    kx_pult/qkx_coeffs.h \
    kx_pult/qpiconfig.h \
    library/pca9685/pca9685.h \
    protocol_BW_AH127C/AH127Cprotocol.h \
    protocol_gans_ZIMA/protocolzima.h \
    protocol_pult/pc_protocol.h \
    protocol_pult/protocol.h \
    protocol_pult/udp_protocol.h \
    coordSSP/coordssp.h
