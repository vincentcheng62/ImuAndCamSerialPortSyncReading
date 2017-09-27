QT = core
QT += serialport

CONFIG += console
CONFIG -= app_bundle

TARGET = creadersync
TEMPLATE = app

LIBS += -L../../lib -lflycapture${D} ${FC2_DEPS}

INCLUDEPATH += ../../include \
            /usr/include/flycapture

SOURCES += \
    main.cpp \
    flycaptureexternaltrigger.cpp \
    stdafx.cpp \
    imu.cpp

target.path = $$[QT_INSTALL_EXAMPLES]/serialport/creadersync
INSTALLS += target

HEADERS += \
    resource.h \
    stdafx.h \
