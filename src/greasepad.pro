QT       += core gui svg printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# CONFIG(release, debug|release):DEFINES += QT_NO_DEBUG_OUTPUT

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


# set assember's flags (MinGW), syntax: -Wa,<comma-separated list>
contains(QMAKE_CC, gcc) {
    # MinGW
    QMAKE_CXXFLAGS += -Wa,-mbig-obj
}


SOURCES += \
    graphics/qconstraints.cpp \
    graphics/qsegment.cpp \
    graphics/qstroke.cpp \
    gui/mainwindow.cpp \
    gui/mainscene.cpp \
    gui/mainview.cpp \
    gui/qformattool.cpp \
    main.cpp \
    reasoning/commands.cpp \
    reasoning/matrix.cpp \
    reasoning/adjustment.cpp \
    reasoning/constraints.cpp \
    reasoning/state.cpp \
    uncertain/quncertain.cpp \
    uncertain/upoint.cpp \
    uncertain/usegment.cpp \
    uncertain/ustraightline.cpp

HEADERS += \
    geometry/aabb.h \
    geometry/acute.h \
    geometry/conics.h \
    geometry/minrot.h \
    geometry/skew.h \
    reasoning/adjustment.h \
    reasoning/commands.h \
    reasoning/conncomp.h \
    reasoning/constants.h \
    reasoning/constraints.h \
    reasoning/find.h \
    reasoning/global.h \
    reasoning/kernel.h \
    reasoning/matfun.h \
    reasoning/matrix.h \
    reasoning/quantiles.h \
    reasoning/state.h \
    reasoning/unique.h \
    graphics/qconstraints.h \
    graphics/qsegment.h \
    graphics/qstroke.h \
    gui/mainscene.h \
    gui/mainview.h \
    gui/mainwindow.h \
    gui/qformattool.h \
    statistics/chisquared.h \
    statistics/exponential.h \
    statistics/gamma.h \
    statistics/iscov.h \
    statistics/normal.h \
    statistics/prob.h \
    statistics/uniform.h \
    uncertain/quncertain.h \
    uncertain/udistance.h \
    uncertain/uelement.h \
    uncertain/upoint.h \
    uncertain/usegment.h \
    uncertain/ustraightline.h

FORMS +=

# Eigen
# INCLUDEPATH += C:/Eigen/eigen-3.4.0/source
INCLUDEPATH += D:/Dev/clone/eigen

# Option
# INCLUDEPATH += "C:/Program Files (x86)/Visual Leak Detector/include/"
# LIBS      += -L"C:/Program Files (x86)/Visual Leak Detector/lib/Win64"

RC_ICONS += icons/Tango/preferences-desktop-peripherals.ico

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += greasepad.qrc
