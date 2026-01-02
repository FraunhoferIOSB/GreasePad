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
    adjustment.cpp \
    constraints.cpp \
    main.cpp \
    mainwindow.cpp \
    mainscene.cpp \
    mainview.cpp \
    commands.cpp \
    matrix.cpp \
    qconstraints.cpp \
    qformattool.cpp \
    qsegment.cpp \
    qstroke.cpp \
    quantiles.cpp \
    state.cpp \
    uncertain.cpp \
    upoint.cpp \
    usegment.cpp \
    ustraightline.cpp

HEADERS += \
    adjustment.h \
    commands.h \
    conncomp.h \
    constraints.h \
    geometry/aabb.h \
    geometry/conics.h \
    geometry/minrot.h \
    geometry/skew.h \
    global.h \
    mainwindow.h \
    mainscene.h \
    mainview.h \
    matfun.h \
    matrix.h \
    qconstraints.h \
    qformattool.h \
    qsegment.h \
    qstroke.h \
    quantiles.h \
    state.h \
    statistics.h \
    statistics/chisquared.h \
    statistics/exponential.h \
    statistics/gamma.h \
    statistics/iscov.h \
    statistics/normal.h \
    statistics/prob.h \
    statistics/uniform.h \
    uncertain.h \
    upoint.h \
    usegment.h \
    ustraightline.h

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
