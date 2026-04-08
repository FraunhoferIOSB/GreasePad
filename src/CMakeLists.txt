cmake_minimum_required(VERSION 3.30.0)

# set the project name
project(GreasePad VERSION 1.2.1 LANGUAGES CXX)


# CONFIG  specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# my installation directories
set(Qt6_DIR "C:/Qt/6.10.0/msvc2022_64/lib/cmake/Qt6")

set(CMAKE_CONFIGURATION_TYPES "Debug;Release;Profile")
set(CMAKE_INSTALL_PREFIX "D:/Dev")
#set(CMAKE_BUILD_TYPE "Debug")


find_package(Qt6 REQUIRED COMPONENTS Core Widgets Svg)

qt_standard_project_setup()

# add the executable
qt_add_executable(GreasePad
    adjustment.cpp
    commands.cpp
    constraints.cpp
    main.cpp
    gui/mainwindow.cpp
    gui/mainscene.cpp
    gui/mainview.cpp
    gui/qformattool.cpp
    matrix.cpp
    qconstraints.cpp
    qsegment.cpp
    qstroke.cpp
    quantiles.cpp
    state.cpp
    uncertain/quncertain.cpp
    uncertain/uncertain.cpp
    uncertain/upoint.cpp
    uncertain/usegment.cpp
    uncertain/ustraightline.cpp
)

qt_add_resources(GreasePad "images"
        PREFIX "/"
	FILES icons/Tango/preferences-desktop-peripherals.svg icons/Tango/Edit-clear.svg
)


# INCLUDEPATH
target_include_directories(GreasePad PUBLIC "D:/Dev/clone/eigen/")
target_include_directories(GreasePad PUBLIC "D:/Dev/clone/GreasePad/src")
target_include_directories(GreasePad PUBLIC "C:/Qt/6.10.0/msvc2022_64/include/QtCore")
target_include_directories(GreasePad PUBLIC "C:/Qt/6.10.0/msvc2022_64/include/QtGui")
target_include_directories(GreasePad PUBLIC "C:/Qt/6.10.0/msvc2022_64/include/QtWidgets")
target_include_directories(GreasePad PUBLIC "C:/Qt/6.10.0/msvc2022_64/include/QtSvg")
target_include_directories(GreasePad PUBLIC "C:/Qt/6.10.0/msvc2022_64/include")





target_link_libraries(GreasePad PRIVATE Qt6::Widgets Qt6::Svg)


install(TARGETS GreasePad DESTINATION bin 
  CONFIGURATIONS Release Debug
)

# in Eingabeaufforderung (not PowerShell)
# set PATH=C:\Qt\6.10.0\msvc2022_64\bin;%PATH%
