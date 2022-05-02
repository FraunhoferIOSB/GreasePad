# CMakeLists.txt
cmake_minimum_required(VERSION 3.1.0)

project(greasepad VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# my configuration
set(Qt5_DIR "C:/Qt/5.11.0/msvc2017_64/lib/cmake/Qt5")
set(CMAKE_INSTALL_PREFIX "D:/git/GreasePad")
set(CMAKE_BUILD_TYPE RELEASE)


if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Svg)

add_executable(helloworld
    src/aabb.cpp
    src/adjustment.cpp 
    src/conics.cpp 
    src/conncomp.cpp 
    src/constraints.cpp 
    src/main.cpp 
    src/mainwindow.cpp 
    src/mainscene.cpp 
    src/mainview.cpp 
    src/commands.cpp 
    src/matrix.cpp 
    src/qconstraints.cpp 
    src/qformattool.cpp 
    src/qsegment.cpp 
    src/qstroke.cpp 
    src/quantiles.cpp 
    src/state.cpp 
    src/statistics.cpp 
    src/uncertain.cpp 
    src/upoint.cpp 
    src/usegment.cpp 
    src/ustraightline.cpp
	src/greasepad.qrc
)

target_link_libraries(greasepad Qt5::Widgets Qt5::Svg Eigen3::Eigen)


install(TARGETS greasepad DESTINATION bin)
