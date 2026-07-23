cmake_minimum_required(VERSION 3.30.0)

# set the project name
project(GreasePad
  VERSION 1.2.1
  LANGUAGES CXX
)

# CONFIG  specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Qt6
find_package(Qt6 REQUIRED COMPONENTS
  Core
  Widgets
  Gui
  # PrintSupport
  Svg
)

# Eigen
find_package(Eigen3 5.0 REQUIRED NO_MODULE)

qt_standard_project_setup()

set(SOURCES
  graphics/qconstraints.cpp
  graphics/qsegment.cpp
  graphics/qstroke.cpp
  gui/mainwindow.cpp
  gui/mainscene.cpp
  gui/mainview.cpp
  gui/qformattool.cpp
  main.cpp
  reasoning/commands.cpp
  reasoning/matrix.cpp
  reasoning/adjustment.cpp
  reasoning/constraints.cpp
  reasoning/state.cpp
  uncertain/quncertain.cpp
  uncertain/upoint.cpp
  uncertain/usegment.cpp
  uncertain/ustraightline.cpp
)

set(HEADERS
  geometry/aabb.h
  geometry/acute.h
  geometry/conics.h
  geometry/minrot.h
  geometry/skew.h
  gui/logger.h
  reasoning/adjustment.h
  reasoning/commands.h
  reasoning/conncomp.h
  reasoning/constants.h
  reasoning/constraints.h
  reasoning/find.h
  reasoning/global.h
  reasoning/kernel.h
  reasoning/matfun.h
  reasoning/matrix.h
  reasoning/quantiles.h
  reasoning/state.h
  reasoning/unique.h
  graphics/qconstraints.h
  graphics/qsegment.h
  graphics/qstroke.h
  gui/mainscene.h
  gui/mainview.h
  gui/mainwindow.h
  gui/qformattool.h
  statistics/chisquared.h
  statistics/exponential.h
  statistics/gamma.h
  statistics/iscov.h
  statistics/normal.h
  statistics/prob.h
  statistics/uniform.h
  uncertain/quncertain.h
  uncertain/udistance.h
  uncertain/uelement.h
  uncertain/upoint.h
  uncertain/usegment.h
  uncertain/ustraightline.h
)

# relative paths
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  # D:/Dev/clone/eigen/
)


# variable-based approach
set(RCFILE greasepad.qrc)
message(STATUS "RCFILE:: ${RCFILE}")
qt_add_resources(RESOURCES ${RCFILE})

add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${HEADERS}
#  ${GENERATED_MOCS}
  ${RESOURCES}
  gui/logger.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  Eigen3::Eigen
  Qt6::Core
  Qt6::Widgets
  Qt6::Gui
  #Qt6::PrintSupport
  Qt6::Svg
  #Threads::Threads
)
