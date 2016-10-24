cmake_minimum_required(VERSION 2.8)
project(calibration)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -Wall -pthread")
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -std=c++11 -O3 -Wall -pthread")
set(CMAKE_PREFIX_PATH "/opt/Qt5.4.2/5.4/gcc_64")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)

aux_source_directory(./src DIR_SRCS)

#message(${DIR_SRCS})
#message(${PROJECT_SOURCE_DIR})
#set(LIBRARY_OUTPUT_PATH )

find_package(Ceres REQUIRED)
find_package(OpenCV 3.0.0 REQUIRED)

find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5OpenGL)
find_package(OpenGL REQUIRED)

include_directories(
        ./include
	3rdparty/sophus
	${CERES_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

qt5_wrap_cpp(UI_MOC ui/mainwindow.h)
qt5_wrap_ui(UI_FORM_H ui/mainwindow.ui)

message(${OpenCV_INCLUDE_DIRS})

add_library(ui_main_lib  ${UI_MOC} ${UI_FORM_H} ui/mainwindow.cpp)
#qt5_use_modules(ui_main_lib Core Gui Widgets OpenGL)
target_link_libraries(ui_main_lib Qt5::Widgets Qt5::Core Qt5::Gui Qt5::OpenGL)

add_executable(calibration ${DIR_SRCS} ui/ui_main_thread.cpp)
target_link_libraries(calibration ui_main_lib ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})

#add_library(vslam SHARED ${DIR_SRCS})

