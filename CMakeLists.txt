cmake_minimum_required(VERSION 2.8)

project(BloomingFlower)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(Boost_USE_STATIC_LIBS  ON)
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED system thread)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenSceneGraph REQUIRED osgViewer osgText osgDB osgGA osgQt osgManipulator osgUtil)
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
#find_package(Qt5Xml REQUIRED)
find_package(Qt5Concurrent REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

# Set Qt files and resources here
set(uis       resource/main_window.ui)
set(resources  resource/main_window.qrc)

# Qt stuff
QT5_WRAP_UI(ui_srcs ${uis})
QT5_ADD_RESOURCES(resource_srcs ${resources})

set(incs    include/main_window.h
            include/threaded_painter.h
            include/osg_viewer_widget.h
            include/toggle_handler.h
            include/renderable.h
            include/update_callback.h
            include/adapter_widget.h
            include/scene_widget.h
            )

set(srcs    src/main.cpp
            src/main_window.cpp
            src/adapter_widget.cpp
            src/threaded_painter.cpp
            src/osg_viewer_widget.cpp
            src/toggle_handler.cpp
            src/renderable.cpp
            src/update_callback.cpp
            src/scene_widget.cpp
            )

# Organize files
SOURCE_GROUP("Resources" FILES ${uis} ${resources})
SOURCE_GROUP("Generated" FILES ${ui_srcs} ${moc_srcs} ${resource_srcs})
SET_SOURCE_FILES_PROPERTIES(${srcs} PROPERTIES OBJECT_DEPENDS "${ui_srcs}")

# Put the ui in the windows project file
IF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
IF (${CMAKE_BUILD_TOOL} MATCHES "devenv")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "devenv")

set(exe_name BloomingFlower)
add_executable(${exe_name} ${ui_srcs} ${moc_srcs} ${resource_srcs} ${srcs} ${incs})
target_link_libraries(${exe_name} 
  ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
  ${OPENSCENEGRAPH_LIBRARIES}
  Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Concurrent
)

if(WIN32 AND MSVC)
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
elseif(CMAKE_SYSTEMname STREQUAL "Darwin")
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set_target_properties(${exe_name} PROPERTIES LINK_FLAGS -Wl)
  endif()
elseif(__COMPILER_PATHSCALE)
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS -mp)
else()
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS -Wl)
endif()

set_target_properties(${exe_name} PROPERTIES DEBUG_POSTFIX _debug)
set_target_properties(${exe_name} PROPERTIES RELEASE_POSTFIX _release)
