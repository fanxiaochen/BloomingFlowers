cmake_minimum_required(VERSION 2.8)

project(BloomingFlower)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#set(Boost_USE_STATIC_LIBS  ON)
#set(Boost_USE_MULTITHREADED ON)  
#set(Boost_USE_STATIC_RUNTIME OFF)
#find_package(Boost REQUIRED system thread)
#include_directories(${Boost_INCLUDE_DIRS})

include(cotire)

find_package(OpenSceneGraph REQUIRED osgViewer osgText osgDB osgGA osgQt osgManipulator osgUtil)
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

find_package(PCL REQUIRED common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Concurrent REQUIRED)

# Set Qt files and resources here
set(uis       resource/main_window.ui)
set(resources  resource/main_window.qrc)

# Qt stuff
QT5_WRAP_UI(ui_srcs ${uis})
QT5_ADD_RESOURCES(resource_srcs ${resources})

find_package(3rdParty)
include_directories(${CPD_INCLUDE_DIR})
include_directories(${ARAP_INCLUDE_DIR})

set(lily_inc    include/lily)
set(lily_src    src/lily)

set(prelily_inc     ${lily_inc}/lily.h)
set(prelily_src     ${lily_src}/lily.cpp)

set(blooming_inc    include/blooming)
set(blooming_src    src/blooming)

set(filesystem_inc  include/filesystem)
set(filesystem_src  src/filesystem)

include_directories(${PROJECT_SOURCE_DIR}/${lily_inc})
include_directories(${PROJECT_SOURCE_DIR}/${blooming_inc})
include_directories(${PROJECT_SOURCE_DIR}/${filesystem_inc})

set(lily_incs       ${lily_inc}/main_window.h
                    ${lily_inc}/threaded_painter.h
                    ${lily_inc}/osg_viewer_widget.h
                    ${lily_inc}/toggle_handler.h
                    ${lily_inc}/renderable.h
                    ${lily_inc}/update_callback.h
                    ${lily_inc}/adapter_widget.h
                    ${lily_inc}/scene_widget.h
                    ${lily_inc}/file_viewer_widget.h
            )

set(lily_srcs       ${lily_src}/main.cpp
                    ${lily_src}/main_window.cpp
                    ${lily_src}/adapter_widget.cpp
                    ${lily_src}/threaded_painter.cpp
                    ${lily_src}/osg_viewer_widget.cpp
                    ${lily_src}/toggle_handler.cpp
                    ${lily_src}/renderable.cpp
                    ${lily_src}/update_callback.cpp
                    ${lily_src}/scene_widget.cpp
                    ${lily_src}/file_viewer_widget.cpp
            )
            
set(blooming_incs   ${blooming_inc}/point_cloud.h
                    ${blooming_inc}/mesh_model.h
                    ${blooming_inc}/types_wrapper.h
                    ${blooming_inc}/tracking_system.h
                    ${blooming_inc}/track_thread.h
                    )

set(blooming_srcs	${blooming_src}/point_cloud.cpp
                    ${blooming_src}/mesh_model.cpp
                    ${blooming_src}/tracking_system.cpp
                    ${blooming_src}/track_thread.cpp
                    )
                    
set(filesystem_incs	${filesystem_inc}/file_system_model.h
                    ${filesystem_inc}/points_file_system.h
                    ${filesystem_inc}/mesh_file_system.h
                    ${filesystem_inc}/tiny_obj_loader.h
                    )
                    
set(filesystem_srcs	${filesystem_src}/file_system_model.cpp
                    ${filesystem_src}/points_file_system.cpp
                    ${filesystem_src}/mesh_file_system.cpp
                    ${filesystem_src}/tiny_obj_loader.cc
                    )

set(incs    ${lily_incs} ${blooming_incs} ${filesystem_incs})
set(srcs    ${lily_srcs} ${blooming_srcs} ${filesystem_srcs})

set(Headers     "Header Files")
set(Sources     "Source Files")

set(header_lily         ${Headers}\\lily)
set(header_blooming     ${Headers}\\blooming)
set(header_filesystem   ${Headers}\\filesystem)

set(source_lily         ${Sources}\\lily)
set(source_blooming     ${Sources}\\blooming)
set(source_filesystem   ${Sources}\\filesystem)

# Organize files
SOURCE_GROUP("Resources" FILES ${uis} ${resources})
SOURCE_GROUP("Generated" FILES ${ui_srcs} ${moc_srcs} ${resource_srcs})
SOURCE_GROUP(${header_lily} FILES ${lily_incs})
SOURCE_GROUP(${header_blooming} FILES ${blooming_incs})
SOURCE_GROUP(${header_filesystem} FILES ${filesystem_incs})
SOURCE_GROUP(${source_lily} FILES ${lily_srcs})
SOURCE_GROUP(${source_blooming} FILES ${blooming_srcs})
SOURCE_GROUP(${source_filesystem} FILES ${filesystem_srcs})


SET_SOURCE_FILES_PROPERTIES(${srcs} PROPERTIES OBJECT_DEPENDS "${ui_srcs}")

# Put the ui in the windows project file
IF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
IF (${CMAKE_BUILD_TOOL} MATCHES "devenv")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "devenv")


set(link_libraies	${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
  ${OPENSCENEGRAPH_LIBRARIES}
  Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Concurrent
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
  ${CPD_LIBRARY} ${ARAP_LIBRARY})

set(exe_name BloomingFlower)
add_executable(${exe_name} ${prelily_src} ${prelily_inc} ${ui_srcs} ${moc_srcs} ${resource_srcs} ${srcs} ${incs})
target_link_libraries(${exe_name} ${link_libraies})

set_target_properties(${exe_name} PROPERTIES COTIRE_CXX_PREFIX_HEADER_INIT ${prelily_inc})
set_target_properties(${exe_name} PROPERTIES COTIRE_UNITY_LINK_LIBRARIES_INIT "COPY")
cotire(${exe_name})

if(WIN32 AND MSVC)
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS /INCREMENTAL:YES)
  #set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/INCREMENTAL:YES")
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
