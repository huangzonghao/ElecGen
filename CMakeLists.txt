cmake_minimum_required(VERSION 3.0)

project(Electronics_Generation)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(Chrono
             COMPONENTS Irrlicht Vehicle
             CONFIG QUIET)
if (NOT Chrono_FOUND)
    message(SEND_ERROR "Error: Could not find Chrono or one of its required modules.")
    message("  Set Chrono_DIR in ${CMAKE_BINARY_DIR}/CMakeCache.txt to the path that contains ChronoConfig.cmake\n\n")
    return()
endif()

find_package(urdfdom QUIET)
if (NOT urdfdom_FOUND)
    if( NOT EXISTS "${CMAKE_SOURCE_DIR}/thirdparty/urdfdom/.git" OR
        NOT EXISTS "${CMAKE_SOURCE_DIR}/thirdparty/urdfdom_headers/.git" OR
        NOT EXISTS "${CMAKE_SOURCE_DIR}/thirdparty/console_bridge/.git")

        execute_process(COMMAND git submodule update --init --recursive)
    endif()

    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty/urdfdom
                        ${CMAKE_BINARY_DIR}/thirdparty/urdfdom_headers
                        ${CMAKE_BINARY_DIR}/thirdparty/console_bridge
                        ${CMAKE_BINARY_DIR}/thirdparty/Install)

    execute_process(COMMAND cmake -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/thirdparty/Install ${CMAKE_SOURCE_DIR}/thirdparty/urdfdom_headers
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty/urdfdom_headers)

    execute_process(COMMAND make install
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty/urdfdom_headers)

    set(urdfdom_headers_DIR ${CMAKE_BINARY_DIR}/thirdparty/Install/lib/urdfdom_headers/cmake
        CACHE PATH "Path to CMake config files" FORCE)

    execute_process(COMMAND cmake -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/thirdparty/Install ${CMAKE_SOURCE_DIR}/thirdparty/console_bridge
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty/console_bridge)

    execute_process(COMMAND make install
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty/console_bridge)

    set(console_bridge_DIR ${CMAKE_BINARY_DIR}/thirdparty/Install/lib/console_bridge/cmake
        CACHE PATH "Path to CMmake config files" FORCE)

    execute_process(COMMAND cmake -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/thirdparty/Install
                                  -Durdfdom_headers_DIR:PATH=${CMAKE_BINARY_DIR}/thirdparty/Install/lib/urdfdom_headers/cmake
                                  -Dconsole_bridge_DIR:PATH=${CMAKE_BINARY_DIR}/thirdparty/Install/lib/console_bridge/cmake
                                  ${CMAKE_SOURCE_DIR}/thirdparty/urdfdom
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/thirdparty/urdfdom)

    execute_process(COMMAND make install
                    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty/urdfdom")

    set(urdfdom_DIR ${CMAKE_BINARY_DIR}/thirdparty/Install/lib/urdfdom/cmake
        CACHE PATH "Path to CMake config files" FORCE)

    find_package(urdfdom QUIET)
endif()

include_directories(${urdfdom_INCLUDE_DIRS})

find_package(Matlab)

if (NOT Matlab_FOUND)
    message(SEND_ERROR "Could not find Matlab.")
    return()
endif()


#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(${CHRONO_INCLUDE_DIRS})

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
    set(EXTRA_COMPILE_FLAGS "/wd4275")             # disable warnings triggered by Irrlicht
else()
    set(EXTRA_COMPILE_FLAGS "")
endif()

# debug flag
add_definitions(-DDEBUG)

# Need to use the filesystem of C++17
set (CMAKE_CXX_STANDARD 17)

matlab_add_mex(NAME mexRun SRC src/mexRun.cpp src/ChUrdfDoc.cpp src/launch_simulation.cpp)

set_target_properties(mexRun PROPERTIES
        COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
        COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
        LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

target_link_libraries(mexRun ${CHRONO_LIBRARIES} ${urdfdom_LIBRARIES})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_DLL_copy_command("${CHRONO_DLLS}")
