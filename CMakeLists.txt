cmake_minimum_required(VERSION 3.16)

project(Electronics_Generation)

#######################################################################
#                        Project Configuration                        #
#######################################################################
if (NOT DEFINED CONFIG_CHRONO_TEST)
    set(CONFIG_CHRONO_TEST True CACHE BOOL "Generate Chrono_Test Target")
endif()
if (NOT DEFINED CONFIG_ELECTRONICS_TEST)
    set(CONFIG_ELECTRONICS_TEST True CACHE BOOL "Generate Electronics_Test Target")
endif()

if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set (PROJECT_IN_FILE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
if (NOT DEFINED PROJECT_IN_FILE_OUTPUT_DIR)
    file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/include)
    set(PROJECT_IN_FILE_OUTPUT_DIR ${PROJECT_BINARY_DIR}/include)
endif()
set(PROJECT_DEP_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/Install)
list(APPEND CMAKE_PREFIX_PATH ${PROJECT_DEP_INSTALL_DIR})

#######################################################################
#                         Setup Dependencies                          #
#######################################################################

# first take care of external dependencies
message(STATUS "Looking for external dependencies")

# eigen3 find script doesn't work well on WIN32
if (UNIX)
    find_package(Eigen3 QUIET)
    if (NOT Eigen3_FOUND)
        message(SEND_ERROR "Error: Could not find Eigen3")
        message("  Set Eigen3_DIR to the directory that contains Eigen3Config.cmake")
        set(NEED_TO_RETURN True)
    else()
        message(STATUS "Eigen3 Found")
    endif()
elseif (WIN32)
    set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" CACHE PATH "Eigen3 root directory")
    if (NOT EIGEN3_INCLUDE_DIR)
        message(SEND_ERROR "Error: Could not find Eigen3")
        message("  Set EIGEN3_INCLUDE_DIR to the root directory of your unzipped eigen library")
        set(NEED_TO_RETURN True)
    else()
        message(STATUS "Eigen3 Found")
    endif()
else()
    message(SEND_ERROR "Unknown system")
    return()
endif()

find_package(Chrono
             COMPONENTS Irrlicht Vehicle
             CONFIG QUIET)
find_package(Gurobi QUIET)
find_package(Boost QUIET)

if (NOT Chrono_FOUND)
    message(SEND_ERROR "Error: Could not find Chrono or one of its required modules.")
    message("  Set Chrono_DIR to the directory that contains ChronoConfig.cmake")
    set(NEED_TO_RETURN True)
    else()
        message(STATUS "Chrono Found")
endif()
if (NOT Gurobi_FOUND)
    message(SEND_ERROR "Could not find Gurobi, set Gurobi_DIR to the directory containing Gurobi's bin, include, lib, etc")
    set(Gurobi_DIR "Set Path to Gurobi Root" CACHE PATH "Gurobi base directory")
    set(NEED_TO_RETURN True)
    else()
        message(STATUS "Gurobi Found")
endif()
if (NOT Boost_FOUND)
    message(SEND_ERROR "Could not find Boost, set BOOST_ROOT to the Boost base directory containing boost, libs, etc")
    set(BOOST_ROOT "Set Path To Boost Root" CACHE PATH "Boost base directory")
    set(NEED_TO_RETURN True)
    else()
        message(STATUS "Boost Found")
endif()

if (NEED_TO_RETURN)
    message("Some dependencies were not found. Set corresponding variables as mentioned above. If you use cmake-cli, edit entries in the CMakeCache.txt")
    return()
endif()

find_package(urdfdom QUIET)
if (NOT urdfdom_FOUND)
    if( NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom/.git" OR
        NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom_headers/.git" OR
        NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/console_bridge/.git")

        execute_process(COMMAND git submodule update --init --recursive --depth 1
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endif()

    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/urdfdom
                        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/urdfdom_headers
                        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/console_bridge
                        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/tinyxml
                        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/Install)

    execute_process(COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_DEP_INSTALL_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom_headers
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/urdfdom_headers)

    execute_process(COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} --target install
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/urdfdom_headers)

    execute_process(COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_DEP_INSTALL_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/console_bridge
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/console_bridge)

    execute_process(COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} --target install
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/console_bridge)

    execute_process(COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_DEP_INSTALL_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/tinyxml
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/tinyxml)

    execute_process(COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} --target install
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/tinyxml)

    if (WIN32) # Windows
        # set(ENV{CL} /D_USE_MATH_DEFINES $ENV{CL})
        # Need to do this due to a flaw of upstream library -- https://github.com/ros/urdfdom/issues/42
        file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/cmake/urdfdom_cmakelists_patched.txt DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom/)
        file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/cmake/urdfparser_cmakelists_patched.txt DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom/urdf_parser)
        file(RENAME ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom/urdfdom_cmakelists_patched.txt ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom/CMakeLists.txt)
        file(RENAME ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom/urdf_parser/urdfparser_cmakelists_patched.txt ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom/urdf_parser/CMakeLists.txt)
    endif()

    execute_process(COMMAND ${CMAKE_COMMAND}
                            -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_DEP_INSTALL_DIR}
                            -DTinyXML_ROOT_DIR:PATH=${PROJECT_DEP_INSTALL_DIR}
                            -DCMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL=True
                            ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/urdfdom
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/urdfdom)

    execute_process(COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} --target install
                    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/thirdparty/urdfdom")

    mark_as_advanced(console_bridge_DIR)
    mark_as_advanced(urdfdom_DIR)
    mark_as_advanced(urdfdom_headers_DIR)

    find_package(urdfdom QUIET)
else()
    message(STATUS "urdfdom Found")
endif()

find_package(Matlab QUIET)
if (NOT Matlab_FOUND)
    message(SEND_ERROR "Could not find Matlab.")
    return()
else()
    message(STATUS "Matlab Found")
endif()
mark_as_advanced(Matlab_ROOT_DIR)

find_package(Protobuf QUIET)
if (NOT Protobuf_FOUND)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/protobuf)
    if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/.git")
        execute_process(COMMAND git submodule update --init --recursive --depth 1
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND}
                            -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_DEP_INSTALL_DIR}
                            -Dprotobuf_BUILD_TESTS=OFF
                            -Dprotobuf_MSVC_STATIC_RUNTIME=OFF
                            -Dprotobuf_WITH_ZLIB=OFF
                            -Dprotobuf_BUILD_SHARED_LIBS=ON
                            ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/protobuf/cmake
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/protobuf)

    execute_process(COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} --target install
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/protobuf)
    find_package(Protobuf QUIET)
else()
    message(STATUS "Protobuf Found")
endif()
mark_as_advanced(Protobuf_SRC_ROOT_FOLDER)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/electronics/proto)

if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/discreture/.git")
    execute_process(COMMAND git submodule update --init --recursive --depth 1
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
set (Discreture_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/discreture/include)

find_package(towr QUIET)
if (NOT towr_FOUND)
    if( NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Ipopt/.git" OR
        NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ifopt/.git" OR
        NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/towr/.git")

        execute_process(COMMAND git submodule update --init --recursive --depth 1
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endif()

    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/towr
                        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/ifopt
                        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/Ipopt)

    # TODO: Ipopt compilation on windows
    if (NOT WIN32)
    execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Ipopt/configure --prefix ${PROJECT_DEP_INSTALL_DIR}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/Ipopt)

    execute_process(COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} --target install
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/Ipopt)
    endif()

    execute_process(COMMAND ${CMAKE_COMMAND}
                            -DIPOPT_DIR:PATH=${PROJECT_DEP_INSTALL_DIR}
                            -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
                            -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_DEP_INSTALL_DIR}
                            ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ifopt
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/ifopt)

    execute_process(COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} --target install
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/ifopt)

    execute_process(COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_DEP_INSTALL_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/towr/towr
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/towr)

    execute_process(COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} --target install
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/towr)

    find_package(towr QUIET)
else()
    message(STATUS "Towr Found")
endif()

include(DLL_Copy_Command)

#######################################################################
#                           Platform Flags                            #
#######################################################################

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

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

# debug flag
# add_definitions(-DDEBUG)

# Need to use the filesystem of C++17
set (CMAKE_CXX_STANDARD 17)

#######################################################################
#                            Source Files                             #
#######################################################################
# set(MATLAB_CPP_GATEWAY ${CHRONOSIM_SRC_PATH}/mexRun.cpp
                       # ${CHRONOSIM_SRC_PATH}/launch_simulation.cpp)

## Matlab Interface
# =================
# matlab_add_mex(NAME mexRun SRC ${MATLAB_CPP_GATEWAY} ${ELECTRONICS_SOURCES} ${CHRONOSIM_SOURCES} ${TRAJECTORY_SOURCES})

# set_target_properties(mexRun PROPERTIES
        # COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
        # COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
        # LINK_FLAGS "${CHRONO_LINKER_FLAGS}")
# if(MSVC)
    # # to make protobuf run with dll
    # target_compile_definitions(mexRun PUBLIC PROTOBUF_USE_DLLS)
# endif()

# target_link_libraries(mexRun
                      # optimized ${Gurobi_CXX_LIBRARY}
                      # debug ${Gurobi_CXX_DEBUG_LIBRARY})

# target_link_libraries(mexRun ${CHRONO_LIBRARIES}
                             # ${urdfdom_LIBRARIES}
                             # # ${towr_LIBRARIES}
                             # ${Gurobi_LIBRARY}
                             # component_structure)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Matlab)
set(ELECGEN_MATLAB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ui/)

# set project specified paths
set(ELECGEN_DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data/)
if (MSVC)
    set(ELECGEN_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE})
else() # UNIX
    set(ELECGEN_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
endif()
set(ELECGEN_EXE_PATH ${ELECGEN_OUTPUT_PATH}/${PROJECT_NAME})

configure_file(${PROJECT_IN_FILE_DIR}/data_dir_path.h.in ${PROJECT_IN_FILE_OUTPUT_DIR}/data_dir_path.h @ONLY)
configure_file(${PROJECT_IN_FILE_DIR}/elec_gen.m.in ${CMAKE_CURRENT_BINARY_DIR}/Matlab/elec_gen.m @ONLY)

#######################################################################
#                           Subdirectories                            #
#######################################################################
add_subdirectory(electronics)
add_subdirectory(chrono_sim)
# add_subdirectory(trajectory)

#######################################################################
#                               TARGET                                #
#######################################################################

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/chrono_sim/src/elec_gen.cpp
                               ${CMAKE_CURRENT_SOURCE_DIR}/electronics/src/testfile.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES
                      COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
                      COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
                      LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

target_link_libraries(${PROJECT_NAME} Electronics Simulator)

if(MSVC)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    file(GLOB PROJECT_DEP_DLLS "${PROJECT_DEP_INSTALL_DIR}/bin/*.dll")
    list(APPEND PROJECT_DEP_DLLS ${Gurobi_DLL_LIBRARY} ${CHRONO_DLLS})
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
    DLL_Copy_Command(${PROJECT_NAME}_COPY_DLLS "${PROJECT_DEP_DLLS}")
endif()
