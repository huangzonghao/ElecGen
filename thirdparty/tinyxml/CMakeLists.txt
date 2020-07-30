cmake_minimum_required(VERSION 3.13.0 FATAL_ERROR)

project(TinyXML VERSION 2.6.2)

message(STATUS "This cmake file compiles only the TinyXML_STL project, for other available projects, look into the sln file")

string(TOLOWER ${PROJECT_NAME} PROJECT_TARGET_NAME)

configure_file(tinyxmlConfig.h.in tinyxmlConfig.h)

################################################################################
# Set target arch type if empty. Visual studio solution generator provides it.
################################################################################
if(NOT CMAKE_VS_PLATFORM_NAME)
    set(CMAKE_VS_PLATFORM_NAME "x64")
endif()
message("${CMAKE_VS_PLATFORM_NAME} architecture in use")

################################################################################
# Global configuration types
################################################################################
set(CMAKE_CONFIGURATION_TYPES
    "Debug"
    "Release"
    CACHE STRING "" FORCE
)

################################################################################
# Global compiler options
################################################################################
if(MSVC)
    # remove default flags provided with CMake for MSVC
    set(CMAKE_CXX_FLAGS "")
    set(CMAKE_CXX_FLAGS_DEBUG "")
    set(CMAKE_CXX_FLAGS_RELEASE "")
endif()

################################################################################
# Global linker options
################################################################################
if(MSVC)
    # remove default flags provided with CMake for MSVC
    set(CMAKE_EXE_LINKER_FLAGS "")
    set(CMAKE_MODULE_LINKER_FLAGS "")
    set(CMAKE_SHARED_LINKER_FLAGS "")
    set(CMAKE_STATIC_LINKER_FLAGS "")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS}")
    set(CMAKE_STATIC_LINKER_FLAGS_DEBUG "${CMAKE_STATIC_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS}")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS}")
endif()

################################################################################
# Nuget packages function stub.
################################################################################
function(use_package TARGET PACKAGE VERSION)
    message(WARNING "No implementation of use_package. Create yours. "
                    "Package \"${PACKAGE}\" with version \"${VERSION}\" "
                    "for target \"${TARGET}\" is ignored!")
endfunction()

################################################################################
# Common utils
################################################################################
include(CMake/Utils.cmake)

################################################################################
# Additional Global Settings(add specific info there)
################################################################################
include(CMake/GlobalSettingsInclude.cmake OPTIONAL)

################################################################################
# Use solution folders feature
################################################################################
set_property(GLOBAL PROPERTY USE_FOLDERS ON)


################################################################################
# Source groups
################################################################################
set(Headers
    "tinystr.h"
    "tinyxml.h"
)
source_group("Headers" FILES ${Headers})

set(Sources
    "tinystr.cpp"
    "tinyxml.cpp"
    "tinyxmlerror.cpp"
    "tinyxmlparser.cpp"
)
source_group("Sources" FILES ${Sources})

set(ALL_FILES ${Headers} ${Sources})

################################################################################
# Target
################################################################################
add_library(${PROJECT_TARGET_NAME} SHARED ${ALL_FILES})

set_target_properties(${PROJECT_TARGET_NAME} PROPERTIES PUBLIC_HEADER tinyxml.h)

use_props(${PROJECT_TARGET_NAME} "${CMAKE_CONFIGURATION_TYPES}" "${DEFAULT_CXX_PROPS}")
################################################################################
# Output directory
################################################################################
set_target_properties(${PROJECT_TARGET_NAME} PROPERTIES
    OUTPUT_DIRECTORY_DEBUG   "${CMAKE_SOURCE_DIR}/$<CONFIG>${PROJECT_TARGET_NAME}/"
    OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/$<CONFIG>${PROJECT_TARGET_NAME}/"
    VERSION ${PROJECT_VERSION}
)
################################################################################
# Compile definitions
################################################################################
target_compile_definitions(${PROJECT_TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:"
        "_DEBUG"
    ">"
    "$<$<CONFIG:Release>:"
        "NDEBUG"
    ">"
    "WIN32;"
    "_LIB;"
    "TIXML_USE_STL;"
    "_MBCS"
)

################################################################################
# Compile and link options
################################################################################
if(MSVC)
    target_compile_options(${PROJECT_TARGET_NAME} PRIVATE
        $<$<CONFIG:Debug>:
            /Gm;
            /Od;
            /RTC1;
            /MTd;
            /W4;
            /ZI
        >
        $<$<CONFIG:Release>:
            /O2;
            /Ob1;
            /GF;
            /MT;
            /Gy;
            /W3;
            ${DEFAULT_CXX_DEBUG_INFORMATION_FORMAT}
        >
        ${DEFAULT_CXX_EXCEPTION_HANDLING}
    )
    target_link_options(${PROJECT_TARGET_NAME} PRIVATE
        /NOLOGO
    )
endif()

install(TARGETS ${PROJECT_TARGET_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
