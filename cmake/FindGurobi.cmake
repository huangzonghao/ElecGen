find_path(Gurobi_INCLUDE_DIRS
          NAMES gurobi_c.h
          HINTS ${Gurobi_DIR} $ENV{Gurobi_HOME}
          PATH_SUFFIXES include)

find_library(Gurobi_LIBRARY
             NAMES gurobi gurobi75 gurobi81 gurobi90
             HINTS ${Gurobi_DIR} $ENV{Gurobi_HOME}
             PATH_SUFFIXES lib)

if(MSVC)
    # determine Visual Studio year
    if(MSVC_TOOLSET_VERSION EQUAL 142)
        set(MSVC_YEAR "2019")
    elseif(MSVC_TOOLSET_VERSION EQUAL 141)
        set(MSVC_YEAR "2017")
    elseif(MSVC_TOOLSET_VERSION EQUAL 140)
        set(MSVC_YEAR "2015")
    endif()

    if(MT)
        set(M_FLAG "mt")
    else()
        set(M_FLAG "md")
    endif()

    find_library(Gurobi_CXX_LIBRARY
        NAMES gurobi_c++${M_FLAG}${MSVC_YEAR}
        HINTS ${Gurobi_DIR} $ENV{Gurobi_HOME}
        PATH_SUFFIXES lib)
    find_library(Gurobi_CXX_DEBUG_LIBRARY
        NAMES gurobi_c++${M_FLAG}d${MSVC_YEAR}
        HINTS ${Gurobi_DIR} $ENV{Gurobi_HOME}
        PATH_SUFFIXES lib)
else()
    find_library(Gurobi_CXX_LIBRARY
        NAMES gurobi_c++
        HINTS ${Gurobi_DIR} $ENV{Gurobi_HOME}
        PATH_SUFFIXES lib)
    set(Gurobi_CXX_DEBUG_LIBRARY ${Gurobi_CXX_LIBRARY})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gurobi DEFAULT_MSG Gurobi_LIBRARY)
