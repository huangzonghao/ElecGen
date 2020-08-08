if(NOT Gurobi_ROOT_DIR AND DEFINED ENV{Gurobi_ROOT_DIR})
  set(Gurobi_ROOT_DIR "$ENV{Gurobi_ROOT_DIR}" CACHE PATH
      "Gurobi base directory location (optional, used for nonstandard installation paths)")
endif()

# Search path for nonstandard package locations
if(Gurobi_ROOT_DIR)
  set(Gurobi_INCLUDE_PATH PATHS "${Gurobi_ROOT_DIR}/include" NO_DEFAULT_PATH)
  set(Gurobi_LIBRARY_PATH PATHS "${Gurobi_ROOT_DIR}/lib"     NO_DEFAULT_PATH)
endif()

# Find headers and libraries
find_path(Gurobi_INCLUDE_DIR NAMES gurobi_c++.h PATH_SUFFIXES "gurobi" ${Gurobi_INCLUDE_PATH})
find_library(Gurobi_LIBRARY  NAMES gurobi75 gurobi90 PATH_SUFFIXES "gurobi" ${Gurobi_LIBRARY_PATH})
find_library(Gurobi_CPP_LIBRARY  NAMES gurobi_c++ gurobi_c++md2017 PATH_SUFFIXES "gurobi" ${Gurobi_LIBRARY_PATH})

mark_as_advanced(Gurobi_INCLUDE_DIR
                 Gurobi_LIBRARY
                 Gurobi_CPP_LIBRARY)

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gurobi DEFAULT_MSG Gurobi_LIBRARY
                                                     Gurobi_CPP_LIBRARY
                                                     Gurobi_INCLUDE_DIR)

set(Gurobi_FOUND ${GUROBI_FOUND}) # Enforce case-correctness: Set appropriately cased variable...
unset(GUROBI_FOUND) # ...and unset uppercase variable generated by find_package_handle_standard_args

if(Gurobi_FOUND)
  set(Gurobi_INCLUDE_DIRS ${Gurobi_INCLUDE_DIR})
  set(Gurobi_LIBRARIES ${Gurobi_LIBRARY} ${Gurobi_CPP_LIBRARY})
endif()
