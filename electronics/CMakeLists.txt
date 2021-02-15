set(CURRENT_TARGET_NAME Electronics)

set(Headers
    include/actuator.h
    include/Circuit.h
    include/Inference.h
    include/Matching.h
    include/typefile.h
    include/utility_functions.h)

set(Sources
    src/actuator.cpp
    src/Circuit.cpp
    src/Inference.cpp
    src/Matching.cpp
    src/utility_functions.cpp)

add_library(${CURRENT_TARGET_NAME} STATIC ${Headers} ${Sources})

if(MSVC)
    source_group("Header Files" FILES ${Headers})
    source_group("Source Files" FILES ${Sources})

    # to make protobuf run with dll
    target_compile_definitions(${CURRENT_TARGET_NAME} PUBLIC PROTOBUF_USE_DLLS)
endif()


target_link_libraries(${CURRENT_TARGET_NAME}
                      optimized ${Gurobi_CXX_LIBRARY}
                      debug ${Gurobi_CXX_DEBUG_LIBRARY})

target_link_libraries(${CURRENT_TARGET_NAME}
                      ${Gurobi_LIBRARY}
                      component_structure)

target_include_directories(${CURRENT_TARGET_NAME}
                           PUBLIC ${Boost_INCLUDE_DIRS}
                                  ${EIGEN3_INCLUDE_DIR}
                                  ${Gurobi_INCLUDE_DIRS}
                                  ${Discreture_INCLUDE_DIRS}
                                  ${PROJECT_IN_FILE_OUTPUT_DIR}
                                  ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (CONFIG_ELECTRONICS_TEST)

project(Electronics_Test)

set(Test_Headers include/testfile.h)

set(Test_Sources src/main.cpp src/testfile.cpp)

add_executable(${PROJECT_NAME} ${Test_Headers} ${Test_Sources})

if(MSVC)
    source_group("Header Files" FILES ${Test_Headers})
    source_group("Source Files" FILES ${Test_Sources})
endif()

target_link_libraries(${PROJECT_NAME} Electronics)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    file(GLOB PROJECT_DEP_DLLS "${PROJECT_DEP_INSTALL_DIR}/bin/*.dll")
    list(APPEND PROJECT_DEP_DLLS ${Gurobi_DLL_LIBRARY} ${CHRONO_DLLS})
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
    DLL_Copy_Command(${PROJECT_NAME}_COPY_DLLS "${PROJECT_DEP_DLLS}")
endif()

endif(CONFIG_ELECTRONICS_TEST)
