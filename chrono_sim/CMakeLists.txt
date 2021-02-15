set(CURRENT_TARGET_NAME Simulator)

set(Headers
    include/ChUrdfDoc.h
    include/LeggedGaits.h
    include/RobotController.h
    include/SimMotor.h
    include/SimulationManager.h)

set(Sources
    src/ChUrdfDoc.cpp
    src/RobotController.cpp
    src/SimMotor.cpp
    src/SimulationManager.cpp)

add_library(${CURRENT_TARGET_NAME} STATIC ${Headers} ${Sources})

if(MSVC)
    source_group("Header Files" FILES ${Headers})
    source_group("Source Files" FILES ${Sources})
endif()

target_link_libraries(${CURRENT_TARGET_NAME}
                      ${CHRONO_LIBRARIES}
                      ${urdfdom_LIBRARIES})

target_include_directories(${CURRENT_TARGET_NAME}
                           PUBLIC ${EIGEN3_INCLUDE_DIR}
                                  ${urdfdom_INCLUDE_DIRS}
                                  ${CHRONO_INCLUDE_DIRS}
                                  ${PROJECT_IN_FILE_OUTPUT_DIR}
                                  ${CMAKE_CURRENT_SOURCE_DIR}/include)

set_target_properties(${CURRENT_TARGET_NAME} PROPERTIES
                      COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
                      COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
                      LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

if (CONFIG_CHRONO_TEST)

project(Simulator_Test)

set(Test_Sources src/test_chrono.cpp)

add_executable(${PROJECT_NAME} ${Test_Sources})

if(MSVC)
    source_group("Source Files" FILES ${Test_Sources})
endif()

target_link_libraries(${PROJECT_NAME} Simulator)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    file(GLOB PROJECT_DEP_DLLS "${PROJECT_DEP_INSTALL_DIR}/bin/*.dll")
    list(APPEND PROJECT_DEP_DLLS ${Gurobi_DLL_LIBRARY} ${CHRONO_DLLS})
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
    DLL_Copy_Command(${PROJECT_NAME}_COPY_DLLS "${PROJECT_DEP_DLLS}")
endif()

endif(CONFIG_CHRONO_TEST)
