set(CURRENT_TARGET_NAME Trajectory)

set(Headers
    eigen_heightmap.h
    quadrupedal_kinematics.h
    quadrupedal_model.h
    traj_optimizer.h)

set(Sources
    eigen_heightmap.cpp
    quadrupedal_kinematics.cpp
    traj_optimizer.cpp)

add_library(${CURRENT_TARGET_NAME} STATIC ${Headers} ${Sources})

if(MSVC)
    source_group("Header Files" FILES ${Headers})
    source_group("Source Files" FILES ${Sources})
endif()

target_link_libraries(${CURRENT_TARGET_NAME} ${towr_LIBRARIES})

target_include_directories(${CURRENT_TARGET_NAME}
                           PUBLIC ${EIGEN3_INCLUDE_DIR}
                                   ${towr_INCLUDE_DIRS}
                                   ${CMAKE_CURRENT_SOURCE_DIR})

