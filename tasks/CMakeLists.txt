# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(viso2_with_imuTaskLib)
ADD_LIBRARY(${VISO2_WITH_IMU_TASKLIB_NAME} SHARED 
    ${VISO2_WITH_IMU_TASKLIB_SOURCES})
add_dependencies(${VISO2_WITH_IMU_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${VISO2_WITH_IMU_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${VISO2_WITH_IMU_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${VISO2_WITH_IMU_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${VISO2_WITH_IMU_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${VISO2_WITH_IMU_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${VISO2_WITH_IMU_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${VISO2_WITH_IMU_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${VISO2_WITH_IMU_TASKLIB_HEADERS}
    DESTINATION include/orocos/viso2_with_imu)

