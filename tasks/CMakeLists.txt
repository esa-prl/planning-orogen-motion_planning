# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(motion_planningTaskLib)
ADD_LIBRARY(${MOTION_PLANNING_TASKLIB_NAME} SHARED 
    ${MOTION_PLANNING_TASKLIB_SOURCES})
add_dependencies(${MOTION_PLANNING_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${MOTION_PLANNING_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${QT_LIBRARIES}
    ${PYTHON_LIBRARIES}
    ${MOTION_PLANNING_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${MOTION_PLANNING_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${MOTION_PLANNING_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${MOTION_PLANNING_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${MOTION_PLANNING_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${MOTION_PLANNING_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${MOTION_PLANNING_TASKLIB_HEADERS}
    DESTINATION include/orocos/motion_planning)

