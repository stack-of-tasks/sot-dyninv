IF(BUILD_PYTHON_INTERFACE)

  CONFIGURE_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/robot_specific.py.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_specific.py"
    )

  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/robot_specific.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/dyninv
    )

  INSTALL(
    FILES ros/sot-concept.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
    )

ENDIF(BUILD_PYTHON_INTERFACE)
