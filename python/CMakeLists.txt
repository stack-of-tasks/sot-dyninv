INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES ros/sot-concept.py ros/ros-walkromeo.py ros/ros-kineromeo.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

