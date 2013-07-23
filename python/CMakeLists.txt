INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INSTALL(
  FILES ros/sot-concept.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/tutorial
)

