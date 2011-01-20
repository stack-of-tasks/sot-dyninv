# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST,
#                 Florent Lamiraux (CNRS/LAAS)
#
# This file is part of sot-dyninv.
# sot-dyninv is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-dyninv is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-dyninv. If not, see <http://www.gnu.org/licenses/>.

INCLUDE(../cmake/python.cmake)

IF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")
  ADD_DEFINITIONS(-DDEBUG=2)
ENDIF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")


FOREACH(lib ${libs})
  ADD_LIBRARY(${lib} SHARED ${lib}.cpp)

  SET_TARGET_PROPERTIES(${lib} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  IF(${lib}_plugins_dependencies)
    ADD_DEPENDENCIES(${lib} "${${lib}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${lib} "${${lib}_plugins_dependencies}")
  ENDIF(${lib}_plugins_dependencies)

  PKG_CONFIG_USE_DEPENDENCY(${lib} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${lib} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${lib} soth)

  INSTALL(TARGETS ${lib} DESTINATION lib/plugin)
  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${lib})
  ADD_DEPENDENCIES(${lib} MKDIR_${PYTHON_LIBRARY_NAME})
  ADD_CUSTOM_TARGET(MKDIR_${PYTHON_LIBRARY_NAME}
    mkdir -p dynamic_graph/sot/dyninv/${PYTHON_LIBRARY_NAME}
    )
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/dyninv/${PYTHON_LIBRARY_NAME}"
    ${lib}
    sot/dyninv/${PYTHON_LIBRARY_NAME}/wrap
    )
ENDFOREACH(lib)

EXEC_PROGRAM(${PYTHON_EXECUTABLE} ARGS "-c \"from distutils import sysconfig; print sysconfig.get_python_lib(0,0,prefix='')\""
  OUTPUT_VARIABLE PYTHON_SITELIB
  )

# Install empty __init__.py files in intermediate directories.
INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dyninv/__init__.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/dyninv
  )
