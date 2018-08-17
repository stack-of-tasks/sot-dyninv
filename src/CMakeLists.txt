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

IF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")
  ADD_DEFINITIONS(-DDEBUG=2)
ENDIF(CMAKE_BUILD_TYPE STREQUAL "DEBUG")

# --- HEADERS
FOREACH(header ${headers})
  get_filename_component(headerName ${header} NAME)
  set(src_path ${${PROJECT_NAME}_SOURCE_DIR}/src/${header})
  set(bin_path ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${headerName})
  set(install_path ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
  execute_process(COMMAND ${CMAKE_COMMAND} -E remove ${bin_path})
  IF(WIN32)
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${src_path} ${bin_path} )
  ELSE(WIN32)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${src_path} ${bin_path} )
  ENDIF(WIN32)
  install(FILES ${src_path} DESTINATION ${install_path}
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDFOREACH(header)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/include)

# --- COMPILATION OF PLUGINS
FOREACH(lib ${libs})
  ADD_LIBRARY(${lib} SHARED ${lib}.cpp)

  SET_TARGET_PROPERTIES(${lib} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  IF(${lib}_plugins_dependencies)
    ADD_DEPENDENCIES(${lib} "${${lib}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${lib} "${${lib}_plugins_dependencies}")
  ENDIF(${lib}_plugins_dependencies)

  IF(${lib}_ext_plugins_dependencies)
    TARGET_LINK_LIBRARIES(${lib} "${${lib}_ext_plugins_dependencies}")
  ENDIF(${lib}_ext_plugins_dependencies)

  PKG_CONFIG_USE_DEPENDENCY(${lib} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${lib} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${lib} soth)

  INSTALL(TARGETS ${lib} DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
  # build python submodule
  IF(BUILD_PYTHON_INTERFACE)
    STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${lib})
    DYNAMIC_GRAPH_PYTHON_MODULE("sot/dyninv/${PYTHON_LIBRARY_NAME}"
      ${lib}
      sot-dyninv-${PYTHON_LIBRARY_NAME}-wrap
      )
  ENDIF(BUILD_PYTHON_INTERFACE)
ENDFOREACH(lib)

# Install empty __init__.py files in intermediate directories.
IF(BUILD_PYTHON_INTERFACE)
  INSTALL(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dyninv/__init__.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dyninv/meta_task_dyn_6d.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dyninv/meta_task_dyn_passing_point.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dyninv/meta_tasks_dyn.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/dyninv/meta_tasks_dyn_relative.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/dyninv
    )
ENDIF(BUILD_PYTHON_INTERFACE)
