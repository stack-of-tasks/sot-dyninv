# Copyright 2011, François Bleibel, Olivier Stasse, JRL, CNRS/AIST
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

ADD_DEFINITIONS(-DDEBUG=2)

SET(tests
  dummy)

FOREACH(test ${tests})
  SET(EXECUTABLE_NAME "${test}_exe")
  ADD_EXECUTABLE(${EXECUTABLE_NAME}
    ${test}.cpp)

  TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
    controller-pd
    )

  PKG_CONFIG_USE_DEPENDENCY(${EXECUTABLE_NAME} sot-core)

  IF(${test}_plugins_dependencies)
    ADD_DEPENDENCIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} "${${test}_plugins_dependencies}")
  ENDIF(${test}_plugins_dependencies)


  ADD_TEST(${test} ${EXECUTABLE_NAME}
    ${samplemodelpath} sample.wrl ${samplespec} ${sampleljr} )

  IF (UNIX)
    SET_PROPERTY(TEST ${test} PROPERTY
      ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_BINARY_DIR}/src:${BOOST_ROOT}/lib")
  ENDIF(UNIX)

ENDFOREACH(test)
