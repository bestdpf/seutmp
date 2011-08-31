SET(AGENT_SOURCE_DIR "${CMAKE_SOURCE_DIR}/src/agent")

INCLUDE_DIRECTORIES(../lib)

ADD_SUBDIRECTORY( action )
ADD_SUBDIRECTORY( controller )
ADD_SUBDIRECTORY( perception )
ADD_SUBDIRECTORY( task )
ADD_SUBDIRECTORY( core )
ADD_SUBDIRECTORY( soccer )
ADD_SUBDIRECTORY( configuration )
INCLUDE_DIRECTORIES(./)
ADD_EXECUTABLE(seu-spark-agent main.cpp)
TARGET_LINK_LIBRARIES(seu-spark-agent
  soccer
  task
  core
  controller
  perception
  action
  configuration
  robot
  socketstream
  sfsexp
  boost_program_options-mt
  boost_thread-mt
)
SET( CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin/")
INSTALL(TARGETS seu-spark-agent 
RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)
INSTALL(DIRECTORY data rsg DESTINATION ${CMAKE_INSTALL_PREFIX})
INSTALL(FILES formation.conf DESTINATION ${CMAKE_INSTALL_PREFIX})
INSTALL( PROGRAMS start.sh kill.sh goalkeeper.sh oneattacker.sh DESTINATION ${CMAKE_INSTALL_PREFIX})
