EXEC_PROGRAM("find ${AGENT_SOURCE_DIR}/core -name *\\.cpp -or -name *\\.h" OUTPUT_VARIABLE SRCS_ORIG )
STRING(REPLACE "\n" ";" SRCS ${SRCS_ORIG})


INCLUDE_DIRECTORIES(../ ${Boost_INCLUDE_DIRS})

ADD_LIBRARY( core STATIC ${SRCS}  )
