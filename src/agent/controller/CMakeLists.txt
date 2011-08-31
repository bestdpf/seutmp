EXEC_PROGRAM("find ${AGENT_SOURCE_DIR}/controller -name *\\.cpp -or -name *\\.h" OUTPUT_VARIABLE SRCS_ORIG )
STRING(REPLACE "\n" ";" SRCS ${SRCS_ORIG})

INCLUDE_DIRECTORIES(../)

ADD_LIBRARY( controller STATIC ${SRCS}  )
