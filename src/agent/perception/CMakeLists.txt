EXEC_PROGRAM("find ${AGENT_SOURCE_DIR}/perception -name *\\.cpp -or -name *\\.h" OUTPUT_VARIABLE SRCS_ORIG )
STRING(REPLACE "\n" ";" SRCS ${SRCS_ORIG})

INCLUDE_DIRECTORIES(../)

ADD_LIBRARY( perception STATIC ${SRCS}  )
