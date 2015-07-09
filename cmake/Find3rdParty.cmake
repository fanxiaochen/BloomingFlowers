set(3RD_PARTY_DIR       ${PROJECT_SOURCE_DIR}/3rdparty)

set(SVD_INCLUDE   ${3RD_PARTY_DIR}/svd)
set(svd_src     ${SVD_INCLUDE}/WunderSVD3x3.cpp)

set(LEMON_ROOT $ENV{LEMON_ROOT})
if(NOT LEMON_ROOT) 
  message(FATAL_ERROR "Could not find LEMON_ROOT environment variable") 
endif(NOT LEMON_ROOT) 


set(LEMON_INCLUDE ${LEMON_ROOT}/include)
set(LEMON_LIBRARY optimized ${LEMON_ROOT}/lib/lemon_release.lib debug ${LEMON_ROOT}/lib/lemon_debug.lib)