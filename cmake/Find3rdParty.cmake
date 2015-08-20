set(3RD_PARTY_DIR       ${PROJECT_SOURCE_DIR}/3rdparty)

set(SVD_INCLUDE   ${3RD_PARTY_DIR}/svd)
set(svd_src     ${SVD_INCLUDE}/WunderSVD3x3.cpp)

set(CURVE_FITTING_INCLUDE ${3RD_PARTY_DIR}/opencurvefitting)
set(curve_fitting_src	${CURVE_FITTING_INCLUDE}/curve.cpp
						${CURVE_FITTING_INCLUDE}/open_curve.cpp)

set(OPENNURBS_DIR "$ENV{OPENNURBS_DIR}")
set(OPENNURBS_INCLUDE_DIR ${OPENNURBS_DIR}/include)
set(OPENNURBS_LIBRARY optimized ${OPENNURBS_DIR}/lib/release/opennurbs.lib
					  debug ${OPENNURBS_DIR}/lib/debug/opennurbs.lib)
					  
					  
set(SUITESPARSE_DIR "$ENV{SUITESPARSE_DIR}")
set(SUITESPARSE_INCLUDE_DIR ${SUITESPARSE_DIR}/include/suitesparse)
set(SUITESPARSE_LIBRARY_DIR ${SUITESPARSE_DIR}/lib64  ${SUITESPARSE_DIR}/lib64/lapack_blas_windows)

set(SUITESPARSE_LIBRARIES_DEBUG 
						  "libamdd.lib"
						  "libbtfd.lib"
						  "libcamdd.lib"
						  "libccolamdd.lib"
						  "libcholmodd.lib"
						  "libcolamdd.lib"
						  "libcxsparsed.lib"
						  "libklud.lib"
						  "libldld.lib"
						  "libspqrd.lib"
						  "libumfpackd.lib"
						  "suitesparseconfigd.lib"
						  "libblas.lib"
						  "liblapack.lib")

set(SUITESPARSE_LIBRARIES_RELEASE 
					      "libamd.lib"
						  "libbtf.lib"
						  "libcamd.lib"
						  "libccolamd.lib"
						  "libcholmod.lib"
						  "libcolamd.lib"
						  "libcxsparse.lib"
						  "libklu.lib"
						  "libldl.lib"
						  "libspqr.lib"
						  "libumfpack.lib"
						  "suitesparseconfig.lib"
						  "libblas.lib"
						  "liblapack.lib")	
						  
set(SUITESPARSE_LIBRARIES  optimized ${SUITESPARSE_LIBRARIES_RELEASE}
						   debug ${SUITESPARSE_LIBRARIES_DEBUG})