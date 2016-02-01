set(3RD_PARTY_DIR       ${PROJECT_SOURCE_DIR}/3rdparty)

set(SVD_INCLUDE   ${3RD_PARTY_DIR}/svd)
set(svd_src     ${SVD_INCLUDE}/WunderSVD3x3.cpp)

set(CURVE_FITTING_INCLUDE ${3RD_PARTY_DIR}/opencurvefitting)
set(curve_fitting_src	${CURVE_FITTING_INCLUDE}/curve.cpp
						${CURVE_FITTING_INCLUDE}/open_curve.cpp)
					  
					  
set(SUITESPARSE_DIR "$ENV{SUITESPARSE_DIR}")
set(SUITESPARSE_INCLUDE_DIR ${SUITESPARSE_DIR}/include/suitesparse)
set(SUITESPARSE_LIBRARY_DIR ${SUITESPARSE_DIR}/lib64  ${SUITESPARSE_DIR}/lib64/lapack_blas_windows)


set(SUITESPARSE_LIBRARIES_METIS debug "metisd.lib" optimized "metis.lib")
set(SUITESPARSE_LIBRARIES_LIBBLAS "libblas.lib")
set(SUITESPARSE_LIBRARIES_LIBLAPACK "liblapack.lib")
set(SUITESPARSE_LIBRARIES_LIBSPQR debug "libspqrd.lib" optimized "libspqr.lib") 
set(SUITESPARSE_LIBRARIES_LIBAMD debug "libamdd.lib" optimized "libamd.lib")
set(SUITESPARSE_LIBRARIES_LIBBTF debug "libbtfd.lib" optimized "libbtf.lib")
set(SUITESPARSE_LIBRARIES_LIBCAMD debug "libcamdd.lib" optimized "libcam.lib")
set(SUITESPARSE_LIBRARIES_LIBCCOLAMD debug "libccolamdd.lib" optimized "libccolam.lib")
set(SUITESPARSE_LIBRARIES_LIBCHOlMOD debug "libcholmodd.lib" optimized "libcholmod.lib")
set(SUITESPARSE_LIBRARIES_LIBCOLAMD debug "libcolamdd.lib" optimized "libcolamd.lib")
set(SUITESPARSE_LIBRARIES_LIBCXSPARSE debug "libcxsparsed.lib" optimized "libcxsparse.lib")
set(SUITESPARSE_LIBRARIES_LIBKLU debug "libklud.lib" optimized "libklu.lib") 
set(SUITESPARSE_LIBRARIES_LIBLDL debug "libldld.lib" optimized "libldl.lib") 
set(SUITESPARSE_LIBRARIES_LIBUMFPACK debug "libumfpackd.lib" optimized "libumfpack.lib")
set(SUITESPARSE_LIBRARIES_SUITESPARSECONFIG debug "suitesparseconfigd.lib" optimized "suitesparseconfig.lib")

						  
set(SUITESPARSE_LIBRARIES  	${SUITESPARSE_LIBRARIES_LIBBLAS}
							${SUITESPARSE_LIBRARIES_LIBLAPACK}
							${SUITESPARSE_LIBRARIES_METIS}
							${SUITESPARSE_LIBRARIES_LIBAMD} 
							${UITESPARSE_LIBRARIES_LIBBTF}
							${SUITESPARSE_LIBRARIES_LIBCAMD}							
							${SUITESPARSE_LIBRARIES_LIBCCOLAMD}
							${SUITESPARSE_LIBRARIES_LIBCHOlMOD}
							${SUITESPARSE_LIBRARIES_LIBCOLAMD}
							${SUITESPARSE_LIBRARIES_LIBCXSPARSE}
							${SUITESPARSE_LIBRARIES_LIBKLU}
							${SUITESPARSE_LIBRARIES_LIBLDL}							
							${SUITESPARSE_LIBRARIES_LIBUMFPACK}
							${SUITESPARSE_LIBRARIES_LIBSPQR}
							${SUITESPARSE_LIBRARIES_SUITESPARSECONFIG} )