set(3RD_PARTY_DIR       ${PROJECT_SOURCE_DIR}/3rdparty)

set(SVD_INCLUDE   ${3RD_PARTY_DIR}/svd)
set(svd_src     ${SVD_INCLUDE}/WunderSVD3x3.cpp)

set(CURVE_FITTING_INCLUDE ${3RD_PARTY_DIR}/opencurvefitting)
set(curve_fitting_src	${CURVE_FITTING_INCLUDE}/open_cubic_b_spline.cpp
						${CURVE_FITTING_INCLUDE}/cubic_b_spline.cpp
						${CURVE_FITTING_INCLUDE}/spline_curve_fitting.cpp)

set(ANN_DIR "$ENV{ANN_DIR}")
set(ANN_INCLUDE_DIR ${ANN_DIR}/include)
set(ANN_LIBRARY ${ANN_DIR}/lib/ANN.lib)