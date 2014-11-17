set(3RD_PARTY_DIR       ${PROJECT_SOURCE_DIR}/3rdparty)

set(CPD_DIR             ${3RD_PARTY_DIR}/cpd)
set(CPD_INCLUDE_DIR     ${CPD_DIR}/include)
set(CPD_LIBRARY_DIR     ${CPD_DIR}/lib)
set(CPD_LIBRARY         optimized ${CPD_LIBRARY_DIR}/figtree_release.lib  
						debug ${CPD_LIBRARY}/figtree_debug.lib)

set(ARAP_DIR            ${3RD_PARTY_DIR}/arap)
set(ARAP_INCLUDE_DIR    ${ARAP_DIR}/include)
set(ARAP_LIBRARY_DIR    ${ARAP_DIR}/lib)
set(ARAP_LIBRARY        optimized ${ARAP_LIBRARY_DIR}/arap_release.lib  
						debug ${ARAP_LIBRARY_DIR}/arap_debug.lib)