# Autogenerated file, run build/tools/setup_cmake.py to regenerate

if(NOT DEFINED CHOLMOD_LIBRARIES)
message(STATUS "Checking for CHOLMOD")

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(CHOLMOD_PKGCONF CHOLMOD)

# Include dir
find_path(CHOLMOD_INCLUDE_DIR
  NAMES ufsparse/cholmod.h
  PATHS ${CHOLMOD_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(CHOLMOD_LIBRARY
  NAMES cholmod amd metis colamd ccolamd camd blas
  PATHS ${CHOLMOD_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(CHOLMOD_PROCESS_INCLUDES CHOLMOD_INCLUDE_DIR)
set(CHOLMOD_PROCESS_LIBS CHOLMOD_LIBRARY)
libfind_process(CHOLMOD)

if (${CHOLMOD_LIBRARY} MATCHES "CHOLMOD_LIBRARY-NOTFOUND"
    OR ${CHOLMOD_INCLUDE_DIR} MATCHES "CHOLMOD_INCLUDE_DIR-NOTFOUND")
  message(STATUS "CHOLMOD not found")
  file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/CHOLMOD" "ok=False")
else()
  message(STATUS "CHOLMOD found " ${CHOLMOD_INCLUDE_DIR} " " ${CHOLMOD_LIBRARY})
  file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/CHOLMOD" "ok=True")
  #set(CHOLMOD_LINK_PATH ${CHOLMOD_LIBRARY_DIRS} CACHE INTERNAL ""  FORCE)
  set(CHOLMOD_INCLUDE_PATH ${CHOLMOD_INCLUDE_DIR} CACHE INTERNAL "" FORCE)
  set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARY} CACHE INTERNAL "" FORCE)
endif()

else()
message(STATUS "CHOLMOD already setup")

endif(NOT DEFINED CHOLMOD_LIBRARIES)