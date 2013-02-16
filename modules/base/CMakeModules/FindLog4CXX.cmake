# Autogenerated file, run build/tools/setup_cmake.py to regenerate

if(NOT DEFINED Log4CXX_LIBRARIES)
message(STATUS "Checking for Log4CXX")

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Log4CXX_PKGCONF Log4CXX)

# Include dir
find_path(Log4CXX_INCLUDE_DIR
  NAMES log4cxx/ndc.h
  PATHS ${Log4CXX_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Log4CXX_LIBRARY
  NAMES log4cxx
  PATHS ${Log4CXX_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Log4CXX_PROCESS_INCLUDES Log4CXX_INCLUDE_DIR)
set(Log4CXX_PROCESS_LIBS Log4CXX_LIBRARY)
libfind_process(Log4CXX)

if (${Log4CXX_LIBRARY} MATCHES "Log4CXX_LIBRARY-NOTFOUND"
    OR ${Log4CXX_INCLUDE_DIR} MATCHES "Log4CXX_INCLUDE_DIR-NOTFOUND")
  message(STATUS "Log4CXX not found")
  file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/Log4CXX" "ok=False")
else()
  message(STATUS "Log4CXX found " ${Log4CXX_INCLUDE_DIR} " " ${Log4CXX_LIBRARY})
  file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/Log4CXX" "ok=True")
  #set(LOG4CXX_LINK_PATH ${Log4CXX_LIBRARY_DIRS} CACHE INTERNAL ""  FORCE)
  set(LOG4CXX_INCLUDE_PATH ${Log4CXX_INCLUDE_DIR} CACHE INTERNAL "" FORCE)
  set(LOG4CXX_LIBRARIES ${Log4CXX_LIBRARY} CACHE INTERNAL "" FORCE)
endif()

else()
message(STATUS "Log4CXX already setup")

endif(NOT DEFINED Log4CXX_LIBRARIES)