# Autogenerated file, run tools/build/setup_cmake.py to regenerate

if(NOT DEFINED ANN_LIBRARIES)
message(STATUS "Checking for ANN")

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(ANN_PKGCONF ANN)

# Include dir
find_path(ANN_INCLUDE_DIR
  NAMES ANN/ANN.h
  PATHS ${ANN_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
foreach(lib ANN)
find_library(${lib}_LIBRARY
  NAMES ${lib}
  PATHS ${ANN_PKGCONF_LIBRARY_DIRS}
)
set(ANN_LIBRARY ${ANN_LIBRARY} ${${lib}_LIBRARY})
endforeach(lib)


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(ANN_PROCESS_INCLUDES ANN_INCLUDE_DIR)
set(ANN_PROCESS_LIBS ANN_LIBRARY)
libfind_process(ANN)

if ("${ANN_LIBRARY}" MATCHES ".*NOTFOUND.*"
    OR "${ANN_INCLUDE_DIR}" MATCHES ".*NOTFOUND.*")
  message(STATUS "ANN not found")
  file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/ANN" "ok=False")
else()
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES "${ANN_LIBRARY}")
  set(CMAKE_REQUIRED_INCLUDES "${ANN_INCLUDE_DIR}")
  set(body "#include <ANN/ANN.h>
int main(int,char*[]) {
  
  return 0;
}")
  check_cxx_source_compiles("${body}"
 ANN_COMPILES)
  if ("ANN_COMPILES" MATCHES "1")
    message(STATUS "ANN found " ${ANN_INCLUDE_DIR} " " ${ANN_LIBRARY})
    file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/ANN" "ok=True")
    #set(ANN_LINK_PATH ${ANN_LIBRARY_DIRS} CACHE INTERNAL ""  FORCE)
    set(ANN_INCLUDE_PATH ${ANN_INCLUDE_DIR} CACHE INTERNAL "" FORCE)
    set(ANN_LIBRARIES ${ANN_LIBRARY} CACHE INTERNAL "" FORCE)
  else()
    file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/ANN" "ok=False")
  endif()
endif()

else()
message(STATUS "ANN already setup")

endif(NOT DEFINED ANN_LIBRARIES)
