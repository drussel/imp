# Autogenerated file, run tools/build/setup_cmake.py to regenerate

if(NOT DEFINED TCMalloc_HeapChecker_LIBRARIES)
message(STATUS "Checking for TCMalloc_HeapChecker")

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(TCMalloc_HeapChecker_PKGCONF tcmalloc_heapchecker)

# Include dir
find_path(TCMalloc_HeapChecker_INCLUDE_DIR
  NAMES gperftools/heap-checker.h
  PATHS ${TCMalloc_HeapChecker_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
foreach(lib )
find_library(${lib}_LIBRARY
  NAMES ${lib}
  PATHS ${TCMalloc_HeapChecker_PKGCONF_LIBRARY_DIRS}
)
set(TCMalloc_HeapChecker_LIBRARY ${TCMalloc_HeapChecker_LIBRARY} ${${lib}_LIBRARY})
endforeach(lib)


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(TCMalloc_HeapChecker_PROCESS_INCLUDES TCMalloc_HeapChecker_INCLUDE_DIR)
set(TCMalloc_HeapChecker_PROCESS_LIBS TCMalloc_HeapChecker_LIBRARY)
libfind_process(TCMalloc_HeapChecker)

if ("${TCMalloc_HeapChecker_LIBRARY}" MATCHES ".*NOTFOUND.*"
    OR "${TCMalloc_HeapChecker_INCLUDE_DIR}" MATCHES ".*NOTFOUND.*")
  message(STATUS "TCMalloc_HeapChecker not found")
  file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/TCMalloc_HeapChecker" "ok=False")
else()
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES "${TCMalloc_HeapChecker_LIBRARY}")
  set(CMAKE_REQUIRED_INCLUDES "${TCMalloc_HeapChecker_INCLUDE_DIR}")
  set(body "#include <gperftools/heap-checker.h>
int main(int,char*[]) {
  new HeapLeakChecker("profiler");
  return 0;
}")
  check_cxx_source_compiles("${body}"
 TCMalloc_HeapChecker_COMPILES)
  if ("TCMalloc_HeapChecker_COMPILES" MATCHES "1")
    message(STATUS "TCMalloc_HeapChecker found " ${TCMalloc_HeapChecker_INCLUDE_DIR} " " ${TCMalloc_HeapChecker_LIBRARY})
    #set(TCMALLOC_HEAPCHECKER_LINK_PATH ${TCMalloc_HeapChecker_LIBRARY_DIRS} CACHE INTERNAL ""  FORCE)
    set(TCMALLOC_HEAPCHECKER_INCLUDE_PATH ${TCMalloc_HeapChecker_INCLUDE_DIR} CACHE INTERNAL "" FORCE)
    set(TCMALLOC_HEAPCHECKER_LIBRARIES ${TCMalloc_HeapChecker_LIBRARY} CACHE INTERNAL "" FORCE)
    file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/TCMalloc_HeapChecker" "ok=True
includepath=\"${TCMalloc_HeapChecker_INCLUDE_PATH}\"
swigpath=\"${TCMalloc_HeapChecker_SWIG_PATH}\"
libpath=\"${TCMalloc_HeapChecker_LIB_PATH}\"
")
  else()
    file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/TCMalloc_HeapChecker" "ok=False")
  endif()
endif()

else()
message(STATUS "TCMalloc_HeapChecker already setup")

endif(NOT DEFINED TCMalloc_HeapChecker_LIBRARIES)
