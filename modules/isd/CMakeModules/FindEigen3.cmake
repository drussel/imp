# Autogenerated file, run tools/build/setup_cmake.py to regenerate

if(NOT DEFINED Eigen3_LIBRARIES)
message(STATUS "Checking for Eigen3")

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Eigen3_PKGCONF eigen3)

# Include dir
find_path(Eigen3_INCLUDE_DIR
  NAMES Eigen/Core
  PATHS ${Eigen3_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
foreach(lib )
find_library(${lib}_LIBRARY
  NAMES ${lib}
  PATHS ${Eigen3_PKGCONF_LIBRARY_DIRS}
)
set(Eigen3_LIBRARY ${Eigen3_LIBRARY} ${${lib}_LIBRARY})
endforeach(lib)


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Eigen3_PROCESS_INCLUDES Eigen3_INCLUDE_DIR)
set(Eigen3_PROCESS_LIBS Eigen3_LIBRARY)
libfind_process(Eigen3)

if ("${Eigen3_LIBRARY}" MATCHES ".*NOTFOUND.*"
    OR "${Eigen3_INCLUDE_DIR}" MATCHES ".*NOTFOUND.*")
  message(STATUS "Eigen3 not found")
  file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/Eigen3" "ok=False")
else()
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES "${Eigen3_LIBRARY}")
  set(CMAKE_REQUIRED_INCLUDES "${Eigen3_INCLUDE_DIR}")
  set(body "#include <Eigen/Core>
int main(int,char*[]) {
  
  return 0;
}")
  check_cxx_source_compiles("${body}"
 Eigen3_COMPILES)
  if ("Eigen3_COMPILES" MATCHES "1")
    message(STATUS "Eigen3 found " ${Eigen3_INCLUDE_DIR} " " ${Eigen3_LIBRARY})
    file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/Eigen3" "ok=True")
    #set(EIGEN3_LINK_PATH ${Eigen3_LIBRARY_DIRS} CACHE INTERNAL ""  FORCE)
    set(EIGEN3_INCLUDE_PATH ${Eigen3_INCLUDE_DIR} CACHE INTERNAL "" FORCE)
    set(EIGEN3_LIBRARIES ${Eigen3_LIBRARY} CACHE INTERNAL "" FORCE)
  else()
    file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/Eigen3" "ok=False")
  endif()
endif()

else()
message(STATUS "Eigen3 already setup")

endif(NOT DEFINED Eigen3_LIBRARIES)
