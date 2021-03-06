# Autogenerated file, run tools/build/setup_cmake.py to regenerate

if(NOT DEFINED ExampleDependency_LIBRARIES)

set(CHECK_COMPILES_BODY "")

check_compiles("_environment" ExampleDependency EXAMPLEDEPENDENCY "#include <example_dependency_header.hh>" "" "example_dependency" ExampleDependency_ok)
if("${ExampleDependency_ok}" MATCHES "1")
message(STATUS "Found ExampleDependency in environment")
else()
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules("ExampleDependency_PKGCONF" "exampledependency")

# Include dir
find_path("ExampleDependency_INCLUDE_DIR"
  NAMES example_dependency_header.hh
  PATHS ${ExampleDependency_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
foreach(lib example_dependency)
find_library("${lib}_LIBRARY"
  NAMES ${lib}
  PATHS ${ExampleDependency_PKGCONF_LIBRARY_DIRS}
)
set("ExampleDependency_LIBRARY" ${ExampleDependency_LIBRARY} ${${lib}_LIBRARY})
endforeach(lib)


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(ExampleDependency_PROCESS_INCLUDES ExampleDependency_INCLUDE_DIR)
set(ExampleDependency_PROCESS_LIBS ExampleDependency_LIBRARY)
libfind_process(ExampleDependency)

if ("${ExampleDependency_LIBRARY}" MATCHES ".*NOTFOUND.*"
    OR "${ExampleDependency_INCLUDE_DIR}" MATCHES ".*NOTFOUND.*")
  message("ExampleDependency not found")
file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/ExampleDependency" "ok=False")
else()
  check_compiles("_found" "ExampleDependency" "EXAMPLEDEPENDENCY" "#include <example_dependency_header.hh>" "${ExampleDependency_INCLUDE_DIR}" "${ExampleDependency_LIBRARY}" ExampleDependency_ok_ok)
  if(${ExampleDependency_ok_ok} MATCHES "1")
    message(STATUS "Found ExampleDependency at ""${ExampleDependency_INCLUDE_DIR}" " " "${ExampleDependency_LIBRARY}")
  else()
    message("ExampleDependency not found")
file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/ExampleDependency" "ok=False")
  endif()
endif()
endif()
else()
message(STATUS "ExampleDependency already setup")

endif(NOT DEFINED ExampleDependency_LIBRARIES)