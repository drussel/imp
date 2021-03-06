# Autogenerated file, run tools/build/setup_cmake.py to regenerate

if(NOT DEFINED libTau_LIBRARIES)

set(CHECK_COMPILES_BODY "")

check_compiles("_environment" libTau LIBTAU "#include <libTAU/PairwiseDockingEngine.h>" "" "TAU" libTau_ok)
if("${libTau_ok}" MATCHES "1")
message(STATUS "Found libTau in environment")
else()
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules("libTau_PKGCONF" "libtau")

# Include dir
find_path("libTau_INCLUDE_DIR"
  NAMES libTAU/PairwiseDockingEngine.h
  PATHS ${libTau_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
foreach(lib TAU)
find_library("${lib}_LIBRARY"
  NAMES ${lib}
  PATHS ${libTau_PKGCONF_LIBRARY_DIRS}
)
set("libTau_LIBRARY" ${libTau_LIBRARY} ${${lib}_LIBRARY})
endforeach(lib)


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(libTau_PROCESS_INCLUDES libTau_INCLUDE_DIR)
set(libTau_PROCESS_LIBS libTau_LIBRARY)
libfind_process(libTau)

if ("${libTau_LIBRARY}" MATCHES ".*NOTFOUND.*"
    OR "${libTau_INCLUDE_DIR}" MATCHES ".*NOTFOUND.*")
  message("libTau not found")
file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/libTau" "ok=False")
else()
  check_compiles("_found" "libTau" "LIBTAU" "#include <libTAU/PairwiseDockingEngine.h>" "${libTau_INCLUDE_DIR}" "${libTau_LIBRARY}" libTau_ok_ok)
  if(${libTau_ok_ok} MATCHES "1")
    message(STATUS "Found libTau at ""${libTau_INCLUDE_DIR}" " " "${libTau_LIBRARY}")
  else()
    message("libTau not found")
file(WRITE "${PROJECT_BINARY_DIR}/data/build_info/libTau" "ok=False")
  endif()
endif()
endif()
else()
message(STATUS "libTau already setup")

endif(NOT DEFINED libTau_LIBRARIES)