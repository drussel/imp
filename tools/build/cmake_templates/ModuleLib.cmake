# Autogenerated file, run tools/build/setup_cmake.py to regenerate

message(STATUS "Setting up module " %(name)s)

FILE(GLOB gensources
     "${PROJECT_BINARY_DIR}/src/%(name)s/*.cpp")

FILE(GLOB genheaders
     "${PROJECT_BINARY_DIR}/include/IMP/%(name)s/*.h")

include_directories(%(includepath)s)
link_directories(%(libpath)s)
add_definitions("-DIMP%(CPPNAME)s_EXPORTS")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${%(NAME)s_CXX_FLAGS}")

set(headers %(headers)s)

if(DEFINED IMP_%(NAME)s_LIBRARY_EXTRA_SOURCES)
  set_source_files_properties(${IMP_%(NAME)s_LIBRARY_EXTRA_SOURCES}
                              PROPERTIES GENERATED 1)
endif()

if(DEFINED IMP_%(name)s_IS_PER_CPP)
  set(sources %(sources)s)

  add_library(imp_%(name)s SHARED ${gensources} ${genheaders}
              ${headers} ${sources}
              ${IMP_%(NAME)s_LIBRARY_EXTRA_SOURCES}
              )
else()

  add_library(imp_%(name)s SHARED ${gensources} ${genheaders}
              ${headers} ${PROJECT_BINARY_DIR}/src/%(name)s_all.cpp
              ${IMP_%(NAME)s_LIBRARY_EXTRA_SOURCES}
              )

endif()
INSTALL(TARGETS imp_%(name)s DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(DEFINED IMP_%(NAME)s_LIBRARY_EXTRA_DEPENDENCIES)
  add_dependencies(imp_%(name)s ${IMP_%(NAME)s_LIBRARY_EXTRA_DEPENDENCIES})
endif()

target_link_libraries(imp_%(name)s
                      %(modules)s
                      %(dependencies)s
  )

set(IMP_%(NAME)s_LIBRARY imp_%(name)s CACHE INTERNAL "" FORCE)
