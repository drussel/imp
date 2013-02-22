# Autogenerated file, run tools/build/setup_cmake.py to regenerate

include_directories(${IMP_PYTHON_INCLUDE_PATH})
add_definitions(-DIMP_SWIG_WRAPPER)

include_directories(%(includepath)s)
link_directories(%(libpath)s)

GET_DIRECTORY_PROPERTY(includes INCLUDE_DIRECTORIES)
foreach(include ${includes})
set(include_path ${include_path}:${include})
endforeach(include)

set(swig_path ${IMP_SWIG_PATH})

# this is needed for some reason that I don't understand
set(swigpath %(swigpath)s)
foreach(swig ${swigpath})
set(swig_path ${swig}:${swig_path})
endforeach(swig)

file(STRINGS "${PROJECT_BINARY_DIR}/src/%(name)s_swig.deps" swigdeps)

set(source ${PROJECT_BINARY_DIR}/src/%(name)s_swig/wrap.cpp
                          ${PROJECT_BINARY_DIR}/src/%(name)s_swig/wrap.h)

add_custom_command(OUTPUT ${source}
                          "${PROJECT_BINARY_DIR}/lib/IMP/%(name)s/__init__.py"
   COMMAND "${PROJECT_SOURCE_DIR}/tools/build/make_swig_wrapper.py"
            "--swig=${SWIG_EXECUTABLE}"
            "--swigpath=${swig_path}"
            "--includepath=${include_path}"
            "--module=%(name)s"
   DEPENDS ${swigdeps}
   WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
   COMMENT "Running swig on %(name)s")

add_library(_IMP_%(name)s MODULE ${source})

set_target_properties(_IMP_%(name)s PROPERTIES PREFIX "")

if(WIN32 AND NOT CYGWIN)
  set_target_properties(_IMP_%(name)s PROPERTIES SUFFIX ".pyd")
endif()

target_link_libraries(_IMP_%(name)s
    imp_%(name)s
    %(modules)s
    %(dependencies)s
    ${SWIG_PYTHON_LIBRARIES}
  )

add_custom_target("imp_%(name)s_wrapper" ALL DEPENDS ${source} _IMP_%(name)s
)

INSTALL(TARGETS _IMP_%(name)s DESTINATION ${CMAKE_INSTALL_PYTHONDIR})
