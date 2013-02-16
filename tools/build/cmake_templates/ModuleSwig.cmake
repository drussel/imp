# Autogenerated file, run build/tools/setup_cmake.py to regenerate

if (${IMP_USE_CUSTOM_CXX_FLAGS})
set(CMAKE_CXX_FLAGS ${IMP_PYTHON_CXX_FLAGS})
endif()

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

execute_process(COMMAND ${PROJECT_SOURCE_DIR}/tools/build/make_swig_deps.py
                    "--swig=${SWIG_EXECUTABLE}"
                    "--includepath=${include_path}"
                    "--swigpath=${swig_path}"
                    "--name=%(name)s"
                    RESULT_VARIABLE swigdeps
                    WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
if( ${swigdeps})
message(FATAL_ERROR " Failed to run swig dependency computation")
endif()

file(STRINGS "${PROJECT_BINARY_DIR}/src/%(name)s_swig.deps" SWIGDEPS)

set(source ${PROJECT_BINARY_DIR}/src/%(name)s_swig/wrap.cpp
                          ${PROJECT_BINARY_DIR}/src/%(name)s_swig/wrap.h)

add_custom_command(OUTPUT ${source}
                          "${PROJECT_BINARY_DIR}/lib/IMP/%(name)s/__init__.py"
   COMMAND "${PROJECT_SOURCE_DIR}/tools/build/make_swig_wrapper.py"
            "--swig=${SWIG_EXECUTABLE}"
            "--swigpath=${swig_path}"
            "--includepath=${include_path}"
            "--module=%(name)s"
   DEPENDS ${SWIGDEPS}
   WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
   COMMENT "Running swig on %(name)s")

add_custom_target("%(name)s_wrapper" ALL DEPENDS ${source}
)

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