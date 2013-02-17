# Autogenerated file, run build/tools/setup_cmake.py to regenerate

if (${IMP_USE_CUSTOM_CXX_FLAGS})
set(CMAKE_CXX_FLAGS ${IMP_BIN_CXX_FLAGS})
endif()

include_directories(%(includepath)s)
link_directories(%(libpath)s)

File(GLOB runtimepytests "${PROJECT_BINARY_DIR}/test/%(name)s/test_*.py")
set(pytests %(pytests)s %(expytests)s)

foreach (test ${runtimepyttests} ${pytests})
   GET_FILENAME_COMPONENT(name ${test} NAME_WE)
  add_test(${name} ${PROJECT_BINARY_DIR}/imppy.sh "python" ${test})
  set_tests_properties(${name} PROPERTIES LABELS %(name)s)
  #add_dependencies(${name} RMFPython)
endforeach(test)

set(cpp_tests %(cpptests)s %(excpptests)s)

foreach (test ${cpp_tests})
   GET_FILENAME_COMPONENT(name ${test} NAME_WE)
   add_executable("${name}" ${test})
   target_link_libraries(${name}     imp_%(name)s
    %(modules)s
    %(dependencies)s)
   set_target_properties(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/test/%(name)s/")
   add_test(${name} ${PROJECT_BINARY_DIR}/imppy.sh "${PROJECT_BINARY_DIR}/test/%(name)s/${name}")
   set_tests_properties(${name} PROPERTIES LABELS %(name)s)
endforeach(test)