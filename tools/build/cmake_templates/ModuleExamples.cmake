# Autogenerated file, run tools/build/setup_cmake.py to regenerate

include_directories(%(includepath)s)
link_directories(%(libpath)s)

set(pytests %(pyexamples)s)

foreach (test ${pytests})
 GET_FILENAME_COMPONENT(name ${test} NAME_WE)
 add_test("%(name)s.${name}" ${IMP_TEST_SETUP} python ${test})
 set_tests_properties("%(name)s.${name}" PROPERTIES LABELS "IMP.%(name)s;example")
  #add_dependencies(${name} RMFPython)
endforeach(test)

set(cpp_tests %(cppexamples)s)

foreach (test ${cpp_tests})
   GET_FILENAME_COMPONENT(name ${test} NAME_WE)
   add_executable("%(name)s.${name}" ${test})
   target_link_libraries("%(name)s.${name}"     imp_%(name)s
    %(modules)s
    %(dependencies)s)
   set_target_properties("%(name)s.${name}" PROPERTIES
                         RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/test/%(name)s/"
                         OUTPUT_NAME "${name}")
   add_test("%(name)s.${name}" ${IMP_TEST_SETUP} "${PROJECT_BINARY_DIR}/test/%(name)s/${name}${CMAKE_EXECUTABLE_SUFFIX}")
   set_tests_properties("%(name)s.${name}" PROPERTIES LABELS "IMP.%(name)s;example")
   set(executables ${executables} "%(name)s.${name}")
endforeach(test)

add_custom_target("imp_%(name)s_examples" ALL DEPENDS ${executables})
