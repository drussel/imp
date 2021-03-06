# Autogenerated file, run tools/build/setup_cmake.py to regenerate

imp_get_process_exit_code("setup_application %(name)s" status ${PROJECT_BINARY_DIR}
                          COMMAND ${PROJECT_SOURCE_DIR}/tools/build/setup_application.py
                          --name=%(name)s
                          --datapath=${IMP_DATAPATH}
                           --source=${PROJECT_SOURCE_DIR})
if(${status} EQUAL 0)
message("Application IMP.%(name)s ok")
include_directories(%(includepath)s)
link_directories(%(libpath)s)

if(${IMP_SPLIT_PYTHON_TESTS})
imp_execute_process("get_python_tests %(name)s" ${PROJECT_BINARY_DIR}
                    COMMAND ${PROJECT_SOURCE_DIR}/tools/build/get_python_tests.py
                          --application=%(name)s
                          ${PROJECT_SOURCE_DIR})
endif()

%(bins)s

set(pybins %(pybins)s)
foreach (pybin ${pybins})
  install(PROGRAMS ${pybin} DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach(pybin)

set(pytests %(pytests)s)
foreach (test ${pytests})
  GET_FILENAME_COMPONENT(name ${test} NAME_WE)
  if(EXISTS "${PROJECT_BINARY_DIR}/test/%(name)s/${name}.pytests")
    FILE(READ "${PROJECT_BINARY_DIR}/test/%(name)s/${name}.pytests" contents)
    STRING(REGEX REPLACE ";" "\\\\;" contents "${contents}")
    STRING(REGEX REPLACE "\n" ";" contents "${contents}")
    foreach(testline ${contents})
      string(REGEX REPLACE "([A-Za-z0-9_]+\\.[A-Za-z0-9_]+) (.*)" 
                           "\\1;\\2" split "${testline}")
      list(GET split 0 methname)
      list(GET split 1 docstring)
      add_test("%(name)s.${name}.${methname}" ${IMP_TEST_SETUP} python ${test} "${methname}")
      set_tests_properties("%(name)s.${name}.${methname}" PROPERTIES LABELS "IMP.%(name)s;test")
      set_tests_properties("%(name)s.${name}.${methname}" PROPERTIES MEASUREMENT "docstring=${docstring}")
    endforeach()
  else()
    add_test("%(name)s.${name}" ${IMP_TEST_SETUP} python ${test})
    set_tests_properties("%(name)s.${name}" PROPERTIES LABELS "IMP.%(name)s;test")
  endif()
endforeach(test)


elseif(${status} EQUAL 1)
message("Application %(name)s disabled")
else()
message(FATAL_ERROR "setup_application failed ${status}")
endif()
