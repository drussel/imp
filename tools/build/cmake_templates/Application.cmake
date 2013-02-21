# Autogenerated file, run tools/build/setup_cmake.py to regenerate

execute_process(COMMAND ${PROJECT_SOURCE_DIR}/tools/build/setup_application.py
                          --name=%(name)s
                          --datapath=${IMP_DATAPATH}
                           --source=${PROJECT_SOURCE_DIR}
                           RESULT_VARIABLE status
                           WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
if(NOT ${status})
message(STATUS "Application %(name)s ok")

include_directories(%(includepath)s)
link_directories(%(libpath)s)

%(bins)s

set(pybins %(pybins)s)
foreach (pybin ${pybins})
  install(PROGRAMS ${pybin} DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach(pybin)

set(pytests %(pytests)s)
foreach (test ${pytests})
  GET_FILENAME_COMPONENT(name ${test} NAME_WE)
  add_test("${name}_%(name)s" ${PROJECT_BINARY_DIR}/imppy.sh "python" ${test})
  set_tests_properties("${name}_%(name)s" PROPERTIES LABELS %(name)s)
endforeach(test)


else()
message(STATUS "Application %(name)s disabled")
endif()
