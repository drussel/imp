# Autogenerated file, run tools/build/setup_cmake.py to regenerate

imp_get_process_exit_code("setup_application multifit_tools" status ${PROJECT_BINARY_DIR}
                          COMMAND ${PROJECT_SOURCE_DIR}/tools/build/setup_application.py
                          --name=multifit_tools
                          --datapath=${IMP_DATAPATH}
                           --source=${PROJECT_SOURCE_DIR})
if(${status} EQUAL 0)
message("Application IMP.multifit_tools ok")
include_directories(${ANN_INCLUDE_PATH}
        ${BOOST.FILESYSTEM_INCLUDE_PATH}
        ${BOOST.GRAPH_INCLUDE_PATH}
        ${BOOST.PROGRAMOPTIONS_INCLUDE_PATH}
        ${BOOST.RANDOM_INCLUDE_PATH}
        ${BOOST.REGEX_INCLUDE_PATH}
        ${BOOST.SYSTEM_INCLUDE_PATH}
        ${CGAL_INCLUDE_PATH}
        ${FFTW3_INCLUDE_PATH}
        ${GPERFTOOLS_INCLUDE_PATH}
        ${HDF5_INCLUDE_PATH}
        ${LOG4CXX_INCLUDE_PATH}
        ${RMF_INCLUDE_PATH}
        ${TCMALLOC_HEAPCHECKER_INCLUDE_PATH}
        ${TCMALLOC_HEAPPROFILER_INCLUDE_PATH} )
link_directories(${ANN_LINK_PATH}
        ${BOOST.FILESYSTEM_LINK_PATH}
        ${BOOST.GRAPH_LINK_PATH}
        ${BOOST.PROGRAMOPTIONS_LINK_PATH}
        ${BOOST.RANDOM_LINK_PATH}
        ${BOOST.REGEX_LINK_PATH}
        ${BOOST.SYSTEM_LINK_PATH}
        ${CGAL_LINK_PATH}
        ${FFTW3_LINK_PATH}
        ${GPERFTOOLS_LINK_PATH}
        ${HDF5_LINK_PATH}
        ${LOG4CXX_LINK_PATH}
        ${RMF_LINK_PATH}
        ${TCMALLOC_HEAPCHECKER_LINK_PATH}
        ${TCMALLOC_HEAPPROFILER_LINK_PATH})

if(${IMP_SPLIT_PYTHON_TESTS})
imp_execute_process("get_python_tests multifit_tools" ${PROJECT_BINARY_DIR}
                    COMMAND ${PROJECT_SOURCE_DIR}/tools/build/get_python_tests.py
                          --application=multifit_tools
                          ${PROJECT_SOURCE_DIR})
endif()


add_custom_target("IMP.multifit_tools" ALL DEPENDS ${bins})


set(pybins ${PROJECT_SOURCE_DIR}/applications/multifit_tools/complex_to_anchor_graph.py
${PROJECT_SOURCE_DIR}/applications/multifit_tools/dependencies.py
${PROJECT_SOURCE_DIR}/applications/multifit_tools/multifit.py)
foreach (pybin ${pybins})
  install(PROGRAMS ${pybin} DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach(pybin)

set(pytests ${PROJECT_SOURCE_DIR}/applications/multifit_tools/test/test_complex.py
${PROJECT_SOURCE_DIR}/applications/multifit_tools/test/expensive_test_3sfd_example.py)
foreach (test ${pytests})
  GET_FILENAME_COMPONENT(name ${test} NAME_WE)
  if(EXISTS "${PROJECT_BINARY_DIR}/test/multifit_tools/${name}.pytests")
    FILE(READ "${PROJECT_BINARY_DIR}/test/multifit_tools/${name}.pytests" contents)
    STRING(REGEX REPLACE ";" "\\\\;" contents "${contents}")
    STRING(REGEX REPLACE "\n" ";" contents "${contents}")
    foreach(testline ${contents})
      string(REGEX REPLACE "([A-Za-z0-9_]+\\.[A-Za-z0-9_]+) (.*)" 
                           "\\1;\\2" split "${testline}")
      list(GET split 0 methname)
      list(GET split 1 docstring)
      add_test("multifit_tools.${name}.${methname}" ${IMP_TEST_SETUP} python ${test} "${methname}")
      set_tests_properties("multifit_tools.${name}.${methname}" PROPERTIES LABELS "IMP.multifit_tools;test")
      set_tests_properties("multifit_tools.${name}.${methname}" PROPERTIES MEASUREMENT "docstring=${docstring}")
    endforeach()
  else()
    add_test("multifit_tools.${name}" ${IMP_TEST_SETUP} python ${test})
    set_tests_properties("multifit_tools.${name}" PROPERTIES LABELS "IMP.multifit_tools;test")
  endif()
endforeach(test)


elseif(${status} EQUAL 1)
message("Application multifit_tools disabled")
else()
message(FATAL_ERROR "setup_application failed ${status}")
endif()
