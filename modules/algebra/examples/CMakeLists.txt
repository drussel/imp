# Autogenerated file, run tools/build/setup_cmake.py to regenerate

include_directories(${ANN_INCLUDE_PATH}
        ${BOOST.FILESYSTEM_INCLUDE_PATH}
        ${BOOST.PROGRAMOPTIONS_INCLUDE_PATH}
        ${BOOST.RANDOM_INCLUDE_PATH}
        ${BOOST.SYSTEM_INCLUDE_PATH}
        ${CGAL_INCLUDE_PATH}
        ${GPERFTOOLS_INCLUDE_PATH}
        ${LOG4CXX_INCLUDE_PATH}
        ${TCMALLOC_HEAPCHECKER_INCLUDE_PATH}
        ${TCMALLOC_HEAPPROFILER_INCLUDE_PATH})
link_directories(${ANN_LINK_PATH}
        ${BOOST.FILESYSTEM_LINK_PATH}
        ${BOOST.PROGRAMOPTIONS_LINK_PATH}
        ${BOOST.RANDOM_LINK_PATH}
        ${BOOST.SYSTEM_LINK_PATH}
        ${CGAL_LINK_PATH}
        ${GPERFTOOLS_LINK_PATH}
        ${LOG4CXX_LINK_PATH}
        ${TCMALLOC_HEAPCHECKER_LINK_PATH}
        ${TCMALLOC_HEAPPROFILER_LINK_PATH})

set(pytests ${PROJECT_SOURCE_DIR}/modules/algebra/examples/geometry.py
${PROJECT_SOURCE_DIR}/modules/algebra/examples/grid_space.py)

math(EXPR timeout "${IMP_TIMEOUT_FACTOR} * 180")

if (${IMP_MAX_CHECKS} MATCHES "INTERNAL")
set(testarg "--run_quick_test")
else()
set(testarg "")
endif()

foreach (test ${pytests})
 GET_FILENAME_COMPONENT(name ${test} NAME)
 add_test("algebra.${name}" ${IMP_TEST_SETUP} python ${test} ${testarg})
 set_tests_properties("algebra.${name}" PROPERTIES LABELS "IMP.algebra;example")
 set_tests_properties("algebra.${name}" PROPERTIES TIMEOUT ${timeout})
 set_tests_properties("algebra.${name}" PROPERTIES COST 3)
  #add_dependencies(${name} RMFPython)
endforeach(test)

set(cpp_tests )

foreach (test ${cpp_tests})
   GET_FILENAME_COMPONENT(name ${test} NAME)
   GET_FILENAME_COMPONENT(name_we ${test} NAME_WE)
   add_executable("algebra.${name_we}" ${test})
   target_link_libraries("algebra.${name_we}"
    imp_algebra
    ${IMP_BASE_LIBRARY}
${IMP_CGAL_LIBRARY}
    ${ANN_LIBRARIES}
${BOOST.FILESYSTEM_LIBRARIES}
${BOOST.PROGRAMOPTIONS_LIBRARIES}
${BOOST.SYSTEM_LIBRARIES}
${GPERFTOOLS_LIBRARIES}
${TCMALLOC_HEAPPROFILER_LIBRARIES}
${TCMALLOC_HEAPCHECKER_LIBRARIES}
${LOG4CXX_LIBRARIES}
${BOOST.RANDOM_LIBRARIES}
${CGAL_LIBRARIES})
   set_target_properties("algebra.${name_we}" PROPERTIES
                         RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/test/algebra/"
                         OUTPUT_NAME "${name_we}")
   add_test("algebra.${name}" ${IMP_TEST_SETUP}
            "${PROJECT_BINARY_DIR}/test/algebra/${name_we}${CMAKE_EXECUTABLE_SUFFIX}" ${testarg})
   set_tests_properties("algebra.${name}" PROPERTIES LABELS "IMP.algebra;example")
   set_tests_properties("algebra.${name}" PROPERTIES TIMEOUT ${timeout})
   set_tests_properties("algebra.${name}" PROPERTIES COST 3)
   set(executables ${executables} "algebra.${name_we}")
endforeach(test)

add_custom_target("imp_algebra_examples" ALL DEPENDS ${executables})
