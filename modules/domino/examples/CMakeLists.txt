# Autogenerated file, run tools/build/setup_cmake.py to regenerate

include_directories(${ANN_INCLUDE_PATH}
        ${BOOST.FILESYSTEM_INCLUDE_PATH}
        ${BOOST.GRAPH_INCLUDE_PATH}
        ${BOOST.PROGRAMOPTIONS_INCLUDE_PATH}
        ${BOOST.RANDOM_INCLUDE_PATH}
        ${BOOST.REGEX_INCLUDE_PATH}
        ${BOOST.SYSTEM_INCLUDE_PATH}
        ${CGAL_INCLUDE_PATH}
        ${GPERFTOOLS_INCLUDE_PATH}
        ${HDF5_INCLUDE_PATH}
        ${LOG4CXX_INCLUDE_PATH}
        ${RMF_INCLUDE_PATH}
        ${TCMALLOC_HEAPCHECKER_INCLUDE_PATH}
        ${TCMALLOC_HEAPPROFILER_INCLUDE_PATH})
link_directories(${ANN_LINK_PATH}
        ${BOOST.FILESYSTEM_LINK_PATH}
        ${BOOST.GRAPH_LINK_PATH}
        ${BOOST.PROGRAMOPTIONS_LINK_PATH}
        ${BOOST.RANDOM_LINK_PATH}
        ${BOOST.REGEX_LINK_PATH}
        ${BOOST.SYSTEM_LINK_PATH}
        ${CGAL_LINK_PATH}
        ${GPERFTOOLS_LINK_PATH}
        ${HDF5_LINK_PATH}
        ${LOG4CXX_LINK_PATH}
        ${RMF_LINK_PATH}
        ${TCMALLOC_HEAPCHECKER_LINK_PATH}
        ${TCMALLOC_HEAPPROFILER_LINK_PATH})

set(pytests ${PROJECT_SOURCE_DIR}/modules/domino/examples/custom_filter.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/domino_approach.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/interactive.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/interactive_with_containers.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/marina_party.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/merge_tree.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/multiscale.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/restraint_cache.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/rigid_body_excluded_volume.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/save_assignments.py
${PROJECT_SOURCE_DIR}/modules/domino/examples/six_particles_optimization.py)

math(EXPR timeout "${IMP_TIMEOUT_FACTOR} * 180")

if (${IMP_MAX_CHECKS} MATCHES "INTERNAL")
set(testarg "--run_quick_test")
else()
set(testarg "")
endif()

foreach (test ${pytests})
 GET_FILENAME_COMPONENT(name ${test} NAME)
 add_test("domino.${name}" ${IMP_TEST_SETUP} python ${test} ${testarg})
 set_tests_properties("domino.${name}" PROPERTIES LABELS "IMP.domino;example")
 set_tests_properties("domino.${name}" PROPERTIES TIMEOUT ${timeout})
 set_tests_properties("domino.${name}" PROPERTIES COST 3)
  #add_dependencies(${name} RMFPython)
endforeach(test)

set(cpp_tests )

foreach (test ${cpp_tests})
   GET_FILENAME_COMPONENT(name ${test} NAME)
   GET_FILENAME_COMPONENT(name_we ${test} NAME_WE)
   add_executable("domino.${name_we}" ${test})
   target_link_libraries("domino.${name_we}"
    imp_domino
    ${IMP_BASE_LIBRARY}
${IMP_CGAL_LIBRARY}
${IMP_ALGEBRA_LIBRARY}
${IMP_KERNEL_LIBRARY}
${IMP_DISPLAY_LIBRARY}
${IMP_SCORE_FUNCTOR_LIBRARY}
${IMP_STATISTICS_LIBRARY}
${IMP_CORE_LIBRARY}
${IMP_CONTAINER_LIBRARY}
${IMP_ATOM_LIBRARY}
${IMP_RMF_LIBRARY}
    ${BOOST.GRAPH_LIBRARIES}
${RMF_LIBRARIES}
${HDF5_LIBRARIES}
${BOOST.FILESYSTEM_LIBRARIES}
${BOOST.PROGRAMOPTIONS_LIBRARIES}
${BOOST.SYSTEM_LIBRARIES}
${GPERFTOOLS_LIBRARIES}
${TCMALLOC_HEAPPROFILER_LIBRARIES}
${TCMALLOC_HEAPCHECKER_LIBRARIES}
${LOG4CXX_LIBRARIES}
${BOOST.RANDOM_LIBRARIES}
${CGAL_LIBRARIES}
${ANN_LIBRARIES}
${BOOST.REGEX_LIBRARIES})
   set_target_properties("domino.${name_we}" PROPERTIES
                         RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/test/domino/"
                         OUTPUT_NAME "${name_we}")
   add_test("domino.${name}" ${IMP_TEST_SETUP}
            "${PROJECT_BINARY_DIR}/test/domino/${name_we}${CMAKE_EXECUTABLE_SUFFIX}" ${testarg})
   set_tests_properties("domino.${name}" PROPERTIES LABELS "IMP.domino;example")
   set_tests_properties("domino.${name}" PROPERTIES TIMEOUT ${timeout})
   set_tests_properties("domino.${name}" PROPERTIES COST 3)
   set(executables ${executables} "domino.${name_we}")
endforeach(test)

add_custom_target("imp_domino_examples" ALL DEPENDS ${executables})
