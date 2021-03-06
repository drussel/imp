# Autogenerated file, run tools/build/setup_cmake.py to regenerate

include_directories(${ANN_INCLUDE_PATH}
        ${BOOST.FILESYSTEM_INCLUDE_PATH}
        ${BOOST.PROGRAMOPTIONS_INCLUDE_PATH}
        ${BOOST.RANDOM_INCLUDE_PATH}
        ${BOOST.REGEX_INCLUDE_PATH}
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
        ${BOOST.REGEX_LINK_PATH}
        ${BOOST.SYSTEM_LINK_PATH}
        ${CGAL_LINK_PATH}
        ${GPERFTOOLS_LINK_PATH}
        ${LOG4CXX_LINK_PATH}
        ${TCMALLOC_HEAPCHECKER_LINK_PATH}
        ${TCMALLOC_HEAPPROFILER_LINK_PATH})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EM_CXX_FLAGS}")

math(EXPR timeout "${IMP_TIMEOUT_FACTOR} * 300")

if (NOT ${IMP_MAX_CHECKS} MATCHES "NONE")
set(testarg "--run_quick_test")
else()
set(testarg "")
endif()

set(cppbenchmarks ${PROJECT_SOURCE_DIR}/modules/em/benchmark/benchmark_density.cpp)

foreach (bin ${cppbenchmarks})
   GET_FILENAME_COMPONENT(name ${bin} NAME_WE)
   add_executable(em.${name} ${bin})
   target_link_libraries(em.${name}     imp_em
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
    ${IMP_BENCHMARK_LIBRARY}
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
   set_target_properties(em.${name} PROPERTIES
                         RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/benchmark/em"
                         OUTPUT_NAME ${name})
   add_test(em.${name} ${IMP_TEST_SETUP}
            "${PROJECT_BINARY_DIR}/benchmark/em/${name}${CMAKE_EXECUTABLE_SUFFIX}" ${testarg})
   set_tests_properties("em.${name}" PROPERTIES LABELS "IMP.em;benchmark")
   set_tests_properties("em.${name}" PROPERTIES TIMEOUT ${timeout})
   set_tests_properties("em.${name}" PROPERTIES COST 1)
   set(executables ${executables} em.${name})
endforeach(bin)

add_custom_target("imp_em_benchmarks" ALL DEPENDS ${executables}
  # add dummy dep as empty targets seem to go away
  imp_em imp_base)

set(pybenchmarks )
foreach (test ${pybenchmarks})
 GET_FILENAME_COMPONENT(name ${test} NAME_WE)
 add_test("em.${name}" ${IMP_TEST_SETUP} python ${test} ${testarg})
 set_tests_properties("em.${name}" PROPERTIES LABELS "IMP.em;benchmark")
 set_tests_properties("em.${name}" PROPERTIES TIMEOUT ${timeout})
 set_tests_properties("em.${name}" PROPERTIES COST 4)
endforeach(test)
