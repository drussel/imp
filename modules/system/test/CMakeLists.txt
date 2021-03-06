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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SYSTEM_CXX_FLAGS}")


File(GLOB runtimepytests "${PROJECT_BINARY_DIR}/test/system/test_*.py")

math(EXPR short_timeout "${IMP_TIMEOUT_FACTOR} * 5")
math(EXPR medium_timeout "${IMP_TIMEOUT_FACTOR} * 15")
math(EXPR expensive_timeout "${IMP_TIMEOUT_FACTOR} * 120")

# should make into function
foreach (test ${runtimepytests} )
  GET_FILENAME_COMPONENT(name ${test} NAME)
  if(EXISTS "${PROJECT_BINARY_DIR}/test/system/${name}tests")
    FILE(READ "${PROJECT_BINARY_DIR}/test/system/${name}tests" contents)
    STRING(REGEX REPLACE ";" "\\\\;" contents "${contents}")
    STRING(REGEX REPLACE "\n" ";" contents "${contents}")
    foreach(testline ${contents})
      string(REGEX REPLACE "([A-Za-z0-9_]+\\.[A-Za-z0-9_]+) (.*)"
                           "\\1;\\2" split "${testline}")
      list(GET split 0 methname)
      list(GET split 1 docstring)
      add_test("system.${name}.${methname}" ${IMP_TEST_SETUP} python ${test} "${methname}")
      set_tests_properties("system.${name}.${methname}" PROPERTIES LABELS "IMP.system;test;length_short")
      set_tests_properties("system.${name}.${methname}" PROPERTIES MEASUREMENT "docstring=${docstring}")
      set_tests_properties("system.${name}.${methname}" PROPERTIES TIMEOUT ${short_timeout})
    endforeach()
  else()
    add_test("system.${name}" ${IMP_TEST_SETUP} python ${test})
    set_tests_properties("system.${name}" PROPERTIES LABELS "IMP.system;test;length_short")
    set_tests_properties("system.${name}" PROPERTIES TIMEOUT ${short_timeout})
    set_tests_properties("system.${name}" PROPERTIES COST 1)
  endif()
endforeach(test)

foreach (test )
  GET_FILENAME_COMPONENT(name ${test} NAME)
  if(EXISTS "${PROJECT_BINARY_DIR}/test/system/${name}tests")
    FILE(READ "${PROJECT_BINARY_DIR}/test/system/${name}tests" contents)
    STRING(REGEX REPLACE ";" "\\\\;" contents "${contents}")
    STRING(REGEX REPLACE "\n" ";" contents "${contents}")
    foreach(testline ${contents})
      string(REGEX REPLACE "([A-Za-z0-9_]+\\.[A-Za-z0-9_]+) (.*)"
                           "\\1;\\2" split "${testline}")
      list(GET split 0 methname)
      list(GET split 1 docstring)
      add_test("system.${name}.${methname}" ${IMP_TEST_SETUP} python ${test} "${methname}")
      set_tests_properties("system.${name}.${methname}" PROPERTIES LABELS "IMP.system;test;length_medium")
      set_tests_properties("system.${name}.${methname}" PROPERTIES MEASUREMENT "docstring=${docstring}")
      set_tests_properties("system.${name}.${methname}" PROPERTIES TIMEOUT ${medium_timeout})
    endforeach()
  else()
    add_test("system.${name}" ${IMP_TEST_SETUP} python ${test})
    set_tests_properties("system.${name}" PROPERTIES LABELS "IMP.system;test;length_medium")
    set_tests_properties("system.${name}" PROPERTIES TIMEOUT ${medium_timeout})
    set_tests_properties("system.${name}" PROPERTIES COST 2)
  endif()
endforeach(test)

foreach (test )
  GET_FILENAME_COMPONENT(name ${test} NAME)
  if(EXISTS "${PROJECT_BINARY_DIR}/test/system/${name}tests")
    FILE(READ "${PROJECT_BINARY_DIR}/test/system/${name}tests" contents)
    STRING(REGEX REPLACE ";" "\\\\;" contents "${contents}")
    STRING(REGEX REPLACE "\n" ";" contents "${contents}")
    foreach(testline ${contents})
      string(REGEX REPLACE "([A-Za-z0-9_]+\\.[A-Za-z0-9_]+) (.*)"
                           "\\1;\\2" split "${testline}")
      list(GET split 0 methname)
      list(GET split 1 docstring)
      add_test("system.${name}.${methname}" ${IMP_TEST_SETUP} python ${test} "${methname}")
      set_tests_properties("system.${name}.${methname}" PROPERTIES LABELS "IMP.system;test;length_expensive")
      set_tests_properties("system.${name}.${methname}" PROPERTIES MEASUREMENT "docstring=${docstring}")
      set_tests_properties("system.${name}.${methname}" PROPERTIES TIMEOUT ${expensive_timeout})
    endforeach()
  else()
    add_test("system.${name}" ${IMP_TEST_SETUP} python ${test})
    set_tests_properties("system.${name}" PROPERTIES LABELS "IMP.system;test;length_expensive")
    set_tests_properties("system.${name}" PROPERTIES TIMEOUT ${expensive_timeout})
    set_tests_properties("system.${name}" PROPERTIES COST 3)
  endif()
endforeach(test)

set(cpp_tests )

foreach (test ${cpp_tests})
   GET_FILENAME_COMPONENT(name ${test} NAME)
   GET_FILENAME_COMPONENT(name_we ${test} NAME_WE)
   add_executable("system.${name}" ${test})
   target_link_libraries("system.${name}"     imp_system
    ${IMP_BASE_LIBRARY}
${IMP_CGAL_LIBRARY}
${IMP_ALGEBRA_LIBRARY}
${IMP_KERNEL_LIBRARY}
    ${BOOST.FILESYSTEM_LIBRARIES}
${BOOST.PROGRAMOPTIONS_LIBRARIES}
${BOOST.SYSTEM_LIBRARIES}
${GPERFTOOLS_LIBRARIES}
${TCMALLOC_HEAPPROFILER_LIBRARIES}
${TCMALLOC_HEAPCHECKER_LIBRARIES}
${LOG4CXX_LIBRARIES}
${BOOST.RANDOM_LIBRARIES}
${CGAL_LIBRARIES}
${ANN_LIBRARIES})
   set_target_properties("system.${name}" PROPERTIES
                         RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/test/system/"
                         OUTPUT_NAME ${name_we})
   add_test("system.${name}" ${IMP_TEST_SETUP}
            "${PROJECT_BINARY_DIR}/test/system/${name_we}${CMAKE_EXECUTABLE_SUFFIX}")
   set_tests_properties("system.${name}" PROPERTIES LABELS "IMP.system;test;length_short")
   set_tests_properties("system.${name}" PROPERTIES TIMEOUT ${short_timeout})
   set_tests_properties("system.${name}" PROPERTIES COST 1)
   set(executables ${executables} "system.${name}")
endforeach(test)


set(mdcpp_tests )

foreach (test ${mdcpp_tests})
   GET_FILENAME_COMPONENT(name ${test} NAME)
   GET_FILENAME_COMPONENT(name_we ${test} NAME_WE)
   add_executable("system.${name}" ${test})
   target_link_libraries("system.${name}"     imp_system
    ${IMP_BASE_LIBRARY}
${IMP_CGAL_LIBRARY}
${IMP_ALGEBRA_LIBRARY}
${IMP_KERNEL_LIBRARY}
    ${BOOST.FILESYSTEM_LIBRARIES}
${BOOST.PROGRAMOPTIONS_LIBRARIES}
${BOOST.SYSTEM_LIBRARIES}
${GPERFTOOLS_LIBRARIES}
${TCMALLOC_HEAPPROFILER_LIBRARIES}
${TCMALLOC_HEAPCHECKER_LIBRARIES}
${LOG4CXX_LIBRARIES}
${BOOST.RANDOM_LIBRARIES}
${CGAL_LIBRARIES}
${ANN_LIBRARIES})
   set_target_properties("system.${name}" PROPERTIES
                         RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/test/system/"
                         OUTPUT_NAME ${name_we})
   add_test("system.${name}" ${IMP_TEST_SETUP}
            "${PROJECT_BINARY_DIR}/test/system/${name_we}${CMAKE_EXECUTABLE_SUFFIX}")
   set_tests_properties("system.${name}" PROPERTIES LABELS "IMP.system;test;length_medium")
   set_tests_properties("system.${name}" PROPERTIES TIMEOUT ${medium_timeout})
   set_tests_properties("system.${name}" PROPERTIES COST 1)
   set(executables ${executables} "system.${name}")
endforeach(test)

set(excpp_tests )

foreach (test ${excpp_tests})
   GET_FILENAME_COMPONENT(name ${test} NAME)
   GET_FILENAME_COMPONENT(name_we ${test} NAME_WE)
   add_executable("system.${name}" ${test})
   target_link_libraries("system.${name}"     imp_system
    ${IMP_BASE_LIBRARY}
${IMP_CGAL_LIBRARY}
${IMP_ALGEBRA_LIBRARY}
${IMP_KERNEL_LIBRARY}
    ${BOOST.FILESYSTEM_LIBRARIES}
${BOOST.PROGRAMOPTIONS_LIBRARIES}
${BOOST.SYSTEM_LIBRARIES}
${GPERFTOOLS_LIBRARIES}
${TCMALLOC_HEAPPROFILER_LIBRARIES}
${TCMALLOC_HEAPCHECKER_LIBRARIES}
${LOG4CXX_LIBRARIES}
${BOOST.RANDOM_LIBRARIES}
${CGAL_LIBRARIES}
${ANN_LIBRARIES})
   set_target_properties("system.${name}" PROPERTIES
                         RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/test/system/"
                         OUTPUT_NAME ${name_we})
   add_test("system.${name}" ${IMP_TEST_SETUP}
            "${PROJECT_BINARY_DIR}/test/system/${name_we}${CMAKE_EXECUTABLE_SUFFIX}")
   set_tests_properties("system.${name}" PROPERTIES LABELS "IMP.system;test;length_expensive")
   set_tests_properties("system.${name}" PROPERTIES TIMEOUT ${expensive_timeout})
   set_tests_properties("system.${name}" PROPERTIES COST 1)
   set(executables ${executables} "system.${name}")
endforeach(test)

add_custom_target("imp_system_tests" ALL DEPENDS ${executables})
