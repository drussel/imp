# Autogenerated file, run tools/build/setup_cmake.py to regenerate



imp_get_process_exit_code("setup_module saxs" status ${PROJECT_BINARY_DIR}
                          COMMAND ${PROJECT_SOURCE_DIR}/tools/build/setup_module.py
                          --name=saxs
                          --datapath=${IMP_DATAPATH}
                          --defines=${IMP_SAXS_CONFIG}:
                           --source=${PROJECT_SOURCE_DIR})
if(${status} EQUAL 0)
message("Module IMP.saxs ok")
imp_execute_process("setup_swig_wrappers saxs" ${PROJECT_BINARY_DIR}
                    COMMAND ${PROJECT_SOURCE_DIR}/tools/build/setup_swig_wrappers.py
                          --module=saxs
                          --datapath=${IMP_DATAPATH}
                           --source=${PROJECT_SOURCE_DIR})

if(${IMP_SPLIT_PYTHON_TESTS})
imp_execute_process("get_python_tests saxs" ${PROJECT_BINARY_DIR}
                    COMMAND ${PROJECT_SOURCE_DIR}/tools/build/get_python_tests.py
                          --module=saxs
                          ${PROJECT_SOURCE_DIR})
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/modules/saxs/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/saxs/test)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/saxs/examples)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/saxs/benchmark)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/saxs/bin)
else()
if(${status} EQUAL 1)
message("Module IMP.saxs disabled")
else()
message(FATAL_ERROR "setup_module returned ${status}")
endif()
set(IMP_SAXS_LIBRARY CACHE INTERNAL "" FORCE)
endif()
