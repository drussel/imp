# Autogenerated file, run tools/build/setup_cmake.py to regenerate

add_subdirectory(${PROJECT_SOURCE_DIR}/modules/rmf/dependency)

imp_get_process_exit_code("setup_module rmf" status ${PROJECT_BINARY_DIR}
                          COMMAND ${PROJECT_SOURCE_DIR}/tools/build/setup_module.py
                          --name=rmf
                          --datapath=${IMP_DATAPATH}
                          --defines=${IMP_RMF_CONFIG}:
                           --source=${PROJECT_SOURCE_DIR})
if(${status} EQUAL 0)
message("Module IMP.rmf ok")
imp_execute_process("setup_swig_wrappers rmf" ${PROJECT_BINARY_DIR}
                    COMMAND ${PROJECT_SOURCE_DIR}/tools/build/setup_swig_wrappers.py
                          --module=rmf
                          --datapath=${IMP_DATAPATH}
                           --source=${PROJECT_SOURCE_DIR})

if(${IMP_SPLIT_PYTHON_TESTS})
imp_execute_process("get_python_tests rmf" ${PROJECT_BINARY_DIR}
                    COMMAND ${PROJECT_SOURCE_DIR}/tools/build/get_python_tests.py
                          --module=rmf
                          ${PROJECT_SOURCE_DIR})
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/modules/rmf/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/rmf/test)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/rmf/examples)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/rmf/benchmark)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/rmf/bin)
else()
if(${status} EQUAL 1)
message("Module IMP.rmf disabled")
else()
message(FATAL_ERROR "setup_module returned ${status}")
endif()
set(IMP_RMF_LIBRARY CACHE INTERNAL "" FORCE)
endif()
