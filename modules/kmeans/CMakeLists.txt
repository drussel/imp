# Autogenerated file, run tools/build/setup_cmake.py to regenerate



imp_get_process_exit_code("setup_module kmeans" status ${PROJECT_BINARY_DIR}
                          COMMAND ${PROJECT_SOURCE_DIR}/tools/build/setup_module.py
                          --name=kmeans
                          --datapath=${IMP_DATAPATH}
                          --defines=${IMP_KMEANS_CONFIG}:
                           --source=${PROJECT_SOURCE_DIR})
if(${status} EQUAL 0)
message("Module IMP.kmeans ok")
imp_execute_process("setup_swig_wrappers kmeans" ${PROJECT_BINARY_DIR}
                    COMMAND ${PROJECT_SOURCE_DIR}/tools/build/setup_swig_wrappers.py
                          --module=kmeans
                          --datapath=${IMP_DATAPATH}
                           --source=${PROJECT_SOURCE_DIR})

if(${IMP_SPLIT_PYTHON_TESTS})
imp_execute_process("get_python_tests kmeans" ${PROJECT_BINARY_DIR}
                    COMMAND ${PROJECT_SOURCE_DIR}/tools/build/get_python_tests.py
                          --module=kmeans
                          ${PROJECT_SOURCE_DIR})
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/modules/kmeans/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/kmeans/test)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/kmeans/examples)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/kmeans/benchmark)
add_subdirectory(${PROJECT_SOURCE_DIR}/modules/kmeans/bin)
else()
if(${status} EQUAL 1)
message("Module IMP.kmeans disabled")
else()
message(FATAL_ERROR "setup_module returned ${status}")
endif()
set(IMP_KMEANS_LIBRARY CACHE INTERNAL "" FORCE)
endif()
