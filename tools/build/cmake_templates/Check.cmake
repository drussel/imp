# Autogenerated file, run tools/build/setup_cmake.py to regenerate

INCLUDE (CheckCXXSourceCompiles)

CHECK_CXX_SOURCE_COMPILES("%(cppsource)s
"
 %(macro)s)

if(${%(macro)s} MATCHES 1)
set(%(macro)s 1 CACHE INTERNAL "" FORCE )
else()
set(%(macro)s 0 CACHE INTERNAL "" FORCE)
endif()