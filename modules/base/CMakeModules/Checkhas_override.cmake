# Autogenerated file, run build/tools/setup_cmake.py to regenerate

INCLUDE (CheckCXXSourceCompiles)

CHECK_CXX_SOURCE_COMPILES("/**
 *  ile nothing.cpp
 *  rief nothing
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

struct Base {
    virtual void some_func(float);
};

struct Derived : Base {
  virtual void some_func(float) override;
};

int main()
{
  auto a=6;
  return 0;
}

"
 IMP_COMPILER_HAS_OVERRIDE)

if(${IMP_COMPILER_HAS_OVERRIDE} MATCHES 1)
set(IMP_COMPILER_HAS_OVERRIDE 1 CACHE INTERNAL "" FORCE )
else()
set(IMP_COMPILER_HAS_OVERRIDE 0 CACHE INTERNAL "" FORCE)
endif()