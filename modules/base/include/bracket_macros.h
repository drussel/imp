/**
 *  \file IMP/base/bracket_macros.h
 *  \brief Various general useful macros for IMP.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_BRACKET_MACROS_H
#define IMPBASE_BRACKET_MACROS_H
#include <IMP/base/base_config.h>


#ifdef IMP_DOXYGEN
/** Implement operator[] for C++ and python. The index type is
    Index and the expression that returns the value is expr. If the
    bounds_check_expr is false, then a UsageException is thrown
    in C++ or and IndexException if called from python.
*/
#define IMP_BRACKET(Value, Index, bounds_check_expr, expr)       \
  const Value operator[](Index) const;                           \
  Value& operator[](Index);

/** Implement operator[] for C++ and python. The index type is
    Index and the expression that returns the value is expr.
    The value returned is not mutable. If the
    bounds_check_expr is false, then a UsageException is thrown
    in C++ or and IndexException if called from python.
*/
#define IMP_CONST_BRACKET(Value, Index, bounds_check_expr, expr)   \
  const Value operator[](Index) const;



#elif !defined(SWIG)
#define IMP_CONST_BRACKET(Value, Index, bounds_check_expr, expr)        \
  const Value& operator[](Index i) const {                              \
    IMP_USAGE_CHECK((bounds_check_expr), "Index out of range: "<< i);   \
    expr;                                                               \
  }                                                                     \
  const Value& __getitem__(Index i) const {                             \
    if (!(bounds_check_expr)) {                                         \
      IMP_THROW("Bad index " << i, IMP::base::IndexException);          \
    }                                                                   \
    expr;                                                               \
  }                                                                     \

#define IMP_BRACKET(Value, Index, bounds_check_expr, expr)              \
  Value& operator[](Index i)  {                                         \
    IMP_USAGE_CHECK((bounds_check_expr), "Index out of range: "<< i);   \
    expr;                                                               \
  }                                                                     \
  void __setitem__(Index i, const Value &v) {                           \
    operator[](i)=v;                                                    \
  }                                                                     \
  IMP_CONST_BRACKET(Value, Index, bounds_check_expr, expr)



#else
#define IMP_CONST_BRACKET(Value, Index, bounds_check_expr, expr)        \
  const Value& __getitem__(Index i) const {                             \
    if (!(bounds_check_expr)) {                                         \
      IMP_THROW("Bad index " << i, IMP::base::IndexException);          \
    }                                                                   \
    expr;                                                               \
  }

#define IMP_BRACKET(Value, Index, bounds_check_expr, expr)              \
  void __setitem__(Index i, const Value &v) {                           \
    operator[](i)=v;                                                    \
  }                                                                     \
  IMP_CONST_BRACKET(Value, Index, bounds_check_expr, expr)

#endif


#endif  /* IMPBASE_BRACKET_MACROS_H */
