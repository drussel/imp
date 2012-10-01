/**
 *  \file IMP/base/tuple_macros.h
 *  \brief Various general useful macros for IMP.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_TUPLE_MACROS_H
#define IMPBASE_TUPLE_MACROS_H
#include "base_config.h"
#include "Value.h"
#include "Showable.h"
#include "hash.h"
#include "hash_macros.h"
#include "showable_macros.h"
#include "value_macros.h"
#include "comparison_macros.h"

/** \name Named tuples
    It is often useful to declare little structures to aid in the passing
    of arguments by name or returning sets of values. One can use
    boost::tuples, but these don't have names for their parts and so
    don't lead to clear code. Instead we provide a macro to aid
    declaring such classes. The resulting class is hashable and
    comparable too.
@{
*/

#define IMP_NAMED_TUPLE_1(Name, Names, type0, var0, invariant)          \
  struct Name: public IMP::base::Value {                                \
    type0 var0;                                                         \
    Name(type0 i0=type0()): var0(i0){invariant;}                        \
    IMP_HASHABLE_INLINE(Name, {                                         \
        std::size_t value= IMP::base::hash_value(var0);                 \
        return value;                                                   \
      });                                                               \
    IMP_COMPARISONS_1(Name, var0);                                      \
    IMP_SHOWABLE_INLINE(Name, out << "(" << #var0 << "="                \
                        << IMP::base::Showable(var0) << ")");           \
  };                                                                    \
  IMP_VALUES(Name, Names)



#define IMP_NAMED_TUPLE_2(Name, Names, type0, var0, type1, var1,        \
                          invariant)                                    \
  struct Name: public IMP::base::Value {                                \
    type0 var0;                                                         \
    type1 var1;                                                         \
    Name(type0 i0=type0(), type1 i1=type1()): var0(i0), var1(i1)        \
    {invariant;}                                                        \
    IMP_HASHABLE_INLINE(Name, {                                         \
        std::size_t value= IMP::base::hash_value(var0);                 \
        boost::hash_combine(value, IMP::base::hash_value(var1));        \
        return value;                                                   \
      });                                                               \
    IMP_SHOWABLE_INLINE(Name, out << "(" << #var0 << "="                \
                        << IMP::base::Showable(var0)                    \
                        << " " <<#var1 << "="                           \
                        << IMP::base::Showable(var1) << ")");           \
    IMP_COMPARISONS_2(Name, var0, var1);                                \
  };                                                                    \
  IMP_VALUES(Name, Names)


#define IMP_NAMED_TUPLE_3(Name, Names, type0, var0, type1, var1,        \
                          type2, var2, invariant)                       \
  struct Name: public IMP::base::Value {                                \
    type0 var0;                                                         \
    type1 var1;                                                         \
    type2 var2;                                                         \
    Name(type0 i0=type0(), type1 i1=type1(),type2 i2=type2()            \
         ): var0(i0), var1(i1), var2(i2){invariant;}                    \
    IMP_HASHABLE_INLINE(Name, {                                         \
        std::size_t value= IMP::base::hash_value(var0);                 \
        boost::hash_combine(value, IMP::base::hash_value(var1));        \
        boost::hash_combine(value, IMP::base::hash_value(var2));        \
        return value;                                                   \
      });                                                               \
    IMP_COMPARISONS_3(Name, var0, var1, var2);                          \
    IMP_SHOWABLE_INLINE(Name, out << "(" << #var0 << "="                \
                        << IMP::base::Showable(var0)                    \
                        << " " <<#var1 << "="                           \
                        << IMP::base::Showable(var1)                    \
                        << " " <<#var2 << "="                           \
                        << IMP::base::Showable(var2) << ")");           \
  };                                                                    \
  IMP_VALUES(Name, Names)


#define IMP_NAMED_TUPLE_4(Name, Names, type0, var0, type1, var1,        \
                          type2, var2, type3, var3, invariant)          \
  struct Name: public IMP::base::Value {                                \
    type0 var0;                                                         \
    type1 var1;                                                         \
    type2 var2;                                                         \
    type3 var3;                                                         \
    Name(type0 i0=type0(), type1 i1=type1(),type2 i2=type2(),           \
         type3 i3=type3()): var0(i0), var1(i1), var2(i2),               \
                            var3(i3) {invariant;}                       \
    IMP_HASHABLE_INLINE(Name, {                                         \
        std::size_t value= IMP::base::hash_value(var0);                 \
        boost::hash_combine(value, IMP::base::hash_value(var1));        \
        boost::hash_combine(value, IMP::base::hash_value(var2));        \
        boost::hash_combine(value, IMP::base::hash_value(var3));        \
        return value;                                                   \
      });                                                               \
    IMP_COMPARISONS(Name);                                              \
    IMP_SHOWABLE_INLINE(Name, out << "(" << #var0 << "="                \
                        << IMP::base::Showable(var0)                    \
                        << " " <<#var1 << "="                           \
                        << IMP::base::Showable(var1)                    \
                        << " " <<#var2 << "="                           \
                        << IMP::base::Showable(var2)                    \
                        << " " <<#var3 << "="                           \
                        << IMP::base::Showable(var3)                    \
                        << ")");                                        \
private:                                                                \
 int compare(const Name &o) const {                                     \
   IMP_COMPARE_ONE(var0, o.var0);                                       \
   IMP_COMPARE_ONE(var1, o.var1);                                       \
   IMP_COMPARE_ONE(var2, o.var2);                                       \
   IMP_COMPARE_ONE(var3, o.var3);                                       \
   return 0;                                                            \
 }                                                                      \
  };                                                                    \
  IMP_VALUES(Name, Names)


#define IMP_NAMED_TUPLE_5(Name, Names, type0, var0, type1, var1,        \
                          type2, var2, type3, var3, type4, var4,        \
                          invariant)                                    \
  struct Name: public IMP::base::Value {                                \
    type0 var0;                                                         \
    type1 var1;                                                         \
    type2 var2;                                                         \
    type2 var3;                                                         \
    type2 var4;                                                         \
    Name(type0 i0=type0(), type1 i1=type1(),type2 i2=type2(),           \
         type3 i3=type3(), type4 i4=type4()): var0(i0), var1(i1),       \
                                              var2(i2),                 \
                                              var3(i3), var4(i4)        \
    {invariant;}                                                        \
    IMP_HASHABLE_INLINE(Name, {                                         \
        std::size_t value= IMP::base::hash_value(var0);                 \
        boost::hash_combine(value, IMP::base::hash_value(var1));        \
        boost::hash_combine(value, IMP::base::hash_value(var2));        \
        boost::hash_combine(value, IMP::base::hash_value(var3));        \
        boost::hash_combine(value, IMP::base::hash_value(var4));        \
        return value;                                                   \
      });                                                               \
    IMP_COMPARISONS(Name);                                              \
    IMP_SHOWABLE_INLINE(Name, out << "(" << #var0 << "="                \
                        << IMP::base::Showable(var0)                    \
                        << " " <<#var1 << "="                           \
                        << IMP::base::Showable(var1)                    \
                        << " " <<#var2 << "="                           \
                        << IMP::base::Showable(var2)                    \
                        << " " <<#var3 << "="                           \
                        << IMP::base::Showable(var3)                    \
                        << " " <<#var4 << "="                           \
                        << IMP::base::Showable(var4)                    \
                        << ")");                                        \
private:                                                                \
 int compare(const Name &o) const {                                     \
   IMP_COMPARE_ONE(var0, o.var0);                                       \
   IMP_COMPARE_ONE(var1, o.var1);                                       \
   IMP_COMPARE_ONE(var2, o.var2);                                       \
   IMP_COMPARE_ONE(var3, o.var3);                                       \
   IMP_COMPARE_ONE(var4, o.var4);                                       \
   return 0;                                                            \
 }                                                                      \
  };                                                                    \
  IMP_VALUES(Name, Names)



/**@}*/

#endif  /* IMPBASE_TUPLE_MACROS_H */
