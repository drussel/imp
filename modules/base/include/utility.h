/**
 *  \file IMP/base/utility.h    \brief Various general useful functions for IMP.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_UTILITY_H
#define IMPBASE_UTILITY_H

#include "base_config.h"
#include <boost/version.hpp>
#include <boost/static_assert.hpp>
#include <boost/utility.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <iostream>
#include "base_macros.h"
#include <IMP/compatibility/vector.h>

#if !defined(_GLIBCXX_USE_C99_MATH) && BOOST_VERSION >= 103500
#include <boost/math/special_functions/fpclassify.hpp>
#endif

IMPBASE_BEGIN_NAMESPACE



#if !defined(IMP_DOXYGEN) && !defined(SWIG)
template <class T>
inline T square(T t) IMP_NO_SIDEEFFECTS;
template <class T>
inline T cube(T t) IMP_NO_SIDEEFFECTS;
template <class T>
inline bool is_nan(const T& a) IMP_NO_SIDEEFFECTS;


//! Compute the square of a number
template <class T>
inline T square(T t)
{
  return t*t;
}

//! Compute the cube of a number
template <class T>
inline T cube(T t)
{
  return t*t*t;
}

//! Return true if a number is NaN
/** With certain compiler settings the compiler can optimize
 out a!=a (and certain intel chips had issues with it too).
 */
template <class T>
inline bool is_nan(const T& a) {
#if defined(_GLIBCXX_USE_C99_MATH)
  // Not all gcc versions include C99 math
  return (std::isnan)(a);
#elif BOOST_VERSION >= 103500
  return (boost::math::isnan)(a);
#else
  return a != a;
#endif
}

//! A version of std::for_each which works with ranges
/** This is needed to apply the functor to a range which is a temporary
    object, since you can't call both begin and end on it.
 */
template <class Range, class Functor>
inline void for_each(const Range &r, const Functor &f) {
  std::for_each(r.begin(), r.end(), f);
}

template <class T>
inline int compare(const T &a, const T &b) {
  return a.compare(b);
}

/** Convert between different types of lists.
 */
template <class Out, class In>
inline Out get_as(const In &in) {
  return Out(in.begin(), in.end());
}
#endif


IMPBASE_END_NAMESPACE

#endif  /* IMPBASE_UTILITY_H */
