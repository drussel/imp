/**
 *  \file IMP/utility.h    \brief Various general useful functions for IMP.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_UTILITY_H
#define IMP_UTILITY_H

#include "kernel_config.h"
#include <boost/version.hpp>
#include <boost/static_assert.hpp>
#include <boost/utility.hpp>
#include <algorithm>

#if IMP_BOOST_VERSION >= 103500
#include <boost/math/special_functions/fpclassify.hpp>
#elif defined(__GNUC__)
#include <cmath>
#endif

IMP_BEGIN_NAMESPACE


#ifndef IMP_DOXYGEN

//! Compute the square of a number
template <class T>
T square(T t)
{
  return t*t;
}

//! Compute the cube of a number
template <class T>
T cube(T t)
{
  return t*t*t;
}


//! Return true if a number is NaN
/** With certain compiler settings the compiler can optimize
 out a!=a (and certain intel chips had issues with it too).
 */
template <class T>
inline bool is_nan(const T& a) {
#if IMP_BOOST_VERSION >= 103500
  return (boost::math::isnan)(a);
#elif defined(_GLIBCXX_USE_C99_MATH)
  // Not all gcc versions include C99 math
  return (std::isnan)(a);
#else
  return a != a;
#endif
}

//! A version of std::for_each which works with ranges
/** This is needed to apply the functor to a range which is a temporary
    object, since you can't call both begin and end on it.
 */
template <class Range, class Functor>
void for_each(const Range &r, const Functor &f) {
  std::for_each(r.begin(), r.end(), f);
}

template <class T>
int compare(const T &a, const T &b) {
  return a.compare(b);
}
#endif

IMP_END_NAMESPACE

#endif  /* IMP_UTILITY_H */
