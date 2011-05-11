/**
 *  \file IMP/base_types.h    \brief Basic types used by IMP.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_BASE_TYPES_H
#define IMP_BASE_TYPES_H

#define IMP_NUM_INLINE 5

#include "kernel_config.h"
#include "Key.h"
#include <string>
#include <vector>

IMP_BEGIN_NAMESPACE

/** \internal \namespace IMP::internal Implementation details.
 */

//! Basic floating-point value (could be float, double...)
typedef double Float;

//! A pair representing a function value with its first derivative
typedef std::pair<double, double> DerivativePair;
typedef std::vector<DerivativePair> DerivativePairs;

//! A generic pair of floats
typedef std::pair<double, double> FloatPair;
typedef std::vector<FloatPair> FloatPairs;

//! A pair representing the allowed range for a Float attribute
typedef std::pair<Float, Float> FloatRange;

//! Basic integer value
typedef int Int;

typedef std::pair<Int, Int> IntRange;
typedef std::vector<IntRange> IntRanges;

typedef std::pair<Int, Int> IntPair;
typedef std::vector<IntPair> IntPairs;


//! Basic string value
typedef std::string String;


//! Standard way to pass a bunch of Float values
typedef std::vector<Float> Floats;
//! Standard way to pass a bunch of Int values
typedef std::vector<Int> Ints;
//! Standard way to pass a bunch of String values
typedef std::vector<String> Strings;


/** @name Attribute Keys
    Each type of attribute has an associated type of key. The keys can
    be constructed from a string. Such construction can be expensive
    and so the resulting keys should be cached.
    @{
 */

//! The type used to identify float attributes in the Particles
IMP_DECLARE_KEY_TYPE(FloatKey, 0);
//! The type used to identify int attributes in the Particles
IMP_DECLARE_KEY_TYPE(IntKey, 1);
//! The type used to identify string attributes in the Particles
IMP_DECLARE_KEY_TYPE(StringKey, 2);
//! The type used to identify a particle attribute in the Particles
IMP_DECLARE_KEY_TYPE(ParticleKey, 3);
//! The type used to identify a particle attribute in the Particles
IMP_DECLARE_KEY_TYPE(ObjectKey, 4);

/** @} */

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
inline void show(std::ostream &out, int i) {
  out << i;
}

inline void show(std::ostream &out, unsigned int i) {
  out << i;
}

inline void show(std::ostream &out, std::string i) {
  out << i;
}

inline void show(std::ostream &out, double i) {
  out << i;
}
template <class T>
inline void show(std::ostream &out, const T& i) {
  out << i;
}
template <class T>
inline void show(std::ostream &out, const T*i) {
  out << i->get_name();
}
template <class T>
inline void show(std::ostream &out, T*i) {
  out << i->get_name();
}
#endif


#ifndef IMP_DOXYGEN
template <class T>
inline std::size_t hash_value(const T &t) {
  return t.__hash__();
}

template <class T>
inline std::ostream &operator<<(std::ostream &out,
                                const std::vector<T> &data) {
  out << "[";
  for (unsigned int i=0; i< data.size(); ++i) {
    if (i != 0) {
      out << ", ";
    }
    out << data[i];
  }
  out << "]";
  return out;
}


template <class T>
inline std::ostream &operator<<(std::ostream &out,
                                const std::vector<T*> &data) {
  out << "[";
  for (unsigned int i=0; i< data.size(); ++i) {
    if (i != 0) {
      out << ", ";
    }
    out << data[i]->get_name();
  }
  out << "]";
  return out;
}
#endif

IMP_END_NAMESPACE

#endif  /* IMP_BASE_TYPES_H */
