/**
 *  \file algebra/utility.h
 *  \brief Functions to deal with very common math operations
 *  \author Javier Velazquez-Muriel
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
*/

#ifndef IMPALGEBRA_UTILITY_H
#define IMPALGEBRA_UTILITY_H

#include <cmath> // abs
#include "config.h"
#include <IMP/exception.h>

IMPALGEBRA_BEGIN_NAMESPACE

//! xor operation between two values
inline bool xorT(bool x, bool y)
{
  return (((x) && !(y)) || (!(x) && (y)));
}



//! Sign of a number. 1 if the number is higher or equal to 0 and -1 otherwise
template<typename T>
int sign(const T& x)
{
  if (x >= 0) return 1;
  return -1;
}

//! Rounds a number to next integer.
/**
 * The result is of type integer but the argument can be of any type. Some
 * examples:
 *
 * \code
 * a = round(-0.7); // a = -1
 * a = round(-0.2); // a = 0
 * a = round(0.2); // a = 0
 * a = round(0.7); // a = 1
 * \endcode
 */
template<typename T>
int round(const T& x)
{
  if (x > 0) {
    return (int)((x) + 0.5);
  } else {
    return (int)((x) - 0.5);
  }
}


//! Constrains a value between two given limits
/*
 * The limits act as the constrain to saturate the value. Examples:
 * with x and constrain(x,-2,2):
 *
 * \code
 * x = constrain(-8,-2,2); // x=-2;
 * x = constrain(-2,-2,2); // x=-2;
 * x = constrain(-1,-2,2); // x=-1;
 * x = constrain(0,-2,2); // x=0;
 * x = constrain(2,-2,2); // x=2;
 * x = constrain(4,-2,2); // x=2;
 * \endcode
 */
template<typename T>
T constrain(const T x, const T x0, const T xF)
{
  if (x < x0) return x0;
  if (x > xF) return xF;
  return x;
}


//! Compares two values (intended for doubles)
/**
 * epsilon is the tolerance allowed to consider the values as equal
 */
inline bool almost_equal(const double a, const double b, const double epsilon)
{
  return (std::abs(a-b) < epsilon);
}

//! Compares two values (intended for floats)
/**
 * epsilon is the tolerance allowed to consider the values as equal
 */
inline bool almost_equal(const float a, const float b, const float epsilon)
{
  return (std::abs(a-b) < epsilon);
}


//! Closest power of 2 that can contain a number x
inline float closest_pow2(float x) {
  float p=1;
  while(p<x) {p*=2;}
  return p;
}

//! Closest power of 2 that can contain a number x
inline double closest_pow2(double x) {
  double p=1;
  while(p<x) {p*=2;}
  return p;
}

IMPALGEBRA_END_NAMESPACE

#endif  /* IMPALGEBRA_UTILITY_H */
