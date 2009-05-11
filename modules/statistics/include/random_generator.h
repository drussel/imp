/**
 *  \file RandomGenerator.h   \brief random number generator
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPSTATISTICS_RANDOM_GENERATOR_H
#define IMPSTATISTICS_RANDOM_GENERATOR_H

#include <cstdlib>
#include <math.h>
#include "IMP/random.h"
#include "config.h"
#include "IMP/macros.h"
#include  <boost/random/normal_distribution.hpp>
#include  <boost/random.hpp>
IMPSTATISTICS_BEGIN_NAMESPACE

//! Generate a random integer number
/**
\param[in] n , the range is [0,n-1]
 */
inline int random_int(int n) {
  ::boost::uniform_int<> rand(0,n-1);
  return rand(random_number_generator);
}
//! Generate a random number in the range [lo,hi]
inline double random_uniform(double lo=0.0, double hi=1.0) {
  ::boost::uniform_real<> rand(lo, hi);
  return rand(random_number_generator);
}
//! Gaussian random number generator
/** Returns a normally distributed with zero mean and unit variance
 */
inline double random_gauss(double mean=0.0,double sigma=1.0) {
  typedef boost::normal_distribution<double> NormalDistribution;
  typedef boost::mt19937 RandomGenerator;

  NormalDistribution norm_dist(mean, sigma);
  norm_dist.reset();
  // Initiate Random Number generator with current time
  RandomGenerator rng(static_cast<unsigned> (time(0)));

  boost::variate_generator<RandomGenerator, NormalDistribution>
    generator(rng,norm_dist);
  return generator();
}

IMPSTATISTICS_END_NAMESPACE
#endif  /* IMPSTATISTICS_RANDOM_GENERATOR_H */
