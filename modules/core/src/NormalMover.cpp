/**
 *  \file NormalMover.cpp
 *  \brief A modifier which perturbs a point with a gaussian.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/core/NormalMover.h>

#include <IMP/random.h>

#include <boost/random/normal_distribution.hpp>

IMPCORE_BEGIN_NAMESPACE

NormalMover::NormalMover(SingletonContainer *sc,
                         const FloatKeys &vars,
                         Float max): MoverBase(sc)
{
  add_float_keys(vars);
  set_sigma(max);
}

void NormalMover::generate_move(float probability)
{
  boost::uniform_real<> rand(0,1);
  boost::normal_distribution<double> mrng(0, stddev_);
  boost::variate_generator<RandomNumberGenerator&,
                           boost::normal_distribution<double> >
                          sampler(random_number_generator, mrng);

  for (unsigned int i = 0;
       i < get_container()->get_number_of_particles(); ++i) {
    if (rand(random_number_generator) > probability) continue;
    for (unsigned int j = 0; j < get_number_of_float_keys(); ++j) {
      Float c = get_float(i, j);
      Float r = sampler();
      // Check for NaN (x!=x when x==NaN) (can only use std::isnan with C99)
      IMP_assert(r == r, "Bad random");
      IMP_assert(c == c, "Bad stored");
      propose_value(i, j, c + r);
    }
  }
}

IMPCORE_END_NAMESPACE
