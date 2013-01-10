/**
 *  \file IMP/score_functor/Harmonic.h
 *  \brief A Score on the distance between a pair of particles.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#ifndef IMPSCORE_FUNCTOR_HARMONIC_H
#define IMPSCORE_FUNCTOR_HARMONIC_H

#include <IMP/score_functor/score_functor_config.h>
#include "Score.h"
#include <IMP/algebra/utility.h>
IMPSCOREFUNCTOR_BEGIN_NAMESPACE


/** A DistanceScore that scores with a harmonic on the distance, centered
    at 0.*/
class Harmonic: public Score {
  double k_;
public:
  Harmonic(double k): k_(k){}
  template <unsigned int D>
  double get_score(Model *, const base::Array<D, ParticleIndex>&,
                   double distance) const {
    return .5*k_*algebra::get_squared(distance);
  }
  template <unsigned int D>
  DerivativePair get_score_and_derivative(Model *m,
                                          const base::Array<D, ParticleIndex>&p,
                                          double distance) const {
    return DerivativePair(get_score(m,p,distance),
                          k_*(distance));
  }
};

IMPSCOREFUNCTOR_END_NAMESPACE

#endif  /* IMPSCORE_FUNCTOR_HARMONIC_H */
