/**
 *  \file PairRestraint.h
 *  \brief Apply a PairScore to a Pair.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_PAIR_RESTRAINT_H
#define IMPCORE_PAIR_RESTRAINT_H

#include "core_config.h"

#include <IMP/Restraint.h>
#include <IMP/Pointer.h>
#include <IMP/PairScore.h>
#include <IMP/internal/container_helpers.h>

#include <iostream>

IMPCORE_BEGIN_NAMESPACE

//! Applies a PairScore to a Pair.
/** This restraint stores a Pair.
    \see PairRestraint
 */
class IMPCOREEXPORT PairRestraint :
  public PairScoreRestraint
{
  IMP::OwnerPointer<PairScore> ss_;
  ParticleIndexPair v_;
public:
  //! Create the restraint.
  /** This function takes the function to apply to the
      stored Pair and the Pair.
   */
  PairRestraint(PairScore *ss,
                     const ParticlePair& vt,
                     std::string name="PairRestraint %1%");

  PairScore* get_score() const {
    return ss_;
  }
  ParticlePair get_argument() const {
    return IMP::internal::get_particle(get_model(), v_);
  }

  IMP_RESTRAINT(PairRestraint);

  double unprotected_evaluate_if_good(DerivativeAccumulator *da,
                                      double max) const;
#ifndef IMP_DOXYGEN
  Restraints do_create_current_decomposition() const;
#endif
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_PAIR_RESTRAINT_H */
