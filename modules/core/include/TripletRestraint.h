/**
 *  \file TripletRestraint.h
 *  \brief Apply a TripletScore to a Triplet.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_TRIPLET_RESTRAINT_H
#define IMPCORE_TRIPLET_RESTRAINT_H

#include "core_config.h"

#include <IMP/Restraint.h>
#include <IMP/Pointer.h>
#include <IMP/TripletScore.h>
#include "internal/triplet_helpers.h"
#include <IMP/internal/container_helpers.h>

#include <iostream>

IMPCORE_BEGIN_NAMESPACE

//! Applies a TripletScore to a Triplet.
/** This restraint stores a Triplet.
    \see TripletRestraint
 */
class IMPCOREEXPORT TripletRestraint :
  public TripletScoreRestraint
{
  IMP::OwnerPointer<TripletScore> ss_;
  ParticleIndexTriplet v_;
public:
  //! Create the restraint.
  /** This function takes the function to apply to the
      stored Triplet and the Triplet.
   */
  TripletRestraint(TripletScore *ss,
                     const ParticleTriplet& vt,
                     std::string name="TripletRestraint %1%");

  TripletScore* get_score() const {
    return ss_;
  }
  ParticleTriplet get_argument() const {
    return IMP::internal::get_particle(get_model(), v_);
  }

  IMP_RESTRAINT(TripletRestraint);

  double unprotected_evaluate_if_good(DerivativeAccumulator *da,
                                      double max) const;

  Restraints get_current_decomposition() const;
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_TRIPLET_RESTRAINT_H */
