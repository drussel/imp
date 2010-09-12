/**
 *  \file PairRestraint.h
 *  \brief Apply a PairScore to a Pair.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_PAIR_RESTRAINT_H
#define IMPCORE_PAIR_RESTRAINT_H

#include "core_config.h"

#include <IMP/Restraint.h>
#include <IMP/Pointer.h>
#include <IMP/PairScore.h>
#include "internal/pair_helpers.h"

#include <iostream>

IMPCORE_BEGIN_NAMESPACE

//! Applies a PairScore to a Pair.
/** This restraint stores a Pair.
    \see PairRestraint
 */
class IMPCOREEXPORT PairRestraint :
  public PairScoreRestraint
{
  IMP::internal::OwnerPointer<PairScore> ss_;
  ParticlePair v_;
  mutable double score_;
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
    return v_;
  }

  IMP_INCREMENTAL_RESTRAINT(PairRestraint);
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_PAIR_RESTRAINT_H */
