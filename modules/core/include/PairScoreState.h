/**
 *  \file PairScoreState.h
 *  \brief Use a PairModifier applied to a ParticlePairs to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_PAIR_SCORE_STATE_H
#define IMPCORE_PAIR_SCORE_STATE_H

#include "config.h"
#include <IMP/PairModifier.h>
#include <IMP/ScoreState.h>
#include <IMP/Particle.h>

IMP_BEGIN_NAMESPACE
// for swig
class PairModifier;
IMP_END_NAMESPACE

IMPCORE_BEGIN_NAMESPACE
//! Apply a PairFunction to a Pair
/** The score state is passed up to two PairModifiers, one to
    apply before evaluation and the other after. The one after
    should take a DerivativeAccumulator as its last argument for
    PairModifier::apply() and will only be called if
    the score was computed with derivatives.

    \ingroup restraint
    \see PairsScoreState
 */
class IMPCOREEXPORT PairScoreState : public ScoreState
{
  IMP::internal::OwnerPointer<PairModifier> f_;
  IMP::internal::OwnerPointer<PairModifier> af_;
  ParticlePair v_;
public:
  /** before and after are the modifiers to apply before and after
      evaluate.
   */
  PairScoreState(PairModifier *before,
                      PairModifier *after, const ParticlePair& vt,
                      std::string name="PairScoreState %1%");

  //! Apply this modifier to all the elements after an evaluate
  void set_after_evaluate_modifier(PairModifier* f) {
    af_=f;
  }

  //! Apply this modifier to all the elements before an evaluate
  void set_before_evaluate_modifier(PairModifier* f) {
    f_=f;
  }

  IMP_SCORE_STATE(PairScoreState, get_module_version_info())
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_PAIR_SCORE_STATE_H */
