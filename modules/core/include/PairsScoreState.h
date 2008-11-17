/**
 *  \file PairsScoreState.h
 *  \brief Use a PairModifier applied to a ParticlePairs to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_PAIRS_SCORE_STATE_H
#define IMPCORE_PAIRS_SCORE_STATE_H

#include "core_exports.h"
#include "PairContainer.h"
#include "PairModifier.h"
#include "internal/core_version_info.h"

#include <IMP/ScoreState.h>

IMPCORE_BEGIN_NAMESPACE

// for swig
class PairContainer;
class PairModifier;

//! Apply a PairFunction to a PairContainer to maintain an invariant
/**
 */
class IMPCOREEXPORT PairsScoreState : public ScoreState
{
  Pointer<PairModifier> f_;
  Pointer<PairContainer> c_;
public:
  PairsScoreState(PairModifier *f, PairContainer *c);

  virtual ~PairsScoreState();

  IMP_SCORE_STATE(internal::core_version_info)
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_PAIRS_SCORE_STATE_H */
