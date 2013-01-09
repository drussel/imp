/**
 *  \file IMP/core/RefinedPairsPairScore.h
 *  \brief Generate pairs by applying a Refiner to the paricles.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_REFINED_PAIRS_PAIR_SCORE_H
#define IMPCORE_REFINED_PAIRS_PAIR_SCORE_H

#include <IMP/core/core_config.h>
#include <IMP/PairScore.h>
#include <IMP/UnaryFunction.h>
#include <IMP/Pointer.h>
#include <IMP/Refiner.h>
#include <IMP/pair_macros.h>

IMPCORE_BEGIN_NAMESPACE

//! Generate pairs to score by applying a Refiner.
/** Each passed particle refined and then the PairScore applied
    to all resulting pairs.
 */
class IMPCOREEXPORT RefinedPairsPairScore : public PairScore
{
  IMP::OwnerPointer<Refiner> r_;
  IMP::OwnerPointer<PairScore> f_;
public:
  /** \param[in] r The Refiner to call on each particle
      \param[in] f The pair score to apply to the generated pairs
   */
  RefinedPairsPairScore(Refiner *r, PairScore *f);
  IMP_INDEX_PAIR_SCORE(RefinedPairsPairScore);
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_REFINED_PAIRS_PAIR_SCORE_H */
