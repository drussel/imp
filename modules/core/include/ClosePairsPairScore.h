/**
 *  \file ClosePairsPairScore.h
 *  \brief Apply a PairScore to close pairs.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_CLOSE_PAIRS_PAIR_SCORE_H
#define IMPCORE_CLOSE_PAIRS_PAIR_SCORE_H

#include "core_config.h"
#include "XYZR.h"

#include <IMP/PairScore.h>
#include <IMP/UnaryFunction.h>
#include <IMP/Pointer.h>
#include <IMP/Refiner.h>

IMPCORE_BEGIN_NAMESPACE

//! Traverse the Refiner hierarchy to find all pairs which are close
/** The Refiner implicitly defines a tree rooted at each particle.
    This PairScore applies another PairScore to all pairs of leaves, one
    taken from each tree such that the leaves are closer than the threshold.
    \note All particles in the tree must be XYZR particles for the
    passed radius. In addition, the ball defined by a particle must contain
    the balls of all its leaves.
    \see ClosePairsScoreState
 */
class IMPCOREEXPORT ClosePairsPairScore : public PairScore
{
  IMP::internal::OwnerPointer<Refiner> r_;
  IMP::internal::OwnerPointer<PairScore> f_;
  Float th_;
  FloatKey rk_;
public:
  /** \param[in] r The Refiner to call on each particle
      \param[in] f The pair score to apply to the generated pairs
      \param[in] max_distance Only score pairs which are close than
      the max_distance
      \param[in] rk The key to use for the radius.
   */
  ClosePairsPairScore(Refiner *r, PairScore *f,
                      Float max_distance,
                      FloatKey rk= XYZR::get_default_radius_key());
  IMP_PAIR_SCORE(ClosePairsPairScore);
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_CLOSE_PAIRS_PAIR_SCORE_H */
