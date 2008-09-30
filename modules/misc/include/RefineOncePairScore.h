/**
 *  \file RefineOncePairScore.h
 *  \brief Refine particles at most once with a ParticleRefiner.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef __IMPMISC_REFINE_ONCE_PAIR_SCORE_H
#define __IMPMISC_REFINE_ONCE_PAIR_SCORE_H

#include "IMP/PairScore.h"
#include "IMP/UnaryFunction.h"
#include "IMP/Pointer.h"
#include "IMP/ParticleRefiner.h"
#include "misc_exports.h"

namespace IMP
{

namespace misc
{

//! Refine the input particles at most once with the ParticleRefiner.
/** Each passed particle is refined once before the resulting pairs
    have the pair score called on them.

    \ingroup pairscore
 */
class IMPMISCEXPORT RefineOncePairScore : public PairScore
{
  Pointer<ParticleRefiner> r_;
  Pointer<PairScore> f_;

public:
  /** \param[in] r The ParticleRefiner to call on each particle
      \param[in] f The pair score to apply to the generated pairs
   */
  RefineOncePairScore(ParticleRefiner *r, PairScore *f);
  virtual ~RefineOncePairScore(){}
  virtual Float evaluate(Particle *a, Particle *b,
                         DerivativeAccumulator *da) const;
  virtual void show(std::ostream &out=std::cout) const;
};

} // namespace misc

} // namespace IMP

#endif  /* __IMPMISC_REFINE_ONCE_PAIR_SCORE_H */
