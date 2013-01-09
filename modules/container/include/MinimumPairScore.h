/**
 *  \file IMP/container/MinimumPairScore.h
 *  \brief Define PairScore.
 *
 *  This file is generated by a script (tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_MINIMUM_PAIR_SCORE_H
#define IMPCONTAINER_MINIMUM_PAIR_SCORE_H

#include <IMP/container/container_config.h>
#include <IMP/PairScore.h>
#include <IMP/pair_macros.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Evaluate the min or max n particle_pair scores of the passed set
/** Each of the set of PairScores is evaluated and the sum of the
    minimum n is returned.
*/
class IMPCONTAINEREXPORT MinimumPairScore : public PairScore
{
  PairScores scores_;
  unsigned int n_;
public:
  MinimumPairScore(const PairScoresTemp &scores,
                       unsigned int n=1,
                       std::string name="PairScore %1%");
  IMP_INDEX_PAIR_SCORE(MinimumPairScore);

  IMP_IMPLEMENT(Restraints
                do_create_current_decomposition(Model *m,
                                                const ParticleIndexPair& vt)
                const IMP_OVERRIDE);
};

IMP_OBJECTS(MinimumPairScore,MinimumPairScores);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_MINIMUM_PAIR_SCORE_H */
