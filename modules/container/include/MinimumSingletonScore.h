/**
 *  \file MinimumSingletonScore.h    \brief Define SingletonScore.
 *
 *  This file is generated by a script (tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_MINIMUM_SINGLETON_SCORE_H
#define IMPCONTAINER_MINIMUM_SINGLETON_SCORE_H

#include "container_config.h"
#include <IMP/SingletonScore.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Evaluate the minimum n singleton scores of the passed set of SingletonScores
/** Each of the set of SingletonScores is evaluated and the sum of the
    minimum n is returned.
*/
class IMPCONTAINEREXPORT MinimumSingletonScore : public SingletonScore
{
  SingletonScores scores_;
  unsigned int n_;
public:
  MinimumSingletonScore(const SingletonScoresTemp &scores,
                       unsigned int n=1,
                       std::string name="SingletonScore %1%");
  IMP_SINGLETON_SCORE(MinimumSingletonScore);
};

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_MINIMUM_SINGLETON_SCORE_H */
