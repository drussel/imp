/**
 *  \file MinimumSingletonScore.h    \brief Define SingletonScore.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_MINIMUM_SINGLETON_SCORE_H
#define IMPCORE_MINIMUM_SINGLETON_SCORE_H

#include "config.h"
#include <IMP/SingletonScore.h>

IMPCORE_BEGIN_NAMESPACE

//! Evaluate the minimum n singleton scores of the passed set of SingletonScores
/** Each of the set of SingletonScores is evaluated and the sum of the
    minimum n is returned.
*/
class IMPEXPORT MinimumSingletonScore : public SingletonScore
{
  SingletonScores scores_;
  unsigned int n_;
public:
  MinimumSingletonScore(const SingletonScoresTemp &scores,
                       unsigned int n=1,
                       std::string name="SingletonScore %1%");
  IMP_SINGLETON_SCORE(MinimumSingletonScore, get_module_version_info());
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_MINIMUM_SINGLETON_SCORE_H */
