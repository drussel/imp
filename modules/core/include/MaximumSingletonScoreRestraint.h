/**
 *  \file MaximumSingletonScoreRestraint.h
 *  \brief Score based on the maximum score over a set of Singletons
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_MAXIMUM_SINGLETON_SCORE_RESTRAINT_H
#define IMPCORE_MAXIMUM_SINGLETON_SCORE_RESTRAINT_H

#include "config.h"
#include <IMP/Restraint.h>
#include <IMP/SingletonScore.h>
#include <IMP/SingletonContainer.h>


IMPCORE_BEGIN_NAMESPACE

//! Score based on the maximum over a set of Singletons
/** The score is evaluated for each of the Singletons in the container
 and the value of the minumum n scores is used. That is,
 if n is 1, the value of the restraint is the value of the lowest
 score over the container.
 \see MinimumSingletonScoreRestraint
 */
class IMPCOREEXPORT MaximumSingletonScoreRestraint
: public Restraint
{
  Pointer<SingletonScore> f_;
  Pointer<SingletonContainer> c_;
  unsigned int n_;
public:
  /** n is the number of minimumal scores to use.
   */
  MaximumSingletonScoreRestraint(SingletonScore *f,
                                 SingletonContainer *c,
                                 unsigned int n=1,
                                 std::string name
                                 ="MaximumSingletonScoreRestraint %1%");

  IMP_RESTRAINT(MaximumSingletonScoreRestraint, get_module_version_info());

  //! Set the number of lowest scores to use.
  void set_n(unsigned int n) { n_=n;}
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_MAXIMUM_SINGLETON_SCORE_RESTRAINT_H */
