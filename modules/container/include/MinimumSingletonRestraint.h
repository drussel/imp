/**
 *  \file MinimumSingletonRestraint.h
 *  \brief Score based on the minimum score over a set of Singletons
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_MINIMUM_SINGLETON_RESTRAINT_H
#define IMPCONTAINER_MINIMUM_SINGLETON_RESTRAINT_H

#include "container_config.h"
#include <IMP/Restraint.h>
#include <IMP/SingletonScore.h>
#include <IMP/SingletonContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Score based on the min or max SingletonScore over a set
/** The score is evaluated for each of the VALUETYPE in the container
    and the value of the min or max n scores is used. That is,
    if n is 1, the value of the restraint is the value of the min or max
    score over the container.
 */
class IMPCONTAINEREXPORT MinimumSingletonRestraint
: public Restraint
{
  IMP::internal::OwnerPointer<SingletonScore> f_;
  IMP::internal::OwnerPointer<SingletonContainer> c_;
  unsigned int n_;
public:
  /** n is the number of LCMinimum scores to use.
   */
  MinimumSingletonRestraint(SingletonScore *f,
                                 SingletonContainer *c,
                                 unsigned int n=1,
                                 std::string name
                                 ="MinimumSingletonRestraint %1%");

  IMP_RESTRAINT(MinimumSingletonRestraint);

  //! Set the number of lowest scores to use.
  void set_n(unsigned int n) { n_=n;}
};

IMP_OBJECTS(MinimumSingletonRestraint,MinimumSingletonRestraints);


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_MINIMUM_SINGLETON_RESTRAINT_H */
