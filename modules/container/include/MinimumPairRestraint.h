/**
 *  \file MinimumPairRestraint.h
 *  \brief Score based on the minimum score over a set of Pairs
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCONTAINER_MINIMUM_PAIR_RESTRAINT_H
#define IMPCONTAINER_MINIMUM_PAIR_RESTRAINT_H

#include "config.h"
#include <IMP/Restraint.h>
#include <IMP/PairScore.h>
#include <IMP/PairContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Score based on the minimum pair over a set of Pairs
/** The score is evaluated for each of the Pairs in the container
    and the value of the minimum n scores is used. That is,
    if n is 1, the value of the restraint is the value of the minimum
    score over the container.
 */
class IMPCONTAINEREXPORT MinimumPairRestraint
: public Restraint
{
  IMP::internal::OwnerPointer<PairScore> f_;
  IMP::internal::OwnerPointer<PairContainer> c_;
  unsigned int n_;
public:
  /** n is the number of minimum scores to use.
   */
  MinimumPairRestraint(PairScore *f,
                                 PairContainer *c,
                                 unsigned int n=1,
                                 std::string name
                                 ="MinimumPairRestraint %1%");

  IMP_RESTRAINT(MinimumPairRestraint,
                get_module_version_info());

  //! Set the number of lowest scores to use.
  void set_n(unsigned int n) { n_=n;}
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_MINIMUM_PAIR_RESTRAINT_H */
