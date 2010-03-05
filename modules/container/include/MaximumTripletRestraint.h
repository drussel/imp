/**
 *  \file MinimumTripletRestraint.h
 *  \brief Score based on the minimum score over a set of Triplets
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_MAXIMUM_TRIPLET_RESTRAINT_H
#define IMPCONTAINER_MAXIMUM_TRIPLET_RESTRAINT_H

#include "container_config.h"
#include <IMP/Restraint.h>
#include <IMP/TripletScore.h>
#include <IMP/TripletContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Score based on the maximum triplet over a set of Triplets
/** The score is evaluated for each of the Triplets in the container
    and the value of the maximum n scores is used. That is,
    if n is 1, the value of the restraint is the value of the maximum
    score over the container.
 */
class IMPCONTAINEREXPORT MaximumTripletRestraint
: public Restraint
{
  IMP::internal::OwnerPointer<TripletScore> f_;
  IMP::internal::OwnerPointer<TripletContainer> c_;
  unsigned int n_;
public:
  /** n is the number of maximum scores to use.
   */
  MaximumTripletRestraint(TripletScore *f,
                                 TripletContainer *c,
                                 unsigned int n=1,
                                 std::string name
                                 ="MaximumTripletRestraint %1%");

  IMP_RESTRAINT(MaximumTripletRestraint);

  //! Set the number of lowest scores to use.
  void set_n(unsigned int n) { n_=n;}
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_MAXIMUM_TRIPLET_RESTRAINT_H */
