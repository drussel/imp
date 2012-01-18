/**
 *  \file CLASSNAMEsPredicateRestraint.h
 *  \brief Apply a CLASSNAMEScore to each CLASSNAME in a list.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCONTAINER_HEADERNAMES_PREDICATE_RESTRAINT_H
#define IMPCONTAINER_HEADERNAMES_PREDICATE_RESTRAINT_H

#include "container_config.h"

#include <IMP/core/internal/CoreCLASSNAMEsRestraint.h>
#include "ListCLASSNAMEContainer.h"
#include <IMP/compatibility/map.h>
#include <IMP/CLASSNAMEPredicate.h>
#include "generic.h"

#include <iostream>

IMPCONTAINER_BEGIN_NAMESPACE

//! Applies a CLASSNAMEScore to each CLASSNAME in a list based on a predicate
/** This restraint uses a passed predicate to choose which score to apply
    to each tuple in the input container. The selections are cached, making it
    substantially faster than using a core::TypedPairScore. */
class IMPCONTAINEREXPORT CLASSNAMEsPredicateRestraint :
public Restraint
{
  OwnerPointer<CLASSNAMEPredicate> predicate_;
  OwnerPointer<CLASSNAMEContainer> input_;
  typedef compatibility::map<unsigned int, Pointer<ListCLASSNAMEContainer> >
      Map;
  Map containers_;
  Pointer<ListCLASSNAMEContainer> unknown_container_;
  Restraints restraints_;
  mutable bool updated_;
  bool error_on_unknown_;
  void update_lists_if_necessary() const;
  void assign_pair(PASSINDEXTYPE index) const;
public:
  CLASSNAMEsPredicateRestraint(CLASSNAMEPredicate *pred,
                      CLASSNAMEContainer *input,
                      std::string name="CLASSNAMEsPredicateRestraint %1%");

  /** This version uses the container::create_restraint() function and so
      is more efficient than the non-template version.*/
  template <class Score>
      void set_score(int predicate_value, Score *score) {
    IMP_USAGE_CHECK(get_is_part_of_model(),
                    "You must add this restraint to the model"
                    << " first, sorry, this can be fixed.");
    IMP_NEW(ListCLASSNAMEContainer, c, (get_model()));
    restraints_.push_back(container::create_restraint(score, c.get()));
    restraints_.back()->set_model(get_model());
    containers_[predicate_value]=c;
  }
template <class Score>
      void set_unknown_score( Score *score) {
  // make sure it gets cleaned up if it is a temporary
  Pointer<Score> pscore(score);
    IMP_USAGE_CHECK(get_is_part_of_model(),
                    "You must add this restraint to the model"
                    << " first, sorry, this can be fixed.");
    IMP_NEW(ListCLASSNAMEContainer, c, (get_model()));
    restraints_.push_back(container::create_restraint(score, c.get()));
    restraints_.back()->set_model(get_model());
    unknown_container_=c;
  }
  /** By default, it is an error if the predicate returns a value that is
      not known. If this is false, then they are silently skipped.
  */
  void set_is_complete(bool tf) {
    error_on_unknown_=tf;
  }

  IMP_RESTRAINT(CLASSNAMEsPredicateRestraint);
private:
  Restraints do_create_current_decomposition() const;
};

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_HEADERNAMES_PREDICATE_RESTRAINT_H */
