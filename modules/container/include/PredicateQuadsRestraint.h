/**
 *  \file IMP/container/PredicateQuadsRestraint.h
 *  \brief Apply a QuadScore to each Quad in a list.
 *
 *  WARNING This file was generated from PredicateNAMEsRestraint.hpp
 *  in tools/maintenance/container_templates/container
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCONTAINER_PREDICATE_QUADS_RESTRAINT_H
#define IMPCONTAINER_PREDICATE_QUADS_RESTRAINT_H

#include <IMP/container/container_config.h>

#include <IMP/internal/InternalDynamicListQuadContainer.h>
#include <IMP/compatibility/map.h>
#include <IMP/QuadPredicate.h>
#include <IMP/restraint_macros.h>
#include "generic.h"

#include <iostream>

IMPCONTAINER_BEGIN_NAMESPACE

//! Applies a QuadScore to each Quad in a list based on a predicate
/** This restraint uses a passed predicate to choose which score to apply
    to each tuple in the input container. The selections are cached, making it
    substantially faster than using a core::TypedPairScore.

    \note The ordering of particles within a tuple may vary depending on the
    input container used. You may need to call set_score() with several
    different predicate values for different orderings.
*/
class IMPCONTAINEREXPORT PredicateQuadsRestraint :
public Restraint
{
  base::OwnerPointer<QuadPredicate> predicate_;
  base::OwnerPointer<QuadContainer> input_;
  typedef IMP::internal::InternalDynamicListQuadContainer List;
  typedef compatibility::map<unsigned int, base::Pointer<List> >
      Map;
  Map containers_;
  base::Pointer<List> unknown_container_;
  Restraints restraints_;
  mutable bool updated_;
  bool error_on_unknown_;
  void update_lists_if_necessary() const;
  bool assign_pair(const ParticleIndexQuad& index) const;
public:
  PredicateQuadsRestraint(QuadPredicate *pred,
                      QuadContainerAdaptor input,
                      std::string name="PredicateQuadsRestraint %1%");

  /** Apply the passed score to all pairs whose predicate values match
      the passed value.

      This version uses the container::create_restraint() function and so
      is more efficient than the non-template version.*/
  template <class Score>
  void set_score(int predicate_value, Score *score) {
    IMP_USAGE_CHECK(get_is_part_of_model(),
                    "You must add this restraint to the model"
                    << " first, sorry, this can be fixed.");
    IMP_NEW(List, c, (input_,
                      score->get_name()+" input"));
    restraints_.push_back(container::create_restraint(score, c.get()));
    restraints_.back()->set_model(get_model());
    restraints_.back()->set_was_used(true);
    containers_[predicate_value]=c;
  }

  /** Apply this score to any pair whose predicate value does not match
      one passed to set_score().*/
  template <class Score>
  void set_unknown_score( Score *score) {
  // make sure it gets cleaned up if it is a temporary
    base::Pointer<Score> pscore(score);
    IMP_USAGE_CHECK(get_is_part_of_model(),
                    "You must add this restraint to the model"
                    << " first, sorry, this can be fixed.");
    IMP_NEW(List, c, (input_,
                      score->get_name()+" input"));
    restraints_.push_back(container::create_restraint(score, c.get()));
    restraints_.back()->set_model(get_model());
    unknown_container_=c;
  }
#ifndef IMP_DOXYGEN
  void set_score(int predicate_value, QuadScore *score) {
    set_score<QuadScore>(predicate_value, score);
  }
  void set_unknown_score(QuadScore *score) {
    set_unknown_score<QuadScore>(score);
  }
#endif
  /** By default, it is an error if the predicate returns a value that is
      not known. If this is false, then they are silently skipped.
  */
  void set_is_complete(bool tf) {
    error_on_unknown_=tf;
  }

  IMP_IMPLEMENT(double get_last_score() const);

  /** return the indexes of all particles for  a given predicate value.*/
  ParticleIndexQuads get_indexes(int predicate_value) const {
    return containers_.find(predicate_value)->second
      ->get_indexes();
  }

  IMP_RESTRAINT_ACCUMULATOR(PredicateQuadsRestraint);
private:
  Restraints do_create_current_decomposition() const;
};

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_PREDICATE_QUADS_RESTRAINT_H */
