/**
 *  \file PredicateTripletsRestraint.cpp   \brief Container for triplet.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/PredicateTripletsRestraint.h"

IMPCONTAINER_BEGIN_NAMESPACE

PredicateTripletsRestraint
::PredicateTripletsRestraint(TripletPredicate *pred,
                               TripletContainer *input,
                               std::string name):
    Restraint(name), predicate_(pred), input_(input), updated_(false),
    error_on_unknown_(true){}

double
PredicateTripletsRestraint
::unprotected_evaluate(DerivativeAccumulator *da) const {
  update_lists_if_necessary();
  double ret=0;
  for (unsigned int i=0; i< restraints_.size(); ++i) {
    ret+=restraints_[i]->unprotected_evaluate(da);
  }
  return ret;
}

ParticlesTemp PredicateTripletsRestraint
::get_input_particles() const {
  // not correct, but correct is complicated
  return input_->get_contained_particles();
}
ContainersTemp PredicateTripletsRestraint
::get_input_containers() const {
  // List containers don't do anything interesting
  return ContainersTemp(1, input_);
}

Restraints PredicateTripletsRestraint
::do_create_current_decomposition() const {
  update_lists_if_necessary();
  return restraints_;
}

void PredicateTripletsRestraint
::assign_pair(const ParticleIndexTriplet& index) const {
  int bin=predicate_->get_value_index(get_model(), index);
  if (containers_.find(bin) == containers_.end()) {
    if (unknown_container_) {
      unknown_container_->add_particle_triplet(index);
    } else if (error_on_unknown_) {
      IMP_THROW("Invalid predicate value of " << bin
                << " encounted for " << index,
                ValueException);
    }
  } else {
    containers_.find(bin)->second->add_particle_triplet(index);
  }
}
void PredicateTripletsRestraint
::update_lists_if_necessary() const {
  if (updated_ && !input_->get_contents_changed()) return;
  updated_=true;
  if (unknown_container_) {
    unknown_container_->clear_particle_triplets();
  }
  for (Map::const_iterator it= containers_.begin();
       it != containers_.end(); ++it) {
    it->second->clear_particle_triplets();
  }
  IMP_FOREACH_TRIPLET_INDEX(input_, {
      assign_pair(_1);
    });
}

void PredicateTripletsRestraint::do_show(std::ostream &) const {
}
IMPCONTAINER_END_NAMESPACE