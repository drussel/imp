/**
 *  \file PredicateSingletonsRestraint.cpp   \brief Container for singleton.
 *
 *  WARNING This file was generated from PredicateNAMEsRestraint.cc
 *  in tools/maintenance/container_templates/container
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/PredicateSingletonsRestraint.h"
#include <IMP/singleton_macros.h>

IMPCONTAINER_BEGIN_NAMESPACE

PredicateSingletonsRestraint
::PredicateSingletonsRestraint(SingletonPredicate *pred,
                               SingletonContainerAdaptor input,
                               std::string name):
  Restraint(input->get_model(), name),
  predicate_(pred), input_(input), updated_(false),
  error_on_unknown_(true){}

void
PredicateSingletonsRestraint
::do_add_score_and_derivatives(ScoreAccumulator sa) const {
  update_lists_if_necessary();
  for (unsigned int i=0; i< restraints_.size(); ++i) {
    restraints_[i]->add_score_and_derivatives(sa);
  }
#pragma omp taskwait
}

double
PredicateSingletonsRestraint
::get_last_score() const {
  double ret=0;
  for (unsigned int i=0; i< restraints_.size(); ++i) {
    ret+=restraints_[i]->get_last_score();
  }
  return ret;
}

ModelObjectsTemp PredicateSingletonsRestraint
::do_get_inputs() const {
  ModelObjectsTemp ret;
  ret+= predicate_->get_inputs(get_model(),
                               input_->get_all_possible_indexes());
 for (unsigned int i=0; i< restraints_.size(); ++i) {
    ret+=restraints_[i]->get_inputs();
  }
  ret.push_back(input_);
  return ret;
}

Restraints PredicateSingletonsRestraint
::do_create_current_decomposition() const {
  Restraints ret;
  for (unsigned int i=0; i< restraints_.size(); ++i) {
    base::Pointer<Restraint> r=restraints_[i]->create_current_decomposition();
    if (r) {
      RestraintSet *rs= dynamic_cast<RestraintSet*>(r.get());
      if (rs) {
        ret+=rs->get_restraints();
        // suppress warning
        rs->set_was_used(true);
      } else {
        ret.push_back(r);
      }
    }
  }
  return ret;
}

bool PredicateSingletonsRestraint
::assign_pair(ParticleIndex index) const {
  int bin=predicate_->get_value_index(get_model(), index);
  Map::const_iterator it= containers_.find(bin);
  if (it == containers_.end()) {
    if (unknown_container_) {
      unknown_container_->add(index);
      return true;
    } else if (error_on_unknown_) {
      IMP_THROW("Invalid predicate value of " << bin
                << " encounted for " << index,
                ValueException);
      return true;
    } else {
      return false;
    }
  } else {
    it->second->add(index);
    return true;
  }
}
void PredicateSingletonsRestraint
::update_lists_if_necessary() const {
  if (updated_ && !input_->get_is_changed()) return;
  updated_=true;
  if (unknown_container_) {
    unknown_container_->clear();
  }
  for (Map::const_iterator it= containers_.begin();
       it != containers_.end(); ++it) {
    it->second->clear();
  }
  int dropped=0;
  IMP_FOREACH_SINGLETON_INDEX(input_, {
      bool added=assign_pair(_1);
      if (!added) ++dropped;
    });
  IMP_IF_CHECK(USAGE_AND_INTERNAL) {
    unsigned int total=dropped;
    for (Map::const_iterator it= containers_.begin();
         it != containers_.end(); ++it) {
      total+=it->second->get_number();
    }
    if (unknown_container_) {
      total+= unknown_container_->get_number();
    } else {
      total+= dropped;
    }
    IMP_INTERNAL_CHECK(input_->get_number()==total,
                       "Wrong number of particles "
                       << total << "!=" << input_->get_number());
  }
}

void PredicateSingletonsRestraint::do_show(std::ostream &) const {
}
IMPCONTAINER_END_NAMESPACE
