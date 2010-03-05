/**
 *  \file ClosePairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. Close rights reserved.
 *
 */

#include "IMP/core/internal/CoreClosePairContainer.h"
#include <IMP/core/internal/DifferenceSingletonContainer.h>
#include <IMP/core/BoxSweepClosePairsFinder.h>
#include <IMP/core/GridClosePairsFinder.h>
#include <IMP/core/internal/CoreListPairContainer.h>
#include <IMP/core/internal/pair_helpers.h>
#include <IMP/core/internal/close_pairs_helpers.h>
#include <IMP/PairModifier.h>
#include <algorithm>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

IMP_LIST_IMPL(CoreClosePairContainer,
             PairFilter,
             pair_filter,
             PairFilter*,
             PairFilters, obj->set_was_used(true);,,)


CoreClosePairContainer::CoreClosePairContainer(SingletonContainer *c,
                                                 Model *m, double distance,
                                                 ClosePairsFinder *cpf,
                                       double slack):
  internal::ListLikePairContainer("ClosePairContainer") {
 initialize(c, distance, slack, m,
             cpf);
}

void CoreClosePairContainer::initialize(SingletonContainer *c, double distance,
                                         double slack, Model *m,
                                         ClosePairsFinder *cpf) {
  set_model(m);
  slack_=slack;
  distance_=distance;
  c_=c;
  cpf_=cpf;
  cpf_->set_distance(distance_+2*slack_);
  first_call_=true;
  moved_= cpf_->get_moved_singleton_container(c_, m, slack_);
}

IMP_ACTIVE_CONTAINER_DEF(CoreClosePairContainer)


ContainersTemp CoreClosePairContainer
::get_state_input_containers() const {
  return cpf_->get_input_containers(c_);
}


ParticlesTemp CoreClosePairContainer::get_state_input_particles() const {
  ParticlesTemp ret(cpf_->get_input_particles(c_));
  if (get_number_of_pair_filters() >0) {
    ParticlePairsTemp all_pairs;
    for (unsigned int i=0; i< ret.size(); ++i) {
      for (unsigned int j=0; j< i; ++j) {
        all_pairs.push_back(ParticlePair(ret[i], ret[j]));
      }
    }
    for (PairFilterConstIterator it= pair_filters_begin();
         it != pair_filters_end(); ++it) {
      for (unsigned int i=0; i< all_pairs.size(); ++i) {
        ParticlesTemp cur= (*it)->get_input_particles(all_pairs[i]);
      ret.insert(ret.end(), cur.begin(), cur.end());
      }
    }
  }
  return ret;
}


void CoreClosePairContainer::do_before_evaluate() {
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(c_);
  IMP_CHECK_OBJECT(cpf_);
  if (first_call_) {
    IMP_LOG(TERSE, "Handling first call of ClosePairContainer." << std::endl);
    ParticlePairsTemp c= cpf_->get_close_pairs(c_);
    internal::filter_close_pairs(this, c);
    moved_->reset();
    IMP_LOG(TERSE, "Found " << c.size() << " pairs." << std::endl);
    update_list(c);
    first_call_=false;
  } else {
    // hack until we have the dependency graph
    moved_->update();
    if (moved_->get_number_of_particles() != 0) {
      if (moved_->get_particles().size() < c_->get_number_of_particles()*.1) {
        IMP_LOG(TERSE, "Handling incremental update of ClosePairContainer."
                << std::endl);
        ParticlePairsTemp ret= cpf_->get_close_pairs(c_, moved_);
        internal::filter_close_pairs(this, ret);
        internal::filter_same(ret);
        IMP_LOG(TERSE, "Found " << ret.size() << " pairs." << std::endl);
        add_to_list(ret);
        moved_->reset_moved();
      } else {
        IMP_LOG(TERSE, "Handling full update of ClosePairContainer."
                << std::endl);
        ParticlePairsTemp ret= cpf_->get_close_pairs(c_);
        internal::filter_close_pairs(this, ret);
        IMP_LOG(TERSE, "Found " << ret.size() << " pairs." << std::endl);
        update_list(ret);
        moved_->reset();
      }
    } else {
      IMP_LOG(TERSE, "No particles moved more than " << slack_ << std::endl);
    }
  }
}


void CoreClosePairContainer::do_after_evaluate() {
  internal::ListLikePairContainer::do_after_evaluate();
}


void CoreClosePairContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "container " << *c_ << std::endl;
}

ContainersTemp CoreClosePairContainer::get_input_containers() const {
  ContainersTemp ret= cpf_->get_input_containers(c_);
  ret.push_back(moved_);
  return ret;
}

IMPCORE_END_INTERNAL_NAMESPACE
