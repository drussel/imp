/**
 *  \file AllPairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/ConsecutivePairContainer.h"
#include <IMP/PairModifier.h>
#include <IMP/internal/container_helpers.h>
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE
namespace {
  // global counter to differentiate keys of overlapping containers
  // TODO: why not a static class variable?
  unsigned int key_count=0;
}
ConsecutivePairContainer::ConsecutivePairContainer(const ParticlesTemp &ps,
                                                   std::string name):
  PairContainer(ps[0]->get_model(),name),
  ps_(IMP::internal::get_index(ps)) {
  init();
}

// add key of this container as attribute to all particles
// if there might be ovrlaps - create a different keys for each instance
void ConsecutivePairContainer::init(){
  std::ostringstream oss;
  oss << "CPC cache " << key_count;
  ++key_count;
  key_= IntKey(oss.str());
  for (unsigned int i= 0; i < ps_.size(); ++i) {
    IMP_USAGE_CHECK(!get_model()->get_has_attribute(key_, ps_[i]),
                    "You must create containers before reading in the "
                    << "saved model: "
                    << get_model()->get_particle(ps_[i])->get_name());
    get_model()->add_attribute(key_, ps_[i], i);
  }
}

void ConsecutivePairContainer::do_before_evaluate() {}

ParticlesTemp ConsecutivePairContainer::get_input_particles() const {
  return ParticlesTemp();
}
ContainersTemp ConsecutivePairContainer::get_input_containers() const {
  return ContainersTemp();
}


bool
ConsecutivePairContainer::get_is_changed() const {
  return false;
}

ParticleIndexPairs ConsecutivePairContainer::get_indexes() const {
  ParticleIndexPairs ret(ps_.size()-1);
  for (unsigned int i=1; i< ps_.size(); ++i) {
    ret[i-1]= ParticleIndexPair(ps_[i-1], ps_[i]);
  }
  return ret;
}

ParticleIndexPairs ConsecutivePairContainer::get_all_possible_indexes() const {
  return get_indexes();
}


void ConsecutivePairContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "num particles: " << ps_.size() << std::endl;
}

ParticlesTemp ConsecutivePairContainer::get_all_possible_particles() const {
  return IMP::internal::get_particle(get_model(), ps_);
}


ConsecutivePairFilter::ConsecutivePairFilter(ConsecutivePairContainer *cpc):
PairPredicate("ConsecutivePairFilter %1%"), cpc_(cpc) {}

ExclusiveConsecutivePairContainer
::ExclusiveConsecutivePairContainer(const ParticlesTemp &ps,
                                    std::string name):
  PairContainer(ps[0]->get_model(),name),
  ps_(IMP::internal::get_index(ps)) {
  init();
}

// add key of this container as attribute to all particles
// if there might be ovrlaps - create a different keys for each instance
void ExclusiveConsecutivePairContainer::init(){
  for (unsigned int i= 0; i < ps_.size(); ++i) {
    IMP_USAGE_CHECK(!get_model()->get_has_attribute(get_exclusive_key(),
                                                    ps_[i]),
                    "You must create containers before reading in the "
                    << "saved model: "
                    << get_model()->get_particle(ps_[i])->get_name());
    get_model()->add_attribute(get_exclusive_key(), ps_[i], i);
    get_model()->add_attribute(get_exclusive_object_key(), ps_[i], this);
  }
}

void ExclusiveConsecutivePairContainer::do_before_evaluate() {}

ParticlesTemp ExclusiveConsecutivePairContainer::get_input_particles() const {
  return ParticlesTemp();
}
ContainersTemp ExclusiveConsecutivePairContainer::get_input_containers() const {
  return ContainersTemp();
}


bool
ExclusiveConsecutivePairContainer::get_is_changed() const {
  return false;
}

ParticleIndexPairs ExclusiveConsecutivePairContainer::get_indexes() const {
  ParticleIndexPairs ret(ps_.size()-1);
  for (unsigned int i=1; i< ps_.size(); ++i) {
    ret[i-1]= ParticleIndexPair(ps_[i-1], ps_[i]);
  }
  return ret;
}

ParticleIndexPairs ExclusiveConsecutivePairContainer
::get_all_possible_indexes() const {
  return get_indexes();
}


void ExclusiveConsecutivePairContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "num particles: " << ps_.size() << std::endl;
}

ParticlesTemp ExclusiveConsecutivePairContainer
::get_all_possible_particles() const {
  return IMP::internal::get_particle(get_model(), ps_);
}

IMPCONTAINER_END_NAMESPACE
