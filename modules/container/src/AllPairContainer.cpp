/**
 *  \file AllPairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/AllPairContainer.h"
#include <IMP/container/AllBipartitePairContainer.h>
#include <IMP/container/PairContainerSet.h>
#include <IMP/PairModifier.h>
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE

AllPairContainer::AllPairContainer(SingletonContainerAdaptor c,
                                   std::string name):
  PairContainer(c->get_model(), name),
  c_(c){
}

ParticleIndexPairs
AllPairContainer::get_indexes() const {
  ParticleIndexes ia= c_->get_indexes();
  ParticleIndexPairs ret; ret.reserve(ia.size()*(ia.size()-1)/2);
  for (unsigned int i=0; i< ia.size(); ++i) {
    for (unsigned int j=0; j< i; ++j) {
      ret.push_back(ParticleIndexPair(ia[i], ia[j]));
    }
  }
  return ret;
}

ParticleIndexPairs
AllPairContainer::get_range_indexes() const {
  ParticleIndexes ia= c_->get_range_indexes();
  ParticleIndexPairs ret; ret.reserve(ia.size()*(ia.size()-1)/2);
  for (unsigned int i=0; i< ia.size(); ++i) {
    for (unsigned int j=0; j< i; ++j) {
      ret.push_back(ParticleIndexPair(ia[i], ia[j]));
    }
  }
  return ret;
}

void AllPairContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "container " << *c_ << std::endl;
}

ParticleIndexes AllPairContainer::get_all_possible_indexes() const {
  return c_->get_all_possible_indexes();
}

ParticlesTemp AllPairContainer::get_input_particles() const {
  return ParticlesTemp();
}
ContainersTemp AllPairContainer::get_input_containers() const {
  return ContainersTemp(1, c_);
}
void AllPairContainer::do_before_evaluate() {
  set_is_changed(c_->get_is_changed());
}
IMPCONTAINER_END_NAMESPACE
