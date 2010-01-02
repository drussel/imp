/**
 *  \file ListPairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/core/internal/pair_helpers.h>
#include <IMP/PairModifier.h>
#include <IMP/PairScore.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE
void ListLikePairContainer::apply(const PairModifier *sm) {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(sm);
  sm->apply(data_);
}
void ListLikePairContainer::apply(const PairModifier *sm,
                                       DerivativeAccumulator &da) {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(sm);
  sm->apply(data_, da);
}
double ListLikePairContainer
::evaluate(const PairScore *s,
           DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate(data_, da);
}
double ListLikePairContainer
::evaluate_change(const PairScore *s,
                  DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate_change(data_, da);
}
double ListLikePairContainer
::evaluate_prechange(const PairScore *s,
                     DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate_prechange(data_, da);
}
unsigned int ListLikePairContainer
::get_number_of_particle_pairs() const {
  IMP_CHECK_OBJECT(this);
  return data_.size();
}
bool ListLikePairContainer
::get_contains_particle_pair(const ParticlePair& vt) const {
  IMP_CHECK_OBJECT(this);
  return std::binary_search(data_.begin(), data_.end(), vt);
}
ParticlePairsTemp ListLikePairContainer
::get_particle_pairs() const {
  IMP_CHECK_OBJECT(this);
  return data_;
}

ParticlePair ListLikePairContainer
::get_particle_pair(unsigned int i) const {
  IMP_CHECK_OBJECT(this);
  return data_[i];
}

VersionInfo ListLikePairContainer
::get_version_info() const {
  return get_module_version_info();
}
void ListLikePairContainer
::show(std::ostream &out) const {
  out << "ListLikeContainer on " << data_.size() << std::endl;
}


ParticlesTemp ListLikePairContainer
::get_contained_particles() const {
  return IMP::internal::flatten(data_);
}

bool ListLikePairContainer
::get_contained_particles_changed() const {
  return !get_added()->data_.empty() || !get_removed()->data_.empty();
}


IMPCORE_END_INTERNAL_NAMESPACE
