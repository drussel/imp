/**
 *  \file ListPairContainer.cpp   \brief A list of ParticlePairsTemp.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/core/internal/pair_helpers.h>
#include <IMP/PairModifier.h>
#include <IMP/PairScore.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE

unsigned int ListLikePairContainer
::get_number_of_particle_pairs() const {
  IMP_CHECK_OBJECT(this);
  IMP_USAGE_CHECK(get_is_up_to_date(),
                  "Attempting to use container "
                  << get_name() << " that is not up to date."
                  << " Call Model::evaluate() first, or something is broken.");
  return data_.size();
}
bool ListLikePairContainer
::get_contains_particle_pair(const ParticlePair& vt) const {
  IMP_CHECK_OBJECT(this);
  update_index();
  return index_.find(vt)!= index_.end();
}

ParticlePair ListLikePairContainer
::get_particle_pair(unsigned int i) const {
  IMP_CHECK_OBJECT(this);
  return data_[i];
}

void ListLikePairContainer
::do_show(std::ostream &out) const {
  out << "contains " << data_.size() << std::endl;
}


ParticlesTemp ListLikePairContainer
::get_contained_particles() const {
  return IMP::internal::flatten(data_);
}

bool ListLikePairContainer
::get_contained_particles_changed() const {
  return !get_added()->data_.empty() || !get_removed()->data_.empty();
}

PairContainerPair ListLikePairContainer
::get_added_and_removed_containers() const {
  return PairContainerPair(new ListLikePairContainer(),
                                new ListLikePairContainer());
}

IMPCORE_END_INTERNAL_NAMESPACE
