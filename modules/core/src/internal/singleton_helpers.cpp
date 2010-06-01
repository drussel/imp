/**
 *  \file ListSingletonContainer.cpp   \brief A list of Particles.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/core/internal/singleton_helpers.h>
#include <IMP/SingletonModifier.h>
#include <IMP/SingletonScore.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE
void ListLikeSingletonContainer::apply(const SingletonModifier *sm) {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(sm);
  sm->apply(data_);
}
void ListLikeSingletonContainer::apply(const SingletonModifier *sm,
                                       DerivativeAccumulator &da) {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(sm);
  sm->apply(data_, da);
}
double ListLikeSingletonContainer
::evaluate(const SingletonScore *s,
           DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate(data_, da);
}
double ListLikeSingletonContainer
::evaluate_subset(const SingletonScore *s,
           DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate_subset(data_, da);
}
double ListLikeSingletonContainer
::evaluate_change(const SingletonScore *s,
                  DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate_change(data_, da);
}
double ListLikeSingletonContainer
::evaluate_prechange(const SingletonScore *s,
                     DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate_prechange(data_, da);
}
unsigned int ListLikeSingletonContainer
::get_number_of_particles() const {
  IMP_CHECK_OBJECT(this);
  IMP_USAGE_CHECK(get_is_up_to_date(),
                  "Attempting to use container "
                  << get_name() << " that is not up to date."
                  << " Call Model::evaluate() first, or something is broken.");
  return data_.size();
}
bool ListLikeSingletonContainer
::get_contains_particle(Particle* vt) const {
  IMP_CHECK_OBJECT(this);
  return std::binary_search(data_.begin(), data_.end(), vt);
}

Particle* ListLikeSingletonContainer
::get_particle(unsigned int i) const {
  IMP_CHECK_OBJECT(this);
  return data_[i];
}

void ListLikeSingletonContainer
::do_show(std::ostream &out) const {
  out << "contains " << data_.size() << std::endl;
}


ParticlesTemp ListLikeSingletonContainer
::get_contained_particles() const {
  return IMP::internal::flatten(data_);
}

bool ListLikeSingletonContainer
::get_contained_particles_changed() const {
  return !get_added()->data_.empty() || !get_removed()->data_.empty();
}


IMPCORE_END_INTERNAL_NAMESPACE
