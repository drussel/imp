/**
 *  \file ListGroupnameContainer.cpp   \brief A list of Classnames.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include <IMP/core/internal/groupname_helpers.h>
#include <IMP/GroupnameModifier.h>
#include <IMP/GroupnameScore.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE
void ListLikeGroupnameContainer::apply(const GroupnameModifier *sm) {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(sm);
  sm->apply(data_);
}
void ListLikeGroupnameContainer::apply(const GroupnameModifier *sm,
                                       DerivativeAccumulator &da) {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(sm);
  sm->apply(data_, da);
}
double ListLikeGroupnameContainer
::evaluate(const GroupnameScore *s,
           DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate(data_, da);
}
double ListLikeGroupnameContainer
::evaluate_change(const GroupnameScore *s,
                  DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate_change(data_, da);
}
double ListLikeGroupnameContainer
::evaluate_prechange(const GroupnameScore *s,
                     DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate_prechange(data_, da);
}
unsigned int ListLikeGroupnameContainer
::get_number_of_classnames() const {
  IMP_CHECK_OBJECT(this);
  return data_.size();
}
bool ListLikeGroupnameContainer
::get_contains_classname(PassValue vt) const {
  IMP_CHECK_OBJECT(this);
  return std::binary_search(data_.begin(), data_.end(), vt);
}

Value ListLikeGroupnameContainer
::get_classname(unsigned int i) const {
  IMP_CHECK_OBJECT(this);
  return data_[i];
}

void ListLikeGroupnameContainer
::do_show(std::ostream &out) const {
  out << "contains " << data_.size() << std::endl;
}


ParticlesTemp ListLikeGroupnameContainer
::get_contained_particles() const {
  return IMP::internal::flatten(data_);
}

bool ListLikeGroupnameContainer
::get_contained_particles_changed() const {
  return !get_added()->data_.empty() || !get_removed()->data_.empty();
}


IMPCORE_END_INTERNAL_NAMESPACE
