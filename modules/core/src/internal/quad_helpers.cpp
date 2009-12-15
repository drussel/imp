/**
 *  \file ListQuadContainer.cpp   \brief A list of ParticleQuads.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/core/internal/quad_helpers.h>
#include <IMP/QuadModifier.h>
#include <IMP/QuadScore.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE
void ListLikeQuadContainer::apply(const QuadModifier *sm) {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(sm);
  sm->apply(data_);
}
void ListLikeQuadContainer::apply(const QuadModifier *sm,
                                       DerivativeAccumulator &da) {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(sm);
  sm->apply(data_, da);
}
double ListLikeQuadContainer
::evaluate(const QuadScore *s,
           DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate(data_, da);
}
double ListLikeQuadContainer
::evaluate_change(const QuadScore *s,
                  DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate_change(data_, da);
}
double ListLikeQuadContainer
::evaluate_prechange(const QuadScore *s,
                     DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(this);
  IMP_CHECK_OBJECT(s);
  return s->evaluate_prechange(data_, da);
}
unsigned int ListLikeQuadContainer
::get_number_of_particle_quads() const {
  IMP_CHECK_OBJECT(this);
  return data_.size();
}
bool ListLikeQuadContainer
::get_contains_particle_quad(const ParticleQuad& vt) const {
  IMP_CHECK_OBJECT(this);
  return std::binary_search(data_.begin(), data_.end(), vt);
}
ObjectsTemp ListLikeQuadContainer
::get_input_objects() const { return ObjectsTemp();}

ParticleQuadsTemp ListLikeQuadContainer
::get_particle_quads() const {
  IMP_CHECK_OBJECT(this);
  return data_;
}

ParticleQuad ListLikeQuadContainer
::get_particle_quad(unsigned int i) const {
  IMP_CHECK_OBJECT(this);
  return data_[i];
}

VersionInfo ListLikeQuadContainer
::get_version_info() const {
  return get_module_version_info();
}
void ListLikeQuadContainer
::show(std::ostream &out) const {
  out << "ListLikeContainer on " << data_.size() << std::endl;
}


IMPCORE_END_INTERNAL_NAMESPACE
