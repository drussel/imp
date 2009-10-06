/**
 *  \file SingletonScoreState.cpp
 *  \brief Use a SingletonModifier applied to a SingletonContainer to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/SingletonScoreState.h"
#include "IMP/internal/container_helpers.h"

IMPCORE_BEGIN_NAMESPACE

SingletonScoreState::SingletonScoreState(SingletonModifier *before,
                                         SingletonModifier *after,
                                         Particle *a,
                                         std::string name):
  ScoreState(name), v_(a){
  if (before) f_=before;
  if (after) af_=after;
}


void SingletonScoreState::do_before_evaluate()
{
  IMP_OBJECT_LOG;
  if (!f_) return;
  IMP_LOG(TERSE, "Begin SingletonsScoreState::update" << std::endl);
  IMP_CHECK_OBJECT(f_);
  IMP::internal::ContainerTraits<Particle>
    ::apply(f_.get(), v_);
  IMP_LOG(TERSE, "End SingletonsScoreState::update" << std::endl);
}

void SingletonScoreState::do_after_evaluate(DerivativeAccumulator *da)
{
  IMP_OBJECT_LOG;
  if (!af_) return;
  IMP_LOG(TERSE, "Begin SingletonsScoreState::after_evaluate" << std::endl);
  IMP_CHECK_OBJECT(af_);
  if (da) {
    IMP::internal::ContainerTraits<Particle>
      ::apply(af_.get(), v_, da);
  }
  IMP_LOG(TERSE, "End SingletonsScoreState::after_evaluate" << std::endl);
}

ParticlesList SingletonScoreState::get_interacting_particles() const {
  ParticlesList ret0, ret1;
  if (f_) ret0= IMP::internal::get_interacting_particles(v_, f_.get());
  if (af_) ret1= IMP::internal::get_interacting_particles(v_, af_.get());
  ret0.insert(ret0.end(), ret1.begin(), ret1.end());
  return ret0;
}

ParticlesTemp SingletonScoreState::get_used_particles() const {
  ParticlesTemp ret0, ret1;
  if (f_) ret0= IMP::internal::get_used_particles(v_, f_.get());
  if (af_) ret1= IMP::internal::get_used_particles(v_, af_.get());
  ret0.insert(ret0.end(), ret1.begin(), ret1.end());
  return ret0;
}

void SingletonScoreState::show(std::ostream &out) const {
  out << "SingletonScoreState with ";
  if (f_) out << *f_;
  else out << "NULL";
  out << " and ";
  if (af_) out << *af_;
  else out << "NULL";
  out << " on ";
  out << IMP::internal::streamable(v_).get_name() << std::endl;
}

IMPCORE_END_NAMESPACE
