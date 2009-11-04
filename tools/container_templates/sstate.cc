/**
 *  \file GroupnameScoreState.cpp
 *  \brief Use a GroupnameModifier applied to a GroupnameContainer to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/GroupnameScoreState.h"
#include "IMP/internal/container_helpers.h"

IMPCORE_BEGIN_NAMESPACE

GroupnameScoreState::GroupnameScoreState(GroupnameModifier *before,
                                         GroupnameModifier *after,
                                         ClassnameArguments,
                                         std::string name):
  ScoreState(name), v_(FromClassnameArguments){
  if (before) f_=before;
  if (after) af_=after;
}


void GroupnameScoreState::do_before_evaluate()
{
  IMP_OBJECT_LOG;
  if (!f_) return;
  IMP_LOG(TERSE, "Begin GroupnamesScoreState::update" << std::endl);
  IMP_CHECK_OBJECT(f_);
  IMP::internal::ContainerTraits<Classname>
    ::apply(f_.get(), v_);
  IMP_LOG(TERSE, "End GroupnamesScoreState::update" << std::endl);
}

void GroupnameScoreState::do_after_evaluate(DerivativeAccumulator *da)
{
  IMP_OBJECT_LOG;
  if (!af_) return;
  IMP_LOG(TERSE, "Begin GroupnamesScoreState::after_evaluate" << std::endl);
  IMP_CHECK_OBJECT(af_);
  if (da) {
    IMP::internal::ContainerTraits<Classname>
      ::apply(af_.get(), v_, *da);
  }
  IMP_LOG(TERSE, "End GroupnamesScoreState::after_evaluate" << std::endl);
}

ParticlesList GroupnameScoreState::get_interacting_particles() const {
  ParticlesList ret0, ret1;
  if (f_) ret0= IMP::internal::get_interacting_particles(v_, f_.get());
  if (af_) ret1= IMP::internal::get_interacting_particles(v_, af_.get());
  ret0.insert(ret0.end(), ret1.begin(), ret1.end());
  return ret0;
}

ObjectsTemp GroupnameScoreState::get_input_objects() const {
  return ObjectsTemp();
}

ObjectsTemp GroupnameScoreState::get_output_objects() const {
  return ObjectsTemp();
}

ParticlesTemp GroupnameScoreState::get_input_particles() const {
  ParticlesTemp ret;
  if (f_) {
    ret= IMP::internal::get_input_particles(v_, f_.get());
    IMP_IF_CHECK(USAGE) {
      if (af_) {
        ParticlesTemp oret= IMP::internal::get_output_particles(v_, af_.get());
        std::sort(ret.begin(), ret.end());
        std::sort(oret.begin(), oret.end());
        ParticlesTemp t;
        std::set_union(ret.begin(), ret.end(), oret.begin(), oret.end(),
                       std::back_inserter(t));
        IMP_USAGE_CHECK(t.size() == oret.size(), "The particles written by "
                        << " the after modifier in " << get_name() << " must "
                        << "be a subset of those read by the before modifier.",
                        UsageException);
      }
    }
  } else {
    ret= IMP::internal::get_output_particles(v_, af_.get());
  }
  return ret;
}

ParticlesTemp GroupnameScoreState::get_output_particles() const {
   ParticlesTemp ret;
  if (f_) {
    ret= IMP::internal::get_output_particles(v_, f_.get());
    IMP_IF_CHECK(USAGE) {
      if (af_) {
        ParticlesTemp oret= IMP::internal::get_input_particles(v_, af_.get());
        std::sort(ret.begin(), ret.end());
        std::sort(oret.begin(), oret.end());
        ParticlesTemp t;
        std::set_union(ret.begin(), ret.end(), oret.begin(), oret.end(),
                       std::back_inserter(t));
        IMP_USAGE_CHECK(t.size() == oret.size(), "The particles read by "
                      << " the after modifier in " << get_name() << " must "
                        << "be a subset of those written by the before modifier.",
                        UsageException);
      }
    }
  } else {
    ret= IMP::internal::get_input_particles(v_, af_.get()); 
  }
  return ret;
}

void GroupnameScoreState::show(std::ostream &out) const {
  out << "GroupnameScoreState with ";
  if (f_) out << *f_;
  else out << "NULL";
  out << " and ";
  if (af_) out << *af_;
  else out << "NULL";
  out << " on ";
  out << IMP::internal::streamable(v_).get_name() << std::endl;
}

IMPCORE_END_NAMESPACE
