/**
 *  \file GroupnamesScoreState.cpp
 *  \brief Use a GroupnameModifier applied to a GroupnameContainer to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/GroupnamesScoreState.h"
#include <utility>

IMPCORE_BEGIN_NAMESPACE

GroupnamesScoreState::GroupnamesScoreState(GroupnameContainer *c,
                                           GroupnameModifier *before,
                                           GroupnameModifier *after,
                                           std::string name):
  ScoreState(name), c_(c) {
  if (before) f_=before;
  if (after) af_=after;
}


void GroupnamesScoreState::do_before_evaluate()
{
  IMP_OBJECT_LOG;
  if (!f_) return;
  IMP_LOG(TERSE, "Begin GroupnamesScoreState::update" << std::endl);
  IMP_CHECK_OBJECT(f_);
  IMP_CHECK_OBJECT(c_);
  c_->apply(f_);
  IMP_LOG(TERSE, "End GroupnamesScoreState::update" << std::endl);
}

void GroupnamesScoreState::do_after_evaluate(DerivativeAccumulator *da)
{
  IMP_OBJECT_LOG;
  if (!af_) return;
  IMP_LOG(TERSE, "Begin GroupnamesScoreState::after_evaluate" << std::endl);
  IMP_CHECK_OBJECT(af_);
  IMP_CHECK_OBJECT(c_);
  if (da) c_->apply(af_, *da);
  IMP_LOG(TERSE, "End GroupnamesScoreState::after_evaluate" << std::endl);
}

ParticlesList GroupnamesScoreState::get_interacting_particles() const {
  IMP_failure("not implemented", ErrorException);
  return ParticlesList();
}

void GroupnamesScoreState::show(std::ostream &out) const {
  out << "GroupnamesScoreState base" << std::endl;
}

IMPCORE_END_NAMESPACE
