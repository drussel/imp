/**
 *  \file SingletonsScoreState.cpp
 *  \brief Use a SingletonModifier applied to a SingletonContainer to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/SingletonsScoreState.h"
#include "IMP/core/internal/container_helpers.h"

IMPCORE_BEGIN_NAMESPACE

SingletonsScoreState::SingletonsScoreState(SingletonModifier *f,
                                           SingletonContainer *c):
  f_(f), c_(c){
}

SingletonsScoreState::~SingletonsScoreState(){}


void SingletonsScoreState::do_before_evaluate()
{
  if (!f_) return;
  IMP_LOG(TERSE, "Begin SingletonsScoreState::update" << std::endl);
  IMP_CHECK_OBJECT(f_);
  IMP_CHECK_OBJECT(c_);
  apply(f_.get(), c_.get());
  IMP_LOG(TERSE, "End SingletonsScoreState::update" << std::endl);
}

void SingletonsScoreState::do_after_evaluate()
{
  if (!af_) return;
  IMP_LOG(TERSE, "Begin SingletonsScoreState::after_evaluate" << std::endl);
  IMP_CHECK_OBJECT(af_);
  IMP_CHECK_OBJECT(c_);
  apply(af_.get(), c_.get());
  IMP_LOG(TERSE, "End SingletonsScoreState::after_evaluate" << std::endl);
}

void SingletonsScoreState::show(std::ostream &out) const {
  out << "SingletonsScoreState base" << std::endl;
}

IMPCORE_END_NAMESPACE
