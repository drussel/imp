/**
 *  \file GroupnameScoreState.cpp
 *  \brief Use a GroupnameModifier applied to a GroupnameContainer to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/GroupnameScoreState.h"
#include "IMP/internal/container_helpers.h"

IMPCORE_BEGIN_NAMESPACE

GroupnameScoreState::GroupnameScoreState(GroupnameModifier *before,
                                         GroupnameModifier *after,
                                         ClassnameArguments):
  v_(FromClassnameArguments){
    if (before) f_=before;
    if (after) af_=after;
}


void GroupnameScoreState::do_before_evaluate()
{
  if (!f_) return;
  IMP_LOG(TERSE, "Begin GroupnamesScoreState::update" << std::endl);
  IMP_CHECK_OBJECT(f_);
  IMP::internal::ContainerTraits<Classname>
    ::apply(f_.get(), v_);
  IMP_LOG(TERSE, "End GroupnamesScoreState::update" << std::endl);
}

void GroupnameScoreState::do_after_evaluate(DerivativeAccumulator *da)
{
  if (!af_) return;
  IMP_LOG(TERSE, "Begin GroupnamesScoreState::after_evaluate" << std::endl);
  IMP_CHECK_OBJECT(af_);
  IMP::internal::ContainerTraits<Classname>
    ::apply(af_.get(), v_, da);
  IMP_LOG(TERSE, "End GroupnamesScoreState::after_evaluate" << std::endl);
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
