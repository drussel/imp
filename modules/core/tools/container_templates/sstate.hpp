/**
 *  \file GroupnameScoreState.h
 *  \brief Use a GroupnameModifier applied to a Classnames to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_GROUPNAME_SCORE_STATE_H
#define IMPCORE_GROUPNAME_SCORE_STATE_H

#include "config.h"
#include "internal/version_info.h"
#include <IMP/GroupnameModifier.h>
#include <IMP/ScoreState.h>

IMP_BEGIN_NAMESPACE
// for swig
class GroupnameModifier;
IMP_END_NAMESPACE

IMPCORE_BEGIN_NAMESPACE
//! Apply a GroupnameFunction to a Groupname
/** The score state is passed up to two GroupnameModifiers, one to
    apply before evaluation and the other after. The one after
    should take a DerivativeAccumulator as its last argument for
    GroupnameModifier::apply() and will only be called if
    the score was computed with derivatives.

    \ingroup restraint
    \see GroupnamesScoreState
 */
class IMPCOREEXPORT GroupnameScoreState : public ScoreState
{
  Pointer<GroupnameModifier> f_;
  Pointer<GroupnameModifier> af_;
  Value v_;
public:
  /** before and after are the modifiers to apply before and after
      evaluate.
   */
  GroupnameScoreState(GroupnameModifier *before,
                      GroupnameModifier *after, ClassnameArguments);

  //! Apply this modifier to all the elements after an evaluate
  void set_after_evaluate_modifier(GroupnameModifier* f) {
    af_=f;
  }

  //! Apply this modifier to all the elements before an evaluate
  void set_before_evaluate_modifier(GroupnameModifier* f) {
    f_=f;
  }

  IMP_SCORE_STATE(GroupnameScoreState, internal::version_info)
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_GROUPNAME_SCORE_STATE_H */
