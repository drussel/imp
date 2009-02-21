/**
 *  \file GroupnamesScoreState.h
 *  \brief Use a GroupnameModifier applied to a Classnames to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_GROUPNAMES_SCORE_STATE_H
#define IMPCORE_GROUPNAMES_SCORE_STATE_H

#include "config.h"
#include "internal/version_info.h"
#include <IMP/GroupnameContainer.h>
#include <IMP/GroupnameModifier.h>
#include <IMP/ScoreState.h>

IMP_BEGIN_NAMESPACE
// for swig
class GroupnameContainer;
class GroupnameModifier;
IMP_END_NAMESPACE

IMPCORE_BEGIN_NAMESPACE
//! Apply a GroupnameFunction to a GroupnameContainer to maintain an invariant
/** \ingroup restraint
    An example showing a how to use such a score state to maintain a cover
    of the atoms of a protein by a sphere per residue.
    \verbinclude simple_examples/cover_particles.py
 */
class IMPCOREEXPORT GroupnamesScoreState : public ScoreState
{
  Pointer<GroupnameModifier> f_;
  Pointer<GroupnameModifier> af_;
  Pointer<GroupnameContainer> c_;
public:
  /** \param[in] c The Container to hold the elements to process
      \param[in] before The GroupnameModifier to apply to all elements
       before evaluate.
      \param[in] after The GroupnameModifier to apply to all elements
       after evaluate.
   */
  GroupnamesScoreState(GroupnameContainer *c, GroupnameModifier *before,
                       GroupnameModifier *after);

  //! Apply this modifier to all the elements after an evaluate
  void set_after_evaluate_modifier(GroupnameModifier* f) {
    af_=f;
  }

  //! Apply this modifier to all the elements before an evaluate
  void set_before_evaluate_modifier(GroupnameModifier* f) {
    f_=f;
  }

  virtual ~GroupnamesScoreState();

  IMP_SCORE_STATE(internal::version_info)
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_GROUPNAMES_SCORE_STATE_H */
