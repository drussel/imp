/**
 *  \file SingletonsScoreState.h
 *  \brief Use a SingletonModifier applied to a Particles to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_SINGLETONS_SCORE_STATE_H
#define IMPCORE_SINGLETONS_SCORE_STATE_H

#include "config.h"
#include "internal/version_info.h"
#include <IMP/SingletonContainer.h>
#include <IMP/SingletonModifier.h>
#include <IMP/ScoreState.h>

IMP_BEGIN_NAMESPACE
// for swig
class SingletonContainer;
class SingletonModifier;
IMP_END_NAMESPACE

IMPCORE_BEGIN_NAMESPACE
//! Apply a SingletonFunction to a SingletonContainer to maintain an invariant
/** \ingroup restraint
    The score state is passed up to two SingletonModifiers, one to
    apply before evaluation and the other after. The one after
    should take a DerivativeAccumulator as its last argument for
    SingletonModifier::apply() and will only be called if
    the score was computed with derivatives.

    An example showing a how to use such a score state to maintain a cover
    of the atoms of a protein by a sphere per residue.
    \htmlinclude cover_particles.py.html
    \see SingletonScoreState
 */
class IMPCOREEXPORT SingletonsScoreState : public ScoreState
{
  Pointer<SingletonModifier> f_;
  Pointer<SingletonModifier> af_;
  Pointer<SingletonContainer> c_;
public:
  /** \param[in] c The Container to hold the elements to process
      \param[in] before The SingletonModifier to apply to all elements
       before evaluate.
      \param[in] after The SingletonModifier to apply to all elements
       after evaluate.
   */
  SingletonsScoreState(SingletonContainer *c, SingletonModifier *before,
                       SingletonModifier *after);

  //! Apply this modifier to all the elements after an evaluate
  void set_after_evaluate_modifier(SingletonModifier* f) {
    af_=f;
  }

  //! Apply this modifier to all the elements before an evaluate
  void set_before_evaluate_modifier(SingletonModifier* f) {
    f_=f;
  }

  IMP_SCORE_STATE(SingletonsScoreState, internal::version_info)
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_SINGLETONS_SCORE_STATE_H */
