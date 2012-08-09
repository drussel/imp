/**
 *  \file IMP/core/SingletonConstraint.h
 *  \brief Use a SingletonModifier applied to a ParticlesTemp to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_SINGLETON_CONSTRAINT_H
#define IMPCORE_SINGLETON_CONSTRAINT_H

#include "core_config.h"
#include <IMP/internal/TupleConstraint.h>
#include <IMP/score_state_macros.h>

IMPCORE_BEGIN_NAMESPACE
//! Apply a SingletonFunction to a Singleton
/** The score state is passed up to two SingletonModifiers, one to
    apply before evaluation and the other after. The one after
    should take a DerivativeAccumulator as its last argument for
    SingletonModifier::apply() and will only be called if
    the score was computed with derivatives.

    \see container::SingletonsConstraint
 */
class IMPCOREEXPORT SingletonConstraint :
#if defined(IMP_DOXYGEN) || defined(SWIG)
public Constraint
#else
public IMP::internal::TupleConstraint<SingletonModifier,
                                      SingletonDerivativeModifier>
#endif
{
public:
  /** before and after are the modifiers to apply before and after
      evaluate.
   */
  SingletonConstraint(SingletonModifier *before,
                      SingletonDerivativeModifier *after,
                      Particle* vt,
                      std::string name="SingletonConstraint %1%"):
      IMP::internal::TupleConstraint<SingletonModifier,
                                      SingletonDerivativeModifier>
      (before, after, vt, name)
      {
  }

#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_CONSTRAINT(SingletonConstraint);
#endif
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_SINGLETON_CONSTRAINT_H */
