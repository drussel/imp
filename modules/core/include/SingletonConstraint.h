/**
 *  \file SingletonConstraint.h
 *  \brief Use a SingletonModifier applied to a Particles to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_SINGLETON_CONSTRAINT_H
#define IMPCORE_SINGLETON_CONSTRAINT_H

#include "config.h"
#include <IMP/SingletonModifier.h>
#include <IMP/Constraint.h>
#include <IMP/Particle.h>

IMP_BEGIN_NAMESPACE
// for swig
class SingletonModifier;
IMP_END_NAMESPACE

IMPCORE_BEGIN_NAMESPACE
//! Apply a SingletonFunction to a Singleton
/** The score state is passed up to two SingletonModifiers, one to
    apply before evaluation and the other after. The one after
    should take a DerivativeAccumulator as its last argument for
    SingletonModifier::apply() and will only be called if
    the score was computed with derivatives.

    \see SingletonsConstraint
 */
class IMPCOREEXPORT SingletonConstraint : public Constraint
{
  IMP::internal::OwnerPointer<SingletonModifier> f_;
  IMP::internal::OwnerPointer<SingletonModifier> af_;
  Pointer<Particle> v_;
public:
  /** before and after are the modifiers to apply before and after
      evaluate.
   */
  SingletonConstraint(SingletonModifier *before,
                      SingletonModifier *after, Particle* vt,
                      std::string name="SingletonConstraint %1%");

  //! Apply this modifier to all the elements after an evaluate
  void set_after_evaluate_modifier(SingletonModifier* f) {
    af_=f;
  }

  //! Apply this modifier to all the elements before an evaluate
  void set_before_evaluate_modifier(SingletonModifier* f) {
    f_=f;
  }

  IMP_CONSTRAINT(SingletonConstraint, get_module_version_info())
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_SINGLETON_CONSTRAINT_H */
