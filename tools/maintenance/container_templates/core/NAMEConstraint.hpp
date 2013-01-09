/**
 *  \file IMP/core/CLASSNAMEConstraint.h
 *  \brief Use a CLASSNAMEModifier applied to a PLURALVARIABLETYPE to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_HEADERNAME_CONSTRAINT_H
#define IMPCORE_HEADERNAME_CONSTRAINT_H

#include <IMP/core/core_config.h>
#include <IMP/internal/TupleConstraint.h>
#include <IMP/CLASSNAMEModifier.h>
#include <IMP/CLASSNAMEDerivativeModifier.h>
#include <IMP/score_state_macros.h>

IMPCORE_BEGIN_NAMESPACE
//! Apply a CLASSNAMEFunction to a CLASSNAME
/** The score state is passed up to two CLASSNAMEModifiers, one to
    apply before evaluation and the other after. The one after
    should take a DerivativeAccumulator as its last argument for
    CLASSNAMEModifier::apply() and will only be called if
    the score was computed with derivatives.

    \see container::CLASSNAMEsConstraint
 */
class IMPCOREEXPORT CLASSNAMEConstraint :
#if defined(IMP_DOXYGEN) || defined(SWIG)
public Constraint
#else
public IMP::internal::TupleConstraint<CLASSNAMEModifier,
                                      CLASSNAMEDerivativeModifier>
#endif
{
public:
  /** before and after are the modifiers to apply before and after
      evaluate.
   */
  CLASSNAMEConstraint(CLASSNAMEModifier *before,
                      CLASSNAMEDerivativeModifier *after,
                      ARGUMENTTYPE vt,
                      std::string name="CLASSNAMEConstraint %1%"):
      IMP::internal::TupleConstraint<CLASSNAMEModifier,
                                      CLASSNAMEDerivativeModifier>
      (before, after, vt, name)
      {
  }

#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_CONSTRAINT(CLASSNAMEConstraint);
#endif
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_HEADERNAME_CONSTRAINT_H */
