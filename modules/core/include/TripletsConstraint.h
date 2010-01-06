/**
 *  \file TripletsConstraint.h
 *  \brief Use a TripletModifier applied to a ParticleTriplets to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_TRIPLETS_CONSTRAINT_H
#define IMPCORE_TRIPLETS_CONSTRAINT_H

#include "config.h"
#include <IMP/TripletContainer.h>
#include <IMP/TripletModifier.h>
#include <IMP/Constraint.h>

IMP_BEGIN_NAMESPACE
// for swig
class TripletContainer;
class TripletModifier;
IMP_END_NAMESPACE

IMPCORE_BEGIN_NAMESPACE
//! Apply a TripletFunction to a TripletContainer to maintain an invariant
/** \advanced

    The score state is passed up to two TripletModifiers, one to
    apply before evaluation and the other after. The one after
    should take a DerivativeAccumulator as its last argument for
    TripletModifier::apply() and will only be called if
    the score was computed with derivatives.

    An example showing a how to use such a score state to maintain a cover
    of the atoms of a protein by a sphere per residue.
    \verbinclude cover_particles.py

    \see TripletConstraint
 */
class IMPCOREEXPORT TripletsConstraint : public Constraint
{
  IMP::internal::OwnerPointer<TripletModifier> f_;
  IMP::internal::OwnerPointer<TripletModifier> af_;
  IMP::internal::OwnerPointer<TripletContainer> c_;
public:
  /** \param[in] c The Container to hold the elements to process
      \param[in] before The TripletModifier to apply to all elements
      before evaluate.
      \param[in] after The TripletModifier to apply to all elements
      after evaluate.
      \param[in] name The object name
   */
  TripletsConstraint(TripletContainer *c, TripletModifier *before,
                       TripletModifier *after,
                       std::string name="TripletConstraint %1%");

  //! Apply this modifier to all the elements after an evaluate
  void set_after_evaluate_modifier(TripletModifier* f) {
    af_=f;
  }

  //! Apply this modifier to all the elements before an evaluate
  void set_before_evaluate_modifier(TripletModifier* f) {
    f_=f;
  }

  IMP_CONSTRAINT(TripletsConstraint, get_module_version_info())
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_TRIPLETS_CONSTRAINT_H */
