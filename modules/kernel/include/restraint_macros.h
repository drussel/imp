/**
 *  \file IMP/restraint_macros.h
 *  \brief Various general useful macros for IMP.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_RESTRAINT_MACROS_H
#define IMPKERNEL_RESTRAINT_MACROS_H
#include <IMP/kernel_config.h>
#include <IMP/base/doxygen_macros.h>
#include <IMP/base/deprecation_macros.h>
#include "internal/scoring_functions.h"
#include "container_base.h"
#include "input_output_macros.h"
#include "constants.h"

/** At this point, you should probably use public:
  void do_add_score_and_derivatives(IMP::ScoreAccumulator sa)
    const IMP_OVERRIDE;
  IMP::ModelObjectsTemp do_get_inputs() const IMP_OVERRIDE;
  IMP_OBJECT_METHODS();
    for new restraints.
*/
#define IMP_RESTRAINT(Name)                                             \
  public:                                                               \
  IMP_IMPLEMENT( double                                                 \
                 unprotected_evaluate(IMP::DerivativeAccumulator *accum)\
                 const);                                                \
  IMP_IMPLEMENT_INLINE(ScoringFunction *                                \
                       create_scoring_function(double weight=1.0,       \
                                               double max               \
                                               = IMP::NO_MAX) const, {  \
                         set_was_used(true);                            \
                         return IMP::internal::create_scoring_function  \
                             (const_cast<Name*>(this),                  \
                              weight, max);                             \
                       });                                              \
  IMP_MODEL_OBJECT_BACKWARDS_MACRO_INPUTS;                              \
  IMP_OBJECT(Name)

#ifdef IMP_USE_DEPRECATED
//! For backwards compatibility
#define IMP_RESTRAINT_2(Name)                                           \
  public:                                                               \
  IMP_IMPLEMENT( double                                                 \
                 unprotected_evaluate(IMP::DerivativeAccumulator *accum) \
                 const);                                                \
  IMP_IMPLEMENT_INLINE(IMP::ScoringFunction *                           \
                       create_scoring_function(double weight=1.0,       \
                                               double max               \
                                               = IMP::NO_MAX) const, {  \
                         return IMP::internal::create_scoring_function  \
                             (const_cast<Name*>(this),                  \
                              weight, max);                             \
                       });                                              \
  IMP_IMPLEMENT(IMP::ModelObjectsTemp do_get_inputs() const);           \
  IMP_OBJECT(Name)

//! For backwards compatability
#define IMP_RESTRAINT_ACCUMULATOR(Name)                                 \
  public:                                                               \
  void do_add_score_and_derivatives(IMP::ScoreAccumulator sa)  const;   \
  IMP_IMPLEMENT_INLINE(IMP::ScoringFunction *                           \
                       create_scoring_function(double weight=1.0,       \
                                               double max               \
                                               = IMP::NO_MAX) const, {  \
                         return IMP::internal::create_scoring_function  \
                             (const_cast<Name*>(this),                  \
                              weight, max);                             \
                       });                                              \
  IMP::ModelObjectsTemp do_get_inputs() const;                          \
  IMP_OBJECT(Name)
#endif

#endif  /* IMPKERNEL_RESTRAINT_MACROS_H */
