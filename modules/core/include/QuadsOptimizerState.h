/**
 *  \file QuadsOptimizerState.h
 *  \brief Use a QuadModifier applied to a ParticleQuads to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_QUADS_OPTIMIZER_STATE_H
#define IMPCORE_QUADS_OPTIMIZER_STATE_H

#include "config.h"
#include <IMP/QuadContainer.h>
#include <IMP/QuadModifier.h>
#include <IMP/OptimizerState.h>

IMP_BEGIN_NAMESPACE
// for swig
class QuadContainer;
class QuadModifier;
IMP_END_NAMESPACE

IMPCORE_BEGIN_NAMESPACE
//! Apply a QuadFunction to a QuadContainer to maintain an invariant
/** \ingroup restraint
    \see QuadOptimizerState
 */
class IMPCOREEXPORT QuadsOptimizerState : public OptimizerState
{
  IMP::internal::OwnerPointer<QuadModifier> f_;
  IMP::internal::OwnerPointer<QuadContainer> c_;
public:
  /** \param[in] c The Container to hold the elements to process
      \param[in] gf The QuadModifier to apply to all elements.
      \param[in] name The name to use for this Object
   */
  QuadsOptimizerState(QuadContainer *c, QuadModifier *gf,
                           std::string name="QuadsOptimizerState %1%");

  IMP_OPTIMIZER_STATE(QuadsOptimizerState, get_module_version_info())
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_QUADS_OPTIMIZER_STATE_H */
