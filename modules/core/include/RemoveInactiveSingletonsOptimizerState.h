/**
 *  \file RemoveInactiveSingletonsOptimizerState.h
 *  \brief Use a SingletonModifier applied to a Particles to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_REMOVE_INACTIVE_SINGLETONS_OPTIMIZER_STATE_H
#define IMPCORE_REMOVE_INACTIVE_SINGLETONS_OPTIMIZER_STATE_H

#include "config.h"
#include "internal/version_info.h"
#include <IMP/core/ListSingletonContainer.h>
#include <IMP/OptimizerState.h>

IMPCORE_BEGIN_NAMESPACE


//! Remove inactive Singletons from a list
/** This optimizer state can be used to clean up particles which have
    been inactived by other OptimizaterState objects.

    \ingroup restraint
    \see SingletonOptimizerState
 */
class IMPCOREEXPORT RemoveInactiveSingletonsOptimizerState :
  public OptimizerState
{
  Pointer<ListSingletonContainer> c_;
public:
  /** \param[in] c The Container to hold the elements to process
   */
  RemoveInactiveSingletonsOptimizerState(ListSingletonContainer *c,
                                         std::string name
                                         ="RemoveInactive %1%");

  IMP_OPTIMIZER_STATE(RemoveInactiveSingletonsOptimizerState,
                      internal::version_info)
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_REMOVE_INACTIVE_SINGLETONS_OPTIMIZER_STATE_H */
