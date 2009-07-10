/**
 *  \file RemoveInactiveGroupnamesOptimizerState.h
 *  \brief Use a GroupnameModifier applied to a Classnames to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_REMOVE_INACTIVE_GROUPNAMES_OPTIMIZER_STATE_H
#define IMPCORE_REMOVE_INACTIVE_GROUPNAMES_OPTIMIZER_STATE_H

#include "config.h"
#include "internal/version_info.h"
#include <IMP/core/ListGroupnameContainer.h>
#include <IMP/OptimizerState.h>

IMP_BEGIN_NAMESPACE
// for swig
class ListGroupnameContainer;
IMP_END_NAMESPACE

IMPCORE_BEGIN_NAMESPACE
//! Remove inactive Groupnames from a list
/** This optimizer state can be used to clean up particles which have
    been inactived by other OptimizaterState objects.

    \ingroup restraint
    \see GroupnameOptimizerState
 */
class IMPCOREEXPORT RemoveInactiveGroupnamesOptimizerState :
  public OptimizerState
{
  Pointer<ListGroupnameContainer> c_;
public:
  /** \param[in] c The Container to hold the elements to process
   */
  RemoveInactiveGroupnamesOptimizerState(ListGroupnameContainer *c);

  IMP_OPTIMIZER_STATE(RemoveInactiveGroupnamesOptimizerState,
                      internal::version_info)
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_REMOVE_INACTIVE_GROUPNAMES_OPTIMIZER_STATE_H */
