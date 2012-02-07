/**
 *  \file RemoveInactiveCLASSNAMEsOptimizerState.h
 *  \brief Use a CLASSNAMEModifier applied to a PLURALVARIABLETYPE to
 *  maintain an invariant
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_REMOVE_INACTIVE_HEADERNAMES_OPTIMIZER_STATE_H
#define IMPCONTAINER_REMOVE_INACTIVE_HEADERNAMES_OPTIMIZER_STATE_H

#include "config.h"
#include <IMP/core/ListCLASSNAMEContainer.h>
#include <IMP/OptimizerState.h>

IMPCONTAINER_BEGIN_NAMESPACE


//! Remove inactive CLASSNAMEs from a list
/** This optimizer state can be used to clean up particles which have
    been inactived by other OptimizaterState objects.

    \ingroup restraint
    \see CLASSNAMEOptimizerState
 */
class IMPCONTAINEREXPORT RemoveInactiveCLASSNAMEsOptimizerState :
  public OptimizerState
{
  IMP::OwnerPointer<ListCLASSNAMEContainer> c_;
public:
  /** \param[in] c The Container to hold the elements to process
      \param[in] name The name to use for this Object
   */
  RemoveInactiveCLASSNAMEsOptimizerState(ListCLASSNAMEContainer *c,
                                         std::string name
                                         ="RemoveInactive %1%");

  IMP_OPTIMIZER_STATE(RemoveInactiveCLASSNAMEsOptimizerState,
                      get_module_version_info())
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_REMOVE_INACTIVE_HEADERNAMES_OPTIMIZER_STATE_H */