/**
 *  \file container/EventCLASSNAMEsOptimizerState.h
 *  \brief Define some predicates.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_EVENT_HEADERNAMES_OPTIMIZER_STATE_H
#define IMPCONTAINER_EVENT_HEADERNAMES_OPTIMIZER_STATE_H

#include "container_config.h"
#include <IMP/CLASSNAMEPredicate.h>
#include <IMP/OptimizerState.h>
#include <IMP/CLASSNAMEContainer.h>
#include <IMP/optimizer_state_macros.h>

IMPCONTAINER_BEGIN_NAMESPACE

/** Raise an IMP::base::EventException when a certain condition is met.
    Currently the supported logic is when the number of items in the
    container for which the predicate returns a certain value is in the
    range [min_count, max_count).
 */
class IMPCONTAINEREXPORT EventCLASSNAMEsOptimizerState:
    public OptimizerState {
  IMP::OwnerPointer<CLASSNAMEPredicate> pred_;
  IMP::OwnerPointer<CLASSNAMEContainer> container_;
  int v_;
  int min_, max_;
public:
  EventCLASSNAMEsOptimizerState(CLASSNAMEPredicate *pred,
                                CLASSNAMEContainerAdaptor container,
                                int value, int min_count, int max_count,
                                std::string name="ConstCLASSNAMEPredicate%1%");
  IMP_OPTIMIZER_STATE(EventCLASSNAMEsOptimizerState);
};

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_EVENT_HEADERNAMES_OPTIMIZER_STATE_H */
