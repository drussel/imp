/**
 *  \file IMP/container/InContainerCLASSNAMEFilter.h
 *  \brief A filter for CLASSNAMEs.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 Sali Lab. All rights reserved.
 */

#ifndef IMPCONTAINER_IN_CONTAINER_HEADERNAME_FILTER_H
#define IMPCONTAINER_IN_CONTAINER_HEADERNAME_FILTER_H

#include "container_config.h"
#include <IMP/CLASSNAMEPredicate.h>
#include <IMP/CLASSNAMEContainer.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/LCCLASSNAME_macros.h>
#include <IMP/base/warning_macros.h>

IMPCONTAINER_BEGIN_NAMESPACE


//! A filter which returns true if a container containers the CLASSNAME
/** This predicate returns 1 if the passed tuple is in the container.
    \note Only the exact tuple, not permutations of it are searched for.

 */
class IMPCONTAINEREXPORT InContainerCLASSNAMEFilter :
    public CLASSNAMEPredicate
{
  IMP::OwnerPointer<CLASSNAMEContainer> c_;
public:
  InContainerCLASSNAMEFilter(CLASSNAMEContainer *c,
                             std::string name="CLASSNAMEFilter %1%");

  IMP_INDEX_HEADERNAME_PREDICATE(InContainerCLASSNAMEFilter,{
      IMP_UNUSED(m);
      return c_->get_contains_index(pi);
    });
};


IMP_OBJECTS(InContainerCLASSNAMEFilter, InContainerCLASSNAMEFilters);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_IN_CONTAINER_HEADERNAME_FILTER_H */