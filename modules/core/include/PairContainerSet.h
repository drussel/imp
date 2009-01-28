/**
 *  \file PairContainerSet.h
 *  \brief Store a set of PairContainers
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_PAIR_CONTAINER_SET_H
#define IMPCORE_PAIR_CONTAINER_SET_H

#include "config.h"
#include "internal/core_version_info.h"
#include <IMP/PairContainer.h>
#include <IMP/container_macros.h>

IMPCORE_BEGIN_NAMESPACE

//! Stores a set of PairContainers
/**
 */
class IMPCOREEXPORT PairContainerSet
  : public PairContainer
{
public:
  //! Construct and empty set
  PairContainerSet();

  virtual ~PairContainerSet();

  IMP_PAIR_CONTAINER(internal::core_version_info);

  IMP_LIST(public, PairContainer, pair_container,
           PairContainer*);
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_PAIR_CONTAINER_SET_H */
