/**
 *  \file PairContainerSet.h
 *  \brief Store a set of PairContainers
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCONTAINER_PAIR_CONTAINER_SET_H
#define IMPCONTAINER_PAIR_CONTAINER_SET_H

#include "config.h"
#include <IMP/PairContainer.h>
#include <IMP/container_macros.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Stores a set of PairContainers
/** The input sets must be disjoint. This can change if there is
    demand for it.

    \usesconstraint
 */
class IMPCONTAINEREXPORT PairContainerSet
  : public PairContainer
{
 // to not have added and removed
 PairContainerSet(bool);
public:
  //! Construct and empty set
  PairContainerSet(std::string name="PairContainerSet %1%");

  PairContainerSet(const PairContainers &in,
                        std::string name="PairContainerSet %1%");

  IMP_PAIR_CONTAINER(PairContainerSet);
 /** @name Methods to control the nested container

     This container merges a set of nested containers. To add
     or remove nested containers, use the methods below.
  */
  /**@{*/
  IMP_LIST(public, PairContainer, pair_container,
           PairContainer*, PairContainers);
  /**@}*/

  static PairContainerSet *create_untracked_container() {
    PairContainerSet *lsc = new PairContainerSet(false);
    return lsc;
  }
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_PAIR_CONTAINER_SET_H */
