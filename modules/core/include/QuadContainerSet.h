/**
 *  \file QuadContainerSet.h
 *  \brief Store a set of QuadContainers
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_QUAD_CONTAINER_SET_H
#define IMPCORE_QUAD_CONTAINER_SET_H

#include "config.h"
#include <IMP/QuadContainer.h>
#include <IMP/container_macros.h>

IMPCORE_BEGIN_NAMESPACE

//! Stores a set of QuadContainers
/**
 */
class IMPCOREEXPORT QuadContainerSet
  : public QuadContainer
{
 // to not have added and removed
 QuadContainerSet(bool);
public:
  //! Construct and empty set
  QuadContainerSet(std::string name="QuadContainerSet %1%");

  QuadContainerSet(const QuadContainers &in,
                        std::string name="QuadContainerSet %1%");

  IMP_QUAD_CONTAINER(QuadContainerSet, get_module_version_info());
 /** @name Methods to control the nested container

     This container merges a set of nested containers. To add
     or remove nested containers, use the methods below.
  */
  /**@{*/
  IMP_LIST(public, QuadContainer, quad_container,
           QuadContainer*, QuadContainers);
  /**@}*/

  static QuadContainerSet *create_untracked_container() {
    QuadContainerSet *lsc = new QuadContainerSet(false);
    return lsc;
  }
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_QUAD_CONTAINER_SET_H */
