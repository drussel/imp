/**
 *  \file GroupnameContainerSet.h
 *  \brief Store a set of GroupnameContainers
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_GROUPNAME_CONTAINER_SET_H
#define IMPCORE_GROUPNAME_CONTAINER_SET_H

#include "config.h"
#include <IMP/GroupnameContainer.h>
#include <IMP/container_macros.h>

IMPCORE_BEGIN_NAMESPACE

//! Stores a set of GroupnameContainers
/**
 */
class IMPCOREEXPORT GroupnameContainerSet
  : public GroupnameContainer
{
  unsigned int rev_;
public:
  //! Construct and empty set
  GroupnameContainerSet(std::string name="GroupnameContainerSet %1%");

  IMP_GROUPNAME_CONTAINER(GroupnameContainerSet, get_module_version_info());
 /** @name Methods to control the nested container

     This container merges a set of nested containers. To add
     or remove nested containers, use the methods below.
  */
  /**@{*/
  IMP_LIST(public, GroupnameContainer, groupname_container,
           GroupnameContainer*, GroupnameContainers);
  /**@}*/
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_GROUPNAME_CONTAINER_SET_H */
