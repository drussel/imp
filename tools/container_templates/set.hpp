/**
 *  \file GroupnameContainerSet.h
 *  \brief Store a set of GroupnameContainers
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_GROUPNAME_CONTAINER_SET_H
#define IMPCONTAINER_GROUPNAME_CONTAINER_SET_H

#include "container_config.h"
#include <IMP/GroupnameContainer.h>
#include <IMP/container_macros.h>
#include <IMP/internal/container_helpers.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Stores a set of GroupnameContainers
/** The input sets must be disjoint. This can change if there is
    demand for it.

    \usesconstraint
*/
class IMPCONTAINEREXPORT GroupnameContainerSet
  : public GroupnameContainer
{
  IMP_CONTAINER_DEPENDENCIES(GroupnameContainerSet,
                             {
                               ret.insert(ret.end(),
                                          back_->groupname_containers_begin(),
                                          back_->groupname_containers_end());
                             });
  // to not have added and removed
  GroupnameContainerSet();
 public:
  //! Construct and empty set
  GroupnameContainerSet(Model *m,
                        std::string name="GroupnameContainerSet %1%");

  GroupnameContainerSet(const GroupnameContainersTemp &pc,
                        std::string name="GroupnameContainerSet %1%");

  IMP_GROUPNAME_CONTAINER(GroupnameContainerSet);
  /** @name Methods to control the nested container

      This container merges a set of nested containers. To add
      or remove nested containers, use the methods below.
  */
  /**@{*/
  IMP_LIST(public, GroupnameContainer, groupname_container,
           GroupnameContainer*, GroupnameContainers);
  /**@}*/

  static GroupnameContainerSet *create_untracked_container() {
    GroupnameContainerSet *lsc = new GroupnameContainerSet();
    return lsc;
  }
#ifndef IMP_DOXYGEN
  bool get_is_up_to_date() const {
    for (unsigned int i=0;
         i< get_number_of_groupname_containers(); ++i) {
      if (!get_groupname_container(i)->get_is_up_to_date()) return false;
    }
    return true;
  }
#endif
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_GROUPNAME_CONTAINER_SET_H */
