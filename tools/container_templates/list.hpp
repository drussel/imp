/**
 *  \file ListGroupnameContainer.h    \brief Store a list of Classnames
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_LIST_GROUPNAME_CONTAINER_H
#define IMPCORE_LIST_GROUPNAME_CONTAINER_H

#include "config.h"
#include <IMP/GroupnameContainer.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/core/internal/groupname_helpers.h>
#include <IMP/ScoreState.h>

IMPCORE_BEGIN_NAMESPACE

//! Store a list of Classnames
/** \note The indexes can change when particles are inserted
    as the list is maintained in sorted order.
 */
class IMPCOREEXPORT ListGroupnameContainer:
#if defined(IMP_DOXYGEN) || defined(SWIG)
public GroupnameContainer
#else
public internal::ListLikeGroupnameContainer
#endif
{
  IMP_ACTIVE_CONTAINER_DECL(ListGroupnameContainer);
  // for the change versions
  ListGroupnameContainer(bool);
public:
  //! construct and pass an initial set of classnames
  ListGroupnameContainer(const Classnames &ps,
                         std::string name= "ListGroupnameContainer %1%");

  ListGroupnameContainer(std::string name= "ListGroupnameContainer %1%");
  ListGroupnameContainer(const char *name);

 /** @name Methods to control the contained objects

     This container stores a list of Classname objects. To manipulate
     the list use these methods.
  */
  /**@{*/
  void add_classname(PassValue vt);
  void add_classnames(const ClassnamesTemp &c);
  void set_classnames(ClassnamesTemp c);
  IMP_NO_DOXYGEN(void add_classnames(const Classnames &c) {
      add_classnames(static_cast<const ClassnamesTemp&>(c));
    })
  IMP_NO_DOXYGEN(void set_classnames(const Classnames &c) {
      set_classnames(static_cast<ClassnamesTemp>(c));
    })
  void clear_classnames();
  /**@}*/

  static ListGroupnameContainer *create_untracked_container() {
    ListGroupnameContainer *lsc = new ListGroupnameContainer(false);
    return lsc;
  }
#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_GROUPNAME_CONTAINER(ListGroupnameContainer, get_module_version_info());
#else
  IMP_LISTLIKE_GROUPNAME_CONTAINER(ListGroupnameContainer,
                                   get_module_version_info());
#endif
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_LIST_GROUPNAME_CONTAINER_H */
