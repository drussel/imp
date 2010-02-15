/**
 *  \file CoreGroupnamesRestraint.h
 *  \brief Apply a GroupnameScore to each Classname in a list.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCORE_INTERNAL_CORE_GROUPNAMES_RESTRAINT_H
#define IMPCORE_INTERNAL_CORE_GROUPNAMES_RESTRAINT_H

#include "../config.h"

#include <IMP/Restraint.h>
#include <IMP/Pointer.h>
#include <IMP/GroupnameScore.h>
#include <IMP/GroupnameContainer.h>

#include <iostream>

IMPCORE_BEGIN_INTERNAL_NAMESPACE

//! Applies a GroupnameScore to each Classname in a list.
/** This restraint stores the used particles in a Classnames.
    The container used can be set so that the list can be shared
    with other containers (or a nonbonded list can be used). By default
    a ListGroupnameContainer is used and the
    {add_, set_, clear_}classname{s} methods can be used.

    \see GroupnameRestraint
 */
class IMPCOREEXPORT CoreGroupnamesRestraint : public Restraint
{
  IMP::internal::OwnerPointer<GroupnameScore> ss_;
  IMP::internal::OwnerPointer<GroupnameContainer> pc_;
  mutable double score_;
public:

 //! Create the restraint with a shared container
  /** \param[in] ss The function to apply to each particle.
      \param[in] pc The container containing the stored particles. This
      container is not copied.
      \param[in] name The object name
   */
  CoreGroupnamesRestraint(GroupnameScore *ss,
                      GroupnameContainer *pc,
                      std::string name="GroupnamesRestraint %1%");

  IMP_INCREMENTAL_RESTRAINT(CoreGroupnamesRestraint, get_module_version_info());

  //! Get the container used to store Particles
  GroupnameContainer* get_groupname_container() {
    return pc_;
  }

  GroupnameScore* get_groupname_score() const {
    return ss_;
  }
};

IMPCORE_END_INTERNAL_NAMESPACE

#endif  /* IMPCORE_CORE_GROUPNAMES_RESTRAINT_H */
