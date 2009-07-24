/**
 *  \file ListGroupnameContainer.h    \brief Store a list of Classnames
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_LIST_GROUPNAME_CONTAINER_H
#define IMPCORE_LIST_GROUPNAME_CONTAINER_H

#include "config.h"
#include "internal/version_info.h"
#include <IMP/GroupnameContainer.h>
#include <IMP/container_macros.h>

IMPCORE_BEGIN_NAMESPACE

//! Store a list of Classnames
/** \note The indexes can change when particles are inserted
    as the list is maintained in sorted order.
 */
class IMPCOREEXPORT ListGroupnameContainer : public GroupnameContainer
{
  bool sorted_;
public:
  //! construct and pass an initial set of classnames
  ListGroupnameContainer(const Classnames &ps= Classnames());
 /** @name Methods to control the contained objects

     This container stores a list of Classname objects. To manipulate
     the list use these methods.
  */
  /**@{*/
  IMP_LIST(public, Classname, classname, Value, Classnames);
  /**@}*/

  //! log n time
  virtual bool get_contains_classname(Value vt) const;

  /** @name Faster editing

      The container keeps it list of elements in a sorted order.
      As this can make for slow insertions, the user has the option
      of disabling the sorting while inserting many objects. To do this,
      call
      \code
      set_is_editing(true);
      // do stuff
      set_is_editing(false);
      \endcode
      \see ListGroupnameContainerEditor
      @{
   */
  void set_is_editing( bool tf);

 bool get_is_editing() const {
    return !sorted_;
  }
  /** @}*/

  IMP::VersionInfo get_version_info() const {
    return internal::version_info;
  }

  virtual void show(std::ostream &out = std::cout) const;

  virtual void apply(const GroupnameModifier *sm);

  virtual void apply(const GroupnameModifier *sm, DerivativeAccumulator *da);

  virtual double evaluate(const GroupnameScore *s,
                          DerivativeAccumulator *da) const;

  virtual Classnames get_classnames() const;


  // for some reason swig gets this wrong
  //IMP_REF_COUNTED_DESTRUCTOR(ListGroupnameContainer)
  ~ListGroupnameContainer(){}
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_LIST_GROUPNAME_CONTAINER_H */
