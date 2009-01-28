/**
 *  \file GroupnameContainerSet.cpp
 *  \brief A set of GroupnameContainers.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/GroupnameContainerSet.h"
#include <algorithm>


IMPCORE_BEGIN_NAMESPACE

GroupnameContainerSet
::GroupnameContainerSet(){
}

GroupnameContainerSet::~GroupnameContainerSet(){}

bool
GroupnameContainerSet
::get_contains_classname(Value vt) const {
  for (GroupnameContainerConstIterator it= groupname_containers_begin();
       it != groupname_containers_end(); ++it) {
    if ((*it)->get_contains_classname(vt)) return true;
  }
  return false;
}

void GroupnameContainerSet::show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "GroupnameContainerSet with "
      << get_number_of_classnames()
      << " classnames." << std::endl;
}

unsigned int
GroupnameContainerSet::get_number_of_classnames() const {
  unsigned int sum=0;
  for (GroupnameContainerConstIterator it= groupname_containers_begin();
       it != groupname_containers_end(); ++it) {
    sum+= (*it)->get_number_of_classnames();
  }
  return sum;
}

Value
GroupnameContainerSet::get_classname(unsigned int i) const {
  for (GroupnameContainerConstIterator it= groupname_containers_begin();
       it != groupname_containers_end(); ++it) {
    if ( i >= (*it)->get_number_of_classnames()) {
      i-= (*it)->get_number_of_classnames();
    } else {
      return (*it)->get_classname(i);
    }
  }
  throw IndexException("out of range");
}



IMP_LIST_IMPL(GroupnameContainerSet,
              GroupnameContainer,
              groupname_container,
              GroupnameContainer*,,,)

IMPCORE_END_NAMESPACE
