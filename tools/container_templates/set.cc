/**
 *  \file GroupnameContainerSet.cpp
 *  \brief A set of GroupnameContainers.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/GroupnameContainerSet.h"
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE

namespace {
  GroupnameContainerSet* get_set(GroupnameContainer* c) {
    return dynamic_cast<GroupnameContainerSet*>(c);
  }
}

GroupnameContainerSet
::GroupnameContainerSet() {
}

GroupnameContainerSet
::GroupnameContainerSet(Model *m, std::string name):
  GroupnameContainer(m, name),
  deps_(new DependenciesScoreState(this), m){
  set_added_and_removed_containers( create_untracked_container(),
                                    create_untracked_container());
}

namespace {
  Model *my_get_model(const GroupnameContainersTemp &in) {
    if (in.empty()) {
      IMP_THROW("Cannot initialize from empty list of containers.",
                IndexException);
    }
    return in[0]->get_model();
  }
}

GroupnameContainerSet
::GroupnameContainerSet(const GroupnameContainersTemp& in,
                        std::string name):
  GroupnameContainer(my_get_model(in), name),
  deps_(new DependenciesScoreState(this), my_get_model(in)){
  set_groupname_containers(in);
  set_added_and_removed_containers( create_untracked_container(),
                                    create_untracked_container());
}


bool
GroupnameContainerSet
::get_contains_classname(PassValue vt) const {
  for (GroupnameContainerConstIterator it= groupname_containers_begin();
       it != groupname_containers_end(); ++it) {
    if ((*it)->get_contains_classname(vt)) return true;
  }
  return false;
}

void GroupnameContainerSet::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << get_number_of_classnames()
      << " containers" << std::endl;
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
              GroupnameContainer*,
              GroupnameContainers,
              {
                if (!get_is_added_or_removed_container()) {
                  get_set(get_added_groupnames_container())
                    ->add_groupname_container(obj
                           ->get_added_groupnames_container());
                }
                obj->set_was_used(true);
              },{},
              if (container
                  && !container->get_is_added_or_removed_container()) {
                get_set(container->get_removed_groupnames_container())
                  ->add_groupname_container(obj
                       ->get_removed_groupnames_container());
              });


void GroupnameContainerSet::apply(const GroupnameModifier *sm) {
  for (unsigned int i=0; i< get_number_of_groupname_containers(); ++i) {
    get_groupname_container(i)->apply(sm);
  }
}

void GroupnameContainerSet::apply(const GroupnameModifier *sm,
                               DerivativeAccumulator &da) {
  for (unsigned int i=0; i< get_number_of_groupname_containers(); ++i) {
    get_groupname_container(i)->apply(sm, da);
  }
}

double GroupnameContainerSet::evaluate(const GroupnameScore *s,
                                       DerivativeAccumulator *da) const {
  double score=0;
  for (unsigned int i=0; i< get_number_of_groupname_containers(); ++i) {
    score+=get_groupname_container(i)->evaluate(s, da);
  }
  return score;
}


double GroupnameContainerSet::evaluate_change(const GroupnameScore *s,
                                              DerivativeAccumulator *da) const {
  double score=0;
  for (unsigned int i=0; i< get_number_of_groupname_containers(); ++i) {
    score+=get_groupname_container(i)->evaluate_change(s, da);
  }
  return score;
}

double GroupnameContainerSet::evaluate_prechange(const GroupnameScore *s,
                                             DerivativeAccumulator *da) const {
  double score=0;
  for (unsigned int i=0; i< get_number_of_groupname_containers(); ++i) {
    score+=get_groupname_container(i)->evaluate_prechange(s, da);
  }
  return score;
}


ParticlesTemp GroupnameContainerSet::get_contained_particles() const {
  ParticlesTemp ret;
  for (unsigned int i=0; i< get_number_of_groupname_containers(); ++i) {
    ParticlesTemp cur= get_groupname_container(i)->get_contained_particles();
    ret.insert(ret.end(), cur.begin(), cur.end());
  }
  return ret;
}

bool GroupnameContainerSet::get_contained_particles_changed() const {
  for (unsigned int i=0; i< get_number_of_groupname_containers(); ++i) {
    if (get_groupname_container(i)->get_contained_particles_changed()) {
      return true;
    }
  }
  return false;
}


IMPCONTAINER_END_NAMESPACE
