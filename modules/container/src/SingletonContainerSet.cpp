/**
 *  \file SingletonContainerSet.cpp
 *  \brief A set of SingletonContainers.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/SingletonContainerSet.h"
#include <algorithm>


IMPCONTAINER_BEGIN_NAMESPACE

SingletonContainerSet
::SingletonContainerSet(Model *m, std::string name):
  SingletonContainer(m, name),
  deps_(new DependenciesScoreState(this), m){
}

namespace {
  Model *my_get_model(const SingletonContainersTemp &in) {
    if (in.empty()) {
      IMP_THROW("Cannot initialize from empty list of containers.",
                IndexException);
    }
    return in[0]->get_model();
  }
}

SingletonContainerSet
::SingletonContainerSet(const SingletonContainersTemp& in,
                        std::string name):
  SingletonContainer(my_get_model(in), name),
  deps_(new DependenciesScoreState(this), my_get_model(in)){
  set_singleton_containers(in);
}


bool
SingletonContainerSet
::get_contains_particle(Particle* vt) const {
  for (SingletonContainerConstIterator it= singleton_containers_begin();
       it != singleton_containers_end(); ++it) {
    if ((*it)->get_contains_particle(vt)) return true;
  }
  return false;
}

void SingletonContainerSet::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << get_number_of_particles()
      << " containers" << std::endl;
}


ParticleIndexes SingletonContainerSet::get_indexes() const {
  ParticleIndexes sum;
  for (SingletonContainerConstIterator it= singleton_containers_begin();
       it != singleton_containers_end(); ++it) {
    ParticleIndexes cur=(*it)->get_indexes();
    sum.insert(sum.end(), cur.begin(), cur.end());
  }
  return sum;
}

ParticleIndexes SingletonContainerSet::get_all_possible_indexes() const {
  ParticleIndexes sum;
  for (SingletonContainerConstIterator it= singleton_containers_begin();
       it != singleton_containers_end(); ++it) {
    ParticleIndexes cur=(*it)->get_all_possible_indexes();
    sum.insert(sum.end(), cur.begin(), cur.end());
  }
  return sum;
}

IMP_LIST_IMPL(SingletonContainerSet,
              SingletonContainer,
              singleton_container,
              SingletonContainer*,
              SingletonContainers);


void SingletonContainerSet::apply(const SingletonModifier *sm) const {
  for (unsigned int i=0; i< get_number_of_singleton_containers(); ++i) {
    get_singleton_container(i)->apply(sm);
  }
}

void SingletonContainerSet::apply(const SingletonDerivativeModifier *sm,
                               DerivativeAccumulator &da) const {
  for (unsigned int i=0; i< get_number_of_singleton_containers(); ++i) {
    get_singleton_container(i)->apply(sm, da);
  }
}

double SingletonContainerSet::evaluate(const SingletonScore *s,
                                       DerivativeAccumulator *da) const {
  return template_evaluate(s, da);
}

double SingletonContainerSet::evaluate_if_good(const SingletonScore *s,
                                               DerivativeAccumulator *da,
                                               double max) const {
  return template_evaluate_if_good(s, da, max);
}


ParticlesTemp SingletonContainerSet::get_contained_particles() const {
  ParticlesTemp ret;
  for (unsigned int i=0; i< get_number_of_singleton_containers(); ++i) {
    ParticlesTemp cur= get_singleton_container(i)
        ->get_contained_particles();
    ret.insert(ret.end(), cur.begin(), cur.end());
  }
  return ret;
}


IMPCONTAINER_END_NAMESPACE
