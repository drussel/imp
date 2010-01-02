/**
 *  \file TripletContainerSet.cpp
 *  \brief A set of TripletContainers.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/TripletContainerSet.h"
#include <algorithm>


IMPCORE_BEGIN_NAMESPACE

namespace {
  TripletContainerSet* get_set(TripletContainer* c) {
    return dynamic_cast<TripletContainerSet*>(c);
  }
}

TripletContainerSet
::TripletContainerSet(bool): TripletContainer("added or removed for set") {
}

TripletContainerSet
::TripletContainerSet(std::string name):
  TripletContainer(name) {
  set_added_and_removed_containers( create_untracked_container(),
                                    create_untracked_container());
}

TripletContainerSet
::TripletContainerSet(const TripletContainers& in,
                        std::string name):
  TripletContainer(name) {
  set_triplet_containers(in);
  set_added_and_removed_containers( create_untracked_container(),
                                    create_untracked_container());
}


bool
TripletContainerSet
::get_contains_particle_triplet(const ParticleTriplet& vt) const {
  for (TripletContainerConstIterator it= triplet_containers_begin();
       it != triplet_containers_end(); ++it) {
    if ((*it)->get_contains_particle_triplet(vt)) return true;
  }
  return false;
}

void TripletContainerSet::show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "TripletContainerSet with "
      << get_number_of_particle_triplets()
      << " particle_triplets." << std::endl;
}

unsigned int
TripletContainerSet::get_number_of_particle_triplets() const {
  unsigned int sum=0;
  for (TripletContainerConstIterator it= triplet_containers_begin();
       it != triplet_containers_end(); ++it) {
    sum+= (*it)->get_number_of_particle_triplets();
  }
  return sum;
}

ParticleTriplet
TripletContainerSet::get_particle_triplet(unsigned int i) const {
  for (TripletContainerConstIterator it= triplet_containers_begin();
       it != triplet_containers_end(); ++it) {
    if ( i >= (*it)->get_number_of_particle_triplets()) {
      i-= (*it)->get_number_of_particle_triplets();
    } else {
      return (*it)->get_particle_triplet(i);
    }
  }
  throw IndexException("out of range");
}



IMP_LIST_IMPL(TripletContainerSet,
              TripletContainer,
              triplet_container,
              TripletContainer*,
              TripletContainers,
              {
                if (!get_is_added_or_removed_container()) {
                  get_set(get_added_triplets_container())
                    ->add_triplet_container(obj
                           ->get_added_triplets_container());
                }
                obj->set_was_owned(true);
              },,
              if (!get_is_added_or_removed_container()) {
                get_set(get_removed_triplets_container())
                  ->add_triplet_container(obj
                       ->get_removed_triplets_container());
              })


void TripletContainerSet::apply(const TripletModifier *sm) {
  for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
    get_triplet_container(i)->apply(sm);
  }
}

void TripletContainerSet::apply(const TripletModifier *sm,
                               DerivativeAccumulator &da) {
  for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
    get_triplet_container(i)->apply(sm, da);
  }
}

double TripletContainerSet::evaluate(const TripletScore *s,
                                       DerivativeAccumulator *da) const {
  double score=0;
  for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
    score+=get_triplet_container(i)->evaluate(s, da);
  }
  return score;
}


double TripletContainerSet::evaluate_change(const TripletScore *s,
                                              DerivativeAccumulator *da) const {
  double score=0;
  for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
    score+=get_triplet_container(i)->evaluate_change(s, da);
  }
  return score;
}

double TripletContainerSet::evaluate_prechange(const TripletScore *s,
                                             DerivativeAccumulator *da) const {
  double score=0;
  for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
    score+=get_triplet_container(i)->evaluate_prechange(s, da);
  }
  return score;
}



ParticleTripletsTemp TripletContainerSet::get_particle_triplets() const {
  ParticleTripletsTemp ret;
  for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
    ParticleTripletsTemp c= get_triplet_container(i)->get_particle_triplets();
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}


ContainersTemp TripletContainerSet::get_input_containers() const {
  return ContainersTemp(triplet_containers_begin(),
                        triplet_containers_end());
}

ParticlesTemp TripletContainerSet::get_contained_particles() const {
  ParticlesTemp ret;
  for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
    ParticlesTemp cur= get_triplet_container(i)->get_contained_particles();
    ret.insert(ret.end(), cur.begin(), cur.end());
  }
  return ret;
}

bool TripletContainerSet::get_contained_particles_changed() const {
  for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
    if (get_triplet_container(i)->get_contained_particles_changed()) {
      return true;
    }
  }
  return false;
}


IMPCORE_END_NAMESPACE
