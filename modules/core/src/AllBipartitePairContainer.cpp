/**
 *  \file AllBipartitePairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. AllBipartite rights reserved.
 *
 */

#include "IMP/core/AllBipartitePairContainer.h"
#include <IMP/core/internal/DifferenceSingletonContainer.h>
#include <IMP/PairModifier.h>
#include <IMP/SingletonModifier.h>
#include <IMP/PairScore.h>
#include <IMP/SingletonScore.h>
#include <algorithm>


#define FOREACH(expr)                                                   \
  unsigned int sza=a_->get_number_of_particles();                       \
  unsigned int szb=b_->get_number_of_particles();                       \
  for (unsigned int i=0; i< sza; ++i) {                                 \
    Particle *a= a_->get_particle(i);                                   \
    for (unsigned int j=0; j< szb; ++j) {                               \
      ParticlePair p(a, b_->get_particle(j));                           \
      expr;                                                             \
    }                                                                   \
  }


IMPCORE_BEGIN_NAMESPACE

AllBipartitePairContainer
::AllBipartitePairContainer( SingletonContainer *a,
                                  SingletonContainer *b,
                                  bool): a_(a), b_(b) {
}

AllBipartitePairContainer
::AllBipartitePairContainer( SingletonContainer *a,
                                  SingletonContainer *b): a_(a), b_(b) {
  SingletonContainer *olda= new internal::DifferenceSingletonContainer(a_,
                             a_->get_removed_singletons_container());
  SingletonContainer *oldb= new internal::DifferenceSingletonContainer(b_,
                                   b_->get_removed_singletons_container());

  PairContainerSet* removed
    = PairContainerSet::create_untracked_container();
  {
    PairContainer *all
      = AllBipartitePairContainer
      ::create_untracked_container(a_->get_removed_singletons_container(),
                                   b_->get_removed_singletons_container());
    removed->add_pair_container(all);
    PairContainer *leftr
      = AllBipartitePairContainer
      ::create_untracked_container(a_->get_removed_singletons_container(),
                                   oldb);
    removed->add_pair_container(leftr);
    PairContainer *rightr
      = AllBipartitePairContainer::create_untracked_container(olda,
                                   b_->get_removed_singletons_container());
    removed->add_pair_container(rightr);
  }
  PairContainerSet* added
    = PairContainerSet::create_untracked_container();
  {
    PairContainer *all
      =AllBipartitePairContainer
      ::create_untracked_container(a_->get_added_singletons_container(),
                                   b_->get_added_singletons_container());
    added->add_pair_container(all);
    PairContainer *leftr
      = AllBipartitePairContainer
      ::create_untracked_container(a_->get_added_singletons_container(),
                                   oldb);
    added->add_pair_container(leftr);
    PairContainer *rightr
      = AllBipartitePairContainer
      ::create_untracked_container(olda,
                                   b_->get_added_singletons_container());
    added->add_pair_container(rightr);
  }
  set_added_and_removed_containers(added, removed);
}

unsigned int
AllBipartitePairContainer::get_number_of_particle_pairs() const {
  return a_->get_number_of_particles()*b_->get_number_of_particles();
}

ParticlePair AllBipartitePairContainer
::get_particle_pair(unsigned int i) const {
  unsigned int a= i/ a_->get_number_of_particles();
  unsigned int b= i% a_->get_number_of_particles();
  return ParticlePair(a_->get_particle(a), b_->get_particle(b));
}

bool
AllBipartitePairContainer
::get_contains_particle_pair(const ParticlePair &p) const {
  return a_->get_contains_particle(p[0])
      && b_->get_contains_particle(p[1]);
}

void AllBipartitePairContainer::show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "AllBipartitePairContainer on ";
  a_->show(out);
  out << " and ";
  b_->show(out);
}


ContainersTemp AllBipartitePairContainer::get_input_containers() const {
  ContainersTemp ret(2);
  ret[0]=a_;
  ret[1]=b_;
  return ret;
}


ParticlesTemp AllBipartitePairContainer::get_contained_particles() const {
  ParticlesTemp ret= a_->get_contained_particles();
  ParticlesTemp b= b_->get_contained_particles();
  ret.insert(ret.end(), b.begin(), b.end());
  return ret;
}

bool AllBipartitePairContainer::get_contained_particles_changed() const {
  return a_->get_contained_particles_changed()
    || b_->get_contained_particles_changed();
}



IMP_PAIR_CONTAINER_METHODS_FROM_FOREACH(AllBipartitePairContainer);



IMPCORE_END_NAMESPACE
