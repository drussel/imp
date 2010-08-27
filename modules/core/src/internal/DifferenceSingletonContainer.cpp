/**
 *  \file DifferenceSingletonContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/internal/DifferenceSingletonContainer.h"
#include <IMP/SingletonModifier.h>
#include <IMP/SingletonContainer.h>
#include <IMP/Particle.h>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

DifferenceSingletonContainer
::DifferenceSingletonContainer(SingletonContainer *a,
                               SingletonContainer *b):
  a_(a),
  b_(b),
  deps_(new DependenciesScoreState(this), a_->get_model()){
}

unsigned int DifferenceSingletonContainer::get_number_of_particles() const {
  return a_->get_number_of_particles()-b_->get_number_of_particles();
}

Particle* DifferenceSingletonContainer::get_particle(unsigned int i) const {
  // cache this later
  int as=-1, ai=0;
  while (as != static_cast<int>(i)) {
    if (!b_->get_contains_particle(a_->get_particle(ai))) {
      ++as;
    }
    ++ai;
  }
  return a_->get_particle(ai);
}

bool
DifferenceSingletonContainer::get_contains_particle(Particle* vt) const {
  return a_->get_contains_particle(vt)
      && !b_->get_contains_particle(vt);
}

void DifferenceSingletonContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "containers " << a_ << " and " << *b_;
}

ParticlesTemp DifferenceSingletonContainer::get_contained_particles() const {
  ParticlesTemp ret= a_->get_contained_particles();
  ParticlesTemp b= b_->get_contained_particles();
  ret.insert(ret.end(), b.begin(), b.end());
  return ret;
}

bool DifferenceSingletonContainer::get_contained_particles_changed() const {
  return a_->get_contained_particles_changed()
    || b_->get_contained_particles_changed();
}


IMPCORE_END_INTERNAL_NAMESPACE
