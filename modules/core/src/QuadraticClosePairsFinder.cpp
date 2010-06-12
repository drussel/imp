/**
 *  \file QuadraticClosePairsFinder.cpp
 *  \brief Test all pairs.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/QuadraticClosePairsFinder.h"
#include "IMP/core/XYZ.h"
#include <IMP/algebra/Sphere3D.h>
#include <IMP/core/utility.h>
#include <cmath>

IMPCORE_BEGIN_NAMESPACE

QuadraticClosePairsFinder::QuadraticClosePairsFinder():
  ClosePairsFinder("QuadraticCPF"){}

ParticlePairsTemp QuadraticClosePairsFinder
::get_close_pairs(SingletonContainer *ca,
                  SingletonContainer *cb) const {
  set_was_used(true);
  IMP_OBJECT_LOG;
  IMP_LOG(TERSE, "Quadratic add_close_pairs called with "
          << ca->get_number_of_particles() << " and "
          << cb->get_number_of_particles() << std::endl);
  ParticlePairsTemp ret;
  for (SingletonContainer::ParticleIterator it = ca->particles_begin();
       it != ca->particles_end(); ++it) {
    for (SingletonContainer::ParticleIterator it2 = cb->particles_begin();
         it2 != cb->particles_end(); ++it2) {
      if (get_are_close(*it, *it2)) {
        ret.push_back(ParticlePair(*it, *it2));
      }
    }
  }
  return ret;
}

ParticlePairsTemp QuadraticClosePairsFinder
::get_close_pairs(SingletonContainer *c) const {
  set_was_used(true);
  IMP_OBJECT_LOG;
  IMP_LOG(TERSE, "Adding close pairs from "
          << c->get_number_of_particles() << " particles with threshold "
          << get_distance() << std::endl);
  ParticlePairsTemp ret;
  for (SingletonContainer::ParticleIterator it = c->particles_begin();
       it != c->particles_end(); ++it) {
    for (SingletonContainer::ParticleIterator it2 = c->particles_begin();
       it2 != it; ++it2) {
      if (get_are_close(*it, *it2)) {
        ret.push_back(ParticlePair(*it, *it2));
      }
    }
  }
  return ret;
}


bool QuadraticClosePairsFinder::get_are_close(Particle *a, Particle *b) const {
  XYZ da(a);
  XYZ db(b);
  Float ra= get_radius(a);
  Float rb= get_radius(b);
  Float sr= ra+rb+get_distance();
  for (unsigned int i=0; i< 3; ++i) {
    double delta=std::abs(da.get_coordinate(i) - db.get_coordinate(i));
    if (delta >= sr) {
      return false;
    }
  }
  return get_interiors_intersect(algebra::SphereD<3>(da.get_coordinates(),
                                               ra+get_distance()),
                             algebra::SphereD<3>(db.get_coordinates(), rb));
}


void QuadraticClosePairsFinder::do_show(std::ostream &out) const {
  out << "distance " << get_distance() << std::endl;
}


ParticlesTemp
QuadraticClosePairsFinder::get_input_particles(SingletonContainer *sc) const {
  ParticlesTemp ret= sc->get_particles();
  return ret;
}

ParticlesTemp
QuadraticClosePairsFinder::get_input_particles(SingletonContainer *a,
                                               SingletonContainer *b) const {
  ParticlesTemp ret0= a->get_particles();
  ParticlesTemp ret1= b->get_particles();
  ret0.insert(ret0.end(), ret1.begin(), ret1.end());
  return ret0;
}


ContainersTemp
QuadraticClosePairsFinder::get_input_containers(SingletonContainer *sc) const {
  ContainersTemp ret(1,sc);
  return ret;
}

ContainersTemp
QuadraticClosePairsFinder::get_input_containers(SingletonContainer *a,
                                                SingletonContainer *b) const {
  ContainersTemp ret(2);
  ret[0]= a;
  ret[1]= b;
  return ret;
}

IMPCORE_END_NAMESPACE
