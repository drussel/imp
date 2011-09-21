/**
 *  \file BondPairFilter.cpp
 *  \brief A fake container that returns true if a pair of particles are bonded
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/atom/BondedPairFilter.h"

IMPATOM_BEGIN_NAMESPACE

BondedPairFilter
::BondedPairFilter(): PairFilter("BondedPairFilter%1%"){
}

ParticlesTemp BondedPairFilter
::get_input_particles( Particle* t) const {
  ParticlesTemp ret;
  ret.push_back(t);
  if (Bonded::particle_is_instance(t)) {
    Bonded b(t);
    for (unsigned int i=0;
         i< b.get_number_of_bonds(); ++i) {
      ret.push_back(b.get_bond(i));
    }
  }
  return ret;
}


ContainersTemp
BondedPairFilter::get_input_containers(Particle*p) const {
  return ContainersTemp(1, p);
}
void BondedPairFilter::do_show(std::ostream &) const {
}

IMPATOM_END_NAMESPACE
