/**
 *  \file BondPairContainer.cpp
 *  \brief A fake container that returns true if a pair of particles are bonded
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/atom/BondPairContainer.h"

IMPATOM_BEGIN_NAMESPACE

BondPairContainer
::BondPairContainer(){
}

bool BondPairContainer
::get_contains_particle_pair(ParticlePair pp) const {
  if (!Bonded::is_instance_of(pp.first)
      || ! Bonded::is_instance_of(pp.second)) {
    return false;
  }

  Bonded ba(pp.first);
  Bonded bb(pp.second);
  Bond bd=get_bond(ba, bb);
  return bd != Bond();
}

unsigned int BondPairContainer
::get_number_of_particle_pairs() const {
  return 0;
}

ParticlePair BondPairContainer
::get_particle_pair(unsigned int i) const {
  throw InvalidStateException("BondPairContainer does" \
                              " not contain any pairs");
  return ParticlePair();
}


void BondPairContainer::show(std::ostream &out) const {
  out << "BondPairContainer" << std::endl;
}

IMPATOM_END_NAMESPACE
