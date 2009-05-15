/**
 *  \file BondEndpointsRefiner.cpp
 *  \brief Return the hierarchy children of a particle.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#include <IMP/atom/BondEndpointsRefiner.h>
#include <IMP/atom/bond_decorators.h>

IMPATOM_BEGIN_NAMESPACE

BondEndpointsRefiner::BondEndpointsRefiner()
{
}


bool BondEndpointsRefiner::get_can_refine(Particle *p) const
{
  return atom::Bond::is_instance_of(p);
}

Particle* BondEndpointsRefiner::get_refined(Particle *p, unsigned int i) const
{
  IMP_assert(get_can_refine(p), "Trying to refine the unrefinable");
  Bond d(p);
  return d.get_bonded(i).get_particle();
}

unsigned int BondEndpointsRefiner::get_number_of_refined(Particle *p) const {
  return 2;
}

const Particles BondEndpointsRefiner::get_refined(Particle *p) const
{
  IMP_assert(get_can_refine(p), "Trying to refine the unrefinable");
  Bond d(p);
  Particles ps(2);
  ps[0]= d.get_bonded(0).get_particle();
  ps[1]= d.get_bonded(1).get_particle();
  return ps;
}

void BondEndpointsRefiner::show(std::ostream &out) const
{
  out << "BondEndpointsRefiner" << std::endl;
}


IMPATOM_END_NAMESPACE
